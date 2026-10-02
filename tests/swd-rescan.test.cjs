const fs = require('node:fs');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const { test } = require('node:test');

// Execute the actual known-pin branch with a mocked wire transport.
const source = fs.readFileSync('main/swd.c', 'utf8');
const flipperSource = fs.existsSync('fz.txt') ? fs.readFileSync('fz.txt', 'utf8') : null;

function transferWaveform(text, write, ack = 1) {
    function body(name) {
        const definition = new RegExp(`static [^\\n]*\\b${name}\\([^;\\n]+\\)\\s*\\{`).exec(text);
        assert.ok(definition, name);
        const begin = text.indexOf('{', definition.index);
        return text.slice(begin, text.indexOf('\n}', begin) + 2)
            .replace('uint8_t request[] = {0};', 'let request = [0];')
            .replaceAll('ctx->', 'ctx.')
            .replaceAll('(uint8_t*)&idle', '[idle]')
            .replaceAll('sizeof(request)', 'request.length')
            .replace(/\b(?:uint8_t|uint32_t|size_t|bool)\s+/g, 'let ')
            .replace(/\bint (?=\w+)/g, 'let ')
            .replace(/\b1U\b/g, '1')
            .replace(/\*data\b/g, 'data.value')
            .replaceAll('data.value >>= 1', 'data.value >>>= 1');
    }
    const events = [];
    const samples = [];
    const directions = [];
    let clock = 0;
    let rises = 0;
    let falls = 0;
    let ticks = 0;
    function parity(value) {
        let result = 0;
        for (let bit = 0; bit < 32; bit++) result ^= (value >>> bit) & 1;
        return result;
    }
    const value = 0x6BA02477;
    const bits = [ack & 1, (ack >> 1) & 1, (ack >> 2) & 1,
        ...Array.from({ length: 32 }, (_, bit) => (value >>> bit) & 1), parity(value)];
    const sandbox = {
        __builtin_parity: parity,
        swd_set_clock: (_, level) => {
            if (clock !== level) {
                clock = level;
                if (level) rises++;
                else falls++;
                events.push(['clock', level, rises, falls, ticks]);
            }
        },
        swd_set_data: (_, level) => events.push(['data', !!level, clock, ticks]),
        swd_clock_delay: () => { ticks++; },
        swd_get_data: () => {
            samples.push({ rises, falls, ticks });
            return bits[rises - 9] ?? 1;
        },
        swd_configure_pins: (_, output) => directions.push({ output, clock, rises, falls, ticks })
    };
    for (const [name, parameters] of [
        ['swd_write_bit', 'ctx, level'], ['swd_read_bit', 'ctx'],
        ['swd_write_byte', 'ctx, data, bits'], ['swd_write', 'ctx, data, bits'],
        ...(text.includes('static void swd_clock_cycle(') ? [['swd_clock_cycle', 'ctx']] : []),
        ['swd_transfer', 'ctx, ap, write, a23, data']
    ]) vm.runInNewContext(`globalThis.${name} = (${parameters}) => ${body(name)};`, sandbox);
    const data = { value: write ? value : 0 };
    const result = sandbox.swd_transfer({ swd_idle_bits: 0, dp_regs: { select_ok: true } }, false, write, 3, data);
    return { result, data, rises, falls, clock, events, samples, directions };
}

test('Flipper comparison: request-to-ACK sample edges are identical; ESP adds settling delays, not an ACK clock', { skip: !flipperSource }, () => {
    const flipper = transferWaveform(flipperSource, false);
    const esp = transferWaveform(source, false);
    assert.equal(flipper.result, 1);
    assert.equal(esp.result, 1);
    assert.equal(flipper.data.value >>> 0, 0x6BA02477);
    assert.equal(esp.data.value >>> 0, 0x6BA02477);
    assert.deepEqual(esp.samples.map(({ rises, falls }) => [rises, falls]),
        flipper.samples.map(({ rises, falls }) => [rises, falls]));
    assert.deepEqual(esp.samples.slice(0, 3).map(sample => sample.rises), [9, 10, 11]);
    assert.equal(esp.samples[0].ticks - flipper.samples[0].ticks, 2);
    assert.equal(flipper.rises, 45);
    assert.equal(esp.rises, 46);
    assert.equal(flipper.directions.at(-1).clock, 1);
    assert.equal(esp.directions.at(-1).clock, 1);
    assert.equal(flipper.events.at(-1)[1], false);
    assert.equal(esp.events.at(-1)[1], true);
});

test('Flipper comparison: write clock count matches but ESP retakes SWDIO while clock is high', { skip: !flipperSource }, () => {
    const flipper = transferWaveform(flipperSource, true);
    const esp = transferWaveform(source, true);
    assert.equal(flipper.result, 1);
    assert.equal(esp.result, 1);
    assert.equal(flipper.rises, 46);
    assert.equal(esp.rises, 46);
    assert.equal(flipper.directions[2].rises, 13);
    assert.equal(esp.directions[2].rises, 13);
    assert.equal(flipper.directions[2].clock, 0);
    assert.equal(esp.directions[2].clock, 1);
});

test('session setup selects GPIO function only for scan pins and leaves them floating', () => {
    const start = source.indexOf('    if (io_mask)', source.indexOf('void swd_init('));
    assert.notEqual(start, -1);
    const body = source.slice(start, source.indexOf('    ctx->loop_count =', start))
        .replace('const gpio_config_t pins =', 'const pins =')
        .replace(/\.(\w+) =/g, '$1:')
        .replace('gpio_config(&pins)', 'gpio_config(pins)');
    const calls = [];
    const sandbox = {
        GPIO_MODE_INPUT: 'input', GPIO_PULLUP_DISABLE: 0,
        GPIO_PULLDOWN_DISABLE: 0, GPIO_INTR_DISABLE: 0,
        gpio_config: pins => calls.push(JSON.parse(JSON.stringify(pins)))
    };
    vm.runInNewContext(`globalThis.initialize = io_mask => { ${body} };`, sandbox);
    sandbox.initialize(3);
    assert.deepEqual(calls, [{ pin_bit_mask: 3, mode: 'input', pull_up_en: 0,
        pull_down_en: 0, intr_type: 0 }]);
    sandbox.initialize(0);
    assert.equal(calls.length, 1);
});
test('RDBUFF WAIT retries completion without posting another AP read; timeout aborts the pending transfer', () => {
    const start = source.indexOf('static uint8_t swd_read_rdbuff(');
    const body = source.slice(source.indexOf('{', start), source.indexOf('\n}', start) + 2)
        .replaceAll('ctx->', 'ctx.')
        .replaceAll('uint8_t ', 'let ')
        .replaceAll('uint32_t ', 'let ')
        .replace('int attempt', 'let attempt')
        .replace('&abort', 'abort');
    function complete(responses, abortAck = 1) {
        const calls = [];
        const sandbox = {
            esp_rom_delay_us() {}, LOG() {},
            swd_transfer: (_, ap, write, address, data) => {
                calls.push([ap, write, address]);
                if (write) { assert.equal(data, 0x1F); return abortAck; }
                const ack = responses.length ? responses.shift() : 2;
                if (ack === 1) data.value = 0xDEADBEEF;
                return ack;
            }
        };
        vm.runInNewContext(`globalThis.complete = (ctx, data) => ${body};`, sandbox);
        const ctx = { dp_regs: { select_ok: true } };
        const data = { value: 0 };
        return { ack: sandbox.complete(ctx, data), calls, ctx, data };
    }
    const ready = complete([2, 2, 1]);
    assert.equal(ready.ack, 1);
    assert.equal(ready.data.value, 0xDEADBEEF);
    assert.deepEqual(ready.calls, Array.from({ length: 3 }, () => [false, false, 3]));
    assert.equal(ready.ctx.dp_regs.select_ok, true);
    const timeout = complete([]);
    assert.equal(timeout.ack, 2);
    assert.equal(timeout.ctx.dp_regs.select_ok, false);
    assert.deepEqual(timeout.calls, [...Array.from({ length: 32 }, () => [false, false, 3]), [false, true, 0]]);
    assert.equal(complete([], 4).ack, 4);
    const fault = complete([4]);
    assert.equal(fault.ack, 4);
    assert.equal(fault.calls.length, 1);
    assert.equal((source.match(/ret = swd_read_rdbuff\(ctx, &rdbuff\);/g) || []).length, 2);
});

test('zero clock delay still provides a settling interval; configured delays are preserved', () => {
    const start = source.indexOf('static void swd_clock_delay(');
    const body = source.slice(source.indexOf('{', start), source.indexOf('\n}', start) + 2)
        .replaceAll('ctx->', 'ctx.');
    const delays = [];
    const sandbox = { esp_rom_delay_us: delay => delays.push(delay) };
    vm.runInNewContext(`globalThis.clockDelay = ctx => ${body};`, sandbox);
    for (const delay of [0, 1, 10, 1000]) sandbox.clockDelay({ swd_clock_delay: delay });
    assert.deepEqual(delays, [1, 1, 10, 1000]);
});
test('write bits complete their falling edge before releasing SWDIO', () => {
    const start = source.indexOf('static void swd_write_bit(');
    const body = source.slice(source.indexOf('{', start), source.indexOf('\n}', start) + 2);
    const calls = [];
    const sandbox = {
        swd_set_clock: (_, level) => calls.push(['clock', level]),
        swd_set_data: (_, level) => calls.push(['data', level]),
        swd_clock_delay: () => calls.push(['delay'])
    };
    vm.runInNewContext(`globalThis.writeBit = (ctx, level) => ${body};`, sandbox);
    sandbox.writeBit({}, true);
    assert.deepEqual(calls, [['clock', 0], ['data', true], ['delay'], ['clock', 1], ['delay'], ['clock', 0]]);
});
test('read bits sample immediately after the falling edge, matching the Flipper probe', () => {
    const start = source.indexOf('static uint32_t swd_read_bit(');
    const body = source.slice(source.indexOf('{', start), source.indexOf('\n}', start) + 2)
        .replace('uint32_t bits', 'const bits');
    const calls = [];
    const sandbox = {
        swd_set_clock: (_, level) => calls.push(['clock', level]),
        swd_clock_delay: () => calls.push(['delay']),
        swd_get_data: () => { calls.push(['sample']); return 2; }
    };
    vm.runInNewContext(`globalThis.readBit = ctx => ${body};`, sandbox);
    assert.equal(sandbox.readBit({}), 2);
    assert.deepEqual(calls, [['clock', 1], ['delay'], ['clock', 0], ['sample'], ['delay'], ['clock', 1]]);
});
const configureSource = source.slice(source.indexOf('    case SWD_UART_OP_CONFIGURE_PINS:'), source.indexOf('    case SWD_UART_OP_DEINIT:'));
const configureBody = configureSource.slice(configureSource.indexOf('{'))
    .replaceAll('(void)', '')
    .replaceAll('uint32_t ', 'let ')
    .replaceAll('bool ', 'let ')
    .replaceAll('int io', 'let io')
    .replaceAll('1U', '1')
    .replaceAll('NULL', 'null')
    .replace('read_u32_le(args + 4)', 'read_u32_le(args.slice(4))')
    .replace('read_u32_le(args + 8)', 'read_u32_le(args.slice(8))')
    .replace('read_u32_le(args + 14)', 'read_u32_le(args.slice(14))');

function configurePins(scan, high, low, drive = 0, length = 13, readPull = 0, clockHz = 100000, clockPin = 255, dataPin = 255) {
    const args = Buffer.alloc(20);
    args.writeUInt32LE(scan, 0);
    args.writeUInt32LE(high, 4);
    args.writeUInt32LE(low, 8);
    args[12] = drive;
    args[13] = readPull;
    args.writeUInt32LE(clockHz, 14);
    args[18] = clockPin;
    args[19] = dataPin;
    const calls = [];
    const sandbox = { gpio_legal_mask: 0x3007FF, swd_open_drain: true, swd_pull_up: true,
        swd_read_pull: 0, swd_clock_hz: 0,
        swd_fixed_swc: 255, swd_fixed_swd: 255,
        GPIO_MODE_INPUT: 'input', GPIO_MODE_OUTPUT: 'output', GPIO_FLOATING: 'floating',
        SWD_UART_STATUS_BAD_LEN: 1, SWD_UART_STATUS_BAD_ARG: 5, SWD_UART_STATUS_OK: 0,
        ESP_ERR_INVALID_SIZE: -1, ESP_ERR_INVALID_ARG: -2, op: 4, seq: 0,
        read_u32_le: data => data.readUInt32LE(0),
        swd_stop_session: () => calls.push(['stop']),
        gpio_set_direction: (pin, mode) => calls.push(['direction', pin, mode]),
        gpio_set_pull_mode: (pin, mode) => calls.push(['pull', pin, mode]),
        gpio_set_level: (pin, level) => calls.push(['level', pin, level]),
        swd_queue_response: (_, __, status) => status };
    vm.runInNewContext(`globalThis.configure = (args, arg_len) => ${configureBody};`, sandbox);
    const status = sandbox.configure(args, length);
    return { calls, status, openDrain: sandbox.swd_open_drain, pullUp: sandbox.swd_pull_up,
        readPull: sandbox.swd_read_pull, clockHz: sandbox.swd_clock_hz,
        clockPin: sandbox.swd_fixed_swc, dataPin: sandbox.swd_fixed_swd };
}

test('GPIO configuration rejects overlapping, reserved and invalid drive masks before touching pins', () => {
    for (const args of [[3, 1, 0], [3, 4, 4], [3, 1 << 18, 0], [3, 0, 0, 3]]) {
        const { calls, status } = configurePins(...args);
        assert.equal(status, -2);
        assert.deepEqual(calls, []);
    }
    assert.equal(configurePins(3, 0, 0, 0, 11).status, -1);
});

test('GPIO configuration releases safe pins, sets idle levels and shares one drive mode', () => {
    for (const drive of [0, 1, 2]) {
        const { calls, status, openDrain, pullUp } = configurePins(3, 4, 8, drive);
        assert.equal(status, 0);
        assert.equal(openDrain, drive !== 1);
        assert.equal(pullUp, drive !== 2);
        assert.deepEqual(calls[0], ['stop']);
        assert.deepEqual(calls.filter(call => call[0] === 'level'), [['level', 2, 1], ['level', 3, 0]]);
        assert.equal(calls.filter(call => call[0] === 'direction' && call[2] === 'input').length, 13);
        assert.equal(calls.some(call => call[1] === 18 || call[1] === 19), false);
        assert.deepEqual(calls.slice(-4), [['level', 2, 1], ['direction', 2, 'output'], ['level', 3, 0], ['direction', 3, 'output']]);
    }
    assert.equal(configurePins(3, 0, 0, 1, 12).openDrain, true);
    assert.equal(configurePins(3, 0, 0, 2, 12).pullUp, true);
});

test('extended configuration validates independent read pull and clock rate before touching pins', () => {
    for (const drive of [0, 1, 2]) {
        for (const readPull of [0, 1]) {
            const result = configurePins(3, 0, 4, drive, 18, readPull, 25000);
            assert.equal(result.status, 0);
            assert.equal(result.openDrain, drive !== 1);
            assert.equal(result.pullUp, drive !== 2);
            assert.equal(result.readPull, readPull);
            assert.equal(result.clockHz, 25000);
        }
    }
    for (const args of [[3, 0, 4, 0, 18, 2, 25000], [3, 0, 4, 3, 18, 0, 25000],
        [3, 0, 4, 0, 18, 0, 499], [3, 0, 4, 0, 18, 0, 500001]]) {
        const result = configurePins(...args);
        assert.equal(result.status, -2);
        assert.deepEqual(result.calls, []);
    }
    assert.equal(configurePins(3, 0, 4, 0, 18, 0, 500).status, 0);
    assert.equal(configurePins(3, 0, 4, 0, 18, 0, 500000).status, 0);
    assert.equal(configurePins(3, 0, 4, 2, 13).readPull, 2);
    assert.equal(configurePins(3, 0, 4, 2, 13).clockHz, 0);
    const init = source.slice(source.indexOf('void swd_init('));
    assert.match(init, /ctx->swd_clock_delay = swd_clock_hz \? \(500000U \+ swd_clock_hz - 1\) \/ swd_clock_hz/);
});

test('fixed pins are validated before GPIO changes and restored on every scan attempt', () => {
    const fixed = configurePins(3, 0, 4, 1, 20, 0, 10000, 0, 1);
    assert.equal(fixed.status, 0);
    assert.equal(fixed.clockPin, 0);
    assert.equal(fixed.dataPin, 1);
    for (const [clock, data] of [[0, 0], [0, 2], [11, 1], [32, 1], [255, 1], [0, 255]]) {
        const result = configurePins(3, 0, 4, 1, 20, 0, 10000, clock, data);
        assert.equal(result.status, -2);
        assert.deepEqual(result.calls, []);
    }
    for (const length of [12, 13, 18, 20]) {
        const result = configurePins(3, 0, 4, 1, length);
        assert.equal(result.clockPin, 255);
        assert.equal(result.dataPin, 255);
    }
    const begin = source.indexOf('    if (ctx->swd_fixed_swc < 32', source.indexOf('void swd_do_scan('));
    const end = source.indexOf('\n    swd_scan(ctx);', begin);
    const block = source.slice(begin, end).replaceAll('ctx->', 'ctx.').replaceAll('1U', '1');
    const ctx = { swd_fixed_swc: 0, swd_fixed_swd: 1, io_num_swc: 255, io_num_swd: 255, detected_device: false };
    let resets = 0;
    const sandbox = { ctx, swd_enter_swd: state => { resets++; assert.equal(state.io_num_swc, 0); assert.equal(state.io_num_swd, 1); } };
    for (const detected of [false, true, false]) {
        ctx.detected_device = detected;
        ctx.io_num_swc = 255;
        ctx.io_num_swd = 255;
        ctx.current_mask = 2;
        vm.runInNewContext(block, sandbox);
        assert.equal(ctx.io_num_swc, 0);
        assert.equal(ctx.io_num_swd, 1);
        assert.equal(ctx.io_swc, 1);
        assert.equal(ctx.io_swd, 2);
        assert.equal(ctx.current_mask, 1);
    }
    assert.equal(resets, 2);
});

test('write mode and read pulls remain independent on candidate and detected pins, including turnaround', () => {
    const start = source.indexOf('static void swd_configure_pins(');
    const body = source.slice(source.indexOf('{', start), source.indexOf('static void swd_set_clock('))
        .replaceAll('ctx->', 'ctx.')
        .replaceAll('gpio_mode_t ', 'const ')
        .replaceAll('gpio_pull_mode_t ', 'const ')
        .replaceAll('uint32_t ', 'const ')
        .replaceAll('int io', 'let io')
        .replaceAll('1U', '1');
    for (const drive of [0, 1, 2]) {
        for (const readPull of [0, 1, 2]) {
        for (const known of [false, true]) {
            for (const output of [false, true]) {
                const calls = [];
                const sandbox = {
                    GPIO_MODE_INPUT: 'input', GPIO_MODE_OUTPUT: 'output', GPIO_MODE_OUTPUT_OD: 'open-drain',
                    GPIO_FLOATING: 'floating', GPIO_PULLUP_ONLY: 'pull-up', GPIO_PULLDOWN_ONLY: 'pull-down',
                    gpio_set_direction: (pin, mode) => calls.push(['direction', pin, mode]),
                    gpio_set_pull_mode: (pin, pull) => calls.push(['pull', pin, pull])
                };
                vm.runInNewContext(`globalThis.configure = (ctx, output) => ${body};`, sandbox);
                sandbox.configure({ swd_open_drain: drive !== 1, swd_pull_up: drive !== 2, swd_read_pull: readPull,
                    io_num_swc: known ? 0 : 255, io_num_swd: known ? 1 : 255,
                    io_selected: 3, io_swc: 3, io_swd: 3, current_mask: 1 }, output);
                const driven = drive === 1 ? 'output' : 'open-drain';
                const dataPull = output ? (drive === 2 ? 'floating' : 'pull-up') :
                    (readPull === 1 ? 'pull-down' : readPull === 0 ? 'pull-up' : 'floating');
                assert.deepEqual(calls, [
                    ['direction', 0, driven], ['pull', 0, drive === 0 ? 'pull-up' : 'floating'],
                    ['direction', 1, output ? driven : 'input'], ['pull', 1, dataPull]
                ]);
            }
        }
        }
    }
});

test('known and candidate driven pins share one output mode', () => {
    const pins = source.slice(source.indexOf('static void swd_configure_pins('), source.indexOf('static void swd_set_clock('));
    assert.match(pins, /ctx->swd_open_drain \? GPIO_MODE_OUTPUT_OD : GPIO_MODE_OUTPUT/);
    assert.match(pins, /gpio_set_direction\(ctx->io_num_swd, output_mode\)/);
    assert.match(pins, /gpio_set_direction\(io, output_mode\)/);
    assert.match(pins, /gpio_set_direction\(ctx->io_num_swc, output_mode\)/);
});
const start = source.indexOf('    if (ctx->io_num_swd < 32 && ctx->io_num_swc < 32)', source.indexOf('static void swd_scan('));
const end = source.indexOf('\n    /* To switch SWJ-DP', start);
const body = source.slice(start, end)
    .replaceAll('ctx->', 'ctx.')
    .replace('uint32_t dpidr = 0;', 'let dpidr = { value: 0 };')
    .replace('uint8_t ack =', 'let ack =')
    .replaceAll('&dpidr', 'dpidr')
    .replace('dpidr != 0 && dpidr != UINT32_MAX', 'dpidr.value != 0 && dpidr.value != UINT32_MAX')
    .replace('? dpidr : 0', '? dpidr.value : 0');

function probe(responses) {
    const calls = [];
    const ctx = { io_num_swd: 1, io_num_swc: 0, detected: true,
        dp_regs: { dpidr_ok: true, dpidr: 0x6BA02477 } };
    const sandbox = { REG_IDCODE: 0, UINT32_MAX: 0xFFFFFFFF,
        swd_transfer: (_, ap, write, addr, data) => {
            assert.equal(ap, false); assert.equal(write, false); assert.equal(addr, 0);
            calls.push('read');
            const [ack, value] = responses.shift(); data.value = value; return ack;
        },
        swd_configure_pins: () => calls.push('output'),
        swd_line_reset: () => calls.push('reset'),
        swd_enter_swd: () => calls.push('select-swd') };
    vm.runInNewContext(`globalThis.scan = ctx => { ${body} };`, sandbox);
    sandbox.scan(ctx);
    return { ctx, calls };
}

test('healthy known-pin rescan reads DPIDR without reset or SWJ selection', () => {
    const { ctx, calls } = probe([[1, 0x6BA02477]]);
    assert.deepEqual(calls, ['read']);
    assert.equal(ctx.detected, true);
    assert.equal(ctx.dp_regs.dpidr, 0x6BA02477);
});
test('failed rescan clears a previously valid cached DPIDR', () => {
    const { ctx, calls } = probe([[7, 0], [7, 0], [7, 0]]);
    assert.deepEqual(calls, ['read', 'output', 'reset', 'read', 'select-swd', 'read']);
    assert.equal(ctx.detected, false);
    assert.equal(ctx.dp_regs.dpidr_ok, false);
    assert.equal(ctx.dp_regs.dpidr, 0);
});
test('line-reset recovery must return a fresh valid DPIDR', () => {
    const { ctx } = probe([[7, 0], [1, 0x6BA02477]]);
    assert.equal(ctx.detected, true);
    assert.equal(ctx.dp_regs.dpidr_ok, true);
});
test('lost SWD mode is restored on the known pin pair', () => {
    const { ctx, calls } = probe([[7, 0], [7, 0], [1, 0x6BA02477]]);
    assert.deepEqual(calls, ['read', 'output', 'reset', 'read', 'select-swd', 'read']);
    assert.equal(ctx.detected, true);
    assert.equal(ctx.dp_regs.dpidr, 0x6BA02477);
});
test('WAIT, FAULT and parity errors do not replay SWJ selection', () => {
    for (const ack of [2, 4, 8]) {
        const { ctx, calls } = probe([[ack, 0], [ack, 0]]);
        assert.deepEqual(calls, ['read', 'output', 'reset', 'read']);
        assert.equal(ctx.detected, false);
    }
});
test('browser does not rescan solely for verbose logging', () => {
    for (const file of ['multiprotocol.html', 'multiprotocol.static.html']) {
        const html = fs.readFileSync(file, 'utf8');
        assert.equal(html.includes('det = await swd.detectPins(ioMask, verbose);'), false, file);
    }
});
