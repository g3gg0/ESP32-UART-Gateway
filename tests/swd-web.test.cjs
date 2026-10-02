const fs = require('node:fs');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const { test } = require('node:test');
// Resolve development-page scripts as well as already-inlined static scripts.
function readPage(file) {
    return fs.readFileSync(file, 'utf8').replace(/<link rel="stylesheet" href="([^"]+)">/g,
        (_, src) => '<style>' + fs.readFileSync(src, 'utf8') + '</style>')
    .replace(/<script src="([^"]+)"><\/script>/g, (tag, src) => {
        if (/^https?:/.test(src)) return tag;
        return '<script>\n' + fs.readFileSync(src, 'utf8') + '\n</script>';
    });
}


const pinsContext = vm.createContext({ Uint8Array, DataView, logToConsole: () => {},
    document: { createElement: () => ({}) } });
vm.runInContext(fs.readFileSync('web/multiprotocol/js/swd/swd.js', 'utf8') + '\nglobalThis.PinControls = SwdPinControls;', pinsContext);

const serialContext = vm.createContext({ Uint8Array, setTimeout, clearTimeout, logToConsole: () => {}, window: {} });
vm.runInContext(fs.readFileSync('EspSerial.js', 'utf8') + '\nglobalThis.Serial = EspSerial;', serialContext);

test('background SWD requests require an idle link and known bank zero', async () => {
    const serial = new serialContext.Serial();
    serial.port = {};
    serial.sendPacket = async () => {};
    const complete = async promise => {
        const [sequence, pending] = serial._swdPending.entries().next().value;
        clearTimeout(pending.timer);
        serial._swdPending.delete(sequence);
        pending.resolve({ status: 0, ack: 1, data: new Uint8Array(4) });
        return await promise;
    };
    const read = new Uint8Array([0, 0, 1, 0, 0, 0, 0, 0]);
    await assert.rejects(serial.swdRequest(3, new Uint8Array([0, 1, 2, 0, 0, 0, 0, 0]), { background: true }), /only supports CTRL\/STAT reads/);
    assert.equal(await serial.swdRequest(3, read, { background: true }), null);
    serial._swdDpBank = 0;
    serial._swdLastActivity = Date.now();
    assert.equal(await serial.swdRequest(3, read, { background: true }), null);
    serial._swdLastActivity = 0;
    const polling = serial.swdRequest(3, read, { background: true });
    assert.equal(serial._swdActivityVersion, 0);
    assert.equal(await serial.swdRequest(3, read, { background: true }), null);
    await complete(polling);
    const bankTwo = serial.swdRequest(3, new Uint8Array([0, 1, 2, 0, 2, 0, 0, 0]));
    assert.equal(serial._swdDpBank, null);
    await complete(bankTwo);
    assert.equal(serial._swdDpBank, 2);
    serial._swdLastActivity = 0;
    assert.equal(await serial.swdRequest(3, read, { background: true }), null);
    await complete(serial.swdRequest(0x11, new Uint8Array(4)));
    assert.equal(serial._swdDpBank, 0);
    assert.equal(serial._swdActivityVersion, 2);
});

for (const file of ['multiprotocol.html', 'multiprotocol.static.html']) {
    const html = readPage(file);
    const scannerStart = html.indexOf('        class MemoryScanner {');
    const scannerEnd = html.indexOf('\n        }', scannerStart) + '\n        }'.length;
    const scannerContext = vm.createContext({ setTimeout });
    vm.runInContext(html.slice(scannerStart, scannerEnd) + '\nglobalThis.Scanner = MemoryScanner;', scannerContext);

    test(`${file}: layout is balanced, AP tabs include types, and disclosures reuse UART styling`, () => {
        const markup = html.slice(html.indexOf('<body>'), html.indexOf('<script>', html.indexOf('<body>'))).replace(/<!--[\s\S]*?-->/g, '');
        assert.equal((markup.match(/<div\b/g) || []).length, (markup.match(/<\/div>/g) || []).length);
        const start = html.indexOf('        function renderApTabs(');
        const end = html.indexOf('        function setActiveAp(', start);
        assert.match(html.slice(start, end), /btn\.textContent = `AP\$\{a\.ap\} \\u00b7 \$\{t\}`/);
        assert.match(html.slice(start, end), /btn\.title =/);
        assert.match(markup, /class="collapsible-group swd-details" id="dpDiagnostics"/);
        assert.match(markup, /class="collapsible-group swd-details" id="swdPinDetails"/);
        assert.match(html, /\.collapsible-group\[open\] > summary::before/);
        assert.doesNotMatch(markup, /btnDpDebug|Refresh now/);
    });

    test(`${file}: DP status does not clear errors, request power, or probe APs`, async () => {
        const start = html.indexOf('            async dpDebugOnce() {');
        const end = html.indexOf('\n            async detectPins', start);
        const calls = [];
        const context = vm.createContext({
            DP_A23_ABORT: 0, DP_A23_SELECT: 2, DP_A23_CTRLSTAT: 1,
            CDBGPWRUPREQ: 0x10000000, CDBGPWRUPACK: 0x20000000,
            CSYSPWRUPREQ: 0x40000000, CSYSPWRUPACK: 0x80000000,
            u32ToHex: value => '0x' + (value >>> 0).toString(16)
        });
        const probe = vm.runInContext(`({${html.slice(start, end)}})`, context);
        probe.ackName = ack => ack === 1 ? 'OK' : 'FAULT';
        probe.transferRaw = async (...args) => {
            calls.push(args);
            return { ack: 1, value: args[2] === 1 ? 0xf0000020 : 0x6ba02477 };
        };
        const lines = await probe.dpDebugOnce();
        assert.deepEqual(calls, [[false, false, 0, 0], [false, true, 2, 0], [false, false, 1, 0]]);
        assert.ok(lines.some(line => line.includes('error=true')));
        assert.ok(lines.some(line => line.includes('request=true acknowledged=true')));
    });

    test(`${file}: DP actions are separate, bank-aware, and restore controls`, async () => {
        const start = html.indexOf('        async function dpAction(action)');
        const end = html.indexOf('\n        async function scanAps()', start);
        const elements = {
            dpActionControls: { disabled: false },
            dpStatusOutput: {}, dpBankInput: { value: '2' }, dpRegisterInput: { value: '1' },
            dpValueInput: { value: '0x50000000' }
        };
        const controls = [{ disabled: false }, { disabled: true }];
        const events = [];
        const context = vm.createContext({
            document: { getElementById: id => elements[id], querySelectorAll: () => controls },
            espSerial: { port: {} }, scanInProgress: false, detectLoopActive: false,
            swd: { clearStickyErrors: async () => events.push('clear'), ensurePowerUp: async () => events.push('power'),
                dpWrite: async (...args) => events.push(['write', ...args]), dpRead: async register => { events.push(['read', register]); return 42; } },
            dpDebug: async () => { assert.equal(controls[0].disabled, true); events.push('status'); },
            DP_A23_CTRLSTAT: 1, DP_A23_SELECT: 2, parseHexOrDec: Number, u32ToHex: value => '0x' + value.toString(16),
            confirm: () => true, logToConsole() {}
        });
        const action = vm.runInContext(`(${html.slice(start, end)})`, context);
        await action('status');
        assert.deepEqual(events.splice(0), ['status']);
        await action('clear');
        assert.deepEqual(events.splice(0), ['clear', 'status']);
        await action('power');
        assert.deepEqual(events.splice(0), ['power', 'status']);
        await action('read');
        assert.deepEqual(events.splice(0), [['write', 2, 2], ['read', 1]]);
        await action('write');
        assert.deepEqual(events.splice(0), [['write', 2, 2], ['write', 1, 0x50000000]]);
        context.confirm = () => false;
        await action('write');
        assert.deepEqual(events, []);
        assert.equal(controls[0].disabled, false);
        assert.equal(controls[1].disabled, true);
        assert.equal(elements.dpActionControls.disabled, false);
        context.swd.dpRead = async () => { throw new Error('SWD FAULT'); };
        await action('read');
        assert.match(elements.dpStatusOutput.textContent, /SWD FAULT/);
        assert.equal(elements.dpActionControls.disabled, false);
    });

    test(`${file}: live DP polling uses 500 ms ticks, decodes lamps, and discards overlapping activity`, async () => {
        const start = html.indexOf('        let dpMonitorTimer = null;');
        const end = html.indexOf('        async function dpDebug()', start);
        const elements = Object.fromEntries(['dpCtrlstat', 'dpWarnings', 'dpDebugPower', 'dpSystemPower', 'dpFaults', 'dpUpdateState', 'dpActionControls', 'btnRunSWDTest', 'dpStatusOutput', 'dpDiagnostics'].map(id => [id, { dataset: {}, disabled: false }]));
        const bits = [0, 1, 4, 5, 6, 7, 26, 27, 28, 29, 30, 31].map(bit => ({ dataset: { dpBit: String(bit) }, disabled: true }));
        const intervals = [];
        const stopped = [];
        const calls = [];
        const data = new Uint8Array(4);
        new DataView(data.buffer).setUint32(0, 0xf0000040, true);
        const response = { ack: 1, status: 0, data };
        const serial = { port: {}, _swdActivityVersion: 0, _swdDpBank: 0,
            swdRequest: async (op, args, options) => { calls.push([op, Array.from(args), options]); return response; } };
        const context = vm.createContext({
            espSerial: serial, swd: { ackName: ack => ack === 4 ? 'FAULT' : 'WAIT' },
            activeProtocolTab: 'swd', scanInProgress: false, detectLoopActive: false,
            document: { getElementById: id => elements[id], querySelectorAll: () => bits },
            setInterval: (callback, delay) => { intervals.push({ callback, delay }); return 1; }, clearInterval: id => stopped.push(id),
            CDBGPWRUPREQ: 0x10000000, CDBGPWRUPACK: 0x20000000, CSYSPWRUPREQ: 0x40000000, CSYSPWRUPACK: 0x80000000,
            SWD_UART_OP_TRANSFER: 3, SWD_UART_STATUS_OK: 0, DP_A23_CTRLSTAT: 1, Uint8Array,
            readU32LE: bytes => new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength).getUint32(0, true),
            u32ToHex: value => '0x' + (value >>> 0).toString(16)
        });
        vm.runInContext(html.slice(start, end), context);
        context.setDpUiEnabled(true);
        context.setDpUiEnabled(true);
        assert.equal(intervals.length, 1);
        assert.equal(intervals[0].delay, 500);
        await intervals[0].callback();
        assert.equal(elements.dpCtrlstat.textContent, '0xf0000040');
        assert.equal(elements.dpDebugPower.dataset.state, 'on');
        assert.equal(elements.dpSystemPower.dataset.state, 'on');
        assert.equal(elements.dpWarnings.hidden, true);
        assert.equal(elements.dpFaults.textContent, 'No sticky flags');
        assert.equal(bits.find(bit => bit.dataset.dpBit === '31').checked, true);
        assert.deepEqual(calls[0].slice(0, 2), [3, [0, 0, 1, 0, 0, 0, 0, 0]]);
        assert.equal(calls[0][2].background, true);
        new DataView(data.buffer).setUint32(0, 0x500000b2, true);
        await context.pollDpStatus();
        assert.equal(elements.dpDebugPower.dataset.state, 'waiting');
        assert.equal(elements.dpFaults.dataset.state, 'error');
        assert.equal(elements.dpWarnings.hidden, false);
        assert.match(elements.dpWarnings.textContent, /Sticky error/);
        assert.match(elements.dpWarnings.textContent, /awaiting acknowledgement/);
        const completed = calls.length;
        context.scanInProgress = true;
        await context.pollDpStatus();
        context.scanInProgress = false;
        elements.btnRunSWDTest.disabled = true;
        await context.pollDpStatus();
        elements.btnRunSWDTest.disabled = false;
        elements.dpActionControls.disabled = true;
        await context.pollDpStatus();
        elements.dpActionControls.disabled = false;
        context.activeProtocolTab = 'uart';
        await context.pollDpStatus();
        context.activeProtocolTab = 'swd';
        assert.equal(calls.length, completed);
        let finish;
        serial.swdRequest = () => new Promise(resolve => { finish = resolve; });
        const pending = context.pollDpStatus();
        const resolver = finish;
        await context.pollDpStatus();
        assert.equal(finish, resolver);
        serial._swdActivityVersion++;
        finish(response);
        await pending;
        assert.equal(elements.dpCtrlstat.textContent, '0x500000b2');
        serial.swdRequest = async () => ({ ack: 4, status: 0 });
        await context.pollDpStatus();
        assert.equal(elements.dpDebugPower.dataset.state, 'unknown');
        assert.match(elements.dpWarnings.textContent, /SWD FAULT/);
        assert.ok(bits.every(bit => bit.indeterminate));
        serial._swdDpBank = 2;
        serial.swdRequest = async () => null;
        await context.pollDpStatus();
        assert.match(elements.dpUpdateState.textContent, /bank 0 not selected/);
        serial.swdRequest = () => new Promise(resolve => { finish = resolve; });
        const beforeDisconnect = context.pollDpStatus();
        context.setDpUiEnabled(false);
        finish(response);
        await beforeDisconnect;
        assert.equal(elements.dpCtrlstat.textContent, '-');
        assert.equal(elements.dpUpdateState.textContent, 'Not checked');
        assert.deepEqual(stopped, [1]);
    });

    test(`${file}: console filters and collapses repeats without losing error messages`, () => {
        const start = html.indexOf('        function renderConsole()');
        const end = html.indexOf('\n        function getTadaaAudio()', start);
        const elements = { consoleDisplay: {}, consoleCount: {}, consoleLevel: { value: 'all' }, consoleFilter: { value: '' } };
        const context = vm.createContext({ document: { getElementById: id => elements[id] }, consoleLineBuffer: [], MAX_CONSOLE_LINES: 3 });
        vm.runInContext(html.slice(start, end), context);
        context.logToConsole('SWD FAULT', 'error');
        context.logToConsole('SWD FAULT', 'error');
        context.logToConsole('Ready', 'info');
        assert.match(elements.consoleDisplay.textContent, /SWD FAULT \(\u00d72\)/);
        assert.equal(elements.consoleCount.textContent, '3 messages');
        elements.consoleLevel.value = 'error';
        context.renderConsole();
        assert.doesNotMatch(elements.consoleDisplay.textContent, /Ready/);
        elements.consoleFilter.value = 'not present';
        context.renderConsole();
        assert.equal(elements.consoleDisplay.textContent, '');
        elements.consoleFilter.value = 'fault';
        context.renderConsole();
        assert.match(elements.consoleDisplay.textContent, /SWD FAULT/);
        context.clearConsole();
        assert.equal(elements.consoleDisplay.textContent, '');
        assert.equal(elements.consoleCount.textContent, '0 messages');
    });

    test(`${file}: clicking any memory region opens hex and caps the read at 4 KiB`, async () => {
        const inputs = { memAddrInput: { value: '' }, memWordsInput: { value: '' } };
        const events = [];
        const callbackStart = html.indexOf('cell.onclick = async () => {', html.indexOf('function renderMemoryMap()')) + 'cell.onclick = '.length;
        const callbackEnd = html.indexOf('\n                        };', callbackStart) + '\n                        }'.length;
        const context = vm.createContext({
            address: 0, size: 0x10000000, openingMemoryRegion: false, scanTask: null,
            scanner: { busy: false, stop: () => events.push('stop') },
            document: { getElementById: id => inputs[id] },
            selectSubTab: async tab => events.push(tab),
            doReadBlock: async () => events.push([inputs.memAddrInput.value, inputs.memWordsInput.value]),
            u32ToHex: value => '0x' + value.toString(16),
            showToast: message => assert.fail(message)
        });
        const click = vm.runInContext(`(${html.slice(callbackStart, callbackEnd)})`, context);
        for (const [address, size, words] of [[0, 0x10000000, 1024], [0x10000, 0x20000, 1024], [0xFFFFF000, 4096, 1024], [0x20000000, 256, 64]]) {
            context.address = address;
            context.size = size;
            await click();
            assert.equal(events.at(-2), 'hex');
            assert.deepEqual(events.at(-1), ['0x' + address.toString(16), String(words)]);
        }
        let finishProbe;
        context.scanner.busy = true;
        context.scanTask = new Promise(resolve => { finishProbe = resolve; });
        const pendingClick = click();
        assert.equal(events.at(-1), 'stop');
        const eventCount = events.length;
        await click();
        assert.equal(events.length, eventCount);
        finishProbe();
        await pendingClick;
        assert.equal(events.at(-2), 'hex');
        assert.equal(context.openingMemoryRegion, false);
    });

    test(`${file}: hex block reads accept 4 KiB without exceeding the cap`, async () => {
        const inputs = { memAddrInput: { value: '0xfffff000' }, memWordsInput: { value: '1024' }, memUseBlockRead: { checked: true } };
        const reads = [];
        const editor = { setData: (bytes, address) => { assert.equal(bytes.length, 4096); assert.equal(address, 0xFFFFF000); } };
        const start = html.indexOf('        async function doReadBlock()');
        const end = html.indexOf('\n        function renderApTabs', start);
        const context = vm.createContext({
            swd: { memReadBlock32Via: async (ap, address, words) => { reads.push([ap, address, words]); return Array(words).fill(0); } },
            activeAp: 2, scannedAps: [{ ap: 2, info: { ap_class: 8 } }], apHexEditors: new Map([[2, editor]]),
            document: { getElementById: id => inputs[id] }, Uint8Array,
            parseHexOrDec: value => Number(value), u32ToHex: value => value.toString(16),
            logToConsole() {}, showToast: message => assert.fail(message)
        });
        const read = vm.runInContext(`(${html.slice(start, end)})`, context);
        await read();
        inputs.memWordsInput.value = '8192';
        await read();
        assert.deepEqual(reads, [[2, 0xFFFFF000, 1024], [2, 0xFFFFF000, 1024]]);
    });

    test(`${file}: expanded memory ranges retain address zero and the rest of the address space`, () => {
        const scanner = new scannerContext.Scanner({});
        const parts = Array.from({ length: 15 }, (_, index) => ({ base: (index + 1) * 65536, size: 65536, depth: 3 }));
        for (const part of parts) scanner.recordResult(part.base, part.size, { state: 'zero', sampledBytes: 64 });
        const view = scanner.getMapView(65536, 15 * 65536, 256, parts);
        assert.equal(view[0].base, 0);
        assert.equal(view[0].size, 65536);
        assert.equal(view[0].context, true);
        assert.equal(view[1].base, 65536);
        assert.equal(view[1].state, 'zero');
        assert.equal(view[1].context, undefined);
        assert.equal(view.at(-1).base + view.at(-1).size, 0x100000000);
        assert.equal(view.at(-1).context, true);
        for (let index = 1; index < view.length; index++) {
            assert.equal(view[index].base, view[index - 1].base + view[index - 1].size);
        }
        const fromZero = scanner.getMapView(0, 65536);
        assert.equal(fromZero[0].base, 0);
        assert.equal(fromZero.at(-1).base + fromZero.at(-1).size, 0x100000000);
        const high = scanner.getMapView(0xFFFF0000, 65536);
        assert.equal(high[0].base, 0);
        assert.equal(high.at(-1).base + high.at(-1).size, 0x100000000);
        assert.equal(high.at(-1).context, undefined);
    });

    test(`${file}: long memory scans preserve early samples after cache eviction`, async () => {
        const scanner = new scannerContext.Scanner({
            read: async (address, words) => Array(words).fill(address === 0 ? 0x12345678 : 0),
            recover: async () => {},
            onUpdate: scan => { if (scan.probes === 10000) scan.stop(); }
        });
        await scanner.scan(0, 0x10000000, 4096);
        assert.equal(scanner.probes, 10000);
        assert.equal(scanner.results.size, 8192);
        assert.equal(scanner.results.has('65536:65536'), false);
        assert.equal(scanner.getResult(65536, 65536).state, 'zero');
        assert.equal(scanner.getResult(0, 65536).state, 'data');
        assert.equal(scanner.getResult(0, 16777216).containsData, true);
        assert.equal(scanner.getResult(16777216, 16777216).containsData, undefined);
        assert.equal(scanner.getResult(0x10000000, 65536), undefined);
        assert.equal(scanner.getResult(65540, 65536), undefined);
        const opened = scanner.getMapSegments(65536, 2 * 65536, 256,
            [{ base: 65536, size: 65536, depth: 3 }, { base: 131072, size: 65536, depth: 3 }]);
        assert.equal(opened.length, 2);
        assert.equal(opened[0].state, 'zero');
        assert.equal(opened[1].state, 'zero');
        assert.ok([...scanner.resultRuns.values()].reduce((count, runs) => count + runs.length, 0) < 20);
        scanner.onUpdate = () => {};
        await scanner.scan(0x10000000, 4096, 4096);
        assert.equal(scanner.getResult(65536, 65536), undefined);
        assert.equal(scanner.dataRanges.size, 0);
    });

    test(`${file}: memory map merges adjacent matching states and preserves drill-down samples`, () => {
        const scanner = new scannerContext.Scanner({});
        scanner.blockSize = 256;
        const states = ['zero', 'zero', 'fault', 'fault', 'ff', 'mixed', 'mixed'];
        states.forEach((state, index) => scanner.results.set(`${index * 4096}:4096`, { state, sampledBytes: state === 'fault' ? 0 : 64 }));
        const segments = scanner.getMapSegments(0, 65536);
        assert.deepEqual(Array.from(segments, segment => segment.state), ['zero', 'fault', 'ff', 'mixed', 'unknown']);
        assert.equal(segments[0].base, 0);
        assert.equal(segments[0].size, 8192);
        assert.equal(segments[0].sampledBytes, 128);
        assert.equal(segments[0].parts.length, 2);
        assert.equal(segments[1].base, 8192);
        assert.equal(segments[1].size, 8192);
        assert.equal(segments[4].base, 7 * 4096);
        assert.equal(segments[4].size, 9 * 4096);
        const opened = scanner.getMapSegments(0, 8192, 256, segments[0].parts);
        assert.equal(opened.length, 2);
        assert.equal(opened[0].result.state, 'zero');
        assert.equal(opened[1].base, 4096);
        scanner.current = { base: 4096, size: 4096 };
        const reading = scanner.getMapSegments(0, 65536);
        assert.equal(reading[0].state, 'zero');
        assert.equal(reading[1].state, 'reading');
        assert.equal(reading[1].base, 4096);
        scanner.current = null;
        scanner.results.clear();
        scanner.results.set('0:268435456', { state: 'data', sampledBytes: 64 });
        const adaptive = scanner.getMapSegments(0, 0x100000000);
        assert.equal(adaptive.length, 2);
        assert.equal(adaptive[0].size, 256);
        assert.equal(adaptive[0].state, 'data');
        assert.equal(adaptive[0].sampledBytes, 64);
        assert.equal(adaptive[1].base, 256);
        assert.equal(adaptive[1].size, 0x100000000 - 256);
        assert.equal(adaptive[1].state, 'unknown');
    });

    test(`${file}: memory map automatically subdivides data ranges down to the configured block size`, () => {
        const scanner = new scannerContext.Scanner({});
        scanner.blockSize = 256;
        scanner.results.set('0:4096', { state: 'zero' });
        scanner.results.set('4096:4096', { state: 'ff' });
        scanner.results.set('8192:4096', { state: 'fault' });
        scanner.results.set('12288:4096', { state: 'data' });
        let ranges = scanner.getVisibleRanges(0, 0x10000);
        assert.equal(ranges.length, 31);
        assert.equal(ranges[0].size, 4096);
        assert.equal(ranges[1].size, 4096);
        assert.equal(ranges[2].size, 4096);
        assert.equal(ranges[3].base, 12288);
        assert.equal(ranges[3].size, 256);
        assert.equal(ranges[18].base, 0x3F00);
        assert.equal(ranges[19].base, 0x4000);
        scanner.results.set('0:4096', { state: 'zero', containsData: true });
        scanner.results.set('0:256', { state: 'data' });
        ranges = scanner.getVisibleRanges(0, 0x10000);
        assert.equal(ranges.length, 46);
        assert.equal(ranges[0].size, 256);
        assert.equal(ranges[0].depth, 1);
        assert.equal(scanner.getVisibleRanges(0, 0x10000, 16).length, 16);
        for (let index = 1; index < ranges.length; index++) {
            assert.equal(ranges[index].base, ranges[index - 1].base + ranges[index - 1].size);
        }
        assert.equal(ranges.at(-1).base + ranges.at(-1).size, 0x10000);
        scanner.results.clear();
        scanner.results.set('0:268435456', { state: 'data' });
        scanner.results.set('0:16777216', { state: 'data' });
        ranges = scanner.getVisibleRanges(0, 0x100000000);
        assert.equal(ranges.length, 46);
        assert.equal(ranges[0].size, 0x100000);
        assert.equal(ranges[0].depth, 2);
        assert.equal(ranges.at(-1).base + ranges.at(-1).size, 0x100000000);
    });

    test(`${file}: memory scan probes high nibbles first across all 4 GiB`, async () => {
        const reads = [];
        const scanner = new scannerContext.Scanner({
            read: async (address, words) => { reads.push([address, words]); return Array(words).fill(0); },
            recover: async () => {},
            onUpdate: scan => { if (scan.probes === 17) scan.stop(); }
        });
        await scanner.scan(0, 0x100000000, 4096);
        assert.deepEqual(reads.slice(0, 16).map(([address]) => address),
            Array.from({ length: 16 }, (_, index) => index * 0x10000000));
        assert.deepEqual(reads[16], [0, 16]);
        assert.equal(scanner.levelSize, 0x1000000);
        assert.equal(scanner.status, 'Stopped');
        assert.equal(scanner.busy, false);
    });

    test(`${file}: memory scan classifies samples and explores children of faults and zeros`, async () => {
        const reads = [];
        let recoveries = 0;
        const scanner = new scannerContext.Scanner({
            read: async (address, words) => {
                reads.push([address, words]);
                if (address === 0) throw new Error('SWD FAULT');
                if (address === 0x1000) return Array(words).fill(0xFFFFFFFF);
                if (address === 0x2000) return Array(words).fill(0xFF00FF00);
                if (address === 0x100 || address === 0x3100) return Array(words).fill(0x1234);
                return Array(words).fill(0);
            },
            recover: async () => { recoveries++; }
        });
        await scanner.scan(0, 0x10000, 256);
        assert.equal(reads.length, 16 + 256);
        assert.equal(recoveries, 2);
        assert.equal(scanner.results.get('0:4096').state, 'fault');
        assert.equal(scanner.results.get('0:4096').containsData, true);
        assert.equal(scanner.results.get('4096:4096').state, 'ff');
        assert.equal(scanner.results.get('8192:4096').state, 'mixed');
        assert.equal(scanner.results.get('12288:4096').state, 'zero');
        assert.equal(scanner.results.get('12288:4096').containsData, true);
        assert.equal(scanner.results.get('256:256').state, 'data');
        assert.equal(scanner.status, 'Complete');
        assert.deepEqual(reads.at(-1), [0xFF00, 64]);
    });

    test(`${file}: memory scan stop waits for the current probe and recovery failures abort`, async () => {
        let complete;
        let reads = 0;
        const scanner = new scannerContext.Scanner({
            read: async () => { reads++; return new Promise(resolve => { complete = resolve; }); },
            recover: async () => {}
        });
        const running = scanner.scan(0, 0x10000, 256);
        await assert.rejects(scanner.scan(), /already running/);
        scanner.stop();
        complete(Array(16).fill(0));
        await running;
        assert.equal(reads, 1);
        assert.equal(scanner.status, 'Stopped');
        scanner.read = async () => { throw new Error('Port disconnected'); };
        scanner.recover = async () => { assert.fail('Transport errors must not be recovered as memory faults'); };
        await assert.rejects(scanner.scan(0, 0x10000, 256), /Port disconnected/);
        assert.equal(scanner.status, 'Failed');
        scanner.read = async () => { throw new Error('SWD bad ACK=4'); };
        scanner.recover = async () => { throw new Error('ABORT failed'); };
        await assert.rejects(scanner.scan(0, 0x10000, 256), /ABORT failed/);
        assert.equal(scanner.status, 'Failed');
        assert.equal(scanner.busy, false);
        await assert.rejects(scanner.scan(0, 4096, 5), /block size/);
        await assert.rejects(scanner.scan(0xFFFFFFFC, 8), /range/);
    });

    const start = html.indexOf('        class HexEditor {');
    const end = html.indexOf('\n        }', start) + '\n        }'.length;
    const context = vm.createContext({ Uint8Array });
    vm.runInContext(html.slice(start, end) + '\nglobalThis.Editor = HexEditor;', context);

    test(`${file}: hex scroll loads before and after the original dump`, async () => {
        const reads = [];
        const editor = new context.Editor({ onReadRange: async (address, length) => {
            reads.push([address, length]);
            return new Uint8Array(length).fill(address === 0x1000 ? 1 : 2);
        } });
        editor.setData(new Uint8Array(1024).fill(3), 0x1400);
        await editor._loadMore(-1);
        assert.equal(editor.baseAddr, 0x1000);
        assert.equal(editor.data[0], 1);
        assert.equal(editor.data[1024], 3);
        await editor._loadMore(1);
        assert.deepEqual(reads, [[0x1000, 1024], [0x1800, 1024]]);
        assert.equal(editor.baseAddr, 0x1000);
        assert.equal(editor.data[2048], 2);
    });

    test(`${file}: hex scroll bounds the window and never wraps the address space`, async () => {
        const reads = [];
        const editor = new context.Editor({ onReadRange: async (address, length) => {
            reads.push([address, length]);
            return new Uint8Array(length);
        } });
        editor.setData(new Uint8Array(16384), 0);
        await editor._loadMore(-1);
        assert.equal(reads.length, 0);
        await editor._loadMore(1);
        assert.equal(editor.data.length, 16384);
        assert.equal(editor.baseAddr, 1024);
        await editor._loadMore(-1);
        assert.equal(editor.baseAddr, 0);
        assert.equal(editor.data.length, 16384);
        editor.setData(new Uint8Array(16), 0xFFFFFFE0);
        await editor._loadMore(1);
        assert.deepEqual(reads.at(-1), [0xFFFFFFF0, 16]);
        const count = reads.length;
        await editor._loadMore(1);
        assert.equal(reads.length, count);
        assert.equal(editor.baseAddr, 0xFFFFFFE0);
    });

    test(`${file}: hex scroll protects edits, coalesces loads and retains data on failure`, async () => {
        let reads = 0;
        let finishRead;
        const errors = [];
        const editor = new context.Editor({
            onReadRange: async () => {
                reads++;
                return new Promise(resolve => { finishRead = resolve; });
            },
            onReadError: error => errors.push(error.message)
        });
        editor.setData(new Uint8Array(16), 0x1000);
        editor.markDirty(0, 1);
        await editor._loadMore(1);
        assert.equal(reads, 0);
        assert.match(errors[0], /pending hex edits/);
        editor.clearDirty();
        const loading = editor._loadMore(1);
        await editor._loadMore(1);
        assert.equal(reads, 1);
        editor.setData(new Uint8Array([7, 8, 9, 10]), 0x2000);
        finishRead(new Uint8Array(1024));
        await loading;
        assert.equal(editor.baseAddr, 0x2000);
        assert.equal(editor.data.length, 4);
        editor.onReadRange = async () => { throw new Error('SWD FAULT'); };
        await editor._loadMore(1);
        assert.equal(editor.data[0], 7);
        assert.equal(editor._loadingRange, false);
        assert.equal(errors.at(-1), 'SWD FAULT');
    });
}

test('WebSerial packet writes are serialized across concurrent requests', async () => {
    const serial = new serialContext.Serial();
    let locked = false;
    let maxConcurrentWrites = 0;
    const packets = [];
    serial.port = { writable: { getWriter() {
        if (locked) throw new Error('writer already locked');
        locked = true;
        return {
            async write(packet) {
                maxConcurrentWrites++;
                await new Promise(resolve => setTimeout(resolve, 5));
                packets.push(Array.from(packet));
                maxConcurrentWrites--;
            },
            releaseLock() { locked = false; }
        };
    } } };

    await Promise.all([
        serial.sendPacket(new Uint8Array([1]), 'first'),
        serial.sendPacket(new Uint8Array([2]), 'second')
    ]);
    assert.deepEqual(packets, [[1], [2]]);
    assert.equal(maxConcurrentWrites, 0);
    assert.equal(locked, false);
});

test('idle pin colors follow GND and 3.3V but never apply to scan pins', () => {
    const controls = Object.create(pinsContext.PinControls.prototype);
    const checkbox = { value: 0, checked: false };
    const select = { dataset: { idle: 'low' }, replaceChildren() {}, appendChild() {}, setAttribute() {} };
    const classes = new Map();
    controls.container = { querySelectorAll: () => [{
        querySelector: selector => selector === 'input' ? checkbox : selector === 'select' ? select : null,
        classList: { toggle: (name, enabled) => classes.set(name, enabled) }
    }] };
    controls.driveMode = 'open-drain';
    controls.refresh();
    assert.equal(classes.get('idle-low'), true);
    assert.equal(classes.get('idle-high'), false);
    select.dataset.idle = 'high';
    controls.refresh();
    assert.equal(classes.get('idle-low'), false);
    assert.equal(classes.get('idle-high'), true);
    checkbox.checked = true;
    controls.refresh();
    assert.equal(classes.get('scanning'), true);
    assert.equal(classes.get('idle-low'), false);
    assert.equal(classes.get('idle-high'), false);
    checkbox.checked = false;
    select.dataset.idle = 'open';
    controls.refresh();
    assert.equal(classes.get('idle-low'), false);
    assert.equal(classes.get('idle-high'), false);
});

test('scan pins never contribute to high or low idle masks', () => {
    const controls = Object.create(pinsContext.PinControls.prototype);
    const items = [[0, true, 'high'], [1, true, 'low'], [2, false, 'high'], [3, false, 'low'], [4, false, 'open']]
        .map(([pin, checked, value]) => ({ querySelector: selector => selector === 'input' ? { value: pin, checked } : { value } }));
    controls.container = { querySelectorAll: () => items };
    assert.deepEqual({ ...controls.masks() }, { scan: 3, high: 4, low: 8 });
});

test('read/write settings are shared above the pin state dropdowns', () => {
    const controls = Object.create(pinsContext.PinControls.prototype);
    const selects = [0, 1].map(() => ({ dataset: { idle: 'open' }, options: [],
        replaceChildren() { this.options = []; }, appendChild(option) { this.options.push(option); },
        setAttribute() {} }));
    controls.readSelect = {};
    controls.writeSelect = {};
    controls.container = { querySelectorAll: () => selects.map((select, pin) => ({
        querySelector: selector => selector === 'input' ? { checked: true, value: pin } :
            selector === 'select' ? select : null,
        classList: { toggle() {} }
    })) };
    for (const mode of ['open-drain', 'open-drain-no-pull', 'push-pull']) {
        controls.driveMode = mode;
        controls.refresh();
        for (const select of selects) {
            assert.equal(select.value, 'scan');
            assert.deepEqual(select.options.map(option => [option.value, option.textContent]), [
                ['scan', 'Scan'], ['open', 'Open'], ['high', '3.3V'], ['low', 'GND'], ['swc', 'SWC'], ['swd', 'SWD']
            ]);
        }
        assert.equal(controls.writeSelect.value, mode);
    }
    for (const readMode of ['pull-up', 'pull-down']) {
        controls.readMode = readMode;
        controls.refresh();
        assert.equal(controls.readSelect.value, readMode);
        assert.equal(controls.writeSelect.value, 'push-pull');
    }
    controls.setBusy(true);
    assert.equal(controls.readSelect.disabled, true);
    assert.equal(controls.writeSelect.disabled, true);
    controls.setBusy(false);
    assert.equal(controls.readSelect.disabled, false);
    assert.equal(controls.writeSelect.disabled, false);
});

test('fixed pin states exclude automatic candidates, pause incomplete pairs and capture queued roles', async () => {
    const controls = Object.create(pinsContext.PinControls.prototype);
    const items = [[0, false, 'swc'], [1, false, 'swd'], [2, false, 'low'], [3, true, 'open']].map(([pin, checked, idle]) => {
        const input = { value: pin, checked };
        const select = { dataset: { idle }, value: idle, replaceChildren() {}, appendChild() {}, setAttribute() {} };
        return { input, select, querySelector: selector => selector === 'input' ? input : select, classList: { toggle() {} } };
    });
    controls.container = { querySelectorAll: () => items };
    controls.refresh();
    assert.deepEqual({ ...controls.masks() }, { scan: 3, high: 0, low: 4 });
    assert.deepEqual({ ...controls.fixedRoles() }, { swc: 0, swd: 1 });
    const packets = [];
    controls.invalidate = async () => {};
    controls.getSerial = () => ({ port: {}, setGatewayMode: async () => {}, swdRequest: async (_, args) => {
        packets.push(Array.from(args)); return { status: 0 };
    } });
    const fixed = controls.apply();
    items[1].select.dataset.idle = 'open';
    controls.refresh();
    assert.deepEqual({ ...controls.masks() }, { scan: 1, high: 0, low: 4 });
    const partial = controls.apply();
    await Promise.all([fixed, partial]);
    assert.equal(packets[0].length, 20);
    assert.deepEqual(packets[0].slice(18), [0, 1]);
    assert.equal(packets[0][8], 4);
    assert.equal(packets[1].length, 18);
    assert.equal(packets[1][0], 1);
});

test('pin configuration sends one global drive byte and little-endian masks', async () => {
    for (const [driveMode, byte] of [['open-drain', 0], ['push-pull', 1], ['open-drain-no-pull', 2]]) {
        const controls = Object.create(pinsContext.PinControls.prototype);
        controls.driveMode = driveMode;
        controls.masks = () => ({ scan: 3, high: 4, low: 8 });
        controls.invalidate = async () => {};
        controls.setBusy = busy => { controls.busy = busy; };
        controls.getSerial = () => ({ port: {}, setGatewayMode: async () => {},
            swdRequest: async (op, args) => {
                assert.equal(op, 4);
                assert.deepEqual(Array.from(args), [3, 0, 0, 0, 4, 0, 0, 0, 8, 0, 0, 0, byte, 0, 0xA0, 0x86, 1, 0]);
                return { status: 0 };
            } });
        await controls.apply();
        assert.equal(controls.busy, false);
        assert.equal(controls.applied, true);
    }
});

test('unsupported GPIO firmware reports a failure and unlocks controls', async () => {
    const controls = Object.create(pinsContext.PinControls.prototype);
    controls.masks = () => ({ scan: 3, high: 0, low: 0 });
    controls.invalidate = async () => {};
    controls.setBusy = busy => { controls.busy = busy; };
    controls.getSerial = () => ({ port: {}, setGatewayMode: async () => {}, swdRequest: async () => ({ status: 2 }) });
    await assert.rejects(controls.apply(), /updated gateway firmware/);
    assert.equal(controls.busy, false);
});

test('rapid pin edits are sent in order with captured drive modes', async () => {
    const controls = Object.create(pinsContext.PinControls.prototype);
    const packets = [];
    let high = 4;
    controls.driveMode = 'open-drain';
    controls.readMode = 'pull-up';
    controls.clockHz = 100000;
    controls.masks = () => ({ scan: 3, high, low: 0 });
    controls.invalidate = async () => {};
    controls.setBusy = busy => { controls.busy = busy; };
    controls.getSerial = () => ({ port: {}, setGatewayMode: async () => {}, swdRequest: async (_, args) => {
        const view = new DataView(args.buffer);
        packets.push([args[4], args[12], args[13], view.getUint32(14, true)]);
        return { status: 0 };
    } });
    const first = controls.apply();
    high = 8;
    controls.driveMode = 'push-pull';
    controls.readMode = 'pull-down';
    controls.clockHz = 25000;
    const second = controls.apply();
    await Promise.all([first, second]);
    assert.deepEqual(packets, [[4, 0, 0, 100000], [8, 1, 1, 25000]]);
});

test('invalid rate does not invalidate or touch the gateway', async () => {
    const controls = Object.create(pinsContext.PinControls.prototype);
    controls.getSerial = () => ({ port: {} });
    controls.masks = () => ({ scan: 3, high: 0, low: 0 });
    controls.invalidate = async () => { throw new Error('unexpected hardware access'); };
    for (const rate of [0, 499, 500001, NaN, 1000.5]) {
        controls.clockHz = rate;
        await assert.rejects(controls.apply(), /SWCLK rate/);
    }
});

test('offline pin edits do not attempt a hardware command', async () => {
    const controls = Object.create(pinsContext.PinControls.prototype);
    controls.getSerial = () => null;
    controls.invalidate = async () => { throw new Error('unexpected hardware access'); };
    assert.equal(await controls.apply(), false);
});

function scanHarness(html) {
    const controls = Object.create(pinsContext.PinControls.prototype);
    let scanMask = 3;
    controls.revision = 0;
    controls.pending = Promise.resolve();
    controls.masks = () => ({ scan: scanMask, high: 0, low: 0 });
    controls.setBusy = busy => { controls.busy = busy; };
    const probes = [];
    const waiters = [];
    const events = [];
    const elements = new Map();
    const sandbox = vm.createContext({ performance, setTimeout, setInterval: () => 1, clearInterval: () => {},
        document: { querySelectorAll: () => [], getElementById: id => {
            if (!elements.has(id)) elements.set(id, { style: {}, dataset: {}, checked: false });
            return elements.get(id);
        } },
        espSerial: { port: {}, setGatewayMode: async () => {}, swdRequest: async () => ({ status: 0 }) },
        detectLoopActive: false, detectLoopToken: 0, scanInProgress: false, swd: null,
        scannedAps: [], activeAp: null, apHexEditors: new Map(), apSubTabState: new Map(), coresightTreeCache: new Map(),
        primeTadaaAudio: () => {}, playTadaa: () => {}, u32ToHex: value => String(value),
        logToConsole: message => events.push(message),
        SwdPinControls: function (_, getSerial, invalidate) {
            controls.getSerial = getSerial;
            controls.invalidate = invalidate;
            return controls;
        },
        Swd: class {
            detectPins(mask) {
                return new Promise(resolve => {
                    const probe = { mask, resolve };
                    probes.push(probe);
                    if (waiters.length) waiters.shift()(probe);
                });
            }
            async postDetectInit() { events.push('initialize'); return { ok: true, ctrlstat: 0xF0000000 }; }
        }
    });
    sandbox.stopSwdOperations = async () => { sandbox.detectLoopActive = false; sandbox.detectLoopToken++; };
    const initializeStart = html.indexOf('        let swdPins;');
    const initializeEnd = html.indexOf('\n        }', initializeStart) + '\n        }'.length;
    vm.runInContext(html.slice(initializeStart, initializeEnd), sandbox);
    sandbox.initializeGpioCheckboxes();
    const runStart = html.indexOf('        async function runSWDTest()');
    vm.runInContext(html.slice(runStart, html.indexOf('        async function dpDebug()', runStart)), sandbox);
    sandbox.CDBGPWRUPREQ = 0x10000000;
    sandbox.CDBGPWRUPACK = 0x20000000;
    sandbox.CSYSPWRUPREQ = 0x40000000;
    sandbox.CSYSPWRUPACK = 0x80000000;
    let consumed = 0;
    return { controls, sandbox, events, probes,
        setMask: mask => { scanMask = mask; },
        nextProbe: () => consumed < probes.length ? Promise.resolve(probes[consumed++])
            : new Promise(resolve => waiters.push(probe => { consumed++; resolve(probe); })) };
}

const detectedTarget = { detected_device: true, dpidr_ok: true, dpidr: 0x6BA02477, swdio_gpio: 1, swclk_gpio: 2 };

for (const file of ['multiprotocol.html', 'multiprotocol.static.html']) {
    const html = readPage(file);
    test(`${file}: editing pins during detection applies and continues with the new mask`, async () => {
        const harness = scanHarness(html);
        const running = harness.sandbox.runSWDTest();
        const oldProbe = await harness.nextProbe();
        assert.equal(oldProbe.mask, 3);
        const token = harness.sandbox.detectLoopToken;
        harness.setMask(6);
        await harness.controls.apply();
        assert.equal(harness.sandbox.detectLoopActive, true);
        assert.equal(harness.sandbox.detectLoopToken, token);
        oldProbe.resolve(detectedTarget);
        const newProbe = await harness.nextProbe();
        assert.equal(newProbe.mask, 6);
        assert.equal(harness.events.includes('initialize'), false);
        newProbe.resolve(detectedTarget);
        await running;
        assert.equal(harness.events.filter(event => event === 'initialize').length, 1);
    });
    test(`${file}: Stop after a pin edit prevents detection from resuming`, async () => {
        const harness = scanHarness(html);
        const running = harness.sandbox.runSWDTest();
        const probe = await harness.nextProbe();
        harness.setMask(6);
        await harness.controls.apply();
        await harness.sandbox.runSWDTest();
        probe.resolve(detectedTarget);
        await running;
        assert.equal(harness.sandbox.detectLoopActive, false);
        assert.equal(harness.probes.length, 1);
        assert.equal(harness.events.includes('initialize'), false);
    });
}

for (const file of ['multiprotocol.html', 'multiprotocol.static.html']) {
    const html = readPage(file);
    const start = html.indexOf('        class Swd {');
    const end = html.indexOf('\n        }', start) + '\n        }'.length;
    const context = vm.createContext({ performance, setTimeout,
        MEMAP_CSW: 0, MEMAP_TAR: 4, MEMAP_DRW: 12,
        DP_A23_SELECT: 2, DP_A23_CTRLSTAT: 1,
        CSYSPWRUPREQ: 0x40000000, CDBGPWRUPREQ: 0x10000000,
        CSYSPWRUPACK: 0x80000000, CDBGPWRUPACK: 0x20000000,
        DP_A23_ABORT: 0, DP_A23_RDBUFF: 3,
        u32ToHex: n => (n >>> 0).toString(16) });
    vm.runInContext(html.slice(start, end) + '\nglobalThis.Swd = Swd;', context);
    // Compile every inline classic script, including bundled static dependencies.
    for (const match of html.matchAll(/<script\b([^>]*)>([\s\S]*?)<\/script>/gi)) {
        if (!/type\s*=\s*["']module["']/i.test(match[1])) new vm.Script(match[2], { filename: file });
    }

    test(`${file}: completed AP result is read once`, async () => {
        const swd = new context.Swd({});
        let reads = 0;
        swd.apWriteStrict = async () => {};
        swd.apReadStrict = async () => { reads++; return 0xDEADBEEF; };
        assert.equal(await swd.memRead32Via(0, 0x1000), 0xDEADBEEF);
        assert.equal(reads, 1);
    });

    if (file.startsWith('multiprotocol')) {
        test(`${file}: subword writes use address-selected DRW lanes and check completion`, async () => {
            for (const [width, start, values, expected] of [
                [8, 0x1000, [0x12, 0x34, 0x56, 0x78], [0x12, 0x3400, 0x560000, 0x78000000]],
                [8, 0x1003, [0xFE, 0xAB], [0xFE000000, 0xAB]],
                [16, 0x1000, [0x1234, 0xABCD], [0x1234, 0xABCD0000]],
                [16, 0x1002, [0xFFFF, 0x5678], [0xFFFF0000, 0x5678]]
            ]) {
                const swd = new context.Swd({});
                const events = [];
                swd.apWriteStrict = async (_, reg, value) => { if (reg === 12) events.push(value); };
                swd.dpRead = async reg => { assert.equal(reg, 3); events.push('completed'); return 0; };
                await swd[`memWriteBlock${width}Via`](2, start, values);
                assert.deepEqual(events, expected.flatMap(value => [value, 'completed']));
            }
        });

        test(`${file}: Write32 reports late bus faults without replaying the write`, async () => {
            const swd = new context.Swd({});
            const writes = [];
            const aborts = [];
            swd.apWriteStrict = async (_, reg, value) => writes.push([reg, value]);
            swd.dpRead = async reg => { assert.equal(reg, 3); throw new Error('SWD FAULT'); };
            swd.clearStickyErrors = async pending => aborts.push(pending);
            swd.recoverLink = async () => { throw new Error('unexpected re-detection'); };
            await assert.rejects(swd.memWrite32Via(2, 0x20000000, 0x12345678), /SWD FAULT/);
            assert.deepEqual(writes, [[0, 0x23000002], [4, 0x20000000], [12, 0x12345678]]);
            assert.deepEqual(aborts, [true]);
            swd.dpRead = async () => 0;
            assert.equal(await swd.memWrite32Via(2, 0x20000000, 0x12345678), true);
            await assert.rejects(swd.memWrite32Via(2, 1, 0), /unaligned/);
        });

        test(`${file}: write-back keeps dirty bytes when completion fails`, async () => {
            let dirty = true;
            let fail = true;
            const logs = [];
            const editor = {
                baseAddr: 0x20000000, isDirty: () => dirty,
                getData: () => new Uint8Array([0x12]), getEditSizeBytes: () => 1,
                setDirtyRegionSize() {}, getDirtyRanges: () => [{ start: 0, length: 1 }],
                clearDirty() { dirty = false; }
            };
            const callbackStart = html.indexOf('wbBtn.onclick = async () => {') + 'wbBtn.onclick = '.length;
            const callbackEnd = html.indexOf('\n                };', callbackStart) + '\n                }'.length;
            const callbackContext = vm.createContext({
                a: { ap: 2 }, apHexEditors: new Map([[2, editor]]),
                swd: { memWriteBlock8Via: async () => { if (fail) throw new Error('SWD FAULT'); } },
                logToConsole: message => logs.push(message), updateDirtyUi() {},
                u32ToHex: value => value.toString(16)
            });
            const writeBack = vm.runInContext(`(${html.slice(callbackStart, callbackEnd)})`, callbackContext);
            await writeBack();
            assert.equal(dirty, true);
            assert.match(logs.at(-1), /write-back error: SWD FAULT/);
            fail = false;
            await writeBack();
            assert.equal(dirty, false);
        });

        test(`${file}: MEM-AP FAULT recovers DP without re-detecting pins`, async () => {
            const swd = new context.Swd({});
            const events = [];
            swd.apWriteStrict = async () => {};
            swd.apReadStrict = async () => {
                if (!events.includes('recovered')) throw new Error('SWD bad ACK=4');
                return 0x12345678;
            };
            swd.recoverDp = async () => { events.push('recovered'); };
            swd.recoverLink = async () => { throw new Error('unexpected pin re-detection'); };
            swd.flushRdbuff = async () => {};
            assert.equal(await swd.memRead32Via(0, 0x08000000), 0x12345678);
            assert.deepEqual(events, ['recovered']);
        });

        test(`${file}: manual DP recovery aborts and powers up without detection`, async () => {
            const swd = new context.Swd({});
            const events = [];
            swd.clearStickyErrors = async abortPending => events.push(['abort', abortPending]);
            swd.ensurePowerUp = async timeout => events.push(['power', timeout]);
            swd.detectPins = async () => { throw new Error('unexpected pin re-detection'); };
            assert.equal(await swd.recoverDp('test'), true);
            assert.deepEqual(events, [['abort', true], ['power', 1200]]);
        });
    }

    test(`${file}: block keeps equal words and repairs TAR wrap`, async () => {
        const swd = new context.Swd({});
        let tar = 0, reads = 0;
        const writes = [];
        swd.apWriteStrict = async (_, reg, val) => { if (reg === 4) { tar = val; writes.push(val); } };
        swd.apReadStrict = async () => {
            reads++;
            const value = tar < 0x400 ? 7 : tar;
            tar = (tar & ~0x3FF) | ((tar + 4) & 0x3FF);
            return value;
        };
        assert.deepEqual(Array.from(await swd.memReadBlock32Via(0, 0x3F8, 4)), [7, 7, 0x400, 0x404]);
        assert.deepEqual(writes, [0x3F8, 0x400]);
        assert.equal(reads, 4);
        writes.length = 0;
        assert.equal((await swd.memReadBlock32Via(0, 0, 0)).length, 0);
        assert.equal(writes.length, 0);
    });

    test(`${file}: failed SELECT stops initialization and AP scan`, async () => {
        const swd = new context.Swd({});
        swd.dpWrite = async () => { throw new Error('SELECT failed'); };
        swd.dpRead = async () => { throw new Error('unexpected read'); };
        await assert.rejects(swd.ensurePowerUp(), /SELECT failed/);
        swd.clearStickyErrors = async () => {};
        swd.apReadStrict = async () => { throw new Error('unexpected AP access'); };
        await assert.rejects(swd.scanAps(), /SELECT failed/);
    });

    test(`${file}: failed ABORT is not hidden`, async () => {
        const swd = new context.Swd({});
        swd.dpWrite = async () => { throw new Error('ABORT failed'); };
        await assert.rejects(swd.clearStickyErrors(), /ABORT failed/);
        swd.ensurePowerUp = async () => { throw new Error('power-up attempted before ABORT'); };
        await assert.rejects(swd.postDetectInit(), /ABORT failed/);
    });

    test(`${file}: initialization aborts pending AP transfer before SELECT can WAIT`, async () => {
        const swd = new context.Swd({});
        const writes = [];
        let pending = true;
        swd.dpWrite = async (reg, value) => {
            writes.push([reg, value]);
            if (reg === 0 && value === 0x1F) pending = false;
            if (pending) throw new Error('dpWrite: too many WAIT retries');
        };
        swd.dpRead = async () => 0xF0000000;
        assert.equal((await swd.postDetectInit()).ok, true);
        assert.deepEqual(writes, [[0, 0x1F], [2, 0]]);
        writes.length = 0;
        await swd.clearStickyErrors();
        assert.deepEqual(writes, [[0, 0x1E]]);
    });

    test(`${file}: power request does not echo sticky status bits`, async () => {
        const swd = new context.Swd({});
        const writes = [];
        swd.dpWrite = async (reg, val) => writes.push([reg, val]);
        let reads = 0;
        swd.dpRead = async () => ++reads === 1 ? 0xA2 : 0xF0000000;
        assert.equal((await swd.ensurePowerUp()).ok, true);
        assert.deepEqual(writes, [[2, 0], [1, 0x50000000]]);
        swd.dpRead = async () => 0;
        await assert.rejects(swd.ensurePowerUp(1), /power-up timeout/);
    });

    test(`${file}: writes reset TAR at boundary for each width`, async () => {
        for (const [width, start] of [[32, 0x3FC], [16, 0x3FE], [8, 0x3FF]]) {
            const swd = new context.Swd({});
            const tar = [];
            swd.apWrite = async (_, reg, val) => { if (reg === 4) tar.push(val); };
            swd.apWriteStrict = swd.apWrite;
            swd.dpRead = async () => 0;
            await swd[`memWriteBlock${width}Via`](0, start, [1, 2, 3]);
            assert.deepEqual(tar, [start, 0x400]);
        }
    });
}
