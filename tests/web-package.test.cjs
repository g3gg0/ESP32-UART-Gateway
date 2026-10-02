const fs = require('node:fs');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const { test } = require('node:test');

test('static page has no external scripts/styles/audio and all scripts compile', () => {
    const before = fs.readFileSync('multiprotocol.static.html');
    const page = before.toString();
    assert.equal(/<script\b[^>]*\bsrc=/.test(page), false);
    assert.equal(/<link\b[^>]*rel="stylesheet"/.test(page), false);
    assert.equal(page.includes("new Audio('tadaa.mp3')"), false);
    assert.equal(/\{\{(?:view:|styles|scripts)/.test(page), false);
    const ids = [...page.slice(0, page.indexOf('<script>')).matchAll(/\bid="([^"]+)"/g)].map(m => m[1]);
    assert.equal(new Set(ids).size, ids.length, 'duplicate DOM IDs');
    for (const id of ['deviceInteraction', 'canInteraction', 'uartInteraction', 'canonEfInteraction']) {
        assert.equal(ids.filter(v => v === id).length, 1);
    }
    for (const match of page.matchAll(/<script>([\s\S]*?)<\/script>/g)) new vm.Script(match[1]);
});

test('all development scripts resolve locally and compile in manifest order', () => {
    const config = JSON.parse(fs.readFileSync('web/multiprotocol/package.json'));
    const page = fs.readFileSync(config.output, 'utf8');
    const scripts = [...page.matchAll(/<script src="([^"]+)"><\/script>/g)].map(m => m[1]);
    assert.deepEqual(scripts, config.scripts);
    for (const file of scripts) new vm.Script(fs.readFileSync(file, 'utf8'), { filename: file });
    new vm.Script(scripts.map(file => fs.readFileSync(file, 'utf8')).join('\n'));
});

test('SWD owns its pin controls, host state and UI implementations', () => {
    assert.equal(fs.existsSync('SwdPins.js'), false);
    const app = fs.readFileSync('web/multiprotocol/js/app.js', 'utf8');
    const protocol = fs.readFileSync('web/multiprotocol/js/swd/protocol.js', 'utf8');
    const swd = fs.readFileSync('web/multiprotocol/js/swd/Swd.js', 'utf8');
    assert.match(swd, /class SwdPinControls/);
    assert.match(swd, /class Swd \{/);
    for (const name of ['runSWDTest', 'scanAps', 'renderApTabs', 'pollDpStatus', 'initializeGpioCheckboxes', 'connectSwdUi', 'resetSwdUi']) {
        const definition = new RegExp('function ' + name + '\\(');
        assert.doesNotMatch(app, definition);
        assert.match(protocol, definition);
    }
    assert.doesNotMatch(app, /const (SWD_|DP_|SCS_|MEMAP_)/);
});

test('CAN, UART and EF own their handlers and controls remain usable after splitting', () => {
    const app = fs.readFileSync('web/multiprotocol/js/app.js', 'utf8');
    const groups = { can: ['decodeCanFrame', 'startCan', 'updateCanUi'],
        uart: ['onUartDataPacket', 'startUart', 'updateUartUi'],
        ef: ['pollCanonEfTelemetry', 'startCanonEf', 'updateEfUi'] };
    for (const [name, functions] of Object.entries(groups)) {
        const source = fs.readFileSync(`web/multiprotocol/js/${name}/protocol.js`, 'utf8');
        for (const func of functions) {
            const pattern = new RegExp('function ' + func + '\\(');
            assert.match(source, pattern);
            assert.doesNotMatch(app, pattern);
        }
    }
    const elements = new Map();
    const context = vm.createContext({ window: {}, TextDecoder, TextEncoder, Uint8Array,
        setTimeout, clearTimeout, setInterval, clearInterval,
        document: { getElementById: id => {
            if (!elements.has(id)) elements.set(id, { value: '', classList: { toggle() {}, remove() {}, add() {} } });
            return elements.get(id);
        } } });
    for (const name of ['can', 'uart', 'ef']) {
        vm.runInContext(fs.readFileSync(`web/multiprotocol/js/${name}/protocol.js`, 'utf8'), context);
    }
    vm.runInContext(app, context);
    vm.runInContext('setCanUiState()', context);
    assert.equal(elements.get('startCanBtn').disabled, true);
    assert.equal(elements.get('uartSendBtn').disabled, true);
    assert.equal(elements.get('canonEfInitializeBtn').disabled, true);
    vm.runInContext('isDeviceConnected = true; setCanUiState()', context);
    assert.equal(elements.get('startCanBtn').disabled, false);
    assert.equal(elements.get('uartSendBtn').disabled, false);
    assert.equal(elements.get('canonEfInitializeBtn').disabled, true);
    vm.runInContext("canonEfRunning = true; canonEfInitialized = true; canonEfView = 'raw'; setCanUiState()", context);
    assert.equal(elements.get('canonEfInitializeBtn').disabled, false);
});
