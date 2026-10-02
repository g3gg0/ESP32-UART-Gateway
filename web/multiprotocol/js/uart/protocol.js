/* UART state, transport handlers and controls. */

let uartRunning = false;

const MAX_UART_RX_LINES = 800;
const uartRxLineBuffer = [];
const uartTextDecoder = new TextDecoder();
let uartLastConfig = null;


function appendUartRxLine(text) {
    const line = String(text || '');
    uartRxLineBuffer.push(line);
    if (uartRxLineBuffer.length > MAX_UART_RX_LINES) {
        uartRxLineBuffer.splice(0, uartRxLineBuffer.length - MAX_UART_RX_LINES);
    }
    const el = document.getElementById('uartRxDisplay');
    if (!el) return;
    el.textContent = uartRxLineBuffer.join('\n');
    el.scrollTop = el.scrollHeight;
}

function clearUartRx() {
    uartRxLineBuffer.length = 0;
    const el = document.getElementById('uartRxDisplay');
    if (el) el.textContent = '';
}


function setSelectValueIfPossible(selectEl, value) {
    if (!selectEl) return;
    const want = String(value);
    const has = Array.from(selectEl.options || []).some(o => o.value === want);
    if (has) selectEl.value = want;
}

function buildUartGpioSelects() {
    const gpios = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 19, 20, 21];
    const targets = [
        { id: 'canRx', includeUnused: false, defaultValue: 21 },
        { id: 'canTx', includeUnused: false, defaultValue: 20 },
        { id: 'canonEfDcl', includeUnused: false, defaultValue: 4 },
        { id: 'canonEfDlc', includeUnused: false, defaultValue: 1 },
        { id: 'canonEfLclkOut', includeUnused: false, defaultValue: 3 },
        { id: 'canonEfLclkIn', includeUnused: false, defaultValue: 2 },
        { id: 'uartCfgTx', includeUnused: false, defaultValue: 20 },
        { id: 'uartCfgRx', includeUnused: false, defaultValue: 21 },
        { id: 'uartCfgReset', includeUnused: true, defaultValue: 255 },
        { id: 'uartCfgControl', includeUnused: true, defaultValue: 255 },
        { id: 'uartCfgLed', includeUnused: true, defaultValue: 255 },
    ];

    for (const t of targets) {
        const el = document.getElementById(t.id);
        if (!el) continue;
        if (el.options && el.options.length > 0) continue;

        if (t.includeUnused) {
            const opt = document.createElement('option');
            opt.value = '255';
            opt.textContent = 'Unused';
            el.appendChild(opt);
        }
        for (const gpio of gpios) {
            const opt = document.createElement('option');
            opt.value = String(gpio);
            opt.textContent = `GPIO ${gpio}`;
            el.appendChild(opt);
        }
        setSelectValueIfPossible(el, t.defaultValue);
    }
}

function onUartConfigPacket(packet) {
    if (!packet) return;

    uartLastConfig = {
        baud_rate: packet.baud_rate >>> 0,
        tx_gpio: packet.tx_gpio >>> 0,
        rx_gpio: packet.rx_gpio >>> 0,
        reset_gpio: packet.reset_gpio >>> 0,
        control_gpio: packet.control_gpio >>> 0,
        led_gpio: packet.led_gpio >>> 0,
    };

    setSelectValueIfPossible(document.getElementById('uartCfgBaud'), uartLastConfig.baud_rate);
    setSelectValueIfPossible(document.getElementById('uartCfgTx'), uartLastConfig.tx_gpio);
    setSelectValueIfPossible(document.getElementById('uartCfgRx'), uartLastConfig.rx_gpio);
    setSelectValueIfPossible(document.getElementById('uartCfgReset'), uartLastConfig.reset_gpio);
    setSelectValueIfPossible(document.getElementById('uartCfgControl'), uartLastConfig.control_gpio);
    setSelectValueIfPossible(document.getElementById('uartCfgLed'), uartLastConfig.led_gpio);

    const resetName = uartLastConfig.reset_gpio === 255 ? 'Unused' : `GPIO${uartLastConfig.reset_gpio}`;
    const ctrlName = uartLastConfig.control_gpio === 255 ? 'Unused' : `GPIO${uartLastConfig.control_gpio}`;
    const ledName = uartLastConfig.led_gpio === 255 ? 'Unused' : `GPIO${uartLastConfig.led_gpio}`;
}

function onUartDataPacket(packet, meta) {
    if (!(packet instanceof Uint8Array) || packet.length === 0) return;

    const packetType = (meta && meta.packetType !== undefined) ? (meta.packetType >>> 0) : 0xFFFF;
    if (packetType !== 0x00) {
        return;
    }

    let text = '';
    try {
        text = uartTextDecoder.decode(packet);
    } catch (e) {
        text = Array.from(packet).map(v => bytesToHex2(v)).join(' ');
    }
    appendUartRxLine(text);
}

async function sendUartText() {
    if (!espSerial || !espSerial.port) {
        logToConsole('UART send: not connected', 'error');
        return;
    }

    const inputEl = document.getElementById('uartTxInput');
    const appendNlEl = document.getElementById('uartAppendNl');
    if (!inputEl) return;

    let text = inputEl.value || '';
    if (!text.length) return;
    if (appendNlEl && appendNlEl.checked) text += '\n';

    try {
        const bytes = new TextEncoder().encode(text);
        await espSerial.sendData(bytes);
        logToConsole(`UART TX: ${bytes.length} bytes`, 'info');
        inputEl.value = '';
    } catch (e) {
        logToConsole(`UART send error: ${e.message}`, 'error');
    }
}

async function requestUartConfig() {
    if (!espSerial || !isDeviceConnected) return;
    try {
        await espSerial.requestConfig();
        logToConsole('UART config requested', 'info');
    } catch (e) {
        logToConsole(`UART config request failed: ${e.message}`, 'error');
    }
}

async function startUart() {
    if (!espSerial || !isDeviceConnected) return;

    if (uartRunning) {
        await stopUart();
        return;
    }

    const baudRate = parseInt(document.getElementById('uartCfgBaud').value, 10);
    const txGpio = parseInt(document.getElementById('uartCfgTx').value, 10);
    const rxGpio = parseInt(document.getElementById('uartCfgRx').value, 10);
    const resetGpio = parseInt(document.getElementById('uartCfgReset').value, 10);
    const controlGpio = parseInt(document.getElementById('uartCfgControl').value, 10);
    const ledGpio = parseInt(document.getElementById('uartCfgLed').value, 10);

    if (Number.isNaN(baudRate) || Number.isNaN(txGpio) || Number.isNaN(rxGpio)) {
        logToConsole('UART config invalid: baud/tx/rx', 'error');
        return;
    }
    if (txGpio === rxGpio) {
        logToConsole('UART config invalid: TX and RX must differ', 'error');
        return;
    }

    try {
        await espSerial.setGatewayMode('UART');
        await espSerial.setConfig({
            baud_rate: baudRate,
            tx_gpio: txGpio,
            rx_gpio: rxGpio,
            reset_gpio: resetGpio,
            control_gpio: controlGpio,
            led_gpio: ledGpio,
            extended_mode: 1
        });
        uartRunning = true;
        setCanUiState();

        try {
            await espSerial.requestConfig();
            logToConsole('UART started', 'info');
        } catch (e) {
            logToConsole(`UART start failed: ${e.message}`, 'error');
        }
    } catch (e) {
        logToConsole(`UART config send failed: ${e.message}`, 'error');
    }
}

async function sendUartBreak() {
    if (!espSerial || !isDeviceConnected) return;
    const input = document.getElementById('uartBreakLenInput');
    let breakMs = input ? parseInt(input.value, 10) : 120;
    if (Number.isNaN(breakMs)) breakMs = 120;
    breakMs = Math.max(0, Math.min(200, breakMs));
    if (input) input.value = String(breakMs);

    try {
        const oldVal = document.getElementById('breakLenInput');
        if (oldVal) oldVal.value = String(breakMs);
        await espSerial.sendBreak();
        logToConsole(`UART break sent: ${breakMs} ms`, 'info');
    } catch (e) {
        logToConsole(`UART break failed: ${e.message}`, 'error');
    }
}

async function setUartReset(value) {
    if (!espSerial || !isDeviceConnected) return;
    try {
        await espSerial.setResetGpio(!!value);
        logToConsole(`UART reset GPIO set to ${value ? 'HIGH' : 'LOW'}`, 'info');
    } catch (e) {
        logToConsole(`UART reset GPIO update failed: ${e.message}`, 'error');
    }
}

async function setUartControl(value) {
    if (!espSerial || !isDeviceConnected) return;
    try {
        await espSerial.setControlGpio(!!value);
        logToConsole(`UART control GPIO set to ${value ? 'HIGH' : 'LOW'}`, 'info');
    } catch (e) {
        logToConsole(`UART control GPIO update failed: ${e.message}`, 'error');
    }
}


function updateUartUi() {
    const uartSendBtn = document.getElementById('uartSendBtn');
    const uartReadCfgBtn = document.getElementById('uartReadCfgBtn');
    const uartStartBtn = document.getElementById('uartStartBtn');
    const uartBreakBtn = document.getElementById('uartBreakBtn');
    const uartResetLowBtn = document.getElementById('uartResetLowBtn');
    const uartResetHighBtn = document.getElementById('uartResetHighBtn');
    const uartCtrlLowBtn = document.getElementById('uartCtrlLowBtn');
    const uartCtrlHighBtn = document.getElementById('uartCtrlHighBtn');
    const uartBreakLenInput = document.getElementById('uartBreakLenInput');
    const legacyBreakInput = document.getElementById('breakLenInput');
    const uartEnabled = !!isDeviceConnected;
    if (uartSendBtn) uartSendBtn.disabled = !uartEnabled;
    if (uartReadCfgBtn) uartReadCfgBtn.disabled = !uartEnabled;
    if (uartStartBtn) {
        uartStartBtn.disabled = !uartEnabled;
        uartStartBtn.textContent = uartRunning ? 'Stop' : 'Start';
    }
    if (uartBreakBtn) uartBreakBtn.disabled = !uartEnabled;
    if (uartResetLowBtn) uartResetLowBtn.disabled = !uartEnabled;
    if (uartResetHighBtn) uartResetHighBtn.disabled = !uartEnabled;
    if (uartCtrlLowBtn) uartCtrlLowBtn.disabled = !uartEnabled;
    if (uartCtrlHighBtn) uartCtrlHighBtn.disabled = !uartEnabled;
    if (legacyBreakInput && uartBreakLenInput) {
        legacyBreakInput.value = uartBreakLenInput.value || '120';
    }
}

async function stopUart(silent = false) {
    if (!espSerial || !isDeviceConnected) {
        uartRunning = false;
        setCanUiState();
        return;
    }
    if (!uartRunning) {
        setCanUiState();
        return;
    }

    try {
        await espSerial.setGatewayMode('NONE');
        uartRunning = false;
        setCanUiState();
        if (!silent) logToConsole('UART stopped', 'info');
    } catch (e) {
        if (!silent) logToConsole(`UART stop failed: ${e.message}`, 'error');
    }
}

