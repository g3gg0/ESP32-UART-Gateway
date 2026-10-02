/* ============= CC3200Serial Class ============= */




/* ============= Global State and UI Functions ============= */
let espSerial = null;
let cc3200Serial = null;
let displayBuffer = new Uint8Array(0);
let displayTimer = null;
let hexDumpOffset = 0;
let sendMode = 'hex';

/* Initialize ESP Serial with callbacks */
function initializeEspSerial() {
    espSerial = new EspSerial();

    /* Set up data callback */
    espSerial.setDataCallback((data) => {
        const hexStr = Array.from(data.slice(0, 32)).map(b => b.toString(16).padStart(2, '0')).join(' ');
        const suffix = data.length > 32 ? ' ...' : '';
        logToConsole(`Data packet: ${hexStr}${suffix}`, 'info');
    });

    /* Set up config callback */
    espSerial.setConfigCallback((packet) => {
        logToConsole(`Config received: ${packet.data.length} bytes`, 'info');
    });

    /* Set up log callback */
    espSerial.setLogCallback((packet) => {
        logToConsole(`[LOG] ${packet.text}`, 'debug');
    });

    return espSerial;
}

function padTo64Bytes(packet) {
    if (packet.length >= 64) {
        return packet;
    }
    const padded = new Uint8Array(64);
    padded.set(packet, 0);
    return padded;
}

function hexdump(buffer) {
    let output = '';
    for (let i = 0; i < buffer.length; i += 16) {
        const chunk = buffer.slice(i, Math.min(i + 16, buffer.length));
        let hex = '';
        let ascii = '';

        for (let j = 0; j < 16; j++) {
            if (j < chunk.length) {
                const byte = chunk[j];
                hex += byte.toString(16).padStart(2, '0').toUpperCase() + ' ';
                ascii += (byte >= 32 && byte <= 126) ? String.fromCharCode(byte) : '.';
            } else {
                hex += '   ';
                ascii += ' ';
            }
            if (j === 7) hex += ' ';
        }

        output += `${(hexDumpOffset + i).toString(16).padStart(8, '0')}:  ${hex} | ${ascii}\n`;
    }
    hexDumpOffset += buffer.length;
    return output;
}

function logToConsole(text, type = 'normal') {
    const consoleDisplay = document.getElementById('consoleDisplay');
    const timestamp = new Date().toLocaleTimeString();
    const prefix = type === 'tx' ? '[TX]' : type === 'rx' ? '[RX]' : type === 'log' ? '[LOG]' : '[INFO]';
    consoleDisplay.textContent += `${timestamp} ${prefix} ${text}\n`;
    consoleDisplay.scrollTop = consoleDisplay.scrollHeight;
}

function flushDisplayBuffer() {
    if (displayBuffer.length > 0) {
        logToConsole(hexdump(displayBuffer), 'rx');
        displayBuffer = new Uint8Array(0);
    }
    displayTimer = null;
}

function scheduleDisplay() {
    if (displayTimer) {
        clearTimeout(displayTimer);
    }
    displayTimer = setTimeout(flushDisplayBuffer, 100);
}

function clearConsole() {
    document.getElementById('consoleDisplay').textContent = '';
    logToConsole('Console cleared', 'info');
}

/* Load a flash image from a file instead of hardware */
async function connectDevice() {
    try {
        espSerial = new EspSerial();
        cc3200Serial = new CC3200Serial(espSerial);
        sffsClient = null;

        /* ESP behaves like a UART: log packets go to UI, raw bytes go to CC3200 + hexdump */
        espSerial.setConfigCallback((parsedConfig) => {
            logToConsole(`Config: Baud=${parsedConfig.baud_rate} TX=${parsedConfig.tx_gpio} RX=${parsedConfig.rx_gpio} Reset=${parsedConfig.reset_gpio} Ctrl=${parsedConfig.control_gpio} LED=${parsedConfig.led_gpio} ExtMode=${parsedConfig.extended_mode}`, 'info');
        });
        espSerial.setLogCallback((packet) => {
            logToConsole(packet.text, 'log');
        });
        espSerial.setDataCallback((data) => {
            cc3200Serial.processCC3200Response(data);
            // displayBuffer = espSerial.appendBuffer(displayBuffer, data);
            // scheduleDisplay();
        });

        /* Optional CC3200 events (don’t duplicate raw bytes in UI) */
        cc3200Serial.setDataCallback((packet) => {
        });

        /* Connect to device */
        if (!await espSerial.connect()) {
            return;
        }

        document.getElementById('connectBtn').disabled = true;
        document.getElementById('disconnectBtn').disabled = false;
        document.getElementById('statusIndicator').textContent = 'Connected';
        document.getElementById('statusIndicator').classList.add('connected');

        logToConsole('Connected to device', 'info');

        await new Promise(resolve => setTimeout(resolve, 500));
        await espSerial.setControlGpio(true);
        await espSerial.setResetGpio(false);
        await new Promise(resolve => setTimeout(resolve, 200));

        /* Start CC3200 sync sequence */
        await cc3200Serial.performSync();
    } catch (err) {
        if (err.name !== 'NotFoundError') {
            logToConsole(`Connection error: ${err.message}`, 'info');
        }
    }
}

async function disconnectDevice() {
    try {
        if (espSerial) {
            await espSerial.disconnect();
        }
        if (cc3200Serial) {
            await cc3200Serial.disconnect();
        }

        /* Flush any pending display buffer */
        if (displayTimer) {
            clearTimeout(displayTimer);
            displayTimer = null;
        }
        flushDisplayBuffer();

        document.getElementById('connectBtn').disabled = false;
        document.getElementById('disconnectBtn').disabled = true;
        document.getElementById('statusIndicator').textContent = 'Disconnected';
        document.getElementById('statusIndicator').classList.remove('connected');
        document.getElementById('resetCheckbox').checked = false;
        document.getElementById('controlCheckbox').checked = false;

        logToConsole('Disconnected from device', 'info');
    } catch (err) {
        logToConsole(`Disconnect error: ${err.message}`, 'info');
    }
}

/* Hexdump formatter for console logging - shows hex and ASCII */
function consoleLogHex(prefix, data) {
    let hexStr = '';
    let asciiStr = '';
    for (let i = 0; i < data.length; i++) {
        const byte = data[i];
        hexStr += byte.toString(16).padStart(2, '0').toUpperCase() + ' ';
        /* ASCII display: printable chars, else . */
        asciiStr += (byte >= 32 && byte < 127) ? String.fromCharCode(byte) : '.';
    }
    /* Only log important messages, not raw packet dumps */
    //if (prefix.includes('Magic') || prefix.includes('Extended')) {
    //logToConsole(`${prefix} [${data.length} bytes] HEX: ${hexStr.trim()} | ASCII: "${asciiStr}"`, 'info');
    //}
}

async function sendCommand() {
    const input = document.getElementById('commandInput');
    const command = input.value;

    if (!command) return;
    if (!espSerial) {
        logToConsole('Not connected', 'info');
        return;
    }

    try {
        const encoder = new TextEncoder();
        const data = encoder.encode(command + '\n');
        await espSerial.sendData(data);
        /* consoleLogHex('TX Text:', data); */

        /* Display sent data as hexdump */
        /* const hexLines = hexdump(data); */
        logToConsole('Text sent', 'tx');
        input.value = '';
    } catch (err) {
        logToConsole(`Send error: ${err.message}`, 'info');
    }
}

function handleKeyPress(event) {
    if (event.key === 'Enter') {
        sendCommand();
    }
}

function handleHexKeyPress(event) {
    if (event.key === 'Enter') {
        sendHex();
    }
}

async function sendBreak() {
    if (!espSerial) {
        logToConsole('Not connected', 'info');
        return;
    }
    await espSerial.sendBreak();
}

async function sendData() {
    const input = document.getElementById('hexInput');
    const hexString = input.value.trim();

    if (!hexString) return;
    if (!espSerial) {
        logToConsole('Not connected', 'info');
        return;
    }

    try {
        /* Parse hex string */
        const bytes = hexString.split(/\s+/)
            .filter(h => h.length > 0)
            .map((h, idx) => {
                const val = parseInt(h, 16);
                if (isNaN(val) || val < 0 || val > 255) {
                    throw new Error(`Invalid hex byte at position ${idx}: ${h}`);
                }
                return val;
            });

        let data = new Uint8Array(bytes);

        /* Send based on mode */
        if (sendMode === 'cc3200') {
            await cc3200Serial.sendCommand(data);
        } else {
            await espSerial.sendData(data);
            /* consoleLogHex('TX Hex:', data); */

            /* const hexLines = hexdump(data); */
            logToConsole('Hex sent', 'tx');
        }

        input.value = '';
    } catch (err) {
        logToConsole(`Send error: ${err.message}`, 'info');
    }
}

async function sendHex() {
    await sendData();
}

async function setResetGpio(state) {
    if (!espSerial) {
        document.getElementById('resetCheckbox').checked = false;
        return;
    }

    try {
        await espSerial.setResetGpio(state);
    } catch (err) {
        logToConsole(`Command error: ${err.message}`, 'info');
        document.getElementById('resetCheckbox').checked = !state;
    }
}

async function setControlGpio(state) {
    if (!espSerial) {
        document.getElementById('controlCheckbox').checked = false;
        return;
    }

    try {
        await espSerial.setControlGpio(state);
    } catch (err) {
        logToConsole(`Command error: ${err.message}`, 'info');
        document.getElementById('controlCheckbox').checked = !state;
    }
}

window.onload = () => {
    initCCTabs();
    logToConsole('Console ready', 'info');
    logToConsole('Connect to start reading UART data', 'info');
};
