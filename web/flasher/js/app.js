function isWebSerialSupported() {
    return typeof navigator !== 'undefined' && !!navigator.serial;
}

function getWebSerialUnsupportedMessage() {
    const ua = (typeof navigator !== 'undefined' && navigator.userAgent) ? navigator.userAgent : '';
    const isFirefox = /firefox/i.test(ua);
    if (isFirefox) {
        return 'WebSerial is not available in Firefox. Please use Microsoft Edge or Google Chrome to flash/configure this device.';
    }
    return 'WebSerial is not available in this browser. Please use Microsoft Edge or Google Chrome to flash/configure this device.';
}

function updateWebSerialUiState() {
    const supported = isWebSerialSupported();
    const msg = supported ? '' : getWebSerialUnsupportedMessage();

    const flashNotice = document.getElementById('webSerialNoticeFlash');
    const configNotice = document.getElementById('webSerialNoticeConfig');
    const connectBtn = document.getElementById('connectBtn');
    const cfgConnectBtn = document.getElementById('cfgConnectBtn');

    if (flashNotice) {
        flashNotice.textContent = msg;
        flashNotice.style.display = supported ? 'none' : 'block';
    }
    if (configNotice) {
        configNotice.textContent = msg;
        configNotice.style.display = supported ? 'none' : 'block';
    }

    if (connectBtn) {
        connectBtn.style.display = supported ? 'inline-block' : 'none';
    }
    if (cfgConnectBtn) {
        cfgConnectBtn.style.display = supported ? 'inline-block' : 'none';
    }

    return supported;
}

/* ============= Hexdump Logging Functions ============= */
function hexdumpByte(b) {
    return b.toString(16).padStart(2, '0').toUpperCase();
}

function hexdumpRow(offset, data, maxLen = 16) {
    const hex = Array.from(data.slice(offset, Math.min(offset + maxLen, data.length)))
        .map(hexdumpByte)
        .join(' ');
    const ascii = Array.from(data.slice(offset, Math.min(offset + maxLen, data.length)))
        .map(b => (b >= 32 && b < 127) ? String.fromCharCode(b) : '.')
        .join('');
    const offsetStr = offset.toString(16).padStart(8, '0').toUpperCase();
    const hexPadded = hex.padEnd(48, ' ');
    return `${offsetStr}  ${hexPadded}  ${ascii}`;
}

function flashHexdump(label, data, maxBytes = 512) {
    if (!data || data.length === 0) {
        console.log(`${label}: [empty]`);
        return;
    }
    const actualLen = Math.min(data.length, maxBytes);
    const isLimited = data.length > maxBytes;
    console.log(`${label}: ${data.length} bytes${isLimited ? ` (showing first ${maxBytes})` : ''}`);
    for (let offset = 0; offset < actualLen; offset += 16) {
        console.log(hexdumpRow(offset, data, 16));
    }
    if (isLimited) {
        console.log(`... (${data.length - maxBytes} more bytes)`);
    }
}

function logPacketTX(label, packet) {
    if (!packet || packet.length === 0) {
        console.log(`TX ${label}: [empty packet]`);
        return;
    }
    const bytes = packet instanceof Uint8Array ? packet : new Uint8Array(packet);
    console.log(`%cTX ${label}`, 'color: #0ea5e9; font-weight: bold;', `(${bytes.length} bytes)`);
    flashHexdump('', bytes, 256);
}

function logPacketRX(label, packet) {
    if (!packet || packet.length === 0) {
        console.log(`RX ${label}: [empty packet]`);
        return;
    }
    const bytes = packet instanceof Uint8Array ? packet : new Uint8Array(packet);
    console.log(`%cRX ${label}`, 'color: #10b981; font-weight: bold;', `(${bytes.length} bytes)`);
    flashHexdump('', bytes, 256);
}


function getEmbeddedBinary(filename) {
    const b64 = EMBEDDED_BINARIES[filename];
    if (!b64) {
        throw new Error(`Embedded binary not found: ${filename}`);
    }
    const raw = atob(b64);
    const bytes = new Uint8Array(raw.length);
    for (let i = 0; i < raw.length; i++) {
        bytes[i] = raw.charCodeAt(i);
    }
    return bytes;
}

function log(message, type = 'info') {
    const logContainer = document.getElementById('logContainer');
    const timestamp = new Date().toLocaleTimeString();
    const entry = document.createElement('div');
    entry.className = `log-entry log-${type}`;
    entry.textContent = `[${timestamp}] ${message}`;
    logContainer.appendChild(entry);
    logContainer.scrollTop = logContainer.scrollHeight;
}

function updateProgress(percent, text) {
    const progressBar = document.getElementById('progressBar');
    progressBar.style.width = percent + '%';
    progressBar.textContent = text || (percent + '%');
}

/* ------------ Tab + Config tool ------------ */
function setActiveTool(tool) {
    const flashPane = document.getElementById('flashTool');
    const configPane = document.getElementById('configTool');
    const flashBtn = document.getElementById('flashTabBtn');
    const configBtn = document.getElementById('configTabBtn');

    if (tool === 'config') {
        flashPane.classList.add('hidden');
        configPane.classList.remove('hidden');
        flashBtn.classList.remove('active');
        configBtn.classList.add('active');
    }
    else {
        configPane.classList.add('hidden');
        flashPane.classList.remove('hidden');
        configBtn.classList.remove('active');
        flashBtn.classList.add('active');
    }
}

window.onload = () => {
    log('ESP32-C3 UART Gateway Flasher ready', 'info');
    log('Loading built-in default configuration...', 'info');
    updateWebSerialUiState();
    autoLoadConfig();
    setActiveTool('flash');
};
