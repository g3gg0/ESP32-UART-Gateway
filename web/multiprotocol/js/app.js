/* ============= Global State and UI Functions ============= */
let espSerial = null;
let tadaaAudio = null;
let tadaaAudioPrimed = false;
let tadaaAudioUrl = null;

const MAX_CONSOLE_LINES = 500;
const consoleLineBuffer = [];

/* audio_embed_start */
if (window.TADAA_MP3_B64 === undefined) window.TADAA_MP3_B64 = null;
/* audio_embed_stop */

let isDeviceConnected = false;
let connectionBusy = false;
let activeProtocolTab = 'swd';

/* Initialize ESP Serial with callbacks */
function initializeEspSerial() {
    espSerial = new EspSerial();

    /* Set up log callback */
    espSerial.setLogCallback((packet) => {
        logToConsole(`${packet.text}`, 'log');
    });

    espSerial.setDataCallback((packet, meta) => {
        onUartDataPacket(packet, meta);
    });

    espSerial.setCanCallback((packet) => {
        onCanPacket(packet);
    });

    espSerial.setConfigCallback((packet) => {
        onUartConfigPacket(packet);
    });

    return espSerial;
}

/* ============= Protocol constants and shared helpers ============= */

function u32ToHex(v) {
    const x = (v >>> 0).toString(16).toUpperCase().padStart(8, '0');
    return `0x${x}`;
}

function readU32LE(bytes, off = 0) {
    return ((bytes[off] >>> 0) |
        ((bytes[off + 1] >>> 0) << 8) |
        ((bytes[off + 2] >>> 0) << 16) |
        ((bytes[off + 3] >>> 0) << 24)) >>> 0;
}

function u32FromBytesLE(bytes, off = 0) {
    return readU32LE(bytes, off) >>> 0;
}

function writeU32LE(dst, off, v) {
    const x = v >>> 0;
    dst[off] = x & 0xFF;
    dst[off + 1] = (x >>> 8) & 0xFF;
    dst[off + 2] = (x >>> 16) & 0xFF;
    dst[off + 3] = (x >>> 24) & 0xFF;
}

function parseHexOrDec(text) {
    const s = (text || '').trim();
    if (s.startsWith('0x') || s.startsWith('0X')) return parseInt(s.substring(2), 16) >>> 0;
    return parseInt(s, 10) >>> 0;
}

function bytesToHex2(b) {
    return (b & 0xFF).toString(16).toUpperCase().padStart(2, '0');
}

async function setActiveProtocolTab(tabName) {
    const target = String(tabName || '').toLowerCase();
    if (!['swd', 'can', 'uart', 'canonef'].includes(target)) return;

    if (activeProtocolTab !== target && espSerial && isDeviceConnected) {
        try {
            if (activeProtocolTab === 'swd') {
                await stopSwd('Switching away from SWD', true);
            } else if (activeProtocolTab === 'can') {
                await stopCan(true);
            } else if (activeProtocolTab === 'uart') {
                await stopUart(true);
            } else if (activeProtocolTab === 'canonef') {
                canonEfRunning = false;
                canonEfInitialized = false;
            }
        } catch (e) {
            /* ignore */
        }
    }

    activeProtocolTab = target;
    updateProtocolVisibility();
}

function updateProtocolVisibility() {
    const tabMap = [
        { tab: 'swd', btnId: 'protoTabSwd', panelId: 'deviceInteraction' },
        { tab: 'can', btnId: 'protoTabCan', panelId: 'canInteraction' },
        { tab: 'uart', btnId: 'protoTabUart', panelId: 'uartInteraction' },
        { tab: 'canonef', btnId: 'protoTabCanonEf', panelId: 'canonEfInteraction' },
    ];

    for (const entry of tabMap) {
        const btn = document.getElementById(entry.btnId);
        const panel = document.getElementById(entry.panelId);
        const isActive = (activeProtocolTab === entry.tab);

        if (btn) {
            btn.classList.toggle('active', isActive);
            btn.disabled = !isDeviceConnected;
        }
        if (panel) {
            panel.style.display = (isDeviceConnected && isActive) ? '' : 'none';
        }
    }
}

// Shared dispatcher retained for existing callers across protocols.
function setCanUiState() {
    updateCanUi();
    updateUartUi();
    updateEfUi();
}

function showToast(message, kind = 'error', opts = {}) {
    const container = document.getElementById('toastContainer');
    if (!container) return;
    const text = (message === null || message === undefined) ? '' : String(message);
    const timeoutMs = (opts && opts.timeoutMs) ? (opts.timeoutMs | 0) : 2200;

    const el = document.createElement('div');
    el.className = `toast ${kind}`;
    el.textContent = text;
    container.appendChild(el);

    /* trigger transition */
    requestAnimationFrame(() => {
        el.classList.add('show');
        if (kind === 'error') el.classList.add('flash');
    });

    setTimeout(() => {
        try { el.classList.remove('show'); } catch (e) { /* ignore */ }
        setTimeout(() => {
            try { el.remove(); } catch (e) { /* ignore */ }
        }, 240);
    }, Math.max(400, timeoutMs));
}

/* ============= Core UI and Connection Functions ============= */

function renderConsole() {
    const consoleEl = document.getElementById('consoleDisplay');
    if (!consoleEl) return;
    const level = document.getElementById('consoleLevel')?.value || 'all';
    const query = (document.getElementById('consoleFilter')?.value || '').toLowerCase();
    const visible = consoleLineBuffer.filter(entry => (level === 'all' || entry.level === level) && entry.message.toLowerCase().includes(query));
    consoleEl.textContent = visible.map(entry => `${entry.time} [${entry.level.toUpperCase()}] ${entry.message}${entry.count > 1 ? ` (\u00d7${entry.count})` : ''}`).join('\n');
    consoleEl.scrollTop = consoleEl.scrollHeight;
    const count = document.getElementById('consoleCount');
    if (count) count.textContent = `${consoleLineBuffer.reduce((total, entry) => total + entry.count, 0)} messages`;
}

function logToConsole(message, level = 'info') {
    message = String(message);
    level = level || 'info';
    const last = consoleLineBuffer.at(-1);
    const time = new Date().toLocaleTimeString();
    if (last && last.message === message && last.level === level) {
        last.count++;
        last.time = time;
    } else {
        consoleLineBuffer.push({ message, level, time, count: 1 });
    }
    if (consoleLineBuffer.length > MAX_CONSOLE_LINES) {
        consoleLineBuffer.splice(0, consoleLineBuffer.length - MAX_CONSOLE_LINES);
    }
    renderConsole();
}

function clearConsole() {
    consoleLineBuffer.length = 0;
    renderConsole();
}

function getTadaaAudio() {
    if (tadaaAudio) return tadaaAudio;

    const embeddedB64 = (window.TADAA_MP3_B64 !== undefined) ? window.TADAA_MP3_B64 : null;
    if (embeddedB64 && typeof embeddedB64 === 'string' && embeddedB64.length) {
        if (!tadaaAudioUrl) {
            const bin = atob(embeddedB64);
            const u8 = new Uint8Array(bin.length);
            for (let i = 0; i < bin.length; i++) u8[i] = bin.charCodeAt(i) & 0xFF;
            const blob = new Blob([u8], { type: 'audio/mpeg' });
            tadaaAudioUrl = URL.createObjectURL(blob);
        }
        const a = new Audio(tadaaAudioUrl);
        a.preload = 'auto';
        tadaaAudio = a;
        return a;
    }

    const a = new Audio('tadaa.mp3');
    a.preload = 'auto';
    tadaaAudio = a;
    return a;
}

function primeTadaaAudio() {
    if (tadaaAudioPrimed) return;
    tadaaAudioPrimed = true;
    try {
        const a = getTadaaAudio();
        a.load();
        const oldVolume = a.volume;
        a.volume = 0;
        const p = a.play();
        if (p && typeof p.then === 'function') {
            p.then(() => {
                a.pause();
                a.currentTime = 0;
                a.volume = oldVolume;
            }).catch(() => {
                a.pause();
                a.currentTime = 0;
                a.volume = oldVolume;
            });
        } else {
            a.pause();
            a.currentTime = 0;
            a.volume = oldVolume;
        }
    } catch (e) {
        /* ignore */
    }
}

function playTadaa() {
    try {
        const a = getTadaaAudio();
        a.currentTime = 0;
        const p = a.play();
        if (p && typeof p.catch === 'function') {
            p.catch(() => { /* ignore autoplay blocking */ });
        }
    } catch (e) {
        /* ignore */
    }
}

function renderConnectionState() {
    const state = connectionBusy ? 'busy' : (isDeviceConnected ? 'connected' : 'disconnected');
    const button = document.getElementById('connectBtn');
    button.disabled = connectionBusy;
    button.textContent = connectionBusy
        ? (isDeviceConnected ? 'Disconnecting?' : 'Connecting?')
        : (isDeviceConnected ? 'Disconnect' : 'Connect');
    document.getElementById('statusIndicator').dataset.state = state;
    document.getElementById('connectionStatusText').textContent = connectionBusy
        ? (isDeviceConnected ? 'Disconnecting' : 'Connecting')
        : (isDeviceConnected ? 'Connected' : 'Disconnected');
}

async function toggleConnection() {
    if (connectionBusy) return;
    await (isDeviceConnected ? disconnectDevice() : connectDevice());
}

async function connectDevice() {
    if (connectionBusy) return;
    connectionBusy = true;
    renderConnectionState();
    try {
        const btn = document.getElementById('connectBtn');

        logToConsole('Connecting...', 'info');

        const esp = initializeEspSerial();
        if (await esp.connect()) {
            logToConsole('Connected successfully', 'info');
            isDeviceConnected = true;
            canRunning = false;
            uartRunning = false;
            canonEfRunning = false;
            canonEfInitialized = false;

            try {
                await espSerial.setGatewayMode('NONE');
            } catch (e) {
                logToConsole(`Failed to enter NONE mode: ${e.message}`, 'error');
            }

            setCanUiState();
            updateProtocolVisibility();

            connectSwdUi();
        } else {
            logToConsole('Connection failed', 'error');
            isDeviceConnected = false;
            canRunning = false;
            uartRunning = false;
            canonEfRunning = false;
            setCanUiState();
            updateProtocolVisibility();
        }
    } catch (err) {
        logToConsole(`Connection error: ${err.message}`, 'error');
        isDeviceConnected = false;
        canRunning = false;
        uartRunning = false;
        canonEfRunning = false;
        canonEfInitialized = false;
        setCanUiState();
        updateProtocolVisibility();
    } finally {
        connectionBusy = false;
        renderConnectionState();
    }
}

async function disconnectDevice() {
    if (connectionBusy) return;
    connectionBusy = true;
    renderConnectionState();
    try {
        const btn = document.getElementById('connectBtn');
        if (canRunning) {
            try {
                await stopCan(true);
            } catch (e) {
                /* ignore */
            }
        }

        await stopSwdOperations('Disconnecting');

        if (espSerial) {
            await espSerial.disconnect();
        }

        isDeviceConnected = false;
        canRunning = false;
        uartRunning = false;
        canonEfRunning = false;
        canonEfInitialized = false;
        uartLastConfig = null;

        resetSwdUi();

        clearCanMessages();
        clearUartRx();
        clearCanonEf();

        logToConsole('Disconnected', 'info');

        setCanUiState();
        updateProtocolVisibility();
    } catch (err) {
        logToConsole(`Disconnect error: ${err.message}`, 'error');
    } finally {
        connectionBusy = false;
        renderConnectionState();
    }
}

window.onload = () => {
    logToConsole('Multi-protocol console ready', 'info');
    logToConsole('Click Connect to begin', 'info');

    /* Initialize GPIO checkboxes */
    initializeGpioCheckboxes();
    buildUartGpioSelects();
    initializeCanonEfAutoConfigure();

    updateProtocolVisibility();
    setCanUiState();

    /* Wire up connect/disconnect buttons */
    const connectBtn = document.getElementById('connectBtn');
    if (connectBtn) connectBtn.onclick = toggleConnection;
    renderConnectionState();

    const uartTxInput = document.getElementById('uartTxInput');
    if (uartTxInput) {
        uartTxInput.addEventListener('keydown', (ev) => {
            if (ev.key === 'Enter') {
                ev.preventDefault();
                sendUartText();
            }
        });
    }

    const canonEfTxInput = document.getElementById('canonEfTxInput');
    if (canonEfTxInput) {
        canonEfTxInput.addEventListener('keydown', (ev) => {
            if (ev.key === 'Enter') {
                ev.preventDefault();
                sendCanonEf();
            }
        });
    }
};
