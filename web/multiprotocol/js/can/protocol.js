/* CAN state, transport handlers and controls. */

let canRunning = false;

const canMessageMap = new Map();


const CAN_UART_OP_START = 0x01;
const CAN_UART_OP_STOP = 0x02;
const CAN_UART_OP_RX_FRAME = 0x10;

const CAN_UART_STATUS_OK = 0x00;

const CAN_UART_FRAME_FLAG_EXTD = 0x01;
const CAN_UART_FRAME_FLAG_RTR = 0x02;
const CAN_UART_FRAME_FLAG_SS = 0x04;
const CAN_UART_FRAME_FLAG_SELF = 0x08;
const CAN_UART_FRAME_FLAG_DLC_NON_COMP = 0x10;

const CAN_UART_EVENT_FLAG_RX_QUEUE_FULL = 0x0001;
const CAN_UART_EVENT_FLAG_RX_FIFO_OVERRUN = 0x0002;
const CAN_UART_EVENT_FLAG_ABOVE_ERR_WARN = 0x0004;
const CAN_UART_EVENT_FLAG_BELOW_ERR_WARN = 0x0008;
const CAN_UART_EVENT_FLAG_ERR_PASS = 0x0010;
const CAN_UART_EVENT_FLAG_ARB_LOST = 0x0020;
const CAN_UART_EVENT_FLAG_BUS_ERROR = 0x0040;
const CAN_UART_EVENT_FLAG_BUS_OFF = 0x0080;
const CAN_UART_EVENT_FLAG_BUS_RECOVERED = 0x0100;


function readU16LE(bytes, off = 0) {
    return ((bytes[off] >>> 0) |
        ((bytes[off + 1] >>> 0) << 8)) >>> 0;
}

function readU64LEAsNumber(bytes, off = 0) {
    const lo = readU32LE(bytes, off);
    const hi = readU32LE(bytes, off + 4);
    return (hi * 4294967296) + lo;
}


function updateCanUi() {
    const startBtn = document.getElementById('startCanBtn');
    const uartEnabled = !!isDeviceConnected;
    if (startBtn) {
        startBtn.disabled = !isDeviceConnected;
        startBtn.textContent = canRunning ? 'Stop' : 'Start';
    }
}

function fmtEspTimestampUs(us) {
    const totalUs = Math.max(0, Math.floor(us || 0));
    const totalMs = Math.floor(totalUs / 1000);
    const ms = totalMs % 1000;
    const totalSec = Math.floor(totalMs / 1000);
    const ss = totalSec % 60;
    const totalMin = Math.floor(totalSec / 60);
    const mm = totalMin % 60;
    const hh = Math.floor(totalMin / 60) % 100;
    const usRem = totalUs % 1000;
    return `${String(hh).padStart(2, '0')}:${String(mm).padStart(2, '0')}:${String(ss).padStart(2, '0')}.${String(ms).padStart(3, '0')}${String(usRem).padStart(3, '0')}`;
}

function decodeCanFrame(data) {
    if (!data || data.length < 32) return null;
    const id = readU32LE(data, 0);
    const dlc = data[4] >>> 0;
    const flags = data[5] >>> 0;
    const eventFlags = readU16LE(data, 6);
    const timestampUs = readU64LEAsNumber(data, 8);
    const rxMissedCount = readU32LE(data, 16);
    const rxOverrunCount = readU32LE(data, 20);
    const payload = data.slice(24, 32);
    return {
        id,
        dlc,
        flags,
        eventFlags,
        timestampUs,
        rxMissedCount,
        rxOverrunCount,
        extd: !!(flags & CAN_UART_FRAME_FLAG_EXTD),
        rtr: !!(flags & CAN_UART_FRAME_FLAG_RTR),
        data: payload
    };
}

function fmtCanData(bytes, dlc) {
    const out = [];
    const n = Math.min(dlc >>> 0, 8);
    for (let i = 0; i < n; i++) out.push(bytesToHex2(bytes[i] & 0xFF));
    return out.join(' ');
}

function fmtCanId(frame) {
    if (frame.extd) return `EXT:${u32ToHex(frame.id)}`;
    return `STD:0x${(frame.id & 0x7FF).toString(16).toUpperCase().padStart(3, '0')}`;
}

function fmtCanFrameFlags(frame) {
    const out = [];
    if (frame.extd) out.push('EXTD');
    if (frame.rtr) out.push('RTR');
    if (frame.flags & CAN_UART_FRAME_FLAG_SS) out.push('SS');
    if (frame.flags & CAN_UART_FRAME_FLAG_SELF) out.push('SELF');
    if (frame.flags & CAN_UART_FRAME_FLAG_DLC_NON_COMP) out.push('DLC_NC');
    if (!out.length) out.push('STD');
    return out.join('|');
}

function fmtCanEventFlags(frame) {
    const f = frame.eventFlags >>> 0;
    const out = [];
    if (f & CAN_UART_EVENT_FLAG_RX_QUEUE_FULL) out.push('RXQ_FULL');
    if (f & CAN_UART_EVENT_FLAG_RX_FIFO_OVERRUN) out.push('RX_OVR');
    if (f & CAN_UART_EVENT_FLAG_ABOVE_ERR_WARN) out.push('WARN+');
    if (f & CAN_UART_EVENT_FLAG_BELOW_ERR_WARN) out.push('WARN-');
    if (f & CAN_UART_EVENT_FLAG_ERR_PASS) out.push('ERR_PASS');
    if (f & CAN_UART_EVENT_FLAG_ARB_LOST) out.push('ARB_LOST');
    if (f & CAN_UART_EVENT_FLAG_BUS_ERROR) out.push('BUS_ERR');
    if (f & CAN_UART_EVENT_FLAG_BUS_OFF) out.push('BUS_OFF');
    if (f & CAN_UART_EVENT_FLAG_BUS_RECOVERED) out.push('RECOVERED');
    if (!out.length) out.push('-');
    return out.join('|');
}

function canFrameRowKey(frame) {
    const tag = frame.extd ? 'E' : 'S';
    return `${tag}:${frame.id >>> 0}`;
}

function upsertCanMessageView(frame) {
    const body = document.getElementById('canMessageTableBody');
    if (!body) return;

    const key = canFrameRowKey(frame);
    let row = canMessageMap.get(key);
    if (!row) {
        row = document.createElement('tr');
        row.innerHTML = `
                    <td></td>
                    <td></td>
                    <td></td>
                    <td></td>
                    <td></td>
                    <td></td>
                    <td></td>
                    <td></td>
                `;
        row.dataset.count = '0';
        body.appendChild(row);
        canMessageMap.set(key, row);
    }

    const nowUs = frame.timestampUs;
    const prevTsUs = Number(row.dataset.lastTsUs || NaN);
    const deltaUs = Number.isFinite(prevTsUs) ? Math.max(0, nowUs - prevTsUs) : NaN;
    const deltaText = Number.isFinite(deltaUs) ? `${(deltaUs / 1000).toFixed(3)} ms` : '—';

    const count = (parseInt(row.dataset.count || '0', 10) + 1) >>> 0;
    row.dataset.count = String(count);
    row.dataset.lastTsUs = String(nowUs);

    row.cells[0].textContent = fmtCanId(frame);
    row.cells[1].textContent = fmtEspTimestampUs(frame.timestampUs);
    row.cells[2].textContent = deltaText;
    row.cells[3].textContent = fmtCanFrameFlags(frame);
    row.cells[4].textContent = fmtCanEventFlags(frame);
    row.cells[5].textContent = String(frame.dlc >>> 0);
    row.cells[6].textContent = frame.rtr ? '-' : fmtCanData(frame.data, frame.dlc);
    row.cells[7].textContent = String(count);
    row.title = `rx_missed=${frame.rxMissedCount} rx_overrun=${frame.rxOverrunCount}`;
}

function onCanPacket(packet) {
    if (!packet) return;
    if (packet.req_op !== CAN_UART_OP_RX_FRAME || packet.status !== CAN_UART_STATUS_OK) return;

    const frame = decodeCanFrame(packet.data);
    if (!frame) return;
    upsertCanMessageView(frame);
}


function clearCanMessages() {
    canMessageMap.clear();
    const body = document.getElementById('canMessageTableBody');
    if (body) body.innerHTML = '';
}

async function startCan() {
    if (!espSerial || !isDeviceConnected) return;

    if (canRunning) {
        await stopCan();
        return;
    }

    try {
        const rx = parseInt(document.getElementById('canRx').value, 10);
        const tx = parseInt(document.getElementById('canTx').value, 10);
        const baudKbit = parseInt(document.getElementById('canBaud').value, 10);

        if (Number.isNaN(rx) || Number.isNaN(tx)) {
            logToConsole('Invalid CAN RX/TX GPIO', 'error');
            return;
        }
        if (Number.isNaN(baudKbit) || baudKbit <= 0) {
            logToConsole('Invalid CAN baud rate (kbit/s)', 'error');
            return;
        }

        const baud = baudKbit * 1000;
        const args = new Uint8Array(8);
        args[0] = rx & 0xFF;
        args[1] = tx & 0xFF;
        args[2] = 0;
        args[3] = 0;
        args[4] = baud & 0xFF;
        args[5] = (baud >>> 8) & 0xFF;
        args[6] = (baud >>> 16) & 0xFF;
        args[7] = (baud >>> 24) & 0xFF;

        await espSerial.setGatewayMode('CAN');

        const rsp = await espSerial.canRequest(CAN_UART_OP_START, args, { timeoutMs: 3000 });
        if (!rsp || rsp.status !== CAN_UART_STATUS_OK) {
            logToConsole(`CAN start failed (status=${rsp ? rsp.status : 'n/a'})`, 'error');
            return;
        }

        canRunning = true;
        setCanUiState();
        logToConsole(`CAN started: RX=${rsp.can_rx_gpio} TX=${rsp.can_tx_gpio} baud=${baudKbit} kbit/s`, 'info');
    } catch (e) {
        logToConsole(`CAN start error: ${e.message}`, 'error');
    }
}

async function stopCan(silent = false) {
    if (!espSerial || !isDeviceConnected) {
        canRunning = false;
        setCanUiState();
        return;
    }
    if (!canRunning) {
        setCanUiState();
        return;
    }
    try {
        const rsp = await espSerial.canRequest(CAN_UART_OP_STOP, new Uint8Array(0), { timeoutMs: 1500 });
        if (!rsp || rsp.status !== CAN_UART_STATUS_OK) {
            if (!silent) logToConsole(`CAN stop failed (status=${rsp ? rsp.status : 'n/a'})`, 'error');
            return;
        }
        try {
            await espSerial.setGatewayMode('NONE');
        } catch (e) {
            if (!silent) logToConsole(`CAN mode NONE failed: ${e.message}`, 'error');
        }
        canRunning = false;
        setCanUiState();
        if (!silent) logToConsole('CAN stopped', 'info');
    } catch (e) {
        if (!silent) logToConsole(`CAN stop error: ${e.message}`, 'error');
    }
}

