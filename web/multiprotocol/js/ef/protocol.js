/* EF state, transport handlers and controls. */

let canonEfRunning = false;
let canonEfInitialized = false;
let canonEfView = 'raw';
let canonEfFocusPollTimer = null;
let canonEfFocusPollInFlight = false;
let canonEfLensOperationInFlight = false;
let canonEfTelemetryPauseDepth = 0;
let canonEfApertureRelativeSteps = null;
let canonEfIsRequested = false;
let canonEfFocusCycleActive = false;
let canonEfFocusCycleTimer = null;
let canonEfFocusCycleNearNext = true;
let canonEfApertureCycleActive = false;
let canonEfApertureCycleTimer = null;
let canonEfApertureCycleOpenNext = true;
let canonEfScanCycleActive = false;
let canonEfScanCycleTimer = null;
let canonEfAutoConfigTimer = null;

function clearCanonEf() {
    const el = document.getElementById('canonEfRxDisplay');
    if (el) el.textContent = '';
}

function setCanonEfView(view) {
    canonEfView = view === 'lens' || view === 'scan' ? view : 'raw';
    if (canonEfView !== 'lens') stopCanonEfCycles();
    if (canonEfView !== 'scan') stopCanonEfScanCycle();
    const raw = canonEfView === 'raw';
    const lens = canonEfView === 'lens';
    document.getElementById('canonEfRawView')?.style.setProperty('display', raw ? '' : 'none');
    document.getElementById('canonEfLensView')?.style.setProperty('display', lens ? '' : 'none');
    document.getElementById('canonEfScanView')?.style.setProperty('display', canonEfView === 'scan' ? '' : 'none');
    document.getElementById('canonEfTabRaw')?.classList.toggle('active', raw);
    document.getElementById('canonEfTabLens')?.classList.toggle('active', lens);
    document.getElementById('canonEfTabScan')?.classList.toggle('active', canonEfView === 'scan');
    updateCanonEfFocusPolling();
}

function parseCanonEfScanByte(text, label) {
    const source = String(text || '').trim();
    if (!/^(?:0x[0-9a-f]{1,2}|[0-9]{1,3})$/i.test(source)) {
        throw new Error(`${label} must be a byte in decimal or 0x00-0xFF form`);
    }
    const value = source.toLowerCase().startsWith('0x')
        ? parseInt(source.slice(2), 16)
        : parseInt(source, 10);
    if (value < 0 || value > 0xFF) throw new Error(`${label} must be between 0x00 and 0xFF`);
    return value;
}

function trimCanonEfScanSync(rx) {
    const response = Array.from(rx);
    while (response[0] === 0xAA) response.shift();
    while (response.length && response[response.length - 1] === 0xAA) response.pop();
    return response;
}

function getCanonEfScanConfig() {
    const start = parseCanonEfScanByte(document.getElementById('canonEfScanStart').value, 'Start opcode');
    const end = parseCanonEfScanByte(document.getElementById('canonEfScanEnd').value, 'End opcode');
    const syncCount = parseInt(document.getElementById('canonEfScanSyncCount').value, 10);
    if (start > end) throw new Error('Start opcode must not exceed end opcode');
    if (!Number.isInteger(syncCount) || syncCount < 1 || syncCount > 32) {
        throw new Error('Sync byte count must be between 1 and 32');
    }
    return { start, end, syncCount };
}

async function buildCanonEfScanText(config) {
    const lines = [];
    for (let opcode = config.start; opcode <= config.end; opcode++) {
        const tx = new Uint8Array(config.syncCount + 1);
        tx[0] = opcode;
        tx.fill(0x0A, 1);
        const response = await espSerial.canonEFLensTransfer(tx, {
            timeoutMs: Math.max(5000, tx.length * 100)
        });
        const commandResponse = trimCanonEfScanSync(canonEfResponseBytes(response));
        const echoedOnly = commandResponse.length === 1 && commandResponse[0] === opcode;
        const value = commandResponse.length && !echoedOnly
            ? commandResponse.map(bytesToHex2).join(' ')
            : '--';
        lines.push(`0x${bytesToHex2(opcode)}${response.status === 0 ? ':' : ` !${bytesToHex2(response.status)}:`} ${value}`);
    }
    return lines.join('\n');
}

function commitCanonEfScanText(text) {
    const output = document.getElementById('canonEfScanResult');
    if (output && output.textContent !== text) output.textContent = text;
}

async function runCanonEfScanPass(cycleMode) {
    if (!espSerial || !isDeviceConnected || !canonEfRunning || canonEfLensOperationInFlight) return false;
    if (!cycleMode) pauseCanonEfTelemetry();
    canonEfLensOperationInFlight = true;
    setCanUiState();
    try {
        const config = getCanonEfScanConfig();
        commitCanonEfScanText(await buildCanonEfScanText(config));
        if (!cycleMode) {
            logToConsole(`Canon EF command scan: ${config.end - config.start + 1} opcode(s)`, 'info');
        }
        return true;
    } catch (e) {
        logToConsole(`Canon EF command scan error: ${e.message}`, 'error');
        return false;
    } finally {
        canonEfLensOperationInFlight = false;
        if (!cycleMode) resumeCanonEfTelemetry();
        setCanUiState();
    }
}

async function scanCanonEfCommands() {
    stopCanonEfScanCycle();
    await runCanonEfScanPass(false);
}

function stopCanonEfScanCycle() {
    const wasActive = canonEfScanCycleActive;
    canonEfScanCycleActive = false;
    if (canonEfScanCycleTimer !== null) clearTimeout(canonEfScanCycleTimer);
    canonEfScanCycleTimer = null;
    document.getElementById('canonEfScanCycleBtn')?.classList.remove('active');
    if (wasActive) resumeCanonEfTelemetry();
}

function toggleCanonEfScanCycle() {
    if (canonEfScanCycleActive) {
        stopCanonEfScanCycle();
        setCanUiState();
        return;
    }
    if (!espSerial || !isDeviceConnected || !canonEfRunning || canonEfLensOperationInFlight) return;
    canonEfScanCycleActive = true;
    document.getElementById('canonEfScanCycleBtn')?.classList.add('active');
    pauseCanonEfTelemetry();
    runCanonEfScanCyclePass();
    setCanUiState();
}

async function runCanonEfScanCyclePass() {
    if (!canonEfScanCycleActive) return;
    const succeeded = await runCanonEfScanPass(true);
    if (!succeeded) {
        stopCanonEfScanCycle();
        setCanUiState();
        return;
    }
    if (canonEfScanCycleActive) {
        canonEfScanCycleTimer = setTimeout(runCanonEfScanCyclePass, 0);
    }
}

function updateCanonEfFocusPolling() {
    const shouldPoll = canonEfView === 'lens' && canonEfRunning && canonEfInitialized &&
        canonEfTelemetryPauseDepth === 0;
    if (!shouldPoll) {
        if (canonEfFocusPollTimer !== null) {
            clearInterval(canonEfFocusPollTimer);
            canonEfFocusPollTimer = null;
        }
        return;
    }
    if (canonEfFocusPollTimer === null) {
        canonEfFocusPollTimer = setInterval(pollCanonEfTelemetry, 200);
        pollCanonEfTelemetry();
    }
}

function pauseCanonEfTelemetry() {
    canonEfTelemetryPauseDepth++;
    updateCanonEfFocusPolling();
}

function resumeCanonEfTelemetry() {
    if (canonEfTelemetryPauseDepth > 0) canonEfTelemetryPauseDepth--;
    updateCanonEfFocusPolling();
}

function formatCanonEfStatus(status) {
    const bitPattern = Array.from({ length: 8 }, (_, index) =>
        (status & (1 << (7 - index))) !== 0 ? 'x' : '-').join('');
    let apertureState = '';
    switch (status & 0x03) {
        case 0x00: apertureState = 'open'; break;
        case 0x01: apertureState = 'uninit'; break;
        case 0x02: apertureState = 'unknown'; break;
        case 0x03: apertureState = 'closing'; break;
        default: apertureState = 'unknown focus state';
    }
    return `${bitPattern}  (0x${bytesToHex2(status)})\n` +
        `| || |||    \n` +
        `| || | \\\\__ aperture ${apertureState}\n` +
        `| ||  \\____ moving\n` +
        `| | \\______ end stop\n` +
        `|  \\_______ accelerating\n` +
        ` \\_________ MF\n`;
}

function formatCanonEfIsStatus(status) {
    const bitPattern = Array.from({ length: 8 }, (_, index) =>
        (status & (1 << (7 - index))) !== 0 ? 'x' : '-').join('');
    return `${bitPattern}  (0x${bytesToHex2(status)})\n` +
        `    |||| \n` +
        `    ||| \\_ unknown\n` +
        `    || \\__ engaged\n` +
        `    | \\___ enabled\n` +
        `     \\____ spinning`;
}

async function pollCanonEfTelemetry() {
    if (canonEfFocusPollInFlight || canonEfLensOperationInFlight || !espSerial || !isDeviceConnected ||
        !canonEfRunning || !canonEfInitialized || canonEfView !== 'lens' || espSerial._lensPending) {
        return;
    }
    canonEfFocusPollInFlight = true;
    try {
        const focusResponse = await sendCanonEfLensTransaction(new Uint8Array([0xC0, 0x00, 0x00]));
        const focusRx = canonEfResponseBytes(focusResponse);
        if (focusRx.length >= 5) {
            setCanonEfLensField('canonEfFocusPosition', String((focusRx[1] << 8) | focusRx[2]));
        }

        const zoomResponse = await sendCanonEfLensTransaction(new Uint8Array([0xA0, 0x00]));
        const zoomRx = canonEfResponseBytes(zoomResponse);
        if (zoomRx.length >= 4) {
            setCanonEfLensField('canonEfZoom', `${(zoomRx[1] << 8) | zoomRx[2]} mm`);
        }

        const distanceResponse = await sendCanonEfLensTransaction(new Uint8Array([0xC2, 0x00, 0x00, 0x00]));
        const distanceRx = canonEfResponseBytes(distanceResponse);
        /* RX[1..4] are C2's four data bytes; the final cycle is the sync reply. */
        if (distanceRx.length >= 6) {
            const valueA = (distanceRx[1] << 8) | distanceRx[2];
            const valueB = (distanceRx[3] << 8) | distanceRx[4];
            if (valueA === 0xFFFF && valueB === 0xFFFF) {
                setCanonEfLensField('canonEfFocusDistance', 'infinity (0xFFFF 0xFFFF)');
            } else if (valueA === 0 && valueB === 0) {
                setCanonEfLensField('canonEfFocusDistance', 'unknown (0x0000 0x0000)');
            } else {
                setCanonEfLensField('canonEfFocusDistance', `${(valueA / 100.0).toFixed(1)}m - ${(valueB / 100.0).toFixed(1)}m (0x${valueA.toString(16).toUpperCase().padStart(4, '0')} 0x${valueB.toString(16).toUpperCase().padStart(4, '0')})`);
            }
        }

        const apertureResponse = await sendCanonEfLensTransaction(new Uint8Array([0xB0, 0x00, 0x00]));
        const apertureRx = canonEfResponseBytes(apertureResponse);
        if (apertureResponse.status === 0 && apertureRx.length >= 4) {
            canonEfApertureRelativeSteps = 0;
            setCanonEfLensField('canonEfAperture',
                `real ${formatCanonEfApertureCode(apertureRx[1])}, ` +
                `disp ${formatCanonEfApertureCode(apertureRx[2])}, ` +
                `min ${formatCanonEfApertureCode(apertureRx[3])}`);
        } else {
            setCanonEfLensField('canonEfAperture', '(limit codes unavailable)');
        }

        const statusResponse = await sendCanonEfLensTransaction(new Uint8Array([0x90, 0xB9, 0x00]));
        const statusRx = canonEfResponseBytes(statusResponse);
        if (statusRx.length >= 5) {
            setCanonEfLensField('canonEfStatus', formatCanonEfStatus(statusRx[2]));
        }

        const isParameter = canonEfIsRequested ? (0xB9 | 0x20) : (0xB9 & ~0x20);
        const extendedStatusResponse = await sendCanonEfLensTransaction(
            new Uint8Array([0x91, isParameter, 0x00, 0x00]));
        const extendedStatusRx = canonEfResponseBytes(extendedStatusResponse);
        if (extendedStatusRx.length >= 6) {
            const isStatus = extendedStatusRx[3];
            setCanonEfLensField('canonEfIsStatus', formatCanonEfIsStatus(isStatus));
        }
    } catch (e) {
        /* Poll failures must not interrupt manual lens control. */
    } finally {
        canonEfFocusPollInFlight = false;
        setCanUiState();
    }
}

function canonEfResponseBytes(response) {
    return response.records && response.records.length
        ? response.records.map(record => record.rx)
        : Array.from(response.data || []);
}

function setCanonEfLensField(id, value) {
    const field = document.getElementById(id);
    if (field) field.textContent = value;
}

function formatCanonEfApertureCode(raw) {
    const fNumber = Math.pow(2, (raw - 8.0) / 16.0);
    return `f/${fNumber.toFixed(2)} (0x${bytesToHex2(raw)})`;
}

function appendCanonEfSyncBytes(tx) {
    const syncTx = new Uint8Array(tx.length + 2);
    syncTx.set(tx);
    syncTx[tx.length] = 0x0A;
    syncTx[tx.length + 1] = 0x0A;
    return syncTx;
}

function isCanonEfSyncReply(value) {
    return value === 0xAA;
}

function formatTime(timeInUs) {
    if (timeInUs < 1000) {
        return `${timeInUs.toFixed(0).toString().padStart(3, ' ')}us`;
    } else if (timeInUs < 1000000) {
        return `${(timeInUs / 1000).toFixed(0).toString().padStart(3, ' ')}ms`;
    } else {
        return `${(timeInUs / 1000000).toFixed(0).toString().padStart(3, ' ')}s`;
    }
}

function formatCanonEfTransferTrace(tx, response) {
    const sent = Array.from(tx || []);
    const records = response.records || [];
    const lines = [];

    if (response.measurement) {
        if (response.malformed_measurement) {
            lines.push(`Malformed measurement payload (${response.data.length} data bytes)`);
        }
        records.forEach((record, index) => {
            let index_str = index.toString().padStart(2, '0');
            const txByte = index < sent.length ? bytesToHex2(sent[index]) : '--';
            const rxByte = bytesToHex2(record.rx);
            const stopped = response.status !== 0 && index === records.length - 1;
            if (record.ack_delay_us === 0 || record.ack_delay_us === 0xFF) {
                lines.push(`[${index_str}] TX ${txByte}, RX ${rxByte}${stopped ? '  <-- stopped' : ''}, NOT ACKNOWLEDGED `);
            } else if (record.ack_duration_us === 0xFFFF) {
                lines.push(`[${index_str}] TX ${txByte}, RX ${rxByte}${stopped ? '  <-- stopped' : ''}, ACK ${formatTime(record.ack_delay_us)}/---`);
            } else {
                lines.push(`[${index_str}] TX ${txByte}, RX ${rxByte}${stopped ? '  <-- stopped' : ''}, ACK ${formatTime(record.ack_delay_us)}/${formatTime(record.ack_duration_us)}`);
            }
        });
        if (records.length === 0 && response.status !== 0 && sent.length) {
            lines.push(`[00] TX ${bytesToHex2(sent[0])}  ATTEMPTED, measurement record missing  <-- stopped`);
        }
        if (response.status !== 0) {
            const firstNotSent = records.length || (sent.length ? 1 : 0);
            for (let index = firstNotSent; index < sent.length; index++) {
                let index_str = index.toString().padStart(2, '0');
                lines.push(`[${index_str}] TX ${bytesToHex2(sent[index])}  NOT SENT`);
            }
        }
    } else {
        const received = Array.from(response.data || []);
        received.forEach((rx, index) => {
            let index_str = index.toString().padStart(2, '0');
            const stopped = response.status !== 0 && index === received.length - 1;
            const ackText = response.ignore_ack
                ? ''
                : stopped ? 'NOT ACKNOWLEDGED' : '';
            lines.push(`[${index_str}] TX ${bytesToHex2(sent[index])}, RX ${bytesToHex2(rx)}${stopped ? '  <-- stopped' : ''} ${ackText} `);
        });
        if (response.status !== 0) {
            for (let index = received.length; index < sent.length; index++) {
                let index_str = index.toString().padStart(2, '0');
                lines.push(`[${index_str}] TX ${bytesToHex2(sent[index])}  NOT SENT`);
            }
        }
    }
    return lines;
}

function describeCanonEfTransferStatus(response) {
    if (response.status === 0) return 'OK';
    if (response.status === 0x04) {
        const record = response.records?.[response.records.length - 1];
        if (record && record.ack_delay_us > 0 && record.ack_delay_us < 0xFF) {
            return `Command failed, Lens started ACK after ${record.ack_delay_us} us but ACK did not complete`;
        }
        return 'Command failed, Lens did not acknowledge command byte';
    }
    if (response.status === 0x07) {
        return 'Command failed, Lens held ACK active too long';
    }
    const descriptions = {
        0x01: 'Invalid transfer length',
        0x02: 'Invalid configuration',
        0x03: 'Lens bus is not configured',
        0x05: 'SPI transfer failed',
        0x06: 'Firmware out of memory',
        0x7F: 'Internal firmware error'
    };
    return descriptions[response.status] || 'Unknown status';
}

async function sendCanonEfLensTransaction(commandTx, logLabel = null) {
    const tx = appendCanonEfSyncBytes(commandTx);
    if (logLabel) {
        logToConsole(`Canon EF ${logLabel} TX: ${Array.from(tx).map(bytesToHex2).join(' ')}`, 'info');
    }
    const response = await espSerial.canonEFLensTransfer(tx, {
        includeAck: document.getElementById('canonEfAckMode')?.value === 'measurement',
        timeoutMs: Math.max(5000, tx.length * 100)
    });
    const rx = canonEfResponseBytes(response);
    if (logLabel) {
        logToConsole(`Canon EF ${logLabel} RX: ${rx.map(bytesToHex2).join(' ')} (status ${bytesToHex2(response.status)})`,
            response.status === 0 ? 'info' : 'error');
    }
    if (logLabel) renderCanonEfLensResult(logLabel, tx, response);
    if (response.status !== 0) {
        throw new Error(describeCanonEfTransferStatus(response));
    }
    if (rx.length !== tx.length ||
        !isCanonEfSyncReply(rx[rx.length - 1])) {
        throw new Error(`Sync reply mismatch: expected AA at final RX byte, got ${rx.map(bytesToHex2).join(' ')}`);
    }
    return response;
}

async function readCanonEfLensName() {
    const nameBytes = [];
    for (let index = 0; index < 64; index++) {
        const opcode = index === 0 ? 0x82 : 0x83;
        const response = await runCanonEfLensCommand(index === 0 ? 'Read lens name' : 'Read next lens-name character',
            new Uint8Array([opcode, 0x00]));
        const rx = canonEfResponseBytes(response);
        const character = rx[1];
        if (response.status !== 0 || character === undefined || character < 0x20 || character > 0x7E) break;
        nameBytes.push(character);
    }
    return String.fromCharCode(...nameBytes);
}

function decodeCanonEfBcdSerial(bytes) {
    if (bytes.length !== 5) return null;
    let serial = '';
    for (const value of bytes) {
        const high = value >> 4;
        const low = value & 0x0F;
        if (high > 9 || low > 9) return null;
        serial += `${high}${low}`;
    }
    return serial;
}

function renderCanonEfLensResult(name, tx, response) {
    const output = document.getElementById('canonEfLensResult');
    if (!output) return;
    const rx = canonEfResponseBytes(response);
    const txText = Array.from(tx).map(bytesToHex2).join(' ');
    const rxText = rx.map(bytesToHex2).join(' ');
    const ascii = rx.map(value => value >= 32 && value <= 126 ? String.fromCharCode(value) : '.').join('');
    const trace = formatCanonEfTransferTrace(tx, response);
    output.textContent = `${name}\nTX ${txText}\nSTATUS ${bytesToHex2(response.status)}\nRX ${rxText} '${ascii}'\n${trace.join('\n')}`;
}

async function waitForCanonEfTransportIdle(timeoutMs = 5000) {
    const deadline = Date.now() + timeoutMs;
    while (canonEfFocusPollInFlight || (espSerial && espSerial._lensPending)) {
        if (Date.now() >= deadline) throw new Error('Canon EF transport did not become idle');
        await new Promise(resolve => setTimeout(resolve, 5));
    }
}

async function runCanonEfLensCommand(name, tx) {
    if (!espSerial || !isDeviceConnected || !canonEfRunning) {
        throw new Error('Configure Canon EF first');
    }
    pauseCanonEfTelemetry();
    try {
        await waitForCanonEfTransportIdle();
        setCanUiState();
        const response = await sendCanonEfLensTransaction(tx, name);
        return response;
    } finally {
        resumeCanonEfTelemetry();
        setCanUiState();
    }
}

async function initializeCanonEfLens() {
    if (canonEfLensOperationInFlight) return;
    canonEfLensOperationInFlight = true;
    pauseCanonEfTelemetry();
    setCanUiState();
    try {
        const reset = await espSerial.canonEFLensReset({ timeoutMs: 5000 });
        if (!reset || reset.status !== 0) throw new Error('Lens reset failed');

        await runCanonEfLensCommand('Initialize', new Uint8Array([0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A]));
        const response = await runCanonEfLensCommand('Initialize', new Uint8Array([0x00, 0x0A, 0x00]));
        const rx = canonEfResponseBytes(response);
        if (rx[2] === 0xAB || rx[rx.length - 1] === 0xAB) {
            throw new Error('Initialization returned AB sync; configure the Canon EF clock above 200 kHz and retry');
        }
        if (rx.length < 5 || rx[2] !== 0xAA || rx[rx.length - 1] !== 0xAA) {
            throw new Error(`Initialization sync mismatch: expected AA at RX[2] and final RX byte, got ${rx.map(bytesToHex2).join(' ')}`);
        }
        canonEfInitialized = true;
        updateCanonEfFocusPolling();
        logToConsole('Canon EF lens initialized', 'info');
    } catch (e) {
        canonEfInitialized = false;
        logToConsole(`Canon EF initialization error: ${e.message}`, 'error');
    } finally {
        canonEfLensOperationInFlight = false;
        resumeCanonEfTelemetry();
        setCanUiState();
    }
}

const lenses = [
    { id: "01", name: "Canon EF 50mm f/1.8" },
    { id: "02", name: "Canon EF 28mm f/2.8" },
    { id: "03", name: "Canon EF 135mm f/2.8 Soft" },
    { id: "04", name: "Canon EF 35-105mm f/3.5-4.5 or Sigma Lens" },
    { id: "04", name: "Sigma UC Zoom 35-135mm f/4-5.6" },
    { id: "05", name: "Canon EF 35-70mm f/3.5-4.5" },
    { id: "06", name: "Canon EF 28-70mm f/3.5-4.5 or Sigma or Tokina Lens" },
    { id: "06", name: "Sigma 18-50mm f/3.5-5.6 DC" },
    { id: "06", name: "Sigma 18-125mm f/3.5-5.6 DC IF ASP" },
    { id: "06", name: "Tokina AF 193-2 19-35mm f/3.5-4.5" },
    { id: "06", name: "Sigma 28-80mm f/3.5-5.6 II Macro" },
    { id: "07", name: "Canon EF 100-300mm f/5.6L" },
    { id: "08", name: "Canon EF 100-300mm f/5.6 or Sigma or Tokina Lens" },
    { id: "08", name: "Sigma 70-300mm f/4-5.6 [APO] DG Macro" },
    { id: "08", name: "Tokina AT-X 242 AF 24-200mm f/3.5-5.6" },
    { id: "09", name: "Canon EF 70-210mm f/4" },
    { id: "09", name: "Sigma 55-200mm f/4-5.6 DC" },
    { id: "0A", name: "Canon EF 50mm f/2.5 Macro or Sigma Lens" },
    { id: "0A", name: "Sigma 50mm f/2.8 EX" },
    { id: "0A", name: "Sigma 28mm f/1.8" },
    { id: "0A", name: "Sigma 105mm f/2.8 Macro EX" },
    { id: "0A", name: "Sigma 70mm f/2.8 EX DG Macro EF" },
    { id: "0B", name: "Canon EF 35mm f/2" },
    { id: "0D", name: "Canon EF 15mm f/2.8 Fisheye" },
    { id: "0E", name: "Canon EF 50-200mm f/3.5-4.5L" },
    { id: "0F", name: "Canon EF 50-200mm f/3.5-4.5" },
    { id: "10", name: "Canon EF 35-135mm f/3.5-4.5" },
    { id: "11", name: "Canon EF 35-70mm f/3.5-4.5A" },
    { id: "12", name: "Canon EF 28-70mm f/3.5-4.5" },
    { id: "14", name: "Canon EF 100-200mm f/4.5A" },
    { id: "15", name: "Canon EF 80-200mm f/2.8L" },
    { id: "16", name: "Canon EF 20-35mm f/2.8L or Tokina Lens" },
    { id: "16", name: "Tokina AT-X 280 AF Pro 28-80mm f/2.8 Aspherical" },
    { id: "17", name: "Canon EF 35-105mm f/3.5-4.5" },
    { id: "18", name: "Canon EF 35-80mm f/4-5.6 Power Zoom" },
    { id: "19", name: "Canon EF 35-80mm f/4-5.6 Power Zoom" },
    { id: "1A", name: "Canon EF 100mm f/2.8 Macro or Other Lens" },
    { id: "1A", name: "Cosina 100mm f/3.5 Macro AF" },
    { id: "1A", name: "Tamron SP AF 90mm f/2.8 Di Macro" },
    { id: "1A", name: "Tamron SP AF 180mm f/3.5 Di Macro" },
    { id: "1A", name: "Zeiss Planar T* 50mm f/1.4" },
    { id: "1B", name: "Canon EF 35-80mm f/4-5.6" },
    { id: "1C", name: "Canon EF 80-200mm f/4.5-5.6 or Tamron Lens" },
    { id: "1C", name: "Tamron SP AF 28-105mm f/2.8 LD Aspherical IF" },
    { id: "1C", name: "Tamron SP AF 28-75mm f/2.8 XR Di LD Aspherical [IF] Macro" },
    { id: "1C", name: "Tamron AF 70-300mm f/4-5.6 Di LD 1:2 Macro" },
    { id: "1C", name: "Tamron AF Aspherical 28-200mm f/3.8-5.6" },
    { id: "1D", name: "Canon EF 50mm f/1.8 II" },
    { id: "1E", name: "Canon EF 35-105mm f/4.5-5.6" },
    { id: "1F", name: "Canon EF 75-300mm f/4-5.6 or Tamron Lens" },
    { id: "1F", name: "Tamron SP AF 300mm f/2.8 LD IF" },
    { id: "20", name: "Canon EF 24mm f/2.8 or Sigma Lens" },
    { id: "20", name: "Sigma 15mm f/2.8 EX Fisheye" },
    { id: "21", name: "Voigtlander or Carl Zeiss Lens" },
    { id: "21", name: "Voigtlander Ultron 40mm f/2 SLII Aspherical" },
    { id: "21", name: "Voigtlander Color Skopar 20mm f/3.5 SLII Aspherical" },
    { id: "21", name: "Voigtlander APO-Lanthar 90mm f/3.5 SLII Close Focus" },
    { id: "21", name: "Zeiss Distagon T* 15mm f/2.8 ZE" },
    { id: "21", name: "Zeiss Distagon T* 18mm f/3.5 ZE" },
    { id: "21", name: "Zeiss Distagon T* 21mm f/2.8 ZE" },
    { id: "21", name: "Zeiss Distagon T* 25mm f/2 ZE" },
    { id: "21", name: "Zeiss Distagon T* 28mm f/2 ZE" },
    { id: "21", name: "Zeiss Distagon T* 35mm f/2 ZE" },
    { id: "10", name: "21 Zeiss Distagon T* 35mm f/1.4 ZE" },
    { id: "11", name: "21 Zeiss Planar T* 50mm f/1.4 ZE" },
    { id: "12", name: "21 Zeiss Makro-Planar T* 50mm f/2 ZE" },
    { id: "13", name: "21 Zeiss Makro-Planar T* 100mm f/2 ZE" },
    { id: "14", name: "21 Zeiss Apo-Sonnar T* 135mm f/2 ZE" },
    { id: "23", name: "Canon EF 35-80mm f/4-5.6" },
    { id: "24", name: "Canon EF 38-76mm f/4.5-5.6" },
    { id: "25", name: "Canon EF 35-80mm f/4-5.6 or Tamron Lens" },
    { id: "25", name: "Tamron 70-200mm f/2.8 Di LD IF Macro" },
    { id: "25", name: "Tamron AF 28-300mm f/3.5-6.3 XR Di VC LD Aspherical [IF] Macro Model A20" },
    { id: "25", name: "Tamron SP AF 17-50mm f/2.8 XR Di II VC LD Aspherical [IF]" },
    { id: "25", name: "Tamron AF 18-270mm f/3.5-6.3 Di II VC LD Aspherical [IF] Macro" },
    { id: "26", name: "Canon EF 80-200mm f/4.5-5.6" },
    { id: "27", name: "Canon EF 75-300mm f/4-5.6" },
    { id: "28", name: "Canon EF 28-80mm f/3.5-5.6" },
    { id: "29", name: "Canon EF 28-90mm f/4-5.6" },
    { id: "2A", name: "Canon EF 28-200mm f/3.5-5.6 or Tamron Lens" },
    { id: "2B", name: "Tamron AF 28-300mm f/3.5-6.3 XR Di VC LD Aspherical [IF] Macro Model A20" },
    { id: "2B", name: "Canon EF 28-105mm f/4-5.6" },
    { id: "2C", name: "Canon EF 90-300mm f/4.5-5.6" },
    { id: "2D", name: "Canon EF-S 18-55mm f/3.5-5.6 [II]" },
    { id: "2E", name: "Canon EF 28-90mm f/4-5.6" },
    { id: "2F", name: "Zeiss Milvus 35mm f/2 or 50mm f/2" },
    { id: "2F", name: "Zeiss Milvus 50mm f/2 Makro" },
    { id: "30", name: "Canon EF-S 18-55mm f/3.5-5.6 IS" },
    { id: "31", name: "Canon EF-S 55-250mm f/4-5.6 IS" },
    { id: "32", name: "Canon EF-S 18-200mm f/3.5-5.6 IS" },
    { id: "33", name: "Canon EF-S 18-135mm f/3.5-5.6 IS" },
    { id: "34", name: "Canon EF-S 18-55mm f/3.5-5.6 IS II" },
    { id: "35", name: "Canon EF-S 18-55mm f/3.5-5.6 III" },
    { id: "36", name: "Canon EF-S 55-250mm f/4-5.6 IS II" },
    { id: "5E", name: "Canon TS-E 17mm f/4L" },
    { id: "5F", name: "Canon TS-E 24.0mm f/3.5 L II" },
    { id: "7C", name: "Canon MP-E 65mm f/2.8 1-5x Macro Photo" },
    { id: "7D", name: "Canon TS-E 24mm f/3.5L" },
    { id: "7E", name: "Canon TS-E 45mm f/2.8" },
    { id: "7F", name: "Canon TS-E 90mm f/2.8" },
    { id: "81", name: "Canon EF 300mm f/2.8L" },
    { id: "82", name: "Canon EF 50mm f/1.0L" },
    { id: "83", name: "Canon EF 28-80mm f/2.8-4L or Sigma Lens" },
    { id: "83", name: "Sigma 8mm f/3.5 EX DG Circular Fisheye" },
    { id: "83", name: "Sigma 17-35mm f/2.8-4 EX DG Aspherical HSM" },
    { id: "83", name: "Sigma 17-70mm f/2.8-4.5 DC Macro" },
    { id: "83", name: "Sigma APO 50-150mm f/2.8 [II] EX DC HSM" },
    { id: "83", name: "Sigma APO 120-300mm f/2.8 EX DG HSM" },
    { id: "83", name: "Sigma 4.5mm f/2.8 EX DC HSM Circular Fisheye" },
    { id: "83", name: "Sigma 70-200mm f/2.8 APO EX HSM" },
    { id: "84", name: "Canon EF 1200mm f/5.6L" },
    { id: "86", name: "Canon EF 600mm f/4L IS" },
    { id: "87", name: "Canon EF 200mm f/1.8L" },
    { id: "88", name: "Canon EF 300mm f/2.8L" },
    { id: "89", name: "Canon EF 85mm f/1.2L or Sigma or Tamron Lens" },
    { id: "89", name: "Sigma 18-50mm f/2.8-4.5 DC OS HSM" },
    { id: "89", name: "Sigma 50-200mm f/4-5.6 DC OS HSM" },
    { id: "89", name: "Sigma 18-250mm f/3.5-6.3 DC OS HSM" },
    { id: "89", name: "Sigma 24-70mm f/2.8 IF EX DG HSM" },
    { id: "89", name: "Sigma 18-125mm f/3.8-5.6 DC OS HSM" },
    { id: "89", name: "Sigma 17-70mm f/2.8-4 DC Macro OS HSM | C" },
    { id: "89", name: "Sigma 17-50mm f/2.8 OS HSM" },
    { id: "89", name: "Sigma 18-200mm f/3.5-6.3 DC OS HSM [II]" },
    { id: "89", name: "Tamron AF 18-270mm f/3.5-6.3 Di II VC PZD" },
    { id: "10", name: "89 Sigma 8-16mm f/4.5-5.6 DC HSM" },
    { id: "11", name: "89 Tamron SP 17-50mm f/2.8 XR Di II VC" },
    { id: "12", name: "89 Tamron SP 60mm f/2 Macro Di II" },
    { id: "13", name: "89 Sigma 10-20mm f/3.5 EX DC HSM" },
    { id: "14", name: "89 Tamron SP 24-70mm f/2.8 Di VC USD" },
    { id: "15", name: "89 Sigma 18-35mm f/1.8 DC HSM" },
    { id: "16", name: "89 Sigma 12-24mm f/4.5-5.6 DG HSM II" },
    { id: "8A", name: "Canon EF 28-80mm f/2.8-4L" },
    { id: "8B", name: "Canon EF 400mm f/2.8L" },
    { id: "8C", name: "Canon EF 500mm f/4.5L" },
    { id: "8D", name: "Canon EF 500mm f/4.5L" },
    { id: "8E", name: "Canon EF 300mm f/2.8L IS" },
    { id: "8F", name: "Canon EF 500mm f/4L IS or Sigma Lens" },
    { id: "8F", name: "Sigma 17-70mm f/2.8-4 DC Macro OS HSM" },
    { id: "90", name: "Canon EF 35-135mm f/4-5.6 USM" },
    { id: "91", name: "Canon EF 100-300mm f/4.5-5.6 USM" },
    { id: "92", name: "Canon EF 70-210mm f/3.5-4.5 USM" },
    { id: "93", name: "Canon EF 35-135mm f/4-5.6 USM" },
    { id: "94", name: "Canon EF 28-80mm f/3.5-5.6 USM" },
    { id: "95", name: "Canon EF 100mm f/2 USM" },
    { id: "96", name: "Canon EF 14mm f/2.8L or Sigma Lens" },
    { id: "96", name: "Sigma 20mm EX f/1.8" },
    { id: "96", name: "Sigma 30mm f/1.4 DC HSM" },
    { id: "96", name: "Sigma 24mm f/1.8 DG Macro EX" },
    { id: "96", name: "Sigma 28mm f/1.8 DG Macro EX" },
    { id: "97", name: "Canon EF 200mm f/2.8L" },
    { id: "98", name: "Canon EF 300mm f/4L IS or Sigma Lens" },
    { id: "98", name: "Sigma 12-24mm f/4.5-5.6 EX DG ASPHERICAL HSM" },
    { id: "98", name: "Sigma 14mm f/2.8 EX Aspherical HSM" },
    { id: "98", name: "Sigma 10-20mm f/4-5.6" },
    { id: "98", name: "Sigma 100-300mm f/4" },
    { id: "99", name: "Canon EF 35-350mm f/3.5-5.6L or Sigma or Tamron Lens" },
    { id: "99", name: "Sigma 50-500mm f/4-6.3 APO HSM EX" },
    { id: "99", name: "Tamron AF 28-300mm f/3.5-6.3 XR LD Aspherical [IF] Macro" },
    { id: "99", name: "Tamron AF 18-200mm f/3.5-6.3 XR Di II LD Aspherical [IF] Macro Model A14" },
    { id: "99", name: "Tamron 18-250mm f/3.5-6.3 Di II LD Aspherical [IF] Macro" },
    { id: "9A", name: "Canon EF 20mm f/2.8 USM or Zeiss Lens" },
    { id: "9A", name: "Zeiss Milvus 21mm f/2.8" },
    { id: "9B", name: "Canon EF 85mm f/1.8 USM" },
    { id: "9C", name: "Canon EF 28-105mm f/3.5-4.5 USM or Tamron Lens" },
    { id: "9C", name: "Tamron SP 70-300mm f/4.0-5.6 Di VC USD" },
    { id: "9C", name: "Tamron SP AF 28-105mm f/2.8 LD Aspherical IF" },
    { id: "A0", name: "Canon EF 20-35mm f/3.5-4.5 USM or Tamron or Tokina Lens" },
    { id: "A0", name: "Tamron AF 19-35mm f/3.5-4.5" },
    { id: "A0", name: "Tokina AT-X 124 AF Pro DX 12-24mm f/4" },
    { id: "A0", name: "Tokina AT-X 107 AF DX 10-17mm f/3.5-4.5 Fisheye" },
    { id: "A0", name: "Tokina AT-X 116 AF Pro DX 11-16mm f/2.8" },
    { id: "A0", name: "Tokina AT-X 11-20 F2.8 PRO DX Aspherical 11-20mm f/2.8" },
    { id: "A1", name: "Canon EF 28-70mm f/2.8L or Sigma or Tamron Lens" },
    { id: "A1", name: "Sigma 24-70mm f/2.8 EX" },
    { id: "A1", name: "Sigma 28-70mm f/2.8 EX" },
    { id: "A1", name: "Sigma 24-60mm f/2.8 EX DG" },
    { id: "A1", name: "Tamron AF 17-50mm f/2.8 Di-II LD Aspherical" },
    { id: "A1", name: "Tamron 90mm f/2.8" },
    { id: "A1", name: "Tamron SP AF 17-35mm f/2.8-4 Di LD Aspherical IF" },
    { id: "A1", name: "Tamron SP AF 28-75mm f/2.8 XR Di LD Aspherical [IF] Macro" },
    { id: "A2", name: "Canon EF 200mm f/2.8L" },
    { id: "A3", name: "Canon EF 300mm f/4L" },
    { id: "A4", name: "Canon EF 400mm f/5.6L" },
    { id: "A5", name: "Canon EF 70-200mm f/2.8 L" },
    { id: "A6", name: "Canon EF 70-200mm f/2.8 L + 1.4x" },
    { id: "A7", name: "Canon EF 70-200mm f/2.8 L + 2x" },
    { id: "A8", name: "Canon EF 28mm f/1.8 USM or Sigma Lens" },
    { id: "A8", name: "Sigma 50-100mm f/1.8 DC HSM | A" },
    { id: "A9", name: "Canon EF 17-35mm f/2.8L or Sigma Lens" },
    { id: "A9", name: "Sigma 18-200mm f/3.5-6.3 DC OS" },
    { id: "A9", name: "Sigma 15-30mm f/3.5-4.5 EX DG Aspherical" },
    { id: "A9", name: "Sigma 18-50mm f/2.8 Macro" },
    { id: "A9", name: "Sigma 50mm f/1.4 EX DG HSM" },
    { id: "A9", name: "Sigma 85mm f/1.4 EX DG HSM" },
    { id: "A9", name: "Sigma 30mm f/1.4 EX DC HSM" },
    { id: "A9", name: "Sigma 35mm f/1.4 DG HSM" },
    { id: "AA", name: "Canon EF 200mm f/2.8L II" },
    { id: "AB", name: "Canon EF 300mm f/4L" },
    { id: "AC", name: "Canon EF 400mm f/5.6L or Sigma Lens" },
    { id: "AC", name: "Sigma 150-600mm f/5-6.3 DG OS HSM | S" },
    { id: "AD", name: "Canon EF 180mm Macro f/3.5L or Sigma Lens" },
    { id: "AD", name: "Sigma 180mm EX HSM Macro f/3.5" },
    { id: "AD", name: "Sigma APO Macro 150mm f/2.8 EX DG HSM" },
    { id: "AE", name: "Canon EF 135mm f/2L or Other Lens" },
    { id: "AE", name: "Sigma 70-200mm f/2.8 EX DG APO OS HSM" },
    { id: "AE", name: "Sigma 50-500mm f/4.5-6.3 APO DG OS HSM" },
    { id: "AE", name: "Sigma 150-500mm f/5-6.3 APO DG OS HSM" },
    { id: "AE", name: "Zeiss Milvus 100mm f/2 Makro" },
    { id: "AF", name: "Canon EF 400mm f/2.8L" },
    { id: "B0", name: "Canon EF 24-85mm f/3.5-4.5 USM" },
    { id: "B1", name: "Canon EF 300mm f/4L IS" },
    { id: "B2", name: "Canon EF 28-135mm f/3.5-5.6 IS" },
    { id: "B3", name: "Canon EF 24mm f/1.4L" },
    { id: "B4", name: "Canon EF 35mm f/1.4L or Other Lens" },
    { id: "B4", name: "Sigma 50mm f/1.4 DG HSM | A" },
    { id: "B4", name: "Sigma 24mm f/1.4 DG HSM | A" },
    { id: "B4", name: "Zeiss Milvus 50mm f/1.4" },
    { id: "B4", name: "Zeiss Milvus 85mm f/1.4" },
    { id: "B4", name: "Zeiss Otus 28mm f/1.4 ZE" },
    { id: "B5", name: "Canon EF 100-400mm f/4.5-5.6L IS + 1.4x or Sigma Lens" },
    { id: "B5", name: "Sigma 150-600mm f/5-6.3 DG OS HSM | S + 1.4x" },
    { id: "B6", name: "Canon EF 100-400mm f/4.5-5.6L IS + 2x or Sigma Lens" },
    { id: "B6", name: "Sigma 150-600mm f/5-6.3 DG OS HSM | S + 2x" },
    { id: "B7", name: "Canon EF 100-400mm f/4.5-5.6L IS or Sigma Lens" },
    { id: "B7", name: "Sigma 150mm f/2.8 EX DG OS HSM APO Macro" },
    { id: "B7", name: "Sigma 105mm f/2.8 EX DG OS HSM Macro" },
    { id: "B7", name: "Sigma 180mm f/2.8 EX DG OS HSM APO Macro" },
    { id: "B7", name: "Sigma 150-600mm f/5-6.3 DG OS HSM | C" },
    { id: "B7", name: "Sigma 150-600mm f/5-6.3 DG OS HSM | S" },
    { id: "B8", name: "Canon EF 400mm f/2.8L + 2x" },
    { id: "B9", name: "Canon EF 600mm f/4L IS" },
    { id: "BA", name: "Canon EF 70-200mm f/4L" },
    { id: "BB", name: "Canon EF 70-200mm f/4L + 1.4x" },
    { id: "BC", name: "Canon EF 70-200mm f/4L + 2x" },
    { id: "BD", name: "Canon EF 70-200mm f/4L + 2.8x" },
    { id: "BE", name: "Canon EF 100mm f/2.8 Macro USM" },
    { id: "BF", name: "Canon EF 400mm f/4 DO IS" },
    { id: "C1", name: "Canon EF 35-80mm f/4-5.6 USM" },
    { id: "C2", name: "Canon EF 80-200mm f/4.5-5.6 USM" },
    { id: "C3", name: "Canon EF 35-105mm f/4.5-5.6 USM" },
    { id: "C4", name: "Canon EF 75-300mm f/4-5.6 USM" },
    { id: "C5", name: "Canon EF 75-300mm f/4-5.6 IS USM" },
    { id: "C6", name: "Canon EF 50mm f/1.4 USM or Zeiss Lens" },
    { id: "C6", name: "Zeiss Otus 55mm f/1.4 ZE" },
    { id: "C6", name: "Zeiss Otus 85mm f/1.4 ZE" },
    { id: "C7", name: "Canon EF 28-80mm f/3.5-5.6 USM" },
    { id: "C8", name: "Canon EF 75-300mm f/4-5.6 USM" },
    { id: "C9", name: "Canon EF 28-80mm f/3.5-5.6 USM" },
    { id: "CA", name: "Canon EF 28-80mm f/3.5-5.6 USM IV" },
    { id: "D0", name: "Canon EF 22-55mm f/4-5.6 USM" },
    { id: "D1", name: "Canon EF 55-200mm f/4.5-5.6" },
    { id: "D2", name: "Canon EF 28-90mm f/4-5.6 USM" },
    { id: "D3", name: "Canon EF 28-200mm f/3.5-5.6 USM" },
    { id: "D4", name: "Canon EF 28-105mm f/4-5.6 USM" },
    { id: "D5", name: "Canon EF 90-300mm f/4.5-5.6 USM or Tamron Lens" },
    { id: "D5", name: "Tamron SP 150-600mm f/5-6.3 Di VC USD" },
    { id: "D5", name: "Tamron 16-300mm f/3.5-6.3 Di II VC PZD Macro" },
    { id: "D5", name: "Tamron SP 35mm f/1.8 Di VC USD" },
    { id: "D5", name: "Tamron SP 45mm f/1.8 Di VC USD" },
    { id: "D6", name: "Canon EF-S 18-55mm f/3.5-5.6 USM" },
    { id: "D7", name: "Canon EF 55-200mm f/4.5-5.6 II USM" },
    { id: "D9", name: "Tamron AF 18-270mm f/3.5-6.3 Di II VC PZD" },
    { id: "E0", name: "Canon EF 70-200mm f/2.8L IS" },
    { id: "E1", name: "Canon EF 70-200mm f/2.8L IS + 1.4x" },
    { id: "E2", name: "Canon EF 70-200mm f/2.8L IS + 2x" },
    { id: "E3", name: "Canon EF 70-200mm f/2.8L IS + 2.8x" },
    { id: "E4", name: "Canon EF 28-105mm f/3.5-4.5 USM" },
    { id: "E5", name: "Canon EF 16-35mm f/2.8L" },
    { id: "E6", name: "Canon EF 24-70mm f/2.8L" },
    { id: "E7", name: "Canon EF 17-40mm f/4L" },
    { id: "E8", name: "Canon EF 70-300mm f/4.5-5.6 DO IS USM" },
    { id: "E9", name: "Canon EF 28-300mm f/3.5-5.6L IS" },
    { id: "EA", name: "Canon EF-S 17-85mm f/4-5.6 IS USM or Tokina Lens" },
    { id: "EA", name: "Tokina AT-X 12-28 PRO DX 12-28mm f/4" },
    { id: "EB", name: "Canon EF-S 10-22mm f/3.5-4.5 USM" },
    { id: "EC", name: "Canon EF-S 60mm f/2.8 Macro USM" },
    { id: "ED", name: "Canon EF 24-105mm f/4L IS" },
    { id: "EE", name: "Canon EF 70-300mm f/4-5.6 IS USM" },
    { id: "EF", name: "Canon EF 85mm f/1.2L II" },
    { id: "F0", name: "Canon EF-S 17-55mm f/2.8 IS USM" },
    { id: "F1", name: "Canon EF 50mm f/1.2L" },
    { id: "F2", name: "Canon EF 70-200mm f/4L IS" },
    { id: "F3", name: "Canon EF 70-200mm f/4L IS + 1.4x" },
    { id: "F4", name: "Canon EF 70-200mm f/4L IS + 2x" },
    { id: "F5", name: "Canon EF 70-200mm f/4L IS + 2.8x" },
    { id: "F6", name: "Canon EF 16-35mm f/2.8L II" },
    { id: "F7", name: "Canon EF 14mm f/2.8L II USM" },
    { id: "F8", name: "Canon EF 200mm f/2L IS or Sigma Lens" },
    { id: "F8", name: "Sigma 24-35mm f/2 DG HSM | A" },
    { id: "F9", name: "Canon EF 800mm f/5.6L IS" },
    { id: "FA", name: "Canon EF 24mm f/1.4L II or Sigma Lens" },
    { id: "FA", name: "Sigma 20mm f/1.4 DG HSM | A" },
    { id: "FB", name: "Canon EF 70-200mm f/2.8L IS II USM" },
    { id: "FC", name: "Canon EF 70-200mm f/2.8L IS II USM + 1.4x" },
    { id: "FD", name: "Canon EF 70-200mm f/2.8L IS II USM + 2x" },
    { id: "FE", name: "Canon EF 100mm f/2.8L Macro IS USM" },
    { id: "FF", name: "Sigma 24-105mm f/4 DG OS HSM | A or Other Sigma Lens" },
    { id: "FF", name: "Sigma 180mm f/2.8 EX DG OS HSM APO Macro" },
    { id: "01E8", name: "Canon EF-S 15-85mm f/3.5-5.6 IS USM" },
    { id: "01E9", name: "Canon EF 70-300mm f/4-5.6L IS USM" },
    { id: "01EA", name: "Canon EF 8-15mm f/4L Fisheye USM" },
    { id: "01EB", name: "Canon EF 300mm f/2.8L IS II USM" },
    { id: "01EC", name: "Canon EF 400mm f/2.8L IS II USM" },
    { id: "01ED", name: "Canon EF 500mm f/4L IS II USM or EF 24-105mm f4L IS USM" },
    { id: "01ED", name: "Canon EF 24-105mm f/4L IS USM" },
    { id: "01EE", name: "Canon EF 600mm f/4.0L IS II USM" },
    { id: "01EF", name: "Canon EF 24-70mm f/2.8L II USM" },
    { id: "01F0", name: "Canon EF 200-400mm f/4L IS USM" },
    { id: "01F3", name: "Canon EF 200-400mm f/4L IS USM + 1.4x" },
    { id: "01F6", name: "Canon EF 28mm f/2.8 IS USM" },
    { id: "01F7", name: "Canon EF 24mm f/2.8 IS USM" },
    { id: "01F8", name: "Canon EF 24-70mm f/4L IS USM" },
    { id: "01F9", name: "Canon EF 35mm f/2 IS USM" },
    { id: "01FA", name: "Canon EF 400mm f/4 DO IS II USM" },
    { id: "01FB", name: "Canon EF 16-35mm f/4L IS USM" },
    { id: "01FC", name: "Canon EF 11-24mm f/4L USM" },
    { id: "02EB", name: "Canon EF 100-400mm f/4.5-5.6L IS II USM" },
    { id: "02EC", name: "Canon EF 100-400mm f/4.5-5.6L IS II USM + 1.4x" },
    { id: "02EE", name: "Canon EF 35mm f/1.4L II USM" },
    { id: "102E", name: "Canon EF-S 18-135mm f/3.5-5.6 IS STM" },
    { id: "102F", name: "Canon EF-M 18-55mm f/3.5-5.6 IS STM or Tamron Lens" },
    { id: "102F", name: "Tamron 18-200mm F/3.5-6.3 Di III VC" },
    { id: "1030", name: "Canon EF 40mm f/2.8 STM" },
    { id: "1031", name: "Canon EF-M 22mm f/2 STM" },
    { id: "1032", name: "Canon EF-S 18-55mm f/3.5-5.6 IS STM" },
    { id: "1033", name: "Canon EF-M 11-22mm f/4-5.6 IS STM" },
    { id: "1034", name: "Canon EF-S 55-250mm f/4-5.6 IS STM" },
    { id: "1035", name: "Canon EF-M 55-200mm f/4.5-6.3 IS STM" },
    { id: "1036", name: "Canon EF-S 10-18mm f/4.5-5.6 IS STM" },
    { id: "1038", name: "Canon EF 24-105mm f/3.5-5.6 IS STM" },
    { id: "1039", name: "Canon EF-M 15-45mm f/3.5-6.3 IS STM" },
    { id: "103A", name: "Canon EF-S 24mm f/2.8 STM" },
    { id: "103B", name: "Canon EF-M 28mm f/3.5 Macro IS STM" },
    { id: "103C", name: "Canon EF 50mm f/1.8 STM" },
    { id: "9030", name: "Canon EF-S 18-135mm f/3.5-5.6 IS USM" }];

async function readCanonEfLensInfo() {
    if (canonEfLensOperationInFlight) return;
    canonEfLensOperationInFlight = true;
    pauseCanonEfTelemetry();
    setCanUiState();
    try {
        const response = await runCanonEfLensCommand('Get lens basic info',
            new Uint8Array([0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]));
        const rx = canonEfResponseBytes(response);
        if (response.status !== 0 || rx.length < 8) throw new Error('Lens basic-info transfer failed');
        const lensType = rx[1];
        /* This capture layout matches the working STM implementation offsets. */
        const minFocal = rx[4];
        const maxFocal = rx[6];
        const focalLength = minFocal === maxFocal ? `${minFocal} mm` : `${minFocal}-${maxFocal} mm`;
        setCanonEfLensField('canonEfLensType', `Type ID ${lensType}`);
        setCanonEfLensField('canonEfLensFocalLength', minFocal > 0 && maxFocal >= minFocal ? focalLength : '(not decoded)');
        setCanonEfLensField('canonEfLensName', 'Reading...');
        const name = await readCanonEfLensName();
        setCanonEfLensField('canonEfLensName', name || '(not available)');
        setCanonEfLensField('canonEfLensSerial', 'Reading...');
        const serialResponse = await runCanonEfLensCommand('Read lens serial number', new Uint8Array([0x85, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A]));
        const serialRx = canonEfResponseBytes(serialResponse);
        const serial = serialResponse.status === 0 && serialRx.length >= 7 ? decodeCanonEfBcdSerial(serialRx.slice(1, 6)) : null;
        setCanonEfLensField('canonEfLensSerial', serial || '(not available)');
        const apertureResponse = await runCanonEfLensCommand('Get aperture limits', new Uint8Array([0xB0, 0x00, 0x00]));
        const apertureRx = canonEfResponseBytes(apertureResponse);
        if (apertureResponse.status === 0 && apertureRx.length >= 4) {
            canonEfApertureRelativeSteps = 0;
            setCanonEfLensField('canonEfAperture',
                `real ${formatCanonEfApertureCode(apertureRx[1])}  ` +
                `disp ${formatCanonEfApertureCode(apertureRx[2])} ` +
                `min ${formatCanonEfApertureCode(apertureRx[3])} `);
        } else {
            setCanonEfLensField('canonEfAperture', '(limit codes unavailable)');
        }
        const tables = await readAllCanonEfTables();
        const tablePanel = document.getElementById('canonEfLensTables');
        if (tablePanel) tablePanel.textContent = formatCanonEfTablesCompact(tables);
    } catch (e) {
        setCanonEfLensField('canonEfLensName', '(read failed)');
        setCanonEfLensField('canonEfLensSerial', '(read failed)');
        const tablePanel = document.getElementById('canonEfLensTables');
        if (tablePanel) tablePanel.textContent = 'Parameter tables: (read failed)';
        logToConsole(`Canon EF lens info error: ${e.message}`, 'error');
    } finally {
        canonEfLensOperationInFlight = false;
        resumeCanonEfTelemetry();
        setCanUiState();
    }
}

async function focusCanonEfNear() {
    try { await runCanonEfLensCommand('Focus near', new Uint8Array([0x06, 0x00, 0x00, 0x00])); }
    catch (e) { logToConsole(`Canon EF focus-near error: ${e.message}`, 'error'); }
}

async function focusCanonEfInfinity() {
    try { await runCanonEfLensCommand('Focus infinity', new Uint8Array([0x05, 0x00, 0x00, 0x00])); }
    catch (e) { logToConsole(`Canon EF focus-infinity error: ${e.message}`, 'error'); }
}

async function setCanonEfIsRequested(enabled) {
    try {
        const parameter = enabled ? (0xB9 | 0x20) : (0xB9 & ~0x20);
        await runCanonEfLensCommand(enabled ? 'Request IS' : 'Stop IS',
            new Uint8Array([0x91, parameter, 0x00, 0x00]));
        canonEfIsRequested = enabled;
        logToConsole(`Canon EF IS request ${enabled ? 'enabled' : 'disabled'}`, 'info');
    } catch (e) {
        logToConsole(`Canon EF IS request error: ${e.message}`, 'error');
    }
}

function stopCanonEfCycles() {
    canonEfFocusCycleActive = false;
    canonEfApertureCycleActive = false;
    if (canonEfFocusCycleTimer !== null) clearTimeout(canonEfFocusCycleTimer);
    if (canonEfApertureCycleTimer !== null) clearTimeout(canonEfApertureCycleTimer);
    canonEfFocusCycleTimer = null;
    canonEfApertureCycleTimer = null;
    document.getElementById('canonEfFocusCycleBtn')?.classList.remove('active');
    document.getElementById('canonEfApertureCycleBtn')?.classList.remove('active');
}

function toggleCanonEfFocusCycle() {
    canonEfFocusCycleActive = !canonEfFocusCycleActive;
    const button = document.getElementById('canonEfFocusCycleBtn');
    button?.classList.toggle('active', canonEfFocusCycleActive);
    if (!canonEfFocusCycleActive) {
        if (canonEfFocusCycleTimer !== null) clearTimeout(canonEfFocusCycleTimer);
        canonEfFocusCycleTimer = null;
        return;
    }
    canonEfFocusCycleNearNext = true;
    runCanonEfFocusCycleStep();
}

async function runCanonEfFocusCycleStep() {
    if (!canonEfFocusCycleActive) return;
    const near = canonEfFocusCycleNearNext;
    canonEfFocusCycleNearNext = !canonEfFocusCycleNearNext;
    try {
        await runCanonEfLensCommand(near ? 'Focus cycle: near' : 'Focus cycle: infinity',
            new Uint8Array([near ? 0x06 : 0x05, 0x00, 0x00, 0x00]));
    } catch (e) {
        logToConsole(`Canon EF focus cycle error: ${e.message}`, 'error');
    } finally {
        if (canonEfFocusCycleActive) {
            canonEfFocusCycleTimer = setTimeout(runCanonEfFocusCycleStep, 1000);
        }
    }
}

function toggleCanonEfApertureCycle() {
    canonEfApertureCycleActive = !canonEfApertureCycleActive;
    const button = document.getElementById('canonEfApertureCycleBtn');
    button?.classList.toggle('active', canonEfApertureCycleActive);
    if (!canonEfApertureCycleActive) {
        if (canonEfApertureCycleTimer !== null) clearTimeout(canonEfApertureCycleTimer);
        canonEfApertureCycleTimer = null;
        return;
    }
    canonEfApertureCycleOpenNext = true;
    runCanonEfApertureCycleStep();
}

async function runCanonEfApertureCycleStep() {
    if (!canonEfApertureCycleActive) return;
    const open = canonEfApertureCycleOpenNext;
    canonEfApertureCycleOpenNext = !canonEfApertureCycleOpenNext;
    try {
        await moveCanonEfAperture(open ? -128 : 127);
    } finally {
        if (canonEfApertureCycleActive) {
            canonEfApertureCycleTimer = setTimeout(runCanonEfApertureCycleStep, 1000);
        }
    }
}

function canonEfJumpSize(id) {
    const value = parseInt(document.getElementById(id)?.value, 10);
    return value === 1 || value === 10 || value === 100 ? value : 1;
}

async function moveCanonEfFocus(delta) {
    if (!Number.isInteger(delta) || delta < -32768 || delta > 32767) {
        logToConsole('Focus delta must be between -32768 and 32767', 'error');
        return;
    }
    const value = delta & 0xFFFF;
    try { await runCanonEfLensCommand('Focus delta', new Uint8Array([0x44, value >> 8, value & 0xFF])); }
    catch (e) { logToConsole(`Canon EF focus error: ${e.message}`, 'error'); }
}

async function moveCanonEfAperture(delta) {
    if (!Number.isInteger(delta) || delta < -128 || delta > 127) {
        logToConsole('Aperture delta must be between -128 and 127', 'error');
        return;
    }
    try {
        const response = await runCanonEfLensCommand('Aperture delta', new Uint8Array([0x13, delta & 0xFF]));
        if (response.status === 0 && canonEfApertureRelativeSteps !== null) {
            canonEfApertureRelativeSteps += delta;
            const current = document.getElementById('canonEfAperture')?.textContent || '';
            setCanonEfLensField('canonEfAperture', current.replace(/; relative -?\d+$/, `; relative ${canonEfApertureRelativeSteps}`));
        }
    }
    catch (e) { logToConsole(`Canon EF aperture error: ${e.message}`, 'error'); }
}

function formatCanonEfTableDump(opcode, bytes) {
    const lines = [`Table ${bytesToHex2(opcode)} (${bytes.length} bytes)`];
    for (let offset = 0; offset < bytes.length; offset += 16) {
        const row = Array.from(bytes.slice(offset, offset + 16));
        const hex = row.map(bytesToHex2).join(' ').padEnd(47, ' ');
        const ascii = row.map(value => value >= 32 && value <= 126 ? String.fromCharCode(value) : '.').join('');
        lines.push(`${offset.toString(16).toUpperCase().padStart(2, '0')}  ${hex}  ${ascii}`);
    }
    return lines.join('\n');
}

function renderCanonEfTableDump(opcode, bytes) {
    const output = document.getElementById('canonEfTableDump');
    if (!output) return;
    output.textContent = formatCanonEfTableDump(opcode, bytes);
}

async function readCanonEfTable(opcode, length) {
    const tx = new Uint8Array(length);
    tx[0] = opcode;
    tx.fill(0xDF, 1);
    const response = await runCanonEfLensCommand(`Read parameter table ${bytesToHex2(opcode)}`, tx);
    const rx = canonEfResponseBytes(response);
    if (rx.length < length + 1) {
        throw new Error(`Table response too short: expected at least ${length + 1} bytes, got ${rx.length}`);
    }
    return rx.slice(1, 1 + length);
}

async function dumpCanonEfTable(opcode, length) {
    if (!Number.isInteger(length) || length < 1 || length > 11 || canonEfLensOperationInFlight) return;
    canonEfLensOperationInFlight = true;
    pauseCanonEfTelemetry();
    setCanUiState();
    try {
        renderCanonEfTableDump(opcode, await readCanonEfTable(opcode, length));
    } catch (e) {
        logToConsole(`Canon EF table ${bytesToHex2(opcode)} error: ${e.message}`, 'error');
    } finally {
        canonEfLensOperationInFlight = false;
        resumeCanonEfTelemetry();
        setCanUiState();
    }
}

async function dumpAllCanonEfTables() {
    if (canonEfLensOperationInFlight) return;
    canonEfLensOperationInFlight = true;
    pauseCanonEfTelemetry();
    setCanUiState();
    try {
        const dumps = formatCanonEfTablesCompact(await readAllCanonEfTables());
        const output = document.getElementById('canonEfTableDump');
        if (output) output.textContent = dumps;
    } catch (e) {
        logToConsole(`Canon EF all-table dump error: ${e.message}`, 'error');
    } finally {
        canonEfLensOperationInFlight = false;
        resumeCanonEfTelemetry();
        setCanUiState();
    }
}

async function readAllCanonEfTables() {
    const definitions = [[0xD0, 11], [0xD1, 11], [0xD2, 11], [0xD3, 11], [0xD4, 6],
    [0xD8, 11], [0xD9, 11], [0xDA, 11], [0xDB, 11], [0xDC, 6]];
    const tables = [];
    for (const [opcode, length] of definitions) {
        tables.push([opcode, await readCanonEfTable(opcode, length)]);
    }
    return tables;
}

function formatCanonEfTablesCompact(tables) {
    return tables.map(([opcode, bytes]) =>
        `0x${bytesToHex2(opcode)}: ${Array.from(bytes).map(bytesToHex2).join('')}`).join('\n');
}

function parseHexBytes(text) {
    const tokens = String(text || '').trim().split(/[\s,;]+/).filter(Boolean);
    const bytes = [];
    for (const token of tokens) {
        if (!/^(?:0x)?[0-9a-f]{1,2}$/i.test(token)) {
            throw new Error(`Invalid byte: ${token}`);
        }
        bytes.push(parseInt(token.replace(/^0x/i, ''), 16));
    }
    return new Uint8Array(bytes);
}

function canonEfMask() {
    let mask = 0;
    if (document.getElementById('canonEfInvertDcl')?.checked) mask |= 1;
    if (document.getElementById('canonEfInvertDlc')?.checked) mask |= 2;
    if (document.getElementById('canonEfInvertLclkOut')?.checked) mask |= 4;
    if (document.getElementById('canonEfInvertLclkIn')?.checked) mask |= 8;
    return mask;
}

function renderCanonEfResponse(tx, response) {
    const output = document.getElementById('canonEfRxDisplay');
    if (!output || !response) return;

    const statusText = describeCanonEfTransferStatus(response);
    const lines = [`STATUS ${bytesToHex2(response.status)} - ${statusText}`];
    const rxBytes = response.records && response.records.length
        ? response.records.map(record => record.rx)
        : Array.from(response.data || []);
    if (rxBytes.length) {
        const hex = rxBytes.map(bytesToHex2).join(' ');
        const ascii = rxBytes.map(value => value >= 32 && value <= 126
            ? String.fromCharCode(value)
            : '.').join('');
        lines.push(`${hex} '${ascii}'`);
    }
    if (response.records && response.records.length) {
        lines.push(...formatCanonEfTransferTrace(tx, response));
    } else {
        lines.push(...formatCanonEfTransferTrace(tx, response));
    }
    output.textContent = lines.join('\n');
    output.scrollTop = output.scrollHeight;
}

async function waitForCanonEfConfigurationIdle(timeoutMs = 10000) {
    const deadline = Date.now() + timeoutMs;
    while (canonEfLensOperationInFlight || canonEfFocusPollInFlight || (espSerial && espSerial._lensPending)) {
        if (Date.now() >= deadline) throw new Error('Canon EF controls did not become idle');
        await new Promise(resolve => setTimeout(resolve, 10));
    }
}

function scheduleCanonEfAutoConfigure(delayMs = 0) {
    if (canonEfAutoConfigTimer !== null) clearTimeout(canonEfAutoConfigTimer);
    canonEfAutoConfigTimer = setTimeout(async () => {
        canonEfAutoConfigTimer = null;
        if (!espSerial || !isDeviceConnected) return;
        await startCanonEf(true);
    }, delayMs);
}

function initializeCanonEfAutoConfigure() {
    const immediateIds = [
        'canonEfDcl', 'canonEfDlc', 'canonEfLclkOut', 'canonEfLclkIn',
        'canonEfInvertDcl', 'canonEfInvertDlc', 'canonEfInvertLclkOut',
        'canonEfInvertLclkIn', 'canonEfAckMode'
    ];
    immediateIds.forEach(id => {
        document.getElementById(id)?.addEventListener('change', () => scheduleCanonEfAutoConfigure(0));
    });
    document.getElementById('canonEfClock')?.addEventListener('input', () => scheduleCanonEfAutoConfigure(400));
}

async function startCanonEf(automatic = false) {
    if (!espSerial || !isDeviceConnected) return;

    stopCanonEfCycles();
    pauseCanonEfTelemetry();
    try {
        await waitForCanonEfConfigurationIdle();
        const clockHz = parseHexOrDec(document.getElementById('canonEfClock').value);
        const config = {
            dcl_gpio: parseInt(document.getElementById('canonEfDcl').value, 10),
            dlc_gpio: parseInt(document.getElementById('canonEfDlc').value, 10),
            lclk_out_gpio: parseInt(document.getElementById('canonEfLclkOut').value, 10),
            lclk_in_gpio: parseInt(document.getElementById('canonEfLclkIn').value, 10),
            inversion_mask: canonEfMask(),
            flags: (() => {
                const mode = document.getElementById('canonEfAckMode')?.value;
                return mode === 'measurement' ? 1 : mode === 'ignore' ? 2 : 0;
            })(),
            clock_hz: clockHz
        };

        if (!Number.isFinite(clockHz) || clockHz < 10000 || clockHz > 500000) {
            throw new Error('Clock must be between 10000 and 500000 Hz');
        }
        const request = espSerial.configureCanonEFLens(config, { timeoutMs: 5000 });
        setCanUiState();
        const response = await request;
        if (!response || response.status !== 0) {
            throw new Error(`Configure failed (status=${response ? response.status : 'n/a'})`);
        }
        canonEfRunning = true;
        canonEfInitialized = false;
        canonEfIsRequested = false;
        setCanonEfLensField('canonEfLensType', '-');
        setCanonEfLensField('canonEfLensFocalLength', '-');
        setCanonEfLensField('canonEfLensName', '-');
        setCanonEfLensField('canonEfLensSerial', '-');
        setCanonEfLensField('canonEfFocusPosition', '-');
        setCanonEfLensField('canonEfZoom', '-');
        setCanonEfLensField('canonEfStatus', '-');
        setCanonEfLensField('canonEfIsStatus', '-');
        setCanonEfLensField('canonEfFocusDistance', '-');
        const tablePanel = document.getElementById('canonEfLensTables');
        if (tablePanel) tablePanel.textContent = 'Parameter tables: -';
        updateCanonEfFocusPolling();
        setCanUiState();
        logToConsole(`Canon EF bus configured${automatic ? ' (settings applied)' : ''}`, 'info');
    } catch (e) {
        canonEfRunning = false;
        logToConsole(`Canon EF configure error: ${e.message}`, 'error');
    } finally {
        resumeCanonEfTelemetry();
        setCanUiState();
    }
}

async function sendCanonEf() {
    if (!espSerial || !isDeviceConnected || !canonEfRunning) {
        logToConsole('Configure Canon EF first', 'error');
        return;
    }
    if (espSerial._lensPending) {
        logToConsole('Canon EF transfer still pending; wait for it to finish', 'error');
        return;
    }

    try {
        const tx = parseHexBytes(document.getElementById('canonEfTxInput').value);
        const request = espSerial.canonEFLensTransfer(tx, {
            includeAck: document.getElementById('canonEfAckMode')?.value === 'measurement',
            timeoutMs: Math.max(5000, tx.length * 100)
        });
        setCanUiState();
        const response = await request;
        renderCanonEfResponse(tx, response);
        logToConsole(`Canon EF transfer: ${tx.length} byte(s)`, 'info');
    } catch (e) {
        logToConsole(`Canon EF transfer error: ${e.message}`, 'error');
    } finally {
        setCanUiState();
    }
}

async function resetCanonEf() {
    if (!espSerial || !isDeviceConnected || !canonEfRunning) {
        logToConsole('Configure Canon EF first', 'error');
        return;
    }
    if (espSerial._lensPending) {
        logToConsole('Canon EF request still pending; wait for it to finish', 'error');
        return;
    }

    try {
        const request = espSerial.canonEFLensReset({ timeoutMs: 5000 });
        setCanUiState();
        const response = await request;
        if (!response || response.status !== 0) {
            throw new Error(`Reset failed (status=${response ? response.status : 'n/a'})`);
        }
        logToConsole('Canon EF lens reset complete', 'info');
    } catch (e) {
        logToConsole(`Canon EF reset error: ${e.message}`, 'error');
    } finally {
        setCanUiState();
    }
}


function updateEfUi() {
    const canonEfStartBtn = document.getElementById('canonEfStartBtn');
    const canonEfSendBtn = document.getElementById('canonEfSendBtn');
    const canonEfResetBtn = document.getElementById('canonEfResetBtn');
    const canonEfInitializeBtn = document.getElementById('canonEfInitializeBtn');
    const canonEfInfoBtn = document.getElementById('canonEfInfoBtn');
    const canonEfScanBtn = document.getElementById('canonEfScanBtn');
    const canonEfScanCycleBtn = document.getElementById('canonEfScanCycleBtn');
    const canonEfFocusNearBtn = document.getElementById('canonEfFocusNearBtn');
    const canonEfFocusMinusBtn = document.getElementById('canonEfFocusMinusBtn');
    const canonEfFocusPlusBtn = document.getElementById('canonEfFocusPlusBtn');
    const canonEfFocusInfinityBtn = document.getElementById('canonEfFocusInfinityBtn');
    const canonEfFocusCycleBtn = document.getElementById('canonEfFocusCycleBtn');
    const canonEfApertureOpenBtn = document.getElementById('canonEfApertureOpenBtn');
    const canonEfApertureMinusBtn = document.getElementById('canonEfApertureMinusBtn');
    const canonEfAperturePlusBtn = document.getElementById('canonEfAperturePlusBtn');
    const canonEfApertureCloseBtn = document.getElementById('canonEfApertureCloseBtn');
    const canonEfApertureCycleBtn = document.getElementById('canonEfApertureCycleBtn');
    const canonEfIsConfigureBtn = document.getElementById('canonEfIsConfigureBtn');
    const canonEfIsEnableBtn = document.getElementById('canonEfIsEnableBtn');
    const canonEfIsDisableBtn = document.getElementById('canonEfIsDisableBtn');

    const uartEnabled = !!isDeviceConnected;
    if (canonEfStartBtn) {
        canonEfStartBtn.disabled = !uartEnabled;
        canonEfStartBtn.textContent = canonEfRunning ? 'Reconfigure' : 'Configure';
    }
    if (canonEfSendBtn) canonEfSendBtn.disabled = !uartEnabled || !canonEfRunning;
    if (canonEfResetBtn) canonEfResetBtn.disabled = !uartEnabled || !canonEfRunning;
    const canonEfBusy = !!(espSerial && espSerial._lensPending);
    if (canonEfStartBtn) canonEfStartBtn.disabled = !uartEnabled || canonEfBusy;
    if (canonEfSendBtn) canonEfSendBtn.disabled = !uartEnabled || !canonEfRunning || canonEfBusy;
    if (canonEfResetBtn) canonEfResetBtn.disabled = !uartEnabled || !canonEfRunning || canonEfBusy;
    if (canonEfInitializeBtn) canonEfInitializeBtn.disabled = !uartEnabled || !canonEfRunning || canonEfBusy || canonEfLensOperationInFlight;
    if (canonEfScanBtn) canonEfScanBtn.disabled = !uartEnabled || !canonEfRunning || canonEfBusy || canonEfLensOperationInFlight || canonEfScanCycleActive;
    if (canonEfScanCycleBtn) {
        canonEfScanCycleBtn.disabled = !uartEnabled || !canonEfRunning || (canonEfLensOperationInFlight && !canonEfScanCycleActive);
        canonEfScanCycleBtn.classList.toggle('active', canonEfScanCycleActive);
        canonEfScanCycleBtn.textContent = canonEfScanCycleActive ? 'Stop cycle' : 'Cycle';
    }
    const lensControlsEnabled = uartEnabled && canonEfRunning && canonEfInitialized && !canonEfBusy && !canonEfLensOperationInFlight;
    if (canonEfInfoBtn) canonEfInfoBtn.disabled = !lensControlsEnabled;
    if (canonEfFocusNearBtn) canonEfFocusNearBtn.disabled = !lensControlsEnabled;
    if (canonEfFocusMinusBtn) canonEfFocusMinusBtn.disabled = !lensControlsEnabled;
    if (canonEfFocusPlusBtn) canonEfFocusPlusBtn.disabled = !lensControlsEnabled;
    if (canonEfFocusInfinityBtn) canonEfFocusInfinityBtn.disabled = !lensControlsEnabled;
    if (canonEfFocusCycleBtn) canonEfFocusCycleBtn.disabled = !lensControlsEnabled;
    if (canonEfApertureOpenBtn) canonEfApertureOpenBtn.disabled = !lensControlsEnabled;
    if (canonEfApertureMinusBtn) canonEfApertureMinusBtn.disabled = !lensControlsEnabled;
    if (canonEfAperturePlusBtn) canonEfAperturePlusBtn.disabled = !lensControlsEnabled;
    if (canonEfApertureCloseBtn) canonEfApertureCloseBtn.disabled = !lensControlsEnabled;
    if (canonEfApertureCycleBtn) canonEfApertureCycleBtn.disabled = !lensControlsEnabled;
    if (canonEfIsConfigureBtn) canonEfIsConfigureBtn.disabled = !lensControlsEnabled;
    if (canonEfIsEnableBtn) canonEfIsEnableBtn.disabled = !lensControlsEnabled;
    if (canonEfIsDisableBtn) canonEfIsDisableBtn.disabled = !lensControlsEnabled;
    if (!uartEnabled || !canonEfRunning || !canonEfInitialized || canonEfView !== 'lens') stopCanonEfCycles();
    if (!uartEnabled || !canonEfRunning || canonEfView !== 'scan') stopCanonEfScanCycle();

    updateCanonEfFocusPolling();

}
