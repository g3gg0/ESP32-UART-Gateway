/* Config tool state */
let cfgSerial = null;
let cfgLastSentConfig = null;

function logToConsole(message, type = 'info') {
    const mapped = type === 'error' ? 'error' : 'info';
    log(message, mapped);
}

function consoleLogHex(prefix, data) {
    if (!data) return;
    const bytes = data instanceof Uint8Array ? data : new Uint8Array(data);
    console.log(`${prefix} [${bytes.length} bytes]`);
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

        output += `${i.toString(16).padStart(8, '0')}:  ${hex} | ${ascii}\n`;
    }
    return output;
}

function cfgUpdateStatus(message, isError = false, isSuccess = false) {
    const statusBox = document.getElementById('cfgStatusBox');
    const statusText = document.getElementById('cfgStatusText');

    if (!message) {
        statusBox.style.display = 'none';
        statusText.textContent = '';
        return;
    }

    statusBox.style.display = 'block';
    statusText.textContent = message;
    statusBox.classList.remove('error', 'success');
    if (isError) statusBox.classList.add('error');
    if (isSuccess) statusBox.classList.add('success');
}

function cfgNormalizeConfig(config) {
    if (!config) return null;
    return {
        baudRate: config.baud_rate ?? 0,
        txGpio: config.tx_gpio ?? 0,
        rxGpio: config.rx_gpio ?? 0,
        resetGpio: config.reset_gpio ?? 0,
        controlGpio: config.control_gpio ?? 0,
        ledGpio: config.led_gpio ?? 0
    };
}

function cfgUpdateReceived(config) {
    const elem = document.getElementById('cfgReceivedConfigText');
    const box = document.getElementById('cfgReceivedConfigBox');

    const normalized = cfgNormalizeConfig(config);

    if (normalized) {
        box.style.display = 'block';
        const matches = cfgLastSentConfig &&
            normalized.baudRate === cfgLastSentConfig.baudRate &&
            normalized.txGpio === cfgLastSentConfig.txGpio &&
            normalized.rxGpio === cfgLastSentConfig.rxGpio &&
            normalized.resetGpio === cfgLastSentConfig.resetGpio &&
            normalized.controlGpio === cfgLastSentConfig.controlGpio &&
            normalized.ledGpio === cfgLastSentConfig.ledGpio;

        if (matches) {
            const resetName = normalized.resetGpio === 255 ? 'Unused' : `GPIO${normalized.resetGpio}`;
            const controlName = normalized.controlGpio === 255 ? 'Unused' : `GPIO${normalized.controlGpio}`;
            const ledName = normalized.ledGpio === 255 ? 'Unused' : `GPIO${normalized.ledGpio}`;
            elem.innerHTML = `<div style="text-align: center; font-size: 48px; color: #27ae60;">✓</div>
                <div style="text-align: center; margin-top: 10px;">Configuration Applied Successfully</div>
                <div style="text-align: center; font-size: 11px; margin-top: 8px;">Baud: ${normalized.baudRate} bps | TX: GPIO${normalized.txGpio} | RX: GPIO${normalized.rxGpio}<br>Reset: ${resetName} | Control: ${controlName} | LED: ${ledName}</div>`;
            box.style.borderLeftColor = '#27ae60';
            box.style.background = '#e6ffe6';
            box.style.color = '#27ae60';
            box.style.transition = 'opacity 0.5s ease-out';
            box.style.opacity = '1';

            // Fade out after 4 seconds
            setTimeout(() => {
                box.style.opacity = '0';
                setTimeout(() => {
                    elem.innerHTML = '';
                    box.style.borderLeftColor = '';
                    box.style.background = '';
                    box.style.color = '';
                }, 500); // Wait for fade out animation to complete
            }, 4000);
        }
        else if (!cfgLastSentConfig) {
            const resetName = normalized.resetGpio === 255 ? 'Unused' : `GPIO${normalized.resetGpio}`;
            const controlName = normalized.controlGpio === 255 ? 'Unused' : `GPIO${normalized.controlGpio}`;
            const ledName = normalized.ledGpio === 255 ? 'Unused' : `GPIO${normalized.ledGpio}`;
            elem.innerHTML = `
                <div style="text-align: center; font-weight: bold; margin-bottom: 8px;">Current Device Config</div>
                <div style="font-size: 12px; text-align: center;">
                    Baud: ${normalized.baudRate} bps<br>
                    TX: GPIO${normalized.txGpio} | RX: GPIO${normalized.rxGpio}<br>
                    Reset: ${resetName} | Control: ${controlName}<br>
                    LED: ${ledName}
                </div>
            `;
            box.style.borderLeftColor = '#2196F3';
            box.style.background = '#e3f2fd';
            box.style.color = '#1565c0';

            document.getElementById('cfgBaudRate').value = normalized.baudRate;
            document.getElementById('cfgTxGpio').value = normalized.txGpio;
            document.getElementById('cfgRxGpio').value = normalized.rxGpio;
            document.getElementById('cfgResetGpio').value = normalized.resetGpio;
            document.getElementById('cfgControlGpio').value = normalized.controlGpio;
            document.getElementById('cfgLedGpio').value = normalized.ledGpio;

            /* Show config section now that we have a valid config */
            document.getElementById('cfgConfigSection').style.display = 'block';
            document.getElementById('cfgActionsSection').style.display = 'block';

        }
        else {
            const resetName = normalized.resetGpio === 255 ? 'Unused' : `GPIO${normalized.resetGpio}`;
            const controlName = normalized.controlGpio === 255 ? 'Unused' : `GPIO${normalized.controlGpio}`;
            const ledName = normalized.ledGpio === 255 ? 'Unused' : `GPIO${normalized.ledGpio}`;
            elem.innerHTML = `
                <div style="text-align: center; font-weight: bold; margin-bottom: 8px;">Device Config Received</div>
                <div style="font-size: 12px; text-align: center;">
                    Baud: ${normalized.baudRate} bps<br>
                    TX: GPIO${normalized.txGpio} | RX: GPIO${normalized.rxGpio}<br>
                    Reset: ${resetName} | Control: ${controlName}<br>
                    LED: ${ledName}
                </div>
            `;
            box.style.borderLeftColor = '#2196F3';
            box.style.background = '#e3f2fd';
            box.style.color = '#1565c0';
        }
    }
    else {
        elem.textContent = 'Invalid configuration packet received';
        box.style.borderLeftColor = '#e74c3c';
        box.style.background = '#ffe6e6';
        box.style.color = '#c0392b';
    }
}

function cfgHandleDisconnect(reason) {
    cfgSerial = null;
    document.getElementById('cfgConnectBtn').disabled = false;
    document.getElementById('cfgDisconnectBtn').disabled = true;
    document.getElementById('cfgSendBtn').disabled = true;
    document.getElementById('cfgConfigSection').style.display = 'none';
    document.getElementById('cfgActionsSection').style.display = 'none';
    document.getElementById('cfgConnectBtn').style.display = 'block';
    document.getElementById('cfgDisconnectBtn').style.display = 'none';
    if (reason) {
        cfgUpdateStatus(`Disconnected (${reason})`, true);
    } else {
        cfgUpdateStatus('Disconnected', true);
    }
}

async function cfgConnectDevice() {
    try {
        if (!isWebSerialSupported()) {
            cfgUpdateStatus(getWebSerialUnsupportedMessage(), true);
            return;
        }
        cfgSerial = new EspSerial();
        cfgSerial.setConfigCallback((packet) => {
            cfgUpdateStatus(null);
            cfgUpdateReceived(packet);
        });

        cfgSerial.setLogCallback((msg) => { cfgUpdateStatus(msg.text, false); });
        cfgSerial.setDataCallback(() => { });
        cfgSerial.setDisconnectCallback((info) => {
            const reason = info && info.reason ? info.reason : 'port closed';
            cfgHandleDisconnect(reason);
        });

        const connected = await cfgSerial.connect();
        if (!connected) {
            throw new Error('Connection cancelled');
        }

        document.getElementById('cfgConnectBtn').disabled = true;
        document.getElementById('cfgDisconnectBtn').disabled = false;
        document.getElementById('cfgSendBtn').disabled = false;
        document.getElementById('cfgConnectBtn').style.display = 'none';
        document.getElementById('cfgDisconnectBtn').style.display = 'block';

        cfgLastSentConfig = null;
        await cfgSerial.requestConfig();
    } catch (err) {
        cfgUpdateStatus('Connection failed: ' + err.message, true);
    }
}

async function cfgDisconnectDevice() {
    try {
        if (cfgSerial) {
            await cfgSerial.disconnect();
        }
        cfgHandleDisconnect('manual');
    } catch (err) {
        cfgUpdateStatus('Disconnect error: ' + err.message, true);
    }
}

async function cfgSendConfig() {
    if (!cfgSerial) {
        return;
    }

    const baudRate = parseInt(document.getElementById('cfgBaudRate').value);
    const txGpio = parseInt(document.getElementById('cfgTxGpio').value);
    const rxGpio = parseInt(document.getElementById('cfgRxGpio').value);
    const resetGpio = parseInt(document.getElementById('cfgResetGpio').value);
    const controlGpio = parseInt(document.getElementById('cfgControlGpio').value);
    const ledGpio = parseInt(document.getElementById('cfgLedGpio').value);

    if (txGpio === rxGpio) {
        cfgUpdateStatus('TX and RX GPIO must be different', true);
        return;
    }

    cfgLastSentConfig = { baudRate, txGpio, rxGpio, resetGpio, controlGpio, ledGpio };

    try {
        await cfgSerial.setConfig({
            baud_rate: baudRate,
            tx_gpio: txGpio,
            rx_gpio: rxGpio,
            reset_gpio: resetGpio,
            control_gpio: controlGpio,
            led_gpio: ledGpio,
            extended_mode: 1
        });
    } catch (err) {
        cfgUpdateStatus('Send failed: ' + err.message, true);
    }
}

