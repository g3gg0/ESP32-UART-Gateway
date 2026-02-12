/* ============= EspSerial Class ============= */
class EspSerial {
    constructor() {
        this.port = null;
        this.reader = null;
        this.rxBuffer = new Uint8Array(0);
        this.inputDone = false;
        this.data_cbr = null; /* Raw UART-like bytes */
        this.config_cbr = null; /* Config packets */
        this.log_cbr = null; /* Parsed ESP log packets */

        /* Extended mode activation magic: type 0x000A packet with 8-byte payload */
        this.EXTMODE_MAGIC = new Uint8Array([
            0x0C, 0x00,  /* length: 12 (4 bytes header + 8 bytes payload) */
            0x0A, 0x00,  /* type: 0x000A */
            0x55, 0x41, 0x52, 0x54,  /* U A R T */
            0x47, 0x57, 0x45, 0x58   /* G W E X */
        ]);
        this.EXTMODE_PACKET_SIZE = 12;

        /* Packet header constants */
        this.PACKET_HEADER_SIZE = 4;  /* uint16_t length + uint16_t type */
        this.PACKET_TYPE_DATA = 0x00;
        this.PACKET_TYPE_CONFIG = 0x01;
        this.PACKET_TYPE_CONTROL = 0x02;
        this.PACKET_TYPE_LOG = 0x03;

        this.CTRL_CMD_SIZE = 16;
        this.LOGMSG_MAX_LEN = 256;

        this.extendedMode = false;

        /* Log message waiting mechanism */
        this.nextLogPromise = null;
        this.nextLogResolve = null;
        this.nextLogTimer = null;
    }

    setDataCallback(callback) {
        this.data_cbr = callback;
    }

    setConfigCallback(callback) {
        this.config_cbr = callback;
    }

    setLogCallback(callback) {
        this.log_cbr = callback;
    }

    async flushSerialData(durationMs) {
        if (!this.port) return;
        try {
            const reader = this.port.readable.getReader();
            const startTime = Date.now();
            let bytesDiscarded = 0;

            while (Date.now() - startTime < durationMs) {
                try {
                    const { value, done } = await Promise.race([
                        reader.read(),
                        new Promise((_, reject) =>
                            setTimeout(() => reject(new Error('flush timeout')), 50)
                        )
                    ]);

                    if (done) break;
                    if (value) {
                        bytesDiscarded += value.length;
                    }
                } catch (err) {
                    /* Timeout waiting for data, continue flushing */
                    if (err.message !== 'flush timeout') throw err;
                }
            }

            reader.releaseLock();
            logToConsole(`Serial flush complete: ${bytesDiscarded} bytes discarded`, 'info');
        } catch (err) {
            logToConsole(`Serial flush error: ${err.message}`, 'info');
        }
    }

    async connect() {
        try {
            this.port = await navigator.serial.requestPort();
            await this.port.open({ baudRate: 921600 });

            /* Ensure RTS and DTR are low after opening */
            try {
                await this.port.setSignals({ requestToSend: false, dataTerminalReady: false });
            } catch (e) {
                logToConsole(`Warning: Failed to set RTS/DTR: ${e.message}`, 'info');
            }

            /* Flush any pending serial data for 250ms */
            await this.flushSerialData(250);

            /* Start reader task (non-blocking) */
            this.readFromPort();

            await new Promise(resolve => setTimeout(resolve, 500));
            logToConsole('ESP Serial: Connected', 'info');

            /* Send extended mode activation magic */
            const magic = this.buildExtModeActivationPacket();
            consoleLogHex('TX ExtMode Magic:', magic);
            await this.sendPacket(magic, 'ExtMode Magic');
            logToConsole('ESP Serial: Extended mode activation sent', 'info');
            this.extendedMode = true;

            /* Wait for first log message to confirm extended mode is working */
            logToConsole('ESP Serial: Waiting for log message...', 'info');
            const logMsg = await this.waitForLogMessage(2000);
            if (!logMsg) {
                throw new Error('No log message received after extended mode activation - device may not be responding');
            }
            logToConsole(`ESP Serial: Confirmed with log: "${logMsg.substring(0, 50)}..."`, 'info');

            /* Request current configuration - MUST succeed */
            await new Promise(resolve => setTimeout(resolve, 100));
            logToConsole('Requesting device configuration...', 'info');
            await this.requestConfig();
            logToConsole('Device configuration received', 'info');

            return true;
        } catch (err) {
            if (err.name !== 'NotFoundError') {
                logToConsole(`ESP Serial: Connection error: ${err.message}`, 'info');
            }
            return false;
        }
    }

    async waitForLogMessage(timeoutMs = 2000) {
        return new Promise((resolve) => {
            this.nextLogResolve = resolve;
            this.nextLogTimer = setTimeout(() => {
                this.nextLogResolve = null;
                resolve(null);
            }, timeoutMs);
        });
    }

    async disconnect() {
        try {
            this.inputDone = true;

            /* Release reader lock if it exists */
            if (this.reader) {
                try {
                    this.reader.cancel();
                } catch (e) {
                    /* Reader might already be released */
                }
                this.reader = null;
            }

            /* Wait a moment for read loop to exit */
            await new Promise(resolve => setTimeout(resolve, 50));

            if (this.port) {
                try {
                    await this.port.close();
                } catch (e) {
                    logToConsole(`Port close error: ${e.message}`, 'info');
                }
                this.port = null;
            }
            logToConsole('ESP Serial: Disconnected', 'info');
        } catch (err) {
            logToConsole(`ESP Serial: Disconnect error: ${err.message}`, 'info');
        }
    }

    appendBuffer(a, b) {
        if (!a || a.length === 0) return new Uint8Array(b);
        const out = new Uint8Array(a.length + b.length);
        out.set(a, 0);
        out.set(b, a.length);
        return out;
    }

    buildExtModeActivationPacket() {
        /* Magic packet is already complete: header + payload */
        return new Uint8Array(this.EXTMODE_MAGIC);
    }

    buildPacketHeader(payloadLength, type) {
        /* Calculate total length: payload + 4-byte header */
        const length = payloadLength + 4;
        const header = new Uint8Array(this.PACKET_HEADER_SIZE);
        header[0] = length & 0xFF;          /* length low byte */
        header[1] = (length >> 8) & 0xFF;   /* length high byte */
        header[2] = type & 0xFF;            /* type low byte */
        header[3] = (type >> 8) & 0xFF;     /* type high byte */
        return header;
    }

    buildPacket(payload, type) {
        /* Build complete packet: header + payload */
        const header = this.buildPacketHeader(payload.length, type);
        return this.appendBuffer(header, payload);
    }

    buildControlPacket(command) {
        const cmdBytes = new TextEncoder().encode(command);
        const payload = new Uint8Array(this.CTRL_CMD_SIZE);
        for (let i = 0; i < Math.min(cmdBytes.length, this.CTRL_CMD_SIZE); i++) {
            payload[i] = cmdBytes[i];
        }
        return this.buildPacket(payload, this.PACKET_TYPE_CONTROL);
    }

    processEspPackets(data) {
        this.rxBuffer = this.appendBuffer(this.rxBuffer, data);

        const emitData = (chunk) => {
            if (!chunk || chunk.length === 0) return;
            if (this.data_cbr) {
                this.data_cbr(chunk);
            }
        };

        const emitConfig = (configBytes) => {
            let parsedConfig = {
                type: 'config_packet',
                data: configBytes,
                baud_rate: 0,
                tx_gpio: 0,
                rx_gpio: 0,
                reset_gpio: 0,
                control_gpio: 0,
                led_gpio: 0,
                extended_mode: 0
            };

            /* Parse config structure if exactly 12 bytes */
            if (configBytes.length === 12) {
                const view = new DataView(configBytes.buffer, configBytes.byteOffset, configBytes.length);
                parsedConfig.baud_rate = view.getUint32(0, true);
                parsedConfig.tx_gpio = configBytes[4];
                parsedConfig.rx_gpio = configBytes[5];
                parsedConfig.reset_gpio = configBytes[6];
                parsedConfig.control_gpio = configBytes[7];
                parsedConfig.led_gpio = configBytes[8];
                parsedConfig.extended_mode = configBytes[11];
            }

            if (this.config_cbr) {
                this.config_cbr(parsedConfig);
            }
        };

        const emitLog = (msgBytes) => {
            const text = new TextDecoder().decode(msgBytes);
            if (this.log_cbr) {
                this.log_cbr({
                    type: 'log_packet',
                    data: msgBytes,
                    text
                });
            }
            /* Resolve any pending log wait */
            if (this.nextLogResolve) {
                this.nextLogResolve(text);
                this.nextLogResolve = null;
                if (this.nextLogTimer) {
                    clearTimeout(this.nextLogTimer);
                    this.nextLogTimer = null;
                }
            }
        };

        if (!this.extendedMode) {
            /* In non-extended mode, just pass through all data */
            if (this.rxBuffer.length > 0) {
                emitData(this.rxBuffer);
                this.rxBuffer = new Uint8Array(0);
            }
            return;
        }

        /* Extended mode: parse packet headers */
        while (this.rxBuffer.length >= this.PACKET_HEADER_SIZE) {
            const length = (this.rxBuffer[0] | (this.rxBuffer[1] << 8)) >>> 0;
            const type = (this.rxBuffer[2] | (this.rxBuffer[3] << 8)) >>> 0;

            /* Validate length (must include header, minimum 4) */
            if (length < 4) {
                /* Invalid packet, discard first byte and resync */
                this.rxBuffer = this.rxBuffer.slice(1);
                continue;
            }

            const payloadLen = length - 4;
            const totalLen = this.PACKET_HEADER_SIZE + payloadLen;

            /* Wait for complete packet */
            if (this.rxBuffer.length < totalLen) {
                break;
            }

            /* Extract payload */
            const payload = this.rxBuffer.slice(this.PACKET_HEADER_SIZE, totalLen);
            this.rxBuffer = this.rxBuffer.slice(totalLen);

            /* Handle packet based on type */
            if (type === this.PACKET_TYPE_DATA) {
                /* Type 0x00: Normal serial data */
                emitData(payload);
            } else if (type === this.PACKET_TYPE_CONFIG) {
                /* Type 0x01: Config response */
                emitConfig(payload);
            } else if (type === this.PACKET_TYPE_LOG) {
                /* Type 0x03: Log message */
                emitLog(payload);
            } else {
                /* Unknown packet type, pass through as data */
                emitData(payload);
            }
        }
    }

    async readFromPort() {
        if (!this.port) return;

        try {
            this.reader = this.port.readable.getReader();
            this.inputDone = false;

            while (!this.inputDone && this.port) {
                try {
                    const { value, done } = await this.reader.read();
                    if (done) break;

                    if (value) {
                        /* Log raw RX bytes at lowest level */
                        consoleLogHex('RX:', value);
                        this.processEspPackets(value);
                    }
                } catch (err) {
                    if (err.name !== 'AbortError') {
                        logToConsole(`Read error: ${err.message}`, 'info');
                    }
                    break;
                }
            }
        } catch (err) {
            logToConsole(`ReadFromPort error: ${err.message}`, 'info');
        } finally {
            if (this.reader) {
                this.reader.releaseLock();
                this.reader = null;
            }
        }
    }

    async sendData(data) {
        if (!this.port) return;
        try {
            const writer = this.port.writable.getWriter();
            try {
                await this.sendDataWithWriter(writer, data);
            } finally {
                writer.releaseLock();
            }
        } catch (err) {
            logToConsole(`Send data error: ${err.message}`, 'info');
        }
    }

    async sendDataWithWriter(writer, data) {
        /* Fragment data into 64k blocks if needed */
        const maxPayloadSize = 65534;
        let offset = 0;

        while (offset < data.length) {
            const chunkSize = Math.min(maxPayloadSize, data.length - offset);
            const chunk = data.slice(offset, offset + chunkSize);

            const packet = this.buildPacket(chunk, this.PACKET_TYPE_DATA);

            /* Log raw TX bytes at lowest level */
            consoleLogHex('TX Data:', packet);
            await writer.write(packet);
            offset += chunkSize;
        }
    }

    async sendPacket(packet, label) {
        if (!this.port) return;
        try {
            const writer = this.port.writable.getWriter();
            try {
                consoleLogHex(`TX ${label}:`, packet);
                await writer.write(packet);
            } finally {
                writer.releaseLock();
            }
        } catch (err) {
            logToConsole(`Send packet error: ${err.message}`, 'info');
        }
    }

    async sendBreak() {
        let breakLen = parseInt(document.getElementById('breakLenInput').value) || 200;
        if (breakLen > 200) breakLen = 200;
        if (breakLen < 0) breakLen = 0;
        const commandStr = 'B:' + breakLen;
        const packet = this.buildControlPacket(commandStr);
        await this.sendPacket(packet, 'Break');
    }

    async setResetGpio(value) {
        const command = value ? 'R:1' : 'R:0';
        const packet = this.buildControlPacket(command);
        await this.sendPacket(packet, 'Reset');
    }

    async setControlGpio(value) {
        const command = value ? 'C:1' : 'C:0';
        const packet = this.buildControlPacket(command);
        await this.sendPacket(packet, 'Control');
    }

    async requestConfig() {
        if (!this.port) throw new Error('Port not connected');
        try {
            const payload = new Uint8Array(12);
            payload.fill(0);

            const packet = this.buildPacket(payload, this.PACKET_TYPE_CONFIG);

            /* Track if we received a config response */
            let configReceived = false;
            const originalConfigCbr = this.config_cbr;
            this.config_cbr = (packet) => {
                configReceived = true;
                if (originalConfigCbr) originalConfigCbr(packet);
            };

            await this.sendPacket(packet, 'Config Request');

            /* Wait up to 2 seconds for config response - MUST receive it */
            let waited = 0;
            while (waited < 2000 && !configReceived) {
                await new Promise(resolve => setTimeout(resolve, 50));
                waited += 50;
            }

            this.config_cbr = originalConfigCbr;

            if (!configReceived) {
                throw new Error('Config request timeout - device did not respond');
            }
        } catch (err) {
            logToConsole(`Config request error: ${err.message}`, 'info');
            throw err;
        }
    }

    async setConfig(configData) {
        if (!this.port) return;
        try {
            /* Build 12-byte payload from config structure */
            const payload = new Uint8Array(12);
            payload.fill(0);

            /* baud_rate at offset 0-3 (little-endian) */
            const baud = configData.baud_rate || 0;
            payload[0] = baud & 0xFF;
            payload[1] = (baud >> 8) & 0xFF;
            payload[2] = (baud >> 16) & 0xFF;
            payload[3] = (baud >> 24) & 0xFF;

            /* GPIO pins at offsets 4-8 */
            payload[4] = configData.tx_gpio || 0;
            payload[5] = configData.rx_gpio || 0;
            payload[6] = configData.reset_gpio || 0;
            payload[7] = configData.control_gpio || 0;
            payload[8] = configData.led_gpio || 0;

            /* Padding at offsets 9-10 */
            payload[9] = 0;
            payload[10] = 0;

            /* extended_mode at offset 11 */
            payload[11] = configData.extended_mode || 0;

            const packet = this.buildPacket(payload, this.PACKET_TYPE_CONFIG);

            const writer = this.port.writable.getWriter();
            await writer.write(packet);
            writer.releaseLock();
        } catch (err) {
            logToConsole(`Config send error: ${err.message}`, 'info');
        }
    }
}

window.EspSerial = EspSerial;
