class CC3200Serial {
    constructor(espSerial) {
        this.espSerial = espSerial;
        this.waitingForResponse = false;
        this.cc3200State = 'idle'; /* idle, waitingForAck, waitingForData, delayingBeforeAck */
        this.rxBuffer = new Uint8Array(0);
        this.cc3200AckTimer = null;
        this.receive_cbr = null;

        this.pendingResponse = null;
        this.responseTimer = null;
        this.byteResponseBuffer = new Uint8Array(0);
        this.packetAssembler = null;
        this.ackDelayMs = 0;
        this.autoFinishAfterAck = false;

        this.pendingResolve = null;
        this.pendingReject = null;

        this.pendingAckPromise = null;
    }

    setDataCallback(callback) {
        this.receive_cbr = callback;
    }

    appendBuffer(a, b) {
        if (!a || a.length === 0) return new Uint8Array(b);
        const out = new Uint8Array(a.length + b.length);
        out.set(a, 0);
        out.set(b, a.length);
        return out;
    }

    _clearResponseTimer() {
        if (this.responseTimer) {
            clearTimeout(this.responseTimer);
            this.responseTimer = null;
        }
    }

    _finishResponse() {
        this._clearResponseTimer();
        this.waitingForResponse = false;
        this.cc3200State = 'idle';
        this.pendingResponse = null;
        this.byteResponseBuffer = new Uint8Array(0);
        this.packetAssembler = null;
        this.rxBuffer = new Uint8Array(0);
        this.autoFinishAfterAck = false;
    }

    _resolvePending(result) {
        if (this.pendingResolve) {
            const r = this.pendingResolve;
            this.pendingResolve = null;
            this.pendingReject = null;
            r(result);
        }
    }

    _rejectPending(err) {
        if (this.pendingReject) {
            const rj = this.pendingReject;
            this.pendingResolve = null;
            this.pendingReject = null;
            rj(err);
        }
    }

    _armResponseTimeout(timeoutMs) {
        this._clearResponseTimer();
        this.responseTimer = setTimeout(() => {
            logToConsole('CC3200 response timeout', 'info');
            this._rejectPending(new Error('CC3200 response timeout'));
            this._finishResponse();
        }, timeoutMs);
    }

    async _sendAckNow() {
        if (!this.espSerial) return;
        const ack = new Uint8Array([0x00, 0xCC]);
        /* Send CC3200 ACK wrapped in ESP DATA packet */
        await this.espSerial.sendData(ack);
        /* consoleLogHex('TX CC3200Ack:', ack); */
    }

    _scheduleAck() {
        if (this.ackDelayMs <= 0) {
            this.pendingAckPromise = this._sendAckNow();
            return this.pendingAckPromise;
        }

        this.pendingAckPromise = new Promise((resolve, reject) => {
            setTimeout(() => {
                this._sendAckNow().then(resolve).catch((err) => {
                    logToConsole(`CC3200 ACK send error: ${err.message}`, 'info');
                    reject(err);
                });
            }, this.ackDelayMs);
        });
        return this.pendingAckPromise;
    }

    processCC3200Response(data) {
        if (!data) return;

        if (data.length > 0) {
            this.rxBuffer = this.appendBuffer(this.rxBuffer, data);
            //consoleLogHex(`RX CC3200(${this.cc3200State}, ${this.rxBuffer.length} buffered, ${this.pendingResponse ? this.pendingResponse.kind : 'none'}):`, data);
        }

        if (this.cc3200State === 'idle') {
            /* Not expecting anything - just ignore */
            this.rxBuffer = new Uint8Array(0);
            return;
        }

        /* During sync, check last two bytes for 00 CC */
        if (this.cc3200State === 'syncing') {
            if (this.rxBuffer.length >= 2) {
                const lastIdx = this.rxBuffer.length - 1;
                if (this.rxBuffer[lastIdx - 1] === 0x00 && this.rxBuffer[lastIdx] === 0xCC) {
                    logToConsole('Sync complete - received 00 CC', 'info');
                    this.cc3200State = 'idle';
                    this.rxBuffer = new Uint8Array(0);
                    return;
                }
                /* Keep only last byte to avoid buffer growth */
                this.rxBuffer = this.rxBuffer.slice(lastIdx);
            }
            return;
        }

        if (this.cc3200State === 'waitingForAck') {
            let ackFound = false;
            for (let i = 0; i < this.rxBuffer.length - 1; i++) {
                if (this.rxBuffer[i] === 0x00 && this.rxBuffer[i + 1] === 0xCC) {
                    //logToConsole('Device ACK received (00 CC)', 'info');
                    this.receive_cbr && this.receive_cbr({
                        type: 'cc3200_ack',
                        data: null
                    });

                    this.rxBuffer = this.rxBuffer.slice(i + 2);
                    if (this.autoFinishAfterAck && !this.pendingResponse) {
                        this._resolvePending({ kind: 'ack' });
                        this._finishResponse();
                        return;
                    }

                    this.cc3200State = 'waitingForData';
                    ackFound = true;
                    break;
                }
            }

            if (ackFound) {
                this.processCC3200Response(new Uint8Array(0));
                return;
            } else {
                if (this.rxBuffer.length > 0 && this.rxBuffer[this.rxBuffer.length - 1] === 0x00) {
                    this.rxBuffer = this.rxBuffer.slice(this.rxBuffer.length - 1);
                } else {
                    this.rxBuffer = new Uint8Array(0);
                }
                return;
            }
        }

        if (this.cc3200State === 'waitingForData') {
            if (!this.pendingResponse) {
                this.rxBuffer = new Uint8Array(0);
                return;
            }

            if (this.pendingResponse.kind === 'bytes') {
                this.byteResponseBuffer = this.appendBuffer(this.byteResponseBuffer, this.rxBuffer);
                this.rxBuffer = new Uint8Array(0);

                if (this.byteResponseBuffer.length < this.pendingResponse.expectedBytes) {
                    return;
                }

                const out = this.byteResponseBuffer.slice(0, this.pendingResponse.expectedBytes);
                this.byteResponseBuffer = this.byteResponseBuffer.slice(this.pendingResponse.expectedBytes);
                if (this.pendingResponse.callback) {
                    try {
                        this.pendingResponse.callback(this.pendingResponse.ctx, out);
                    } catch (err) {
                        logToConsole(`CC3200 response callback error: ${err.message}`, 'info');
                    }
                }
                this._resolvePending({ kind: 'bytes', data: out });
                this._finishResponse();
                return;
            }

            if (this.pendingResponse.kind === 'packet') {
                if (!this.packetAssembler) {
                    this.packetAssembler = new CC3200PacketAssembler(this.pendingResponse.expectedLen ?? null);
                }

                try {
                    const payload = this.packetAssembler.push(this.rxBuffer);
                    this.rxBuffer = new Uint8Array(0);
                    if (!payload) {
                        return;
                    }

                    if (this.pendingResponse.callback) {
                        try {
                            this.pendingResponse.callback(this.pendingResponse.ctx, payload);
                        } catch (err) {
                            logToConsole(`CC3200 response callback error: ${err.message}`, 'info');
                        }
                    }
                    const ackPromise = this._scheduleAck();
                    this._resolvePending({ kind: 'packet', data: payload, ackPromise });
                    this._finishResponse();
                    return;
                } catch (err) {
                    logToConsole(`CC3200 packet error: ${err.message}`, 'info');
                    this._rejectPending(err);
                    this._finishResponse();
                    return;
                }
            }
        }
    }

    async sendCommand(data, expectResponse = false) {
        if (!this.espSerial) return;
        try {
            const len = data.length + 2;
            const checksum = (data.reduce((sum, byte) => sum + byte, 0)) & 0xFF;
            const packet = new Uint8Array(3 + data.length);
            packet[0] = (len >> 8) & 0xFF;
            packet[1] = len & 0xFF;
            packet[2] = checksum;
            packet.set(data, 3);

            this.waitingForResponse = true;
            this.cc3200State = 'waitingForAck';
            this.autoFinishAfterAck = !expectResponse;

            /* Send CC3200 frame wrapped in ESP DATA packet */
            await this.espSerial.sendData(packet);
            /* consoleLogHex('TX CC3200Cmd:', packet); */

            //logToConsole(`CC3200 command sent: ${hexdump(packet)}`, 'tx');

        } catch (err) {
            logToConsole(`CC3200 send error: ${err.message}`, 'info');
        }
    }

    async sendCommandWaitAck(data, timeoutMs = 1000) {
        if (this.pendingReject) {
            this._rejectPending(new Error('CC3200 previous command cancelled'));
        }

        const res = await new Promise(async (resolve, reject) => {
            this.pendingResolve = resolve;
            this.pendingReject = reject;
            this.pendingResponse = null;
            try {
                await this.sendCommand(data, false);
            } catch (err) {
                this._rejectPending(err);
                this._finishResponse();
                return;
            }
            this._armResponseTimeout(timeoutMs);
        });
        return res;
    }

    async sendCommandWithResponse(data, responseSpec) {
        if (this.pendingReject) {
            this._rejectPending(new Error('CC3200 previous command cancelled'));
        }

        const res = await new Promise(async (resolve, reject) => {
            this.pendingResolve = resolve;
            this.pendingReject = reject;
            this.pendingResponse = responseSpec;
            const timeout = responseSpec?.timeoutMs ?? 1000;
            try {
                await this.sendCommand(data, true);
            } catch (err) {
                this._rejectPending(err);
                this._finishResponse();
                return;
            }
            this._armResponseTimeout(timeout);
        });

        if (res && res.ackPromise) {
            await res.ackPromise;
        }
        return res;
    }

    async performSync() {
        if (!this.espSerial) return;

        logToConsole('Starting CC3200 sync sequence...', 'info');
        this.cc3200State = 'syncing';

        try {
            /* Outer retry loop - 5 times */
            for (let outerRetry = 0; outerRetry < 5; outerRetry++) {
                if (this.cc3200State !== 'syncing') break;

                logToConsole(`Sync attempt ${outerRetry + 1}/5`, 'info');

                await this.espSerial.setControlGpio(true);
                await this.espSerial.setResetGpio(false);
                await new Promise(resolve => setTimeout(resolve, 50));
                await this.espSerial.setResetGpio(true);
                await new Promise(resolve => setTimeout(resolve, 250));

                /* Inner retry loop - 5 breaks */
                for (let breakRetry = 0; breakRetry < 3; breakRetry++) {
                    //logToConsole(`Break ${breakRetry + 1}/5`, 'info');
                    await this.espSerial.sendBreak();

                    /* Wait 500ms for 00 CC response */
                    const synced = await this.waitForSyncResponse();
                    if (synced) {
                        logToConsole('Sync successful!', 'info');
                        return;
                    }
                }
            }
            logToConsole('Sync failed after all retries', 'info');
            this.cc3200State = 'idle';
        } catch (err) {
            logToConsole(`Sync error: ${err.message}`, 'info');
            this.cc3200State = 'idle';
        }
    }

    async waitForSyncResponse() {
        return new Promise((resolve) => {
            const startTime = Date.now();
            const checkInterval = setInterval(() => {
                if (this.cc3200State === 'idle') {
                    clearInterval(checkInterval);
                    resolve(true);
                    return;
                }

                if (Date.now() - startTime >= 500) {
                    clearInterval(checkInterval);
                    resolve(false);
                }
            }, 10);
        });
    }

    async disconnect() {
        if (this.cc3200AckTimer) {
            clearTimeout(this.cc3200AckTimer);
            this.cc3200AckTimer = null;
        }
        this._rejectPending(new Error('CC3200 disconnected'));
        this._clearResponseTimer();
        this.waitingForResponse = false;
        this.cc3200State = 'idle';
        this.pendingResponse = null;
        this.byteResponseBuffer = new Uint8Array(0);
        this.packetAssembler = null;
        this.rxBuffer = new Uint8Array(0);

        this.pendingAckPromise = null;
    }
}
