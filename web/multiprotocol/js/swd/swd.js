class SwdPinControls {
    constructor(container, getSerial, invalidate) {
        this.container = container;
        this.getSerial = getSerial;
        this.invalidate = invalidate;
        this.busy = false;
        this.applied = false;
        this.pending = Promise.resolve();
        this.revision = 0;
        let saved = {};
        try {
            saved = JSON.parse(localStorage.getItem('swd-pin-states') || '{}') || {};
        } catch (_) { }
        this.driveMode = ['open-drain', 'open-drain-no-pull', 'push-pull'].includes(saved.driveMode) ? saved.driveMode : 'open-drain';
        this.readMode = saved.readMode === 'pull-down' ? 'pull-down' : 'pull-up';
        this.clockHz = Number.isInteger(saved.clockHz) && saved.clockHz >= 500 && saved.clockHz <= 500000 ? saved.clockHz : 100000;
        const change = () => {
            this.applied = false;
            this.refresh();
            this.save();
            this.apply().catch(error => logToConsole(error.message, 'error'));
        };
        const settings = document.createElement('div');
        settings.id = 'swdPinSettings';
        settings.className = 'swd-pin-settings';
        const rateLabel = document.createElement('label');
        rateLabel.className = 'swd-clock-control';
        rateLabel.append(document.createTextNode('SWCLK max. (kHz)'));
        this.rateInput = document.createElement('input');
        this.rateInput.type = 'number';
        this.rateInput.required = true;
        this.rateInput.id = 'swdClockRate';
        this.rateInput.min = '0.5';
        this.rateInput.max = '500';
        this.rateInput.step = '0.5';
        this.rateInput.value = this.clockHz / 1000;
        this.rateInput.title = 'SWCLK upper bound; GPIO overhead reduces the actual rate';
        this.rateInput.addEventListener('change', () => {
            if (!this.rateInput.checkValidity()) {
                this.rateInput.reportValidity();
                return;
            }
            const clockHz = Math.round(Number(this.rateInput.value) * 1000);
            if (!Number.isFinite(clockHz) || clockHz < 500 || clockHz > 500000) return;
            this.clockHz = clockHz;
            change();
        });
        document.getElementById('swdPinSettings')?.remove();
        document.getElementById('swdClockRate')?.parentElement.remove();
        rateLabel.append(this.rateInput);
        settings.append(rateLabel);
        for (const [name, property, values] of [
            ['Read', 'readSelect', [['pull-up', 'pull up'], ['pull-down', 'pull down']]],
            ['Write', 'writeSelect', [['push-pull', 'push-pull'], ['open-drain-no-pull', 'open-drain'], ['open-drain', 'open-drain (pull up)']]]
        ]) {
            const label = document.createElement('label');
            label.className = 'swd-clock-control';
            label.append(document.createTextNode(name));
            const select = document.createElement('select');
            select.id = property === 'readSelect' ? 'swdReadMode' : 'swdWriteMode';
            select.title = `Shared ${name.toLowerCase()} mode for active SWD pins`;
            for (const [value, text] of values) {
                const option = document.createElement('option');
                option.value = value;
                option.textContent = text;
                select.appendChild(option);
            }
            this[property] = select;
            select.addEventListener('change', () => {
                if (property === 'readSelect') this.readMode = select.value;
                else this.driveMode = select.value;
                change();
            });
            label.append(select);
            settings.append(label);
        }
        container.before(settings);
        for (const gpio of [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 20, 21]) {
            const item = document.createElement('div');
            item.className = 'swd-pin';
            const label = document.createElement('label');
            label.className = 'swd-pin-choice';
            const checkbox = document.createElement('input');
            checkbox.type = 'checkbox';
            checkbox.value = gpio;
            checkbox.checked = typeof saved[gpio]?.scan === 'boolean' ? saved[gpio].scan : gpio >= 20;
            checkbox.setAttribute('aria-label', `Scan GPIO${gpio}`);
            label.append(checkbox, document.createTextNode(`GPIO${gpio}`));
            const select = document.createElement('select');
            select.dataset.idle = ['open', 'high', 'low', 'swc', 'swd'].includes(saved[gpio]?.idle) ? saved[gpio].idle : 'open';
            if (['swc', 'swd'].includes(select.dataset.idle)) checkbox.checked = false;
            checkbox.addEventListener('change', () => {
                if (checkbox.checked && ['swc', 'swd'].includes(select.dataset.idle)) select.dataset.idle = 'open';
                change();
            });
            select.addEventListener('change', () => {
                checkbox.checked = select.value === 'scan';
                if (!checkbox.checked) select.dataset.idle = select.value;
                if (['swc', 'swd'].includes(select.value)) {
                    for (const other of container.querySelectorAll('.swd-pin')) {
                        const state = other.querySelector('select');
                        if (state !== select && state.dataset.idle === select.value) state.dataset.idle = 'open';
                    }
                }
                change();
            });
            item.append(label, select);
            container.appendChild(item);
            this.refresh();
        }
    }

    refresh() {
        if (this.readSelect) this.readSelect.value = this.readMode || 'pull-up';
        if (this.writeSelect) this.writeSelect.value = this.driveMode;
        for (const item of this.container.querySelectorAll('.swd-pin')) {
            const checkbox = item.querySelector('input');
            const select = item.querySelector('select');
            const options = [['scan', 'Scan'], ['open', 'Open'], ['high', '3.3V'], ['low', 'GND'], ['swc', 'SWC'], ['swd', 'SWD']];
            select.replaceChildren();
            for (const [value, text] of options) {
                const option = document.createElement('option');
                option.value = value;
                option.textContent = text;
                select.appendChild(option);
            }
            select.value = checkbox.checked ? 'scan' : select.dataset.idle;
            select.disabled = this.busy;
            select.setAttribute('aria-label', `GPIO${checkbox.value} pin state`);
            select.title = 'Scan, idle level or fixed SWC/SWD role';
            item.classList.toggle('scanning', checkbox.checked || ['swc', 'swd'].includes(select.dataset.idle));
            item.classList.toggle('idle-low', !checkbox.checked && select.dataset.idle === 'low');
            item.classList.toggle('idle-high', !checkbox.checked && select.dataset.idle === 'high');
        }
    }

    masks() {
        const result = { scan: 0, high: 0, low: 0 };
        let fixedMask = 0;
        for (const item of this.container.querySelectorAll('.swd-pin')) {
            const checkbox = item.querySelector('input');
            const mode = checkbox.checked ? 'scan' : item.querySelector('select').value;
            if (mode === 'swc' || mode === 'swd') fixedMask |= 1 << Number(checkbox.value);
            else if (mode !== 'open') result[mode] = (result[mode] | (1 << Number(checkbox.value))) >>> 0;
        }
        if (fixedMask) result.scan = fixedMask >>> 0;
        return result;
    }

    fixedRoles() {
        const roles = {};
        for (const item of this.container?.querySelectorAll('.swd-pin') || []) {
            const state = item.querySelector('select').value;
            if (state === 'swc' || state === 'swd') roles[state] = Number(item.querySelector('input').value);
        }
        return roles;
    }

    save() {
        const saved = { driveMode: this.driveMode, readMode: this.readMode, clockHz: this.clockHz };
        for (const item of this.container.querySelectorAll('.swd-pin')) {
            const checkbox = item.querySelector('input');
            saved[checkbox.value] = { scan: checkbox.checked, idle: item.querySelector('select').dataset.idle };
        }
        try {
            localStorage.setItem('swd-pin-states', JSON.stringify(saved));
        } catch (_) { }
    }

    setBusy(busy) {
        this.busy = busy;
        if (this.rateInput) this.rateInput.disabled = busy;
        if (this.readSelect) this.readSelect.disabled = busy;
        if (this.writeSelect) this.writeSelect.disabled = busy;
        for (const item of this.container.querySelectorAll('.swd-pin')) {
            const checkbox = item.querySelector('input');
            checkbox.disabled = busy;
            item.querySelector('select').disabled = busy;
        }
    }

    apply() {
        const serial = this.getSerial();
        if (!serial?.port) return Promise.resolve(false);
        this.revision = (this.revision || 0) + 1;
        const masks = this.masks();
        const driveMode = this.driveMode;
        const readMode = this.readMode || 'pull-up';
        const clockHz = this.clockHz ?? 100000;
        const roles = this.fixedRoles();
        const fixed = Number.isInteger(roles.swc) && Number.isInteger(roles.swd);
        if (!Number.isInteger(clockHz) || clockHz < 500 || clockHz > 500000) {
            return Promise.reject(new Error('SWCLK rate must be between 0.5 and 500 kHz'));
        }
        this.pending = (this.pending || Promise.resolve()).catch(() => { }).then(() => this.applyState(serial, masks, driveMode, readMode, clockHz, fixed ? roles : null));
        return this.pending;
    }

    async applyState(serial, masks, driveMode, readMode, clockHz, roles) {
        this.setBusy(true);
        try {
            await this.invalidate();
            await serial.setGatewayMode('SWD');
            const args = new Uint8Array(roles ? 20 : 18);
            const view = new DataView(args.buffer);
            view.setUint32(0, masks.scan, true);
            view.setUint32(4, masks.high, true);
            view.setUint32(8, masks.low, true);
            args[12] = driveMode === 'push-pull' ? 1 : driveMode === 'open-drain-no-pull' ? 2 : 0;
            args[13] = readMode === 'pull-down' ? 1 : 0;
            view.setUint32(14, clockHz, true);
            if (roles) {
                args[18] = roles.swc;
                args[19] = roles.swd;
            }
            const response = await serial.swdRequest(0x04, args, { timeoutMs: 1500 });
            if (response.status === 1 || response.status === 2) throw new Error('Pin modes, fixed roles and clock rate require updated gateway firmware');
            if (response.status !== 0) throw new Error(`Pin configuration failed: status=${response.status}`);
            this.applied = true;
            return true;
        } finally {
            this.setBusy(false);
        }
    }
}

class Swd {
    constructor(esp, logger) {
        this.esp = esp;
        this.log = logger || (() => { });
        this.memAp = null;
        this.lastIoMask = null;
        this.lastVerbose = false;
    }

    _expectOkStatus(rsp, what) {
        if (!rsp) throw new Error(`${what}: no response`);
        if (rsp.status !== SWD_UART_STATUS_OK) {
            throw new Error(`${what}: status=${rsp.status}`);
        }
    }

    _checkAckOrRetry(ack) {
        if (ack === 1) return { ok: true, retry: false };
        if (ack === 2) return { ok: false, retry: true };
        if (ack === 4) return { ok: false, retry: false, fault: true };
        if (ack === 8) throw new Error('SWD parity mismatch');
        throw new Error(`SWD bad ACK=${ack}`);
    }

    async transfer(ap, write, a23, value = 0) {
        const args = new Uint8Array(8);
        args[0] = ap ? 1 : 0;
        args[1] = write ? 1 : 0;
        args[2] = a23 & 0x03;
        args[3] = 0;
        writeU32LE(args, 4, value >>> 0);
        const rsp = await this.esp.swdRequest(SWD_UART_OP_TRANSFER, args, { timeoutMs: 1500 });
        this._expectOkStatus(rsp, `transfer(ap=${ap ? 1 : 0},write=${write ? 1 : 0},a23=${a23})`);
        const outVal = (rsp.data && rsp.data.length >= 4) ? readU32LE(rsp.data, 0) : 0;
        return { ack: rsp.ack, value: outVal >>> 0, rsp };
    }

    async transferRaw(ap, write, a23, value = 0) {
        const args = new Uint8Array(8);
        args[0] = ap ? 1 : 0;
        args[1] = write ? 1 : 0;
        args[2] = a23 & 0x03;
        args[3] = 0;
        writeU32LE(args, 4, value >>> 0);
        const rsp = await this.esp.swdRequest(SWD_UART_OP_TRANSFER, args, { timeoutMs: 1500 });
        const outVal = (rsp && rsp.data && rsp.data.length >= 4) ? readU32LE(rsp.data, 0) : 0;
        return { ack: rsp ? rsp.ack : 0, value: outVal >>> 0, rsp };
    }

    async dpRead(a23) {
        for (let attempt = 0; attempt < 32; attempt++) {
            const t = await this.transfer(false, false, a23, 0);
            const ar = this._checkAckOrRetry(t.ack);
            if (ar.retry) {
                await new Promise(r => setTimeout(r, 10));
                continue;
            }
            if (ar.fault) {
                throw new Error('SWD FAULT');
            }
            return t.value >>> 0;
        }
        throw new Error('dpRead: too many WAIT retries');
    }

    async dpWrite(a23, value) {
        for (let attempt = 0; attempt < 32; attempt++) {
            const t = await this.transfer(false, true, a23, value >>> 0);
            const ar = this._checkAckOrRetry(t.ack);
            if (ar.retry) {
                await new Promise(r => setTimeout(r, 10));
                continue;
            }
            if (ar.fault) {
                throw new Error('SWD FAULT');
            }
            return true;
        }
        throw new Error('dpWrite: too many WAIT retries');
    }

    async clearStickyErrors(abortPending = false) {
        // Propagate failed ABORTs so initialization cannot report success.
        await this.dpWrite(DP_A23_ABORT, abortPending ? 0x1F : 0x1E);
    }

    async ensurePowerUp(timeoutMs = 800) {
        /* Mirrors Flipper app swd_ensure_powerup():
         * request debug + system power-up and wait for ACKs.
         * Without this, a cold target may return APIDR/APBASE as 0.
         */
        /* DP CTRL/STAT is a banked DP register (A[3:2]=0b01).
         * If firmware previously selected a different DP bank (e.g. to read TARGETID),
         * reading/writing A[3:2]=0b01 here may hit the wrong register.
         * Force SELECT=0 so DPBANKSEL=0 and we really access CTRL/STAT.
         *
         * Note: writing DP SELECT from the host is safe in this firmware; it explicitly
         * invalidates/updates the cached SELECT value.
         */
        await this.dpWrite(DP_A23_SELECT, 0);

        const start = performance.now();
        let last = 0;

        while (performance.now() - start < timeoutMs) {
            last = await this.dpRead(DP_A23_CTRLSTAT);

            const want = (CSYSPWRUPREQ | CDBGPWRUPREQ) >>> 0;
            const haveReq = (last & want) === want;
            const haveAck = ((last & (CSYSPWRUPACK | CDBGPWRUPACK)) === (CSYSPWRUPACK | CDBGPWRUPACK));

            if (!haveReq) {
                // Write request bits only: do not echo sticky/W1C status bits.
                const v = want;
                await this.dpWrite(DP_A23_CTRLSTAT, v);
                await new Promise(r => setTimeout(r, 20));
                continue;
            }

            if (haveAck) {
                return { ok: true, ctrlstat: last >>> 0 };
            }

            await new Promise(r => setTimeout(r, 20));
        }

        throw new Error(`DP power-up timeout: CTRL/STAT=${u32ToHex(last)}`);
    }

    async postDetectInit(opts = {}) {
        const timeoutMs = (opts && opts.timeoutMs) ? (opts.timeoutMs | 0) : 1200;
        const trace = !!(opts && opts.trace);
        const allowRecover = (opts && opts.allowRecover !== undefined) ? !!opts.allowRecover : true;

        const tryOnce = async () => {
            await this.clearStickyErrors(true);
            const pu = await this.ensurePowerUp(timeoutMs);
            this._trace(trace, `DP power-up: ok=${pu.ok ? 1 : 0} CTRL/STAT=${u32ToHex(pu.ctrlstat)}`);
            return pu;
        };

        try {
            return await tryOnce();
        } catch (e) {
            const msg = (e && e.message) ? e.message : String(e);
            if (allowRecover && msg.includes('SWD bad ACK=7')) {
                /* Link looks unstable immediately after detect. Re-run detect once and retry.
                 * IMPORTANT: avoid recursion (recoverLink() normally runs postDetectInit()).
                 */
                await this.recoverLink('post-detect init: bad ACK=7', { skipPostInit: true });
                return await tryOnce();
            }
            throw e;
        }
    }

    async flushRdbuff() {
        /* If a posted AP read completed but the subsequent DP RDBUFF read failed,
         * a stale value can remain pending and shift later reads. Drain it best-effort.
         */
        try {
            await this.transferRaw(false, false, DP_A23_RDBUFF, 0);
            await this.transferRaw(false, false, DP_A23_RDBUFF, 0);
        } catch (e) {
            /* ignore */
        }
    }

    ackName(ack) {
        if (ack === 1) return 'OK';
        if (ack === 2) return 'WAIT';
        if (ack === 4) return 'FAULT';
        if (ack === 8) return 'PARITY';
        return `BAD(${ack})`;
    }

    async dpDebugOnce() {
        const lines = [];

        const dpidr = await this.transferRaw(false, false, DP_A23_ABORT, 0);
        lines.push(`DP DPIDR/IDCODE: ack=${this.ackName(dpidr.ack)} value=${u32ToHex(dpidr.value)}`);

        // SELECT is synchronized with the firmware cache by raw transfers.
        const select = await this.transferRaw(false, true, DP_A23_SELECT, 0);
        lines.push(`DP SELECT(0): ack=${this.ackName(select.ack)}`);
        const ctrl0 = await this.transferRaw(false, false, DP_A23_CTRLSTAT, 0);
        lines.push(`DP CTRL/STAT:   ack=${this.ackName(ctrl0.ack)} value=${u32ToHex(ctrl0.value)}`);
        this.dpStatus = { ack: select.ack === 1 ? ctrl0.ack : select.ack, value: ctrl0.value };
        if (ctrl0.ack === 1 && select.ack === 1) {
            const value = ctrl0.value >>> 0;
            lines.push(`Debug power: request=${!!(value & CDBGPWRUPREQ)} acknowledged=${!!(value & CDBGPWRUPACK)}`);
            lines.push(`System power: request=${!!(value & CSYSPWRUPREQ)} acknowledged=${!!(value & CSYSPWRUPACK)}`);
            lines.push(`Sticky flags: overrun=${!!(value & 2)} compare=${!!(value & 16)} error=${!!(value & 32)} write-data=${!!(value & 128)}`);
        } else {
            lines.push('CTRL/STAT decoding unavailable: bank selection or read failed');
        }

        return lines;
    }

    async detectPins(ioMask, verboseLogs = false) {
        this.lastIoMask = ioMask >>> 0;
        this.lastVerbose = !!verboseLogs;
        const args = new Uint8Array(4);
        writeU32LE(args, 0, ioMask >>> 0);
        const flags = verboseLogs ? SWD_UART_FLAG_VERBOSE_LOG : 0;
        const rsp = await this.esp.swdRequest(SWD_UART_OP_DETECT_PINS, args, { flags, timeoutMs: 15000 });
        this._expectOkStatus(rsp, 'detectPins');
        if (!rsp.data || rsp.data.length < 12) {
            throw new Error('detectPins: short response data');
        }

        const det = {
            swdio_gpio: rsp.swdio_gpio,
            swclk_gpio: rsp.swclk_gpio,
            dpidr: readU32LE(rsp.data, 0),
            targetid: readU32LE(rsp.data, 4),
            dpidr_ok: rsp.data[8] ? true : false,
            targetid_ok: rsp.data[9] ? true : false,
            detected_device: rsp.data[10] ? true : false
        };

        return det;
    }

    async recoverLink(reason = '', opts = {}) {
        if (this.lastIoMask === null || this.lastIoMask === undefined) {
            throw new Error('recoverLink: no previous ioMask');
        }
        const msg = reason ? ` (${reason})` : '';
        this.log(`SWD link recovery: re-detecting pins${msg}...`, 'info');
        const det = await this.detectPins(this.lastIoMask, this.lastVerbose);
        this.log(`SWD link recovery: DPIDR=${u32ToHex(det.dpidr)} ok=${det.dpidr_ok ? 1 : 0}`, 'info');
        if (!det.dpidr_ok || !det.detected_device) {
            throw new Error('SWD link recovery: target not detected');
        }
        const skipPostInit = !!(opts && opts.skipPostInit);
        if (!skipPostInit) {
            await this.postDetectInit({ timeoutMs: 1200, allowRecover: false });
        }
        return det;
    }

    async recoverDp(reason = '') {
        const msg = reason ? ` (${reason})` : '';
        this.log(`SWD DP recovery: clearing sticky state${msg}...`, 'info');
        await this.clearStickyErrors(true);
        await this.ensurePowerUp(1200);
        this.log('SWD DP recovery: complete; pins were not re-detected', 'info');
        return true;
    }

    _trace(enabled, message) {
        if (!enabled) return;
        if (this.log) this.log(message, 'info');
    }

    async deinit() {
        const rsp = await this.esp.swdRequest(SWD_UART_OP_DEINIT, new Uint8Array(0), { timeoutMs: 1000 });
        this._expectOkStatus(rsp, 'deinit');
        this.memAp = null;
        return true;
    }

    async apRead(ap, apOff, single = false) {
        const args = new Uint8Array(4);
        args[0] = ap & 0xFF;
        args[1] = apOff & 0xFF;
        args[2] = 0;
        args[3] = 0;

        const op = single ? SWD_UART_OP_AP_READ_SINGLE : SWD_UART_OP_AP_READ;

        for (let attempt = 0; attempt < 12; attempt++) {
            const rsp = await this.esp.swdRequest(op, args, { timeoutMs: 1500 });
            this._expectOkStatus(rsp, `apRead(ap=${ap},off=0x${apOff.toString(16)})`);
            let ar;
            try {
                ar = this._checkAckOrRetry(rsp.ack);
            } catch (e) {
                const m = (e && e.message) ? e.message : String(e);
                if (m.includes('SWD bad ACK=7') && attempt < 2) {
                    await this.recoverLink('bad ACK=7 during apRead');
                    await this.flushRdbuff();
                    await new Promise(r => setTimeout(r, 20));
                    continue;
                }
                throw e;
            }
            if (ar.retry) {
                await new Promise(r => setTimeout(r, 10));
                continue;
            }
            if (ar.fault) {
                if (attempt < 2) {
                    await this.clearStickyErrors();
                    await new Promise(r => setTimeout(r, 10));
                    continue;
                }
                throw new Error('SWD FAULT');
            }
            if (!rsp.data || rsp.data.length < 4) {
                throw new Error('apRead: short data');
            }
            return readU32LE(rsp.data, 0);
        }
        throw new Error('apRead: too many retries');
    }

    async apReadStrict(ap, apOff, single = false) {
        /* C-like semantics: do not attempt link recovery here.
         * Let the caller decide whether to restart a larger operation.
         */
        const args = new Uint8Array(4);
        args[0] = ap & 0xFF;
        args[1] = apOff & 0xFF;
        args[2] = 0;
        args[3] = 0;

        const op = single ? SWD_UART_OP_AP_READ_SINGLE : SWD_UART_OP_AP_READ;

        for (let attempt = 0; attempt < 64; attempt++) {
            const rsp = await this.esp.swdRequest(op, args, { timeoutMs: 1500 });
            this._expectOkStatus(rsp, `apReadStrict(ap=${ap},off=0x${apOff.toString(16)})`);

            if (rsp.ack === 1) {
                if (!rsp.data || rsp.data.length < 4) {
                    throw new Error('apReadStrict: short data');
                }
                return readU32LE(rsp.data, 0) >>> 0;
            }

            if (rsp.ack === 2) {
                await new Promise(r => setTimeout(r, 10));
                continue;
            }

            if (rsp.ack === 8) {
                throw new Error('SWD parity mismatch');
            }

            /* FAULT, BAD(7), etc -> fail fast */
            throw new Error(`SWD bad ACK=${rsp.ack}`);
        }

        throw new Error('apReadStrict: too many WAIT retries');
    }

    async apWrite(ap, apOff, value) {
        const args = new Uint8Array(8);
        args[0] = ap & 0xFF;
        args[1] = apOff & 0xFF;
        args[2] = 0;
        args[3] = 0;
        writeU32LE(args, 4, value >>> 0);

        for (let attempt = 0; attempt < 12; attempt++) {
            const rsp = await this.esp.swdRequest(SWD_UART_OP_AP_WRITE, args, { timeoutMs: 1500 });
            this._expectOkStatus(rsp, `apWrite(ap=${ap},off=0x${apOff.toString(16)})`);
            let ar;
            try {
                ar = this._checkAckOrRetry(rsp.ack);
            } catch (e) {
                const m = (e && e.message) ? e.message : String(e);
                if (m.includes('SWD bad ACK=7') && attempt < 2) {
                    await this.recoverLink('bad ACK=7 during apWrite');
                    await this.flushRdbuff();
                    await new Promise(r => setTimeout(r, 20));
                    continue;
                }
                throw e;
            }
            if (ar.retry) {
                await new Promise(r => setTimeout(r, 10));
                continue;
            }
            if (ar.fault) {
                if (attempt < 2) {
                    await this.clearStickyErrors();
                    await new Promise(r => setTimeout(r, 10));
                    continue;
                }
                throw new Error('SWD FAULT');
            }
            return true;
        }
        throw new Error('apWrite: too many WAIT retries');
    }

    async apWriteStrict(ap, apOff, value) {
        const args = new Uint8Array(8);
        args[0] = ap & 0xFF;
        args[1] = apOff & 0xFF;
        args[2] = 0;
        args[3] = 0;
        writeU32LE(args, 4, value >>> 0);

        for (let attempt = 0; attempt < 64; attempt++) {
            const rsp = await this.esp.swdRequest(SWD_UART_OP_AP_WRITE, args, { timeoutMs: 1500 });
            this._expectOkStatus(rsp, `apWriteStrict(ap=${ap},off=0x${apOff.toString(16)})`);

            if (rsp.ack === 1) {
                return true;
            }
            if (rsp.ack === 2) {
                await new Promise(r => setTimeout(r, 10));
                continue;
            }
            if (rsp.ack === 8) {
                throw new Error('SWD parity mismatch');
            }
            throw new Error(`SWD bad ACK=${rsp.ack}`);
        }

        throw new Error('apWriteStrict: too many WAIT retries');
    }

    async scanAps(maxAps = 16, opts = {}) {
        const fullScan = !!(opts && opts.fullScan);
        const trace = !!(opts && opts.trace);

        /* Best-effort: clear sticky flags before scanning. */
        await this.clearStickyErrors();

        // AP results are meaningful only after DP initialization succeeds.
        const pu = await this.ensurePowerUp(800);
        this._trace(trace, `DP power-up: ok=1 CTRL/STAT=${u32ToHex(pu.ctrlstat)}`);

        const aps = [];
        let badAckRecoveries = 0;
        let consecutiveFailures = 0;
        let consecutiveEmptyIdr = 0;
        for (let ap = 0; ap < maxAps; ap++) {
            this._trace(trace, `AP scan: probing AP${ap}...`);
            let idr = 0;
            try {
                idr = await this.apReadStrict(ap, AP_IDR, true);
                consecutiveFailures = 0;
                this._trace(trace, `AP${ap}: IDR=${u32ToHex(idr)}`);
            } catch (e) {
                const msg = (e && e.message) ? e.message : String(e);
                if (msg.includes('SWD bad ACK=7')) {
                    /* If the line is unstable (invalid SWD ACK), don't loop forever.
                     * Try a bounded number of recoveries, then abort the scan.
                     */
                    if (badAckRecoveries >= 2) {
                        throw new Error('AP scan aborted: repeated SWD bad ACK=7 (link unstable)');
                    }
                    badAckRecoveries++;
                    this._trace(trace, `AP${ap}: bad ACK=7, attempting link recovery (${badAckRecoveries}/2) then retry...`);
                    await this.recoverLink('bad ACK=7 during scan');
                    await new Promise(r => setTimeout(r, 80));
                    /* retry same AP index */
                    ap--;
                    continue;
                }
                consecutiveFailures++;
                this._trace(trace, `AP${ap}: read failed (${msg}), consecutiveFailures=${consecutiveFailures}`);
                /* If nothing was found yet, bail out early to avoid destabilizing the link by probing
                 * many non-existent APSEL values.
                 */
                if (!fullScan && aps.length === 0 && consecutiveFailures >= 4) {
                    throw e;
                }
                /* If we already found at least one AP, stop after a few consecutive failures. */
                if (!fullScan && aps.length > 0 && consecutiveFailures >= 3) {
                    break;
                }
                continue;
            }
            if (idr === 0 || idr === 0xFFFFFFFF) {
                this._trace(trace, `AP${ap}: IDR ignored (${u32ToHex(idr)})`);

                if (!fullScan && aps.length > 0) {
                    consecutiveEmptyIdr++;
                    if (consecutiveEmptyIdr >= 8) {
                        this._trace(trace, `AP scan: stopping after ${consecutiveEmptyIdr} empty IDRs`);
                        break;
                    }
                }
                continue;
            }

            consecutiveEmptyIdr = 0;
            const info = decodeApIdr(idr);

            let base = null;
            try {
                base = await this.apReadStrict(ap, AP_BASE, true);
                this._trace(trace, `AP${ap}: BASE=${u32ToHex(base)}`);
            } catch (e) {
                const msg = (e && e.message) ? e.message : String(e);
                this._trace(trace, `AP${ap}: BASE read failed (${msg})`);
                base = null;
            }
            const entry = { ap, idr, info, base };
            aps.push(entry);
        }

        const mem = aps.find(a => a.info && a.info.ap_class === 0x08) || null;
        this.memAp = mem ? mem.ap : null;
        return { aps, memAp: this.memAp };
    }

    async memRead32Via(ap, address) {
        const csw = 0x23000002;
        const base = address >>> 0;

        for (let attempt = 0; attempt < 3; attempt++) {
            try {
                await this.apWriteStrict(ap, MEMAP_CSW, csw);
                await this.apWriteStrict(ap, MEMAP_TAR, base);
                // Firmware already completes AP reads through DP RDBUFF.
                const v = await this.apReadStrict(ap, MEMAP_DRW, true);
                return v >>> 0;
            } catch (e) {
                const msg = (e && e.message) ? e.message : String(e);
                if (attempt < 2) {
                    if (msg.includes('SWD bad ACK=4') || msg.includes('SWD FAULT')) {
                        await this.recoverDp(`MEM-AP read fault (${msg})`);
                    } else {
                        await this.recoverLink(`memRead32Via restart (${msg})`);
                    }
                    await this.flushRdbuff();
                    await new Promise(r => setTimeout(r, 20));
                    continue;
                }
                throw e;
            }
        }

        throw new Error('memRead32Via: unreachable');
    }

    async memReadBlock32Via(ap, address, words) {
        const base = address >>> 0;
        const count = words >>> 0;
        if ((base & 3) !== 0) throw new Error('memReadBlock32Via: unaligned address');
        if (count === 0) return [];
        // Each firmware AP helper returns the completed result, not a dummy.
        // Use the guaranteed 1 KiB TAR window even on APs with larger windows.
        await this.apWriteStrict(ap, MEMAP_CSW, 0x23000012);
        const result = [];
        for (let i = 0; i < count; i++) {
            const addr = (base + i * 4) >>> 0;
            if (i === 0 || (addr & 0x3FF) === 0) {
                await this.apWriteStrict(ap, MEMAP_TAR, addr);
            }
            result.push((await this.apReadStrict(ap, MEMAP_DRW, true)) >>> 0);
        }
        return result;
    }

    async memWriteBlock32Via(ap, address, words) {
        const csw = 0x23000012;
        if ((address & 3) !== 0) throw new Error('memWriteBlock32Via: unaligned address');
        if (!words.length) return true;
        await this.apWriteStrict(ap, MEMAP_CSW, csw);
        await this.apWriteStrict(ap, MEMAP_TAR, address >>> 0);
        for (let i = 0; i < words.length; i++) {
            const current = ((address >>> 0) + i * 4) >>> 0;
            if (i > 0 && (current & 0x3FF) === 0) {
                await this.apWriteStrict(ap, MEMAP_TAR, current);
            }
            await this.apWriteStrict(ap, MEMAP_DRW, (words[i] >>> 0));
            await this.completeMemWrite();
        }
        return true;
    }

    async memWriteBlock16Via(ap, address, halfwords) {
        const csw = 0x23000011;
        const addr = address >>> 0;
        if ((addr & 1) !== 0) throw new Error('memWriteBlock16Via: unaligned address');
        if (!halfwords.length) return true;
        await this.apWriteStrict(ap, MEMAP_CSW, csw);
        await this.apWriteStrict(ap, MEMAP_TAR, addr);
        for (let i = 0; i < halfwords.length; i++) {
            const current = ((address >>> 0) + i * 2) >>> 0;
            if (i > 0 && (current & 0x3FF) === 0) {
                await this.apWriteStrict(ap, MEMAP_TAR, current);
            }
            await this.apWriteStrict(ap, MEMAP_DRW, ((halfwords[i] & 0xFFFF) << ((current & 2) * 8)) >>> 0);
            await this.completeMemWrite();
        }
        return true;
    }

    async memWriteBlock8Via(ap, address, bytes) {
        const csw = 0x23000010;
        const addr = address >>> 0;
        if (!bytes.length) return true;
        await this.apWriteStrict(ap, MEMAP_CSW, csw);
        await this.apWriteStrict(ap, MEMAP_TAR, addr);
        for (let i = 0; i < bytes.length; i++) {
            const current = ((address >>> 0) + i * 1) >>> 0;
            if (i > 0 && (current & 0x3FF) === 0) {
                await this.apWriteStrict(ap, MEMAP_TAR, current);
            }
            await this.apWriteStrict(ap, MEMAP_DRW, ((bytes[i] & 0xFF) << ((current & 3) * 8)) >>> 0);
            await this.completeMemWrite();
        }
        return true;
    }

    async completeMemWrite() {
        try {
            await this.dpRead(DP_A23_RDBUFF);
        } catch (error) {
            try {
                await this.clearStickyErrors(true);
            } catch (recoveryError) {
                this.log(`SWD write cleanup failed: ${recoveryError.message}`, 'error');
            }
            throw error;
        }
    }

    async memWrite32Via(ap, address, value) {
        const csw = 0x23000002;
        const base = address >>> 0;
        if ((base & 3) !== 0) throw new Error('memWrite32Via: unaligned address');
        await this.apWriteStrict(ap, MEMAP_CSW, csw);
        await this.apWriteStrict(ap, MEMAP_TAR, base);
        await this.apWriteStrict(ap, MEMAP_DRW, value >>> 0);
        await this.completeMemWrite();
        return true;
    }

    async coreHaltVia(ap) {
        const dhcsr = (SCS_DHCSR_KEY | SCS_DHCSR_C_HALT | SCS_DHCSR_C_DEBUGEN) >>> 0;
        await this.memWrite32Via(ap, SCS_DHCSR, dhcsr);
        const v = await this.memRead32Via(ap, SCS_DHCSR);
        return { dhcsr: v >>> 0, halted: !!(v & SCS_DHCSR_S_HALT) };
    }

    async coreContinueVia(ap) {
        const v = await this.memRead32Via(ap, SCS_DHCSR);
        if (!(v & SCS_DHCSR_S_HALT)) {
            return { dhcsr: v >>> 0, continued: false, reason: 'not halted' };
        }
        await this.memWrite32Via(ap, SCS_DHCSR, SCS_DHCSR_KEY);
        const v2 = await this.memRead32Via(ap, SCS_DHCSR);
        return { dhcsr: v2 >>> 0, continued: true };
    }

    async coreStepVia(ap) {
        const v = await this.memRead32Via(ap, SCS_DHCSR);
        if (!(v & SCS_DHCSR_S_HALT)) {
            return { dhcsr: v >>> 0, stepped: false, reason: 'not halted' };
        }
        const dhcsr = (SCS_DHCSR_KEY | SCS_DHCSR_C_STEP | SCS_DHCSR_C_MASKINTS | SCS_DHCSR_C_DEBUGEN) >>> 0;
        await this.memWrite32Via(ap, SCS_DHCSR, dhcsr);
        const v2 = await this.memRead32Via(ap, SCS_DHCSR);
        return { dhcsr: v2 >>> 0, stepped: true };
    }

    async coreCpuidVia(ap) {
        const v = await this.memRead32Via(ap, SCS_CPUID);
        return v >>> 0;
    }

    async _coreWaitRegReadyVia(ap, timeoutMs = 200) {
        const start = performance.now();
        while (performance.now() - start < timeoutMs) {
            const dhcsr = await this.memRead32Via(ap, SCS_DHCSR);
            if (dhcsr & SCS_DHCSR_S_REGRDY) return true;
            await new Promise(r => setTimeout(r, 5));
        }
        return false;
    }

    async coreRegReadVia(ap, regsel) {
        await this.memWrite32Via(ap, SCS_DCRSR, (SCS_DCRSR_RD | (regsel & 0xFFFF)) >>> 0);
        await this._coreWaitRegReadyVia(ap, 250);
        const v = await this.memRead32Via(ap, SCS_DCRDR);
        return v >>> 0;
    }

    async coreRegWriteVia(ap, regsel, value) {
        await this.memWrite32Via(ap, SCS_DCRDR, value >>> 0);
        await this.memWrite32Via(ap, SCS_DCRSR, (SCS_DCRSR_WR | (regsel & 0xFFFF)) >>> 0);
        await this._coreWaitRegReadyVia(ap, 250);
        return true;
    }

    async adiGetPidrVia(ap, base) {
        const b = base >>> 0;
        const pidrs = [];
        for (const off of CS_PIDR_OFFS) {
            pidrs.push(await this.memRead32Via(ap, (b + off) >>> 0));
        }

        const designer = ((((pidrs[4] & 0x0F) << 7) | ((pidrs[2] & 0x07) << 4) | ((pidrs[1] >> 4) & 0x0F)) & 0x3FF) >>> 0;
        const part = (((pidrs[0] & 0xFF) | ((pidrs[1] & 0x0F) << 8)) & 0xFFFF) >>> 0;
        const revand = (((pidrs[3] >> 4) & 0x0F) & 0xFF) >>> 0;
        const cmod = ((pidrs[3] & 0x0F) & 0xFF) >>> 0;
        const revision = (((pidrs[2] >> 4) & 0x0F) & 0xFF) >>> 0;
        const size = (((pidrs[2] >> 4) & 0x0F) & 0xFF) >>> 0;

        return { designer, part, revision, cmod, revand, size };
    }

    async adiGetClassVia(ap, base) {
        const b = base >>> 0;
        const cidrs = [];
        for (const off of CS_CIDR_OFFS) {
            cidrs.push(await this.memRead32Via(ap, (b + off) >>> 0));
        }

        if ((cidrs[0] & 0xFF) !== 0x0D) return null;
        if ((cidrs[1] & 0x0F) !== 0x00) return null;
        if ((cidrs[2] & 0xFF) !== 0x05) return null;
        if ((cidrs[3] & 0xFF) !== 0xB1) return null;

        return ((cidrs[1] >> 4) & 0x0F) >>> 0;
    }

    async adiRomtableEntryCountVia(ap, base) {
        const b = base >>> 0;
        let count = 0;
        for (let pos = 0; pos < 960; pos++) {
            const entry = await this.memRead32Via(ap, (b + (pos * 4)) >>> 0);
            if ((entry & 1) === 0) break;
            if (entry & 0x00000FFC) break;
            count++;
        }
        return count >>> 0;
    }

    async adiRomtableGetVia(ap, base, pos) {
        const entry = await this.memRead32Via(ap, (base + ((pos >>> 0) * 4)) >>> 0);
        return (base + (entry & 0xFFFFF000)) >>> 0;
    }

    async memRead32(address) {
        if (this.memAp === null || this.memAp === undefined) {
            throw new Error('No MEM-AP selected');
        }
        const ap = this.memAp;
        const csw = 0x23000002;
        await this.apWrite(ap, MEMAP_CSW, csw);
        await this.apWrite(ap, MEMAP_TAR, address >>> 0);
        const v = await this.apRead(ap, MEMAP_DRW, false);
        return v >>> 0;
    }

    async memReadBlock32(address, words) {
        if (this.memAp === null || this.memAp === undefined) {
            throw new Error('No MEM-AP selected');
        }
        return await this.memReadBlock32Via(this.memAp, address >>> 0, words >>> 0);
    }
}
