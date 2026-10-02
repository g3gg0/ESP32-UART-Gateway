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
        } catch (_) {}
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
        } catch (_) {}
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
        this.pending = (this.pending || Promise.resolve()).catch(() => {}).then(() => this.applyState(serial, masks, driveMode, readMode, clockHz, fixed ? roles : null));
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