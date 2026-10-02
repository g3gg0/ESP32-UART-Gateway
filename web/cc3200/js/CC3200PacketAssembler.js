class CC3200PacketAssembler {
    constructor(expectedLen = null) {
        this.expectedLen = expectedLen;
        this.buffer = new Uint8Array(0);

        this.debug = false;

        this._dbg(`ctor(expectedLen=${this.expectedLen})`);
    }

    _dbg(msg) {
        if (!this.debug) return;
        try {
            logToConsole(`CC3200PacketAssembler: ${msg}`, 'info');
        } catch (e) {
            /* ignore */
        }
    }

    appendBuffer(a, b) {
        if (!a || a.length === 0) return new Uint8Array(b);
        const out = new Uint8Array(a.length + b.length);
        out.set(a, 0);
        out.set(b, a.length);
        return out;
    }

    push(data) {
        this._dbg(`push(data_len=${data ? data.length : 0}) buffer_len=${this.buffer.length}`);
        if (!data || data.length === 0) {
            this._dbg('no data, return null');
            return null;
        }
        this.buffer = this.appendBuffer(this.buffer, data);

        if (this.debug) {
            const peekLen = Math.min(16, this.buffer.length);
            const peek = Array.from(this.buffer.slice(0, peekLen)).map(b => b.toString(16).padStart(2, '0').toUpperCase()).join(' ');
            this._dbg(`buffer_append -> buffer_len=${this.buffer.length} peek[0..${peekLen - 1}]=${peek}${this.buffer.length > peekLen ? ' ...' : ''}`);
        }

        if (this.buffer.length < 3) {
            this._dbg('need header(3), return null');
            return null;
        }

        const len = (this.buffer[0] << 8) | this.buffer[1];
        const csum = this.buffer[2];
        const dataLen = len - 2;
        this._dbg(`hdr: len=0x${len.toString(16).padStart(4, '0')} (${len}) csum=0x${csum.toString(16).padStart(2, '0')} dataLen=${dataLen}`);
        if (dataLen < 0) {
            this._dbg('dataLen < 0, drop 1 byte and resync');
            this.buffer = this.buffer.slice(1);
            return null;
        }

        const totalLen = 3 + dataLen;
        if (this.buffer.length < totalLen) {
            this._dbg(`need totalLen=${totalLen}, have=${this.buffer.length}, return null`);
            return null;
        }

        const payload = this.buffer.slice(3, 3 + dataLen);
        let sum = 0;
        for (let i = 0; i < payload.length; i++) {
            sum = (sum + payload[i]) & 0xFF;
        }
        this._dbg(`checksum calc=0x${sum.toString(16).padStart(2, '0')} payload_len=${payload.length}`);
        if (sum !== csum) {
            this._dbg(`checksum mismatch got=0x${csum.toString(16).padStart(2, '0')} calc=0x${sum.toString(16).padStart(2, '0')}`);
            throw new Error(`CC3200 RX csum failed (got 0x${csum.toString(16).padStart(2, '0')}, calc 0x${sum.toString(16).padStart(2, '0')})`);
        }

        const remainder = this.buffer.slice(totalLen);
        this.buffer = remainder;
        this._dbg(`packet ok, consumed=${totalLen}, remainder_len=${this.buffer.length}`);

        if (this.expectedLen !== null && payload.length !== this.expectedLen) {
            this._dbg(`expectedLen mismatch expected=${this.expectedLen} got=${payload.length}`);
            logToConsole(`CC3200 RX packet length mismatch (expected ${this.expectedLen}, got ${payload.length})`, 'info');
        }

        return payload;
    }
}
