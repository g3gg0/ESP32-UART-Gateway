class HexEditor {
    constructor(opts = {}) {
        this.bytesPerRow = opts.bytesPerRow || 16;
        this.baseAddr = 0;
        this.data = new Uint8Array(0);
        this.onChange = opts.onChange || null;
        this.onModeChange = opts.onModeChange || null;
        this.onReadRange = opts.onReadRange || null;
        this.onReadError = opts.onReadError || null;
        this._loadingRange = false;
        this._maxWindowBytes = 16384;
        this.editMode = 'byte';
        /* Dirty tracking is per-byte. UI/write-back can group bytes into
         * 8/16/32-bit regions depending on selected width.
         */
        this._dirtyBytes = new Uint8Array(0);
        this._dirtyCount = 0;
        this._dirtyRegionSize = this._editSizeBytes(); /* bytes: 1,2,4 */
        this._root = null;
        this._view = null;
        this._selIndex = null;
        this._selKind = 'hex';
        this._selNibble = null;
        this._overlay = null;
        this._boundKeyDown = (e) => this._onKeyDown(e);
        this._boundClick = (e) => this._onClick(e);
        this._boundDblClick = (e) => this._onDblClick(e);
    }

    _targetCellEl(rawTarget) {
        /* Some browsers report a Text node as the event target when double-clicking.
         * Normalize to the nearest element carrying data-kind/data-index.
         */
        if (!rawTarget) return null;
        let el = rawTarget;
        try {
            if (el.nodeType === 3) el = el.parentElement; /* TEXT_NODE */
        } catch (e) {
            /* ignore */
        }
        if (!el) return null;
        if (el.dataset && el.dataset.kind && el.dataset.index !== undefined) return el;
        try {
            if (el.closest) {
                return el.closest('[data-kind][data-index]');
            }
        } catch (e) {
            /* ignore */
        }
        return null;
    }

    render(containerEl) {
        if (!containerEl) return;
        containerEl.innerHTML = '';

        const root = document.createElement('div');
        root.className = 'hex-editor';
        root.tabIndex = 0;

        const toolbar = document.createElement('div');
        toolbar.className = 'hex-toolbar';

        const title = document.createElement('div');
        title.style.fontWeight = '800';
        title.textContent = 'Hex viewer';

        const modeLabel = document.createElement('label');
        modeLabel.textContent = 'Edit:';
        const modeSel = document.createElement('select');
        const addOpt = (v, t) => {
            const o = document.createElement('option');
            o.value = v;
            o.textContent = t;
            modeSel.appendChild(o);
        };
        addOpt('byte', 'byte');
        addOpt('word', 'word (16-bit)');
        addOpt('dword', 'dword (32-bit)');
        modeSel.value = this.editMode;
        modeSel.onchange = () => {
            this.editMode = modeSel.value;
            this.setDirtyRegionSize(this._editSizeBytes());
            if (this.onModeChange) {
                try { this.onModeChange({ editMode: this.editMode, widthBytes: this._editSizeBytes() }); } catch (e) { /* ignore */ }
            }
        };
        modeLabel.appendChild(modeSel);

        const meta = document.createElement('div');
        meta.style.marginLeft = 'auto';
        meta.style.color = '#9ca3af';
        meta.textContent = 'Tip: click to select, double-click to edit, type in ASCII view';

        toolbar.appendChild(title);
        toolbar.appendChild(modeLabel);
        toolbar.appendChild(meta);

        const view = document.createElement('div');
        view.className = 'hex-view';
        view.addEventListener('scroll', () => {
            if (view.scrollTop > 0 && view.scrollTop + view.clientHeight >= view.scrollHeight - 2) {
                this._loadMore(1);
            }
        });
        view.addEventListener('wheel', (event) => {
            const direction = Math.sign(event.deltaY);
            const atEdge = direction < 0 ? view.scrollTop <= 0 :
                view.scrollTop + view.clientHeight >= view.scrollHeight - 2;
            if (direction && atEdge && this.data.length && this.onReadRange) {
                event.preventDefault();
                this._loadMore(direction);
            }
        }, { passive: false });

        root.appendChild(toolbar);
        root.appendChild(view);

        root.addEventListener('keydown', this._boundKeyDown);
        root.addEventListener('click', this._boundClick);
        root.addEventListener('dblclick', this._boundDblClick);

        this._root = root;
        this._view = view;
        containerEl.appendChild(root);
        this._renderView();
    }

    setData(bytes, baseAddr = 0) {
        this.data = bytes ? new Uint8Array(bytes) : new Uint8Array(0);
        this.baseAddr = baseAddr >>> 0;
        this._selIndex = null;
        this._selKind = 'hex';
        this._selNibble = null;
        this._dirtyBytes = new Uint8Array(this.data.length);
        this._dirtyCount = 0;
        this._renderView();
    }

    async _loadMore(direction) {
        if (this._loadingRange || !this.onReadRange || !this.data.length) return;
        if (this.isDirty()) {
            if (this.onReadError) this.onReadError(new Error('Write back pending hex edits before loading more memory'));
            return;
        }
        const end = this.baseAddr + this.data.length;
        const length = Math.min(1024, direction < 0 ? this.baseAddr : 0x100000000 - end);
        if (length <= 0) return;
        const address = direction < 0 ? this.baseAddr - length : end;
        const originalData = this.data;
        const originalBase = this.baseAddr;
        const view = this._view;
        const scrollTop = view ? view.scrollTop : 0;
        const rowHeight = view && view.querySelector('.hex-row') ?
            view.querySelector('.hex-row').getBoundingClientRect().height : 19.2;
        this._loadingRange = true;
        try {
            const bytes = new Uint8Array(await this.onReadRange(address, length));
            if (this.data !== originalData || this.baseAddr !== originalBase || this.isDirty()) return;
            if (bytes.length !== length) throw new Error('Short memory read while scrolling');
            const joined = new Uint8Array(this.data.length + length);
            joined.set(direction < 0 ? bytes : this.data);
            joined.set(direction < 0 ? this.data : bytes, direction < 0 ? length : this.data.length);
            const excess = Math.max(0, joined.length - this._maxWindowBytes);
            const removedTop = direction > 0 ? excess : 0;
            this.data = joined.slice(removedTop, joined.length - (direction < 0 ? excess : 0));
            this.baseAddr = address - (direction > 0 ? this.data.length - length : 0);
            this._dirtyBytes = new Uint8Array(this.data.length);
            this._selIndex = null;
            this._selNibble = null;
            this._renderView();
            if (view) view.scrollTop = scrollTop + ((direction < 0 ? length : -removedTop) / this.bytesPerRow) * rowHeight;
        } catch (error) {
            if (this.onReadError) this.onReadError(error);
        } finally {
            this._loadingRange = false;
        }
    }

    isDirty() {
        return this._dirtyCount > 0;
    }

    clearDirty() {
        if (this._dirtyBytes && this._dirtyBytes.length) {
            this._dirtyBytes.fill(0);
        }
        this._dirtyCount = 0;
    }

    markDirty(startByte, lengthBytes) {
        const s = Math.max(0, startByte | 0);
        const e = Math.min(this.data.length, (s + (lengthBytes | 0)) | 0);
        if (e <= s) return;
        if (!this._dirtyBytes || this._dirtyBytes.length !== this.data.length) {
            this._dirtyBytes = new Uint8Array(this.data.length);
            this._dirtyCount = 0;
        }
        for (let i = s; i < e; i++) {
            if (this._dirtyBytes[i] === 0) {
                this._dirtyBytes[i] = 1;
                this._dirtyCount++;
            }
        }
    }

    setDirtyRegionSize(bytes) {
        const v = bytes | 0;
        if (v === 1 || v === 2 || v === 4) {
            this._dirtyRegionSize = v;
            this._renderView();
        }
    }

    getDirtyRegionSize() {
        return this._dirtyRegionSize | 0;
    }

    _isRegionDirty(byteIndex, regionSize) {
        if (!this._dirtyBytes || this._dirtyCount === 0) return false;
        const rs = (regionSize | 0) || 1;
        const start = (byteIndex & ~(rs - 1)) | 0;
        const end = Math.min(this.data.length, start + rs);
        for (let i = start; i < end; i++) {
            if (this._dirtyBytes[i]) return true;
        }
        return false;
    }

    getDirtyRanges(regionSizeBytes) {
        const rs = (regionSizeBytes | 0) || 1;
        if (!(rs === 1 || rs === 2 || rs === 4)) throw new Error('getDirtyRanges: bad region size');
        if (!this._dirtyBytes || this._dirtyCount === 0) return [];

        const regions = new Set();
        for (let i = 0; i < this._dirtyBytes.length; i++) {
            if (!this._dirtyBytes[i]) continue;
            regions.add((i & ~(rs - 1)) | 0);
        }

        if (regions.size === 0) return [];
        const starts = Array.from(regions).sort((a, b) => a - b);
        const ranges = [];
        let start = starts[0];
        let prev = starts[0];
        for (let i = 1; i < starts.length; i++) {
            const v = starts[i];
            if (v === (prev + rs)) {
                prev = v;
                continue;
            }
            ranges.push({ start, length: (prev - start + rs) });
            start = v;
            prev = v;
        }
        ranges.push({ start, length: (prev - start + rs) });
        return ranges;
    }

    countDirtyRegions(regionSizeBytes) {
        const rs = (regionSizeBytes | 0) || 1;
        const ranges = this.getDirtyRanges(rs);
        let total = 0;
        for (const rg of ranges) total += (rg.length / rs) | 0;
        return total | 0;
    }

    getData() {
        return new Uint8Array(this.data);
    }

    _renderView() {
        if (!this._view) return;
        const view = this._view;
        view.innerHTML = '';

        if (!this.data || this.data.length === 0) {
            const empty = document.createElement('div');
            empty.style.color = '#9ca3af';
            empty.textContent = '(no data)';
            view.appendChild(empty);
            return;
        }

        const bpr = this.bytesPerRow;
        const rows = Math.ceil(this.data.length / bpr);
        for (let r = 0; r < rows; r++) {
            const rowStart = r * bpr;
            const rowEnd = Math.min(this.data.length, rowStart + bpr);

            const row = document.createElement('div');
            row.className = 'hex-row';

            const off = document.createElement('div');
            off.className = 'hex-off';
            off.textContent = u32ToHex((this.baseAddr + rowStart) >>> 0);

            const hex = document.createElement('div');
            hex.className = 'hex-bytes';
            const ascii = document.createElement('div');
            ascii.className = 'hex-ascii';

            for (let i = 0; i < bpr; i++) {
                const idx = rowStart + i;

                if (i === 8) {
                    const gap = document.createTextNode(' ');
                    hex.appendChild(gap);
                }

                if (idx < rowEnd) {
                    const b = this.data[idx] & 0xFF;
                    const dirty = this._isRegionDirty(idx, this._dirtyRegionSize);

                    const hx = bytesToHex2(b);

                    const s = document.createElement('span');
                    s.className = 'byte';
                    s.dataset.kind = 'hex';
                    s.dataset.index = String(idx);
                    const n0 = document.createElement('span');
                    n0.className = 'nib';
                    n0.dataset.kind = 'hex';
                    n0.dataset.index = String(idx);
                    n0.dataset.nib = '0';
                    n0.textContent = hx[0];

                    const n1 = document.createElement('span');
                    n1.className = 'nib';
                    n1.dataset.kind = 'hex';
                    n1.dataset.index = String(idx);
                    n1.dataset.nib = '1';
                    n1.textContent = hx[1];

                    if (this._selIndex === idx && this._selKind === 'hex' && (this._selNibble === 0 || this._selNibble === 1)) {
                        if (this._selNibble === 0) n0.classList.add('nsel');
                        if (this._selNibble === 1) n1.classList.add('nsel');
                    }

                    s.appendChild(n0);
                    s.appendChild(n1);
                    if (dirty) s.classList.add('dirty');
                    if (this._selIndex === idx && this._selKind === 'hex') s.classList.add('sel');
                    hex.appendChild(s);
                    hex.appendChild(document.createTextNode(' '));

                    const a = document.createElement('span');
                    a.className = 'ascii';
                    a.dataset.kind = 'ascii';
                    a.dataset.index = String(idx);
                    a.textContent = isPrintableAscii(b) ? String.fromCharCode(b) : '.';
                    if (dirty) a.classList.add('dirty');
                    if (this._selIndex === idx && this._selKind === 'ascii') a.classList.add('sel');
                    ascii.appendChild(a);
                } else {
                    hex.appendChild(document.createTextNode('   '));
                    ascii.appendChild(document.createTextNode(' '));
                }
            }

            row.appendChild(off);
            row.appendChild(hex);
            row.appendChild(ascii);
            view.appendChild(row);
        }
    }

    _select(index, kind) {
        if (index === null || index === undefined) return;
        const idx = index | 0;
        if (idx < 0 || idx >= this.data.length) return;
        this._selIndex = idx;
        this._selKind = kind || 'hex';
        this._selNibble = null;
        this._renderView();
    }

    _nextIndex(delta) {
        if (this._selIndex === null || this._selIndex === undefined) return;
        const n = Math.max(0, Math.min(this.data.length - 1, (this._selIndex + delta) | 0));
        this._selIndex = n;
        this._selNibble = null;
        this._renderView();
    }

    _onClick(e) {
        /* When double-clicking, the browser fires click twice then dblclick.
         * If we re-render on the 2nd click, we detach the target element and the
         * dblclick overlay positioning breaks. Let dblclick handle it.
         */
        if (e && e.detail && e.detail > 1) return;
        const t = this._targetCellEl(e.target);
        if (!t || !t.dataset) return;
        const kind = t.dataset.kind;
        const idx = t.dataset.index;
        if (kind && idx !== undefined) {
            this._select(parseInt(idx, 10), kind);
            if (this._root) this._root.focus();
        }
    }

    _onDblClick(e) {
        let t = this._targetCellEl(e.target);

        /* Some dblclicks land on whitespace/text nodes between spans.
         * Try resolving the actual element under the pointer.
         */
        try {
            if ((!t || !t.dataset) && e && typeof e.clientX === 'number' && typeof e.clientY === 'number') {
                const under = document.elementFromPoint(e.clientX, e.clientY);
                t = this._targetCellEl(under);
            }
        } catch (err) {
            /* ignore */
        }

        /* Final fallback: start edit at current selection. */
        if (!t || !t.dataset) {
            if (this._selIndex === null || this._selIndex === undefined) return;
            try {
                const q = this._view ? this._view.querySelector(`[data-kind="${this._selKind}"][data-index="${this._selIndex}"]`) : null;
                if (q && q.dataset) t = q;
            } catch (err) {
                /* ignore */
            }
        }

        if (!t || !t.dataset) return;
        const kind = t.dataset.kind;
        const idxStr = t.dataset.index;
        if (!kind || idxStr === undefined) return;
        const idx = parseInt(idxStr, 10);

        /* IMPORTANT: do not call _select() here.
         * _select() re-renders the view, which detaches the clicked element and breaks
         * getBoundingClientRect() positioning for the overlay input.
         */
        this._selIndex = idx;
        this._selKind = kind;

        if (kind === 'hex') {
            /* Inline nibble overwrite editing: start at first nibble. */
            this._selNibble = 0;
            this._renderView();
            return;
        }

        this._selNibble = null;
        if (kind === 'ascii') {
            this._beginAsciiEditAtTarget(t, idx);
        }
    }

    _editSizeBytes() {
        if (this.editMode === 'dword') return 4;
        if (this.editMode === 'word') return 2;
        return 1;
    }

    getEditSizeBytes() {
        return this._editSizeBytes() | 0;
    }

    _alignedIndex(index) {
        const size = this._editSizeBytes();
        if (size <= 1) return index;
        return index & ~(size - 1);
    }

    _beginHexEditAtTarget(targetEl, index) {
        const start = this._alignedIndex(index);
        const size = this._editSizeBytes();
        if (start < 0 || (start + size) > this.data.length) return;

        const rect = targetEl.getBoundingClientRect();
        const overlay = document.createElement('div');
        overlay.className = 'hex-input-overlay';
        overlay.style.left = `${Math.max(8, rect.left)}px`;
        overlay.style.top = `${Math.max(8, rect.top - 2)}px`;

        const input = document.createElement('input');
        const current = [];
        for (let i = 0; i < size; i++) current.push(bytesToHex2(this.data[start + i]));
        input.value = current.join('');
        input.maxLength = size * 2;
        input.placeholder = (size === 1) ? '00' : (size === 2) ? '0000' : '00000000';

        const finish = (commit) => {
            this._endOverlay();
            if (!commit) return;
            const text = (input.value || '').trim();
            if (!/^[0-9a-fA-F]+$/.test(text) || text.length !== (size * 2)) return;
            for (let i = 0; i < size; i++) {
                const byteText = text.substr(i * 2, 2);
                this.data[start + i] = parseInt(byteText, 16) & 0xFF;
            }
            this.markDirty(start, size);
            if (this.onChange) this.onChange({ start, length: size, kind: 'hex' });
            this._select(start, 'hex');
        };

        input.addEventListener('keydown', (ev) => {
            if (ev.key === 'Enter') {
                ev.preventDefault();
                finish(true);
            } else if (ev.key === 'Escape') {
                ev.preventDefault();
                finish(false);
            }
        });

        input.addEventListener('blur', () => finish(true));

        overlay.appendChild(input);
        document.body.appendChild(overlay);
        this._overlay = overlay;
        input.focus();
        input.select();
    }

    _beginAsciiEditAtTarget(targetEl, index) {
        if (index < 0 || index >= this.data.length) return;
        const rect = targetEl.getBoundingClientRect();
        const overlay = document.createElement('div');
        overlay.className = 'hex-input-overlay';
        overlay.style.left = `${Math.max(8, rect.left)}px`;
        overlay.style.top = `${Math.max(8, rect.top - 2)}px`;

        const input = document.createElement('input');
        input.value = isPrintableAscii(this.data[index]) ? String.fromCharCode(this.data[index]) : '';
        input.maxLength = 1;
        input.placeholder = '.';
        input.style.width = '40px';

        const finish = (commit) => {
            this._endOverlay();
            if (!commit) return;
            const v = (input.value || '');
            if (!v.length) return;
            const code = v.charCodeAt(0) & 0xFF;
            this.data[index] = code;
            this.markDirty(index, 1);
            if (this.onChange) this.onChange({ start: index, length: 1, kind: 'ascii' });
            this._select(Math.min(this.data.length - 1, index + 1), 'ascii');
        };

        input.addEventListener('keydown', (ev) => {
            if (ev.key === 'Enter') {
                ev.preventDefault();
                finish(true);
            } else if (ev.key === 'Escape') {
                ev.preventDefault();
                finish(false);
            }
        });
        input.addEventListener('blur', () => finish(true));

        overlay.appendChild(input);
        document.body.appendChild(overlay);
        this._overlay = overlay;
        input.focus();
        input.select();
    }

    _endOverlay() {
        if (!this._overlay) return;
        try {
            this._overlay.remove();
        } catch (e) {
            /* ignore */
        }
        this._overlay = null;
        if (this._root) this._root.focus();
        this._renderView();
    }

    _onKeyDown(e) {
        if (this._overlay) return;
        if (this._selIndex === null || this._selIndex === undefined) return;

        if (e.key === 'ArrowLeft') {
            e.preventDefault();
            this._nextIndex(-1);
            return;
        }
        if (e.key === 'ArrowRight') {
            e.preventDefault();
            this._nextIndex(1);
            return;
        }
        if (e.key === 'ArrowUp') {
            e.preventDefault();
            this._nextIndex(-this.bytesPerRow);
            return;
        }
        if (e.key === 'ArrowDown') {
            e.preventDefault();
            this._nextIndex(this.bytesPerRow);
            return;
        }

        if (this._selKind === 'ascii' && e.key && e.key.length === 1) {
            const code = e.key.charCodeAt(0) & 0xFF;
            if (isPrintableAscii(code)) {
                e.preventDefault();
                this.data[this._selIndex] = code;
                this.markDirty(this._selIndex, 1);
                if (this.onChange) this.onChange({ start: this._selIndex, length: 1, kind: 'ascii' });
                this._selIndex = Math.min(this.data.length - 1, this._selIndex + 1);
                this._renderView();
            }
        }

        if (this._selKind === 'hex' && e.key && e.key.length === 1) {
            const ch = e.key.toUpperCase();
            if (ch >= '0' && ch <= '9' || ch >= 'A' && ch <= 'F') {
                e.preventDefault();

                const nib = parseInt(ch, 16) & 0xF;
                let which = (this._selNibble === 0 || this._selNibble === 1) ? this._selNibble : 0;

                const oldByte = this.data[this._selIndex] & 0xFF;
                let newByte = oldByte;
                if (which === 0) {
                    newByte = ((nib << 4) | (oldByte & 0x0F)) & 0xFF;
                } else {
                    newByte = ((oldByte & 0xF0) | nib) & 0xFF;
                }
                this.data[this._selIndex] = newByte;
                this.markDirty(this._selIndex, 1);
                if (this.onChange) this.onChange({ start: this._selIndex, length: 1, kind: 'hex' });

                /* advance to next nibble */
                if (which === 0) {
                    this._selNibble = 1;
                } else {
                    this._selNibble = 0;
                    this._selIndex = Math.min(this.data.length - 1, this._selIndex + 1);
                }
                this._renderView();
            }
        }
    }
}
