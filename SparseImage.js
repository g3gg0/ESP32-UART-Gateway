
/**
 * SparseImage - Abstraction layer for accessing binary data with caching
 * Acts like a Uint8Array but lazily loads data from a device/source through a callback
 * 
 * Use cases:
 * 1. Reading from slow devices (e.g., flash memory over serial)
 * 2. Working with large files where only portions are needed
 * 3. Caching frequently accessed regions
 * 
 * Example usage:
 * ```javascript
 * // Create a SparseImage with a read callback
 * const sparseImage = new SparseImage(1024 * 1024, (address, size) => {
 *     // This callback is called when data is not in cache
 *     // Read from your device here
 *     return deviceRead(address, size); // Should return Uint8Array
 * });
 * 
 * // Wrap in proxy for array-like access
 * const buffer = SparseImage._createProxy(sparseImage);
 * 
 * // Access like a normal Uint8Array - data is fetched automatically
 * const byte = buffer[0x1000];
 * const chunk = buffer.subarray(0x1000, 0x2000);
 * ```
 * 
 * Architecture:
 * - ReadBuffer: Array of {address, data} segments containing cached read data
 * - ReadData callback: Called to fetch missing data from device/source
 * - Automatic merging: Adjacent/overlapping segments are merged to optimize memory
 * 
 * Future enhancement:
 * - WriteBuffer: Parallel buffer for tracking writes before committing to device
 *   - Reads check WriteBuffer first, then ReadBuffer
 *   - Allows batching writes and deferred commit operations
 */
class SparseImage {
    constructor(size, readDataCallback = null, writeDataCallback = null, flushPrepareCallback = null, sectorSize = 0x1000) {
        this.size = size;
        this.readDataCallback = readDataCallback;
        this.writeDataCallback = writeDataCallback;
        this.flushPrepareCallback = flushPrepareCallback;
        this.dataUpdatedCallback = null;
        this.sectorSize = sectorSize || 0x1000;
        this.readBuffer = []; // Array of {address, data} structures
        this.writeBuffer = []; // Array of {address, data} structures
        this.length = size;
        /* Lock to ensure _ensureData executes serially */
        this._ensureDataLock = Promise.resolve();

        this.logMessage = (msg) => { };
        this.logDebug = (msg) => { };
        this.logError = (msg) => { };
    }

    /**
     * Initialize from an existing ArrayBuffer/Uint8Array
     */
    static fromBuffer(arrayBuffer, sectorSize = 0x1000) {
        const buffer = arrayBuffer instanceof Uint8Array ? arrayBuffer : new Uint8Array(arrayBuffer);
        const sparseImage = new SparseImage(buffer.length, null, null, null, sectorSize);
        sparseImage.readBuffer.push({
            address: 0,
            data: buffer
        });
        return sparseImage;
    }

    /**
     * Find which buffer segment contains the given address
     */
    _findSegment(address, list = this.readBuffer) {
        for (const segment of list) {
            const endAddress = segment.address + segment.data.length;
            if (address >= segment.address && address < endAddress) {
                return segment;
            }
        }
        return null;
    }

    /**
     * Check if a range is fully covered by existing segments
     */
    _isRangeCovered(address, size, list = this.readBuffer) {
        let checkPos = address;
        const endAddress = address + size;

        while (checkPos < endAddress) {
            const segment = this._findSegment(checkPos, list);
            if (!segment) {
                return false;
            }
            checkPos = segment.address + segment.data.length;
        }
        return true;
    }

    /**
     * Check if a range is covered by either write or read buffers
     */
    _isRangeCoveredAny(address, size) {
        let checkPos = address;
        const endAddress = address + size;
        while (checkPos < endAddress) {
            const w = this._findSegment(checkPos, this.writeBuffer);
            if (w) {
                checkPos = Math.min(endAddress, w.address + w.data.length);
                continue;
            }
            const r = this._findSegment(checkPos, this.readBuffer);
            if (r) {
                checkPos = Math.min(endAddress, r.address + r.data.length);
                continue;
            }
            return false;
        }
        return true;
    }

    /**
     * Find the first gap within [address, address+size) not covered by write/read buffers.
     * Returns { start, size } or null if fully covered.
     */
    _findFirstGapRange(address, size) {
        const endAddress = address + size;
        let pos = address;
        while (pos < endAddress) {
            const w = this._findSegment(pos, this.writeBuffer);
            if (w) {
                pos = Math.min(endAddress, w.address + w.data.length);
                continue;
            }
            const r = this._findSegment(pos, this.readBuffer);
            if (r) {
                pos = Math.min(endAddress, r.address + r.data.length);
                continue;
            }
            /* pos is not covered: determine gap end at next segment start or endAddress */
            let nextStart = endAddress;
            for (const s of this.writeBuffer) {
                if (s.address > pos && s.address < nextStart) nextStart = s.address;
            }
            for (const s of this.readBuffer) {
                if (s.address > pos && s.address < nextStart) nextStart = s.address;
            }
            return { start: pos, size: nextStart - pos };
        }
        return null;
    }

    _mergeSegmentsGeneric(list) {
        if (list.length <= 1) return list;

        const indexed = list.map((segment, idx) => ({ ...segment, _idx: idx }));
        indexed.sort((a, b) => {
            if (a.address === b.address) {
                return a._idx - b._idx;
            }
            return a.address - b.address;
        });

        const merged = [];
        let current = indexed[0];

        for (let i = 1; i < indexed.length; i++) {
            const next = indexed[i];
            const currentEnd = current.address + current.data.length;

            if (next.address <= currentEnd) {
                const mergedEnd = Math.max(currentEnd, next.address + next.data.length);
                const mergedSize = mergedEnd - current.address;
                const mergedData = new Uint8Array(mergedSize);
                mergedData.set(current.data, 0);
                const nextOffset = next.address - current.address;
                mergedData.set(next.data, nextOffset);
                current = {
                    address: current.address,
                    data: mergedData
                };
            } else {
                merged.push({ address: current.address, data: current.data });
                current = next;
            }
        }
        merged.push({ address: current.address, data: current.data });
        return merged;
    }

    /**
     * Merge adjacent or overlapping segments in the readBuffer
     */
    _mergeSegments() {
        this.readBuffer = this._mergeSegmentsGeneric(this.readBuffer);
    }

    _mergeWriteSegments() {
        this.writeBuffer = this._mergeSegmentsGeneric(this.writeBuffer);
    }

    _effectiveByte(pos) {
        const w = this._findSegment(pos, this.writeBuffer);
        if (w) return w.data[pos - w.address] & 0xFF;
        const r = this._findSegment(pos, this.readBuffer);
        if (r) return r.data[pos - r.address] & 0xFF;
        return 0xFF;
    }

    _materializeRange(start, end) {
        const len = end - start;
        const out = new Uint8Array(len);
        out.fill(0xFF);

        for (const seg of this.readBuffer) {
            const s0 = seg.address;
            const s1 = seg.address + seg.data.length;
            const o0 = Math.max(start, s0);
            const o1 = Math.min(end, s1);
            if (o0 < o1) {
                const dstOff = o0 - start;
                const srcOff = o0 - s0;
                out.set(seg.data.subarray(srcOff, srcOff + (o1 - o0)), dstOff);
            }
        }

        for (const seg of this.writeBuffer) {
            const s0 = seg.address;
            const s1 = seg.address + seg.data.length;
            const o0 = Math.max(start, s0);
            const o1 = Math.min(end, s1);
            if (o0 < o1) {
                const dstOff = o0 - start;
                const srcOff = o0 - s0;
                out.set(seg.data.subarray(srcOff, srcOff + (o1 - o0)), dstOff);
            }
        }

        return out;
    }

    _materializeReadRange(start, end) {
        const len = end - start;
        const out = new Uint8Array(len);
        out.fill(0xFF);

        for (const seg of this.readBuffer) {
            const s0 = seg.address;
            const s1 = seg.address + seg.data.length;
            const o0 = Math.max(start, s0);
            const o1 = Math.min(end, s1);
            if (o0 < o1) {
                const dstOff = o0 - start;
                const srcOff = o0 - s0;
                out.set(seg.data.subarray(srcOff, srcOff + (o1 - o0)), dstOff);
            }
        }

        return out;
    }

    _addSegment(list, address, data) {
        list.push({ address, data });
        return this._mergeSegmentsGeneric(list);
    }

    /**
     * Read data from the sparse image, fetching from device if necessary
     */
    async _ensureData(address, size) {
        /* Acquire lock to ensure only one _ensureData executes at a time */
        const run = () => this._ensureDataUnlocked(address, size);
        this._ensureDataLock = this._ensureDataLock.then(run, run);
        return this._ensureDataLock;
    }

    /**
     * Internal _ensureData implementation (unlocked)
     * @private
     */
    async _ensureDataUnlocked(address, size) {
        if (address < 0 || address >= this.size) {
            throw new RangeError(`Address ${address} out of bounds [0, ${this.size})`);
        }

        // Clamp size to available data
        size = Math.min(size, this.size - address);

        // If range is already covered by write or read cache, nothing to do
        if (this._isRangeCoveredAny(address, size)) return;

        // Fill gaps: either by read callback (preferred) or zero-fill if no callback
        let safety = 64;
        while (!this._isRangeCoveredAny(address, size) && safety-- > 0) {
            const gap = this._findFirstGapRange(address, size);
            if (!gap || gap.size <= 0) break;

            if (!this.readDataCallback) {
                /* No callback - create zero segment only for the uncovered gap */
                const data = new Uint8Array(gap.size);
                this.readBuffer = this._addSegment(this.readBuffer, gap.start, data);
                continue;
            }

            /* Call the callback; it may return more/less and with its own base address */
            const res = await this.readDataCallback(gap.start, gap.size);
            let a = null;
            let d = null;
            if (res instanceof Uint8Array) {
                a = gap.start;
                d = res;
            } else if (res && res.buffer instanceof ArrayBuffer && res.byteLength !== undefined) {
                /* Accept ArrayBufferView-like */
                a = gap.start;
                d = new Uint8Array(res.buffer, res.byteOffset || 0, res.byteLength);
            } else if (res && typeof res === 'object') {
                const rAddr = res.address !== undefined ? res.address : gap.start;
                const rData = res.data;
                if (rData instanceof Uint8Array) {
                    a = rAddr;
                    d = rData;
                } else if (rData && rData.buffer instanceof ArrayBuffer && rData.byteLength !== undefined) {
                    a = rAddr;
                    d = new Uint8Array(rData.buffer, rData.byteOffset || 0, rData.byteLength);
                }
            }

            if (d && d.length > 0) {
                this.readBuffer = this._addSegment(this.readBuffer, a, d);
                // loop will re-check coverage
            } else {
                // No progress possible from callback, avoid infinite loop
                break;
            }

            this.dataUpdatedCallback && this.dataUpdatedCallback(a, d.length);
        }
    }

    write(address, data) {
        if (address < 0 || address >= this.size) {
            throw new RangeError(`Address ${address} out of bounds [0, ${this.size})`);
        }
        const normalized = data instanceof Uint8Array ? data : new Uint8Array(data);
        const start = address;
        const end = Math.min(address + normalized.length, this.size);
        if (end <= start) return;

        const fmtRanges = (list) => list.map(s => `[0x${s.address.toString(16)}-0x${(s.address + s.data.length).toString(16)})`).join(', ');
        const preRanges = fmtRanges(this.writeBuffer);
        // this.logDebug('SparseImage.write start', { address: start, length: normalized.length, preRanges });
        const sectorSize = this.sectorSize || 0x1000;
        const touchedSectors = new Set();

        /* Helper: find write buffer segment that covers pos */
        const findWriteSeg = (pos) => this._findSegment(pos, this.writeBuffer);

        /* Helper: find any cached segment (write preferred, then read) that covers pos */
        const findCachedSeg = (pos) => findWriteSeg(pos) || this._findSegment(pos, this.readBuffer);

        /* Helper: merge touching/overlapping segments after modifications */
        const mergeWrites = () => {
            this.writeBuffer = this._mergeSegmentsGeneric(this.writeBuffer);
        };

        /* Helper: mark sectors touched by a range */
        const markSectors = (rangeStart, rangeEnd) => {
            for (let s = Math.floor(rangeStart / sectorSize) * sectorSize; s < rangeEnd; s += sectorSize) {
                touchedSectors.add(s);
            }
        };

        /* Helper: compute next boundary where coverage changes */
        const nextBoundary = (pos, limit) => {
            let nb = limit;
            for (const s of [...this.readBuffer, ...this.writeBuffer]) {
                if (s.address > pos && s.address < nb) nb = s.address;
                const sEnd = s.address + s.data.length;
                if (sEnd > pos && sEnd < nb) nb = sEnd;
            }
            const sectorEnd = Math.min(limit, (Math.floor(pos / sectorSize) + 1) * sectorSize);
            if (sectorEnd > pos && sectorEnd < nb) nb = sectorEnd;
            return nb;
        };

        let pos = start;
        let remaining = end - start;

        while (remaining > 0) {
            /* Case 1: existing write buffer covers current position */
            const wseg = findWriteSeg(pos);
            if (wseg) {
                const offset = pos - wseg.address;
                const span = Math.min(remaining, wseg.data.length - offset);
                wseg.data.set(normalized.subarray(pos - start, pos - start + span), offset);
                markSectors(pos, pos + span);
                pos += span;
                remaining -= span;
                continue;
            }

            /* Case 2: cached (read) data covers current position */
            const cseg = findCachedSeg(pos);
            if (cseg) {
                const offset = pos - cseg.address;
                const span = Math.min(remaining, cseg.data.length - offset);

                /* Check matching prefix */
                let matchLen = 0;
                while (matchLen < span) {
                    const desired = normalized[pos - start + matchLen] & 0xFF;
                    if (cseg.data[offset + matchLen] !== desired) break;
                    matchLen++;
                }

                if (matchLen > 0) {
                    pos += matchLen;
                    remaining -= matchLen;
                    continue;
                }

                /* Mismatch: see if full sector is FULLY cached in readBuffer */
                const sectorStart = Math.floor(pos / sectorSize) * sectorSize;
                const sectorEnd = Math.min(sectorStart + sectorSize, this.size);
                if (this._isRangeCovered(sectorStart, sectorEnd - sectorStart, this.readBuffer)) {
                    const sectorBuf = this._materializeRange(sectorStart, sectorEnd);
                    const writeStart = pos;
                    const writeEnd = Math.min(end, sectorEnd);
                    sectorBuf.set(
                        normalized.subarray(writeStart - start, writeEnd - start),
                        writeStart - sectorStart
                    );
                    this.writeBuffer = this._addSegment(this.writeBuffer, sectorStart, sectorBuf);
                    markSectors(sectorStart, sectorEnd);
                    pos = writeEnd;
                    remaining = end - pos;
                    continue;
                }

                /* Not fully cached: create a new segment until the next boundary */
                const boundary = nextBoundary(pos, end);
                const writeEnd = Math.min(boundary, end);
                const slice = normalized.slice(pos - start, writeEnd - start);
                this.writeBuffer = this._addSegment(this.writeBuffer, pos, slice);
                markSectors(pos, writeEnd);
                mergeWrites();
                pos = writeEnd;
                remaining = end - pos;
                continue;
            }

            /* Case 3: no cache coverage; create a new segment up to next sector/boundary */
            const boundary = nextBoundary(pos, end);
            const writeEnd = Math.min(boundary, end);
            const slice = normalized.slice(pos - start, writeEnd - start);
            this.writeBuffer = this._addSegment(this.writeBuffer, pos, slice);
            markSectors(pos, writeEnd);
            mergeWrites();
            pos = writeEnd;
            remaining = end - pos;
        }

        /* Cleanup: remove sectors we touched that are identical to cached data */
        for (const sectorStart of touchedSectors) {
            const sectorEnd = Math.min(sectorStart + sectorSize, this.size);

            /* Only prune if the sector is fully backed by real read cache and matches */
            const readCovered = this._isRangeCovered(sectorStart, sectorEnd - sectorStart, this.readBuffer);
            if (!readCovered) {
                continue;
            }

            const baseline = this._materializeReadRange(sectorStart, sectorEnd);
            const combined = this._materializeRange(sectorStart, sectorEnd);

            let identical = baseline.length === combined.length;
            if (identical) {
                for (let i = 0; i < combined.length; i++) {
                    if (combined[i] !== baseline[i]) {
                        identical = false;
                        break;
                    }
                }
            }

            if (identical) {
                const pruned = [];
                for (const seg of this.writeBuffer) {
                    const segStart = seg.address;
                    const segEnd = seg.address + seg.data.length;
                    if (segEnd <= sectorStart || segStart >= sectorEnd) {
                        pruned.push(seg);
                        continue;
                    }

                    if (segStart < sectorStart) {
                        const left = seg.data.slice(0, sectorStart - segStart);
                        if (left.length) pruned.push({ address: segStart, data: left });
                    }

                    if (segEnd > sectorEnd) {
                        const right = seg.data.slice(sectorEnd - segStart);
                        if (right.length) pruned.push({ address: sectorEnd, data: right });
                    }
                }
                this.writeBuffer = this._mergeSegmentsGeneric(pruned);
            }
        }

        /* Ensure buffers are merged after all operations */
        mergeWrites();

        const postRanges = fmtRanges(this.writeBuffer);
        // this.logDebug('SparseImage.write done', { address: start, length: normalized.length, preRanges, postRanges });
    }

    fill(value, start = 0, end = this.size) {
        if (start < 0 || start >= this.size) {
            throw new RangeError(`Address ${start} out of bounds [0, ${this.size})`);
        }
        end = Math.min(end, this.size);
        if (end <= start) return;

        const desired = value & 0xFF;
        const len = end - start;
        const buf = new Uint8Array(len);
        buf.fill(desired);
        // this.logDebug('SparseImage.fill', { start, end, len, desired });
        this.write(start, buf);
    }

    async flush() {
        if (!this.writeBuffer.length) return;

        // Consolidate write segments first (touching/overlapping writes coalesce)
        this._mergeWriteSegments();

        /* Call prepare callback if provided */
        if (this.flushPrepareCallback) {
            await this.flushPrepareCallback(this);
        }

        // Flush to backing store if provided
        if (this.writeDataCallback) {
            // Deterministic order: ascending address
            const toWrite = [...this.writeBuffer].sort((a, b) => a.address - b.address);
            for (const segment of toWrite) {
                await this.writeDataCallback(segment.address, segment.data);
            }
        }

        // Merge read+write with explicit priority: write data overrides read data
        this.readBuffer = this._mergeReadAndWriteWithPriority(this.readBuffer, this.writeBuffer);

        // Clear pending writes
        this.writeBuffer = [];
    }

    async clear() {
        this.readBuffer = [];
        this.writeBuffer = [];
    }

    /**
     * Merge read and write buffers into a single read buffer, ensuring
     * write data has priority over read data in any overlap. Touching
     * segments are merged into a single continuous segment.
     */
    _mergeReadAndWriteWithPriority(readList, writeList) {
        if ((!readList || readList.length === 0) && (!writeList || writeList.length === 0)) {
            return [];
        }

        const annotated = [];
        if (readList && readList.length) {
            for (const s of readList) annotated.push({ address: s.address, data: s.data, _src: 'r' });
        }
        if (writeList && writeList.length) {
            for (const s of writeList) annotated.push({ address: s.address, data: s.data, _src: 'w' });
        }

        // Sort by address to form contiguous/touching groups
        annotated.sort((a, b) => a.address - b.address);

        const result = [];
        let group = [];
        let groupStart = null;
        let groupEnd = null;

        const flushGroup = () => {
            if (!group.length) return;
            const length = groupEnd - groupStart;
            const mergedData = new Uint8Array(length);

            // Overlay order: read first, then write (write overrides)
            for (const seg of group) {
                if (seg._src !== 'r') continue;
                const off = seg.address - groupStart;
                mergedData.set(seg.data, off);
            }
            for (const seg of group) {
                if (seg._src !== 'w') continue;
                const off = seg.address - groupStart;
                mergedData.set(seg.data, off);
            }

            result.push({ address: groupStart, data: mergedData });
            group = [];
            groupStart = null;
            groupEnd = null;
        };

        for (const seg of annotated) {
            const segStart = seg.address;
            const segEnd = seg.address + seg.data.length;
            if (groupStart === null) {
                // start new group
                groupStart = segStart;
                groupEnd = segEnd;
                group.push(seg);
                continue;
            }
            // Merge if overlapping or touching
            if (segStart <= groupEnd) {
                group.push(seg);
                if (segEnd > groupEnd) groupEnd = segEnd;
            } else {
                // Gap: finalize previous group
                flushGroup();
                // start new group
                groupStart = segStart;
                groupEnd = segEnd;
                group.push(seg);
            }
        }

        flushGroup();
        return result;
    }

    /**
     * Get a single byte at the given offset (Uint8Array-like interface)
     * NOTE: Assumes data is already loaded. Use async methods to ensure data first.
     */
    _get(index) {
        if (index < 0 || index >= this.size) {
            return undefined;
        }

        // Write buffer overrides read buffer
        const wseg = this._findSegment(index, this.writeBuffer);
        if (wseg) {
            return wseg.data[index - wseg.address];
        }

        const segment = this._findSegment(index, this.readBuffer);
        if (!segment) {
            return 0; // Return 0 for unread data
        }

        return segment.data[index - segment.address];
    }

    /**
     * Proxy handler to make SparseImage act like a Uint8Array
     */
    static _createProxy(sparseImage) {
        return new Proxy(sparseImage, {
            get(target, prop) {
                if (typeof prop === 'symbol') {
                    return target[prop];
                }
                // Handle numeric indices
                const index = Number(prop);
                if (Number.isInteger(index) && index >= 0) {
                    return target._get(index);
                }

                // Handle standard properties and methods
                if (prop in target) {
                    const value = target[prop];
                    return typeof value === 'function' ? value.bind(target) : value;
                }

                return undefined;
            },

            set(target, prop, value) {
                if (typeof prop === 'symbol') {
                    target[prop] = value;
                    return true;
                }
                const index = Number(prop);
                if (Number.isInteger(index) && index >= 0) {
                    const byte = Number(value) & 0xFF;
                    target.write(index, Uint8Array.of(byte));
                    return true;
                }

                target[prop] = value;
                return true;
            },

            has(target, prop) {
                if (typeof prop === 'symbol') {
                    return prop in target;
                }
                const index = Number(prop);
                if (Number.isInteger(index) && index >= 0 && index < target.size) {
                    return true;
                }
                return prop in target;
            }
        });
    }

    /**
     * Get a subarray (similar to Uint8Array.subarray)
     * SYNC version - assumes data is already loaded via prefetch/ensureData
     */
    subarray(start, end) {
        start = start || 0;
        end = end === undefined ? this.size : end;

        const size = end - start;

        const result = new Uint8Array(size);
        for (let pos = start, idx = 0; pos < end; pos++, idx++) {
            result[idx] = this._get(pos);
        }

        return result;
    }

    /**
     * Get a subarray asynchronously (ensures data is loaded first)
     */
    async subarray_async(start, end) {
        start = start || 0;
        end = end === undefined ? this.size : end;

        const size = end - start;

        // Ensure all data is loaded first
        await this._ensureData(start, size);

        const result = new Uint8Array(size);
        for (let pos = start, idx = 0; pos < end; pos++, idx++) {
            result[idx] = this._get(pos);
        }

        return result;
    }

    /**
     * Get a slice (creates a copy, similar to Uint8Array.slice)
     * SYNC version - assumes data is already loaded
     */
    slice(start, end) {
        return this.subarray(start, end);
    }

    /**
     * Get a slice asynchronously (ensures data is loaded first)
     */
    async slice_async(start, end) {
        return await this.subarray_async(start, end);
    }

    /**
     * Create a DataView for this SparseImage
     */
    createDataView() {
        return new SparseImageDataView(this);
    }

    /**
     * Pre-fetch a range of data
     */
    async prefetch(address, size) {
        return await this._ensureData(address, size);
    }
}
