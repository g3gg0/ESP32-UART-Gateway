class MemoryScanner {
    constructor(opts) {
        this.read = opts.read;
        this.recover = opts.recover;
        this.onUpdate = opts.onUpdate || (() => { });
        this.running = false;
        this.status = 'Ready';
        this.results = new Map();
        this.resultRuns = new Map();
        this.dataRanges = new Set();
        this.blockSize = 4096;
        this.probes = 0;
        this.counts = { data: 0, zero: 0, ff: 0, mixed: 0, fault: 0 };
    }

    stop() {
        this.running = false;
    }

    getResult(base, size) {
        const key = `${base}:${size}`;
        let result = this.results.get(key);
        if (!result) {
            const runs = this.resultRuns.get(size) || [];
            let lower = 0, upper = runs.length;
            while (lower < upper) {
                const middle = Math.floor((lower + upper) / 2);
                if (runs[middle].end <= base) lower = middle + 1;
                else upper = middle;
            }
            const run = runs[lower];
            if (run && base >= run.base && base + size <= run.end && (base - run.base) % size === 0) result = run.result;
        }
        return result && this.dataRanges.has(key) ? { ...result, containsData: true } : result;
    }

    recordResult(base, size, result) {
        this.results.set(`${base}:${size}`, result);
        if (!this.resultRuns.has(size)) this.resultRuns.set(size, []);
        const runs = this.resultRuns.get(size);
        const previous = runs[runs.length - 1];
        if (previous && previous.end === base && previous.result.state === result.state &&
            previous.result.sampledBytes === result.sampledBytes && previous.result.requestedBytes === result.requestedBytes &&
            previous.result.error === result.error) {
            previous.end += size;
        } else {
            runs.push({ base, end: base + size, result: { ...result } });
        }
        if (this.results.size > 8192) this.results.delete(this.results.keys().next().value);
    }

    getVisibleRanges(base, size, maxRows = 256, initialRanges = null) {
        const ranges = initialRanges ? initialRanges.map(range => ({ ...range })) : [];
        const step = Math.min(size, Math.max(this.blockSize, size / 16));
        if (!initialRanges) {
            for (let address = base; address < base + size; address += step) {
                ranges.push({ base: address, size: Math.min(step, base + size - address), depth: 0 });
            }
        }
        for (let index = 0; index < ranges.length; index++) {
            const range = ranges[index];
            const result = this.getResult(range.base, range.size) || range.sample;
            if (range.size <= this.blockSize || !result || (!result.containsData && result.state !== 'data')) continue;
            const childSize = Math.max(this.blockSize, range.size / 16);
            const childCount = Math.ceil(range.size / childSize);
            if (ranges.length + childCount - 1 > maxRows) continue;
            const children = [];
            for (let address = range.base; address < range.base + range.size; address += childSize) {
                const sample = address === range.base && result.sampledBytes > 0 && result.sampledBytes <= childSize
                    ? { ...result, containsData: false } : undefined;
                children.push({ base: address, size: Math.min(childSize, range.base + range.size - address), depth: range.depth + 1, sample });
            }
            ranges.splice(index, 1, ...children);
            index--;
        }
        return ranges;
    }

    getMapSegments(base, size, maxRows = 256, initialRanges = null) {
        const segments = [];
        for (const range of this.getVisibleRanges(base, size, maxRows, initialRanges)) {
            const result = this.getResult(range.base, range.size) || range.sample;
            const reading = this.current && this.current.base >= range.base && this.current.base < range.base + range.size;
            const state = reading ? 'reading' : result ? result.containsData ? 'data' : result.state : 'unknown';
            const previous = segments[segments.length - 1];
            if (!initialRanges && previous && previous.base + previous.size === range.base && previous.state === state && state !== 'reading') {
                previous.size += range.size;
                previous.parts.push(range);
                previous.sampledBytes += result ? result.sampledBytes || 0 : 0;
                previous.containsData = previous.containsData || !!(result && result.containsData);
            } else {
                segments.push({
                    ...range, state, parts: [range], result,
                    sampledBytes: result ? result.sampledBytes || 0 : 0,
                    containsData: !!(result && result.containsData)
                });
            }
        }
        return segments;
    }

    getMapView(base, size, maxRows = 256, initialRanges = null) {
        if (base === 0 && size === 0x100000000 && !initialRanges) return this.getMapSegments(base, size, maxRows);
        const end = base + size;
        const before = [], after = [];
        for (const segment of this.getMapSegments(0, 0x100000000, maxRows)) {
            const segmentEnd = segment.base + segment.size;
            const addContext = (start, finish, target) => {
                if (start >= finish) return;
                const partial = start !== segment.base || finish !== segmentEnd;
                const result = partial ? start === segment.base && segment.result
                    ? { ...segment.result, containsData: false } : this.getResult(start, finish - start) : segment.result;
                const reading = this.current && this.current.base >= start && this.current.base < finish;
                target.push({
                    ...segment, base: start, size: finish - start, context: true,
                    result, containsData: !partial && segment.containsData,
                    state: reading ? 'reading' : partial ? result ? result.state : 'unknown' : segment.state,
                    parts: [{ base: start, size: finish - start, depth: 0 }],
                    sampledBytes: partial ? result ? result.sampledBytes || 0 : 0 : segment.sampledBytes
                });
            };
            addContext(segment.base, Math.min(segmentEnd, base), before);
            addContext(Math.max(segment.base, end), segmentEnd, after);
        }
        return [...before, ...this.getMapSegments(base, size, maxRows, initialRanges), ...after];
    }

    async scan(base = 0, size = 0x100000000, blockSize = 4096) {
        if (this.busy) throw new Error('Memory scan already running');
        if (![256, 1024, 4096, 65536].includes(blockSize)) throw new Error('Invalid scan block size');
        if (!Number.isSafeInteger(base) || !Number.isSafeInteger(size) || base < 0 || size < 4 ||
            base % 4 || size % 4 || base + size > 0x100000000) throw new Error('Invalid scan range');
        this.busy = true;
        this.running = true;
        this.status = 'Scanning';
        this.blockSize = blockSize;
        this.results.clear();
        this.resultRuns.clear();
        this.dataRanges.clear();
        this.probes = 0;
        this.counts = { data: 0, zero: 0, ff: 0, mixed: 0, fault: 0 };
        const levels = [];
        try {
            for (let regionSize = Math.max(blockSize, size / 16); ; regionSize = Math.max(blockSize, regionSize / 16)) {
                regionSize = Math.min(size, regionSize);
                this.levelSize = regionSize;
                levels.push(regionSize);
                this.levelTotal = Math.ceil(size / regionSize);
                this.levelDone = 0;
                for (let address = base; address < base + size && this.running; address += regionSize) {
                    const length = Math.min(regionSize > blockSize ? 64 : blockSize, base + size - address);
                    const key = `${address}:${Math.min(regionSize, base + size - address)}`;
                    this.current = { base: address, size: Math.min(regionSize, base + size - address) };
                    this.onUpdate(this);
                    let result;
                    try {
                        const words = await this.read(address, length / 4);
                        if (words.length !== length / 4) throw new Error('Short memory scan read');
                        let zero = true, ff = true, data = false;
                        for (const word of words) {
                            zero = zero && (word >>> 0) === 0;
                            ff = ff && (word >>> 0) === 0xFFFFFFFF;
                            for (let shift = 0; shift < 32; shift += 8) {
                                const byte = (word >>> shift) & 0xFF;
                                if (byte !== 0 && byte !== 0xFF) data = true;
                            }
                        }
                        result = { state: zero ? 'zero' : ff ? 'ff' : data ? 'data' : 'mixed', sampledBytes: length };
                    } catch (error) {
                        if (!/ACK=4\b|\bFAULT\b/.test(error.message)) throw error;
                        result = { state: 'fault', sampledBytes: 0, requestedBytes: length, error: error.message };
                        await this.recover();
                    }
                    this.recordResult(address, this.current.size, result);
                    if (result.state === 'data') {
                        for (const parentSize of levels.slice(0, -1)) {
                            const parentBase = base + Math.floor((address - base) / parentSize) * parentSize;
                            const parentKey = `${parentBase}:${Math.min(parentSize, base + size - parentBase)}`;
                            this.dataRanges.add(parentKey);
                            const parent = this.results.get(parentKey);
                            if (parent) parent.containsData = true;
                        }
                    }
                    this.probes++;
                    this.counts[result.state]++;
                    this.levelDone++;
                    this.current = null;
                    this.onUpdate(this);
                    if ((this.probes & 15) === 0) await new Promise(resolve => setTimeout(resolve, 0));
                }
                if (!this.running || regionSize <= blockSize) break;
            }
            this.status = this.running ? 'Complete' : 'Stopped';
        } catch (error) {
            this.status = 'Failed';
            throw error;
        } finally {
            this.running = false;
            this.busy = false;
            this.current = null;
            this.onUpdate(this);
        }
    }
}
