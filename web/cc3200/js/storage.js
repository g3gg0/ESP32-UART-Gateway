const CC3200_STORAGE_BITS = [
    { bit: 0x02, storageId: CC3200_STORAGE.FLASH, name: 'FLASH' },
    { bit: 0x04, storageId: CC3200_STORAGE.SFLASH, name: 'SFLASH' },
    { bit: 0x80, storageId: CC3200_STORAGE.SRAM, name: 'SRAM' }
];


async function rawReadRangeToU8(ctx) {
    const storageId = (ctx.storageId ?? CC3200_STORAGE.SFLASH) >>> 0;
    const offset = (ctx.offset ?? 0) >>> 0;
    const size = (ctx.size ?? 0) >>> 0;
    const onChunk = (typeof ctx?.onChunk === 'function') ? ctx.onChunk : null;

    /* If SparseImage is available and reading from SFLASH, use it */
    if (cc3200FlashImage && storageId === CC3200_STORAGE.SFLASH) {
        /* Prefetch data with progress reporting */
        if (onChunk) {
            const blockSize = 4096;
            let done = 0;
            while (done < size) {
                const chunkSize = Math.min(blockSize, size - done);
                await cc3200FlashImage.prefetch(offset + done, chunkSize);
                done += chunkSize;
                onChunk({ storageId, offset: offset + done - chunkSize, chunkSize, done, total: size, data: null });
            }
        }
        /* Get the complete data */
        return await cc3200FlashImage.subarray_async(offset, offset + size);
    }

    /* Fall back to direct RawRead commands */
    const blockSize = 4096;
    let remaining = size;
    let currentOffset = offset;
    const onChunkInternal = (typeof ctx?.onChunk === 'function') ? ctx.onChunk : null;

    const chunks = [];
    const total = remaining;
    let done = 0;

    while (remaining > 0) {
        const chunkSize = Math.min(blockSize, remaining);
        const cmdPayload = u8Concat([
            new Uint8Array([CC3200_OPCODES.RawRead]),
            be32(storageId >>> 0),
            be32(currentOffset >>> 0),
            be32(chunkSize >>> 0)
        ]);

        const timeoutMs = Math.max(2000, 1500 + Math.ceil(chunkSize / 50));
        const res = await cc3200Serial.sendCommandWithResponse(cmdPayload, {
            kind: 'packet',
            expectedLen: chunkSize,
            timeoutMs,
            callback: null,
            ctx: { cmdId: 'RawReadMemChunk' }
        });

        if (!res || !res.data || res.data.length !== chunkSize) {
            throw new Error(`raw_read chunk failed (got ${res?.data?.length ?? 0}, expected ${chunkSize})`);
        }

        chunks.push(res.data);
        done += chunkSize;

        if (onChunkInternal) {
            try {
                onChunkInternal({ storageId, offset: currentOffset, chunkSize, done, total, data: res.data });
            } catch (e) {
                /* ignore */
            }
        }

        currentOffset = (currentOffset + chunkSize) >>> 0;
        remaining -= chunkSize;
    }

    return u8Concat(chunks);
}






function downloadU8AsFile(u8, filename) {
    const safeName = (filename && filename.trim()) ? filename.trim() : 'download.bin';
    const blob = new Blob([u8], { type: 'application/octet-stream' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = safeName;
    document.body.appendChild(a);
    a.click();
    a.remove();
    setTimeout(() => URL.revokeObjectURL(url), 5000);
}

async function rawReadRangeToFile(ctx) {
    const blockSize = 4096;
    const storageId = ctx.storageId >>> 0;
    let offset = ctx.offset >>> 0;
    let remaining = ctx.size >>> 0;
    const fname = (ctx.filename && ctx.filename.trim()) ? ctx.filename.trim() : 'raw_read.bin';

    const onChunk = (typeof ctx?.onChunk === 'function') ? ctx.onChunk : null;

    let writable = null;
    const chunks = [];
    try {
        if (ctx.fileHandle) {
            writable = await ctx.fileHandle.createWritable();
        }

        const total = remaining;
        let done = 0;

        while (remaining > 0) {
            const chunkSize = Math.min(blockSize, remaining);
            const cmdPayload = new Uint8Array([
                CC3200_OPCODES.RawRead,
                (storageId >>> 24) & 0xFF,
                (storageId >>> 16) & 0xFF,
                (storageId >>> 8) & 0xFF,
                storageId & 0xFF,
                (offset >>> 24) & 0xFF,
                (offset >>> 16) & 0xFF,
                (offset >>> 8) & 0xFF,
                offset & 0xFF,
                (chunkSize >>> 24) & 0xFF,
                (chunkSize >>> 16) & 0xFF,
                (chunkSize >>> 8) & 0xFF,
                chunkSize & 0xFF
            ]);

            const timeoutMs = Math.max(2000, 1500 + Math.ceil(chunkSize / 50));
            const res = await cc3200Serial.sendCommandWithResponse(cmdPayload, {
                kind: 'packet',
                expectedLen: chunkSize,
                timeoutMs,
                callback: null,
                ctx: { cmdId: 'RawReadChunk' }
            });

            if (!res || !res.data || res.data.length !== chunkSize) {
                throw new Error(`raw_read chunk failed (got ${res?.data?.length ?? 0}, expected ${chunkSize})`);
            }

            if (writable) {
                await writable.write(res.data);
            } else {
                chunks.push(res.data);
            }

            done += chunkSize;
            if (onChunk) {
                try {
                    onChunk({
                        storageId,
                        offset,
                        chunkSize,
                        done,
                        total,
                        data: res.data
                    });
                } catch (e) {
                    /* ignore user callback errors */
                }
            }

            offset = (offset + chunkSize) >>> 0;
            remaining -= chunkSize;

            if ((done % (blockSize * 32)) === 0 || done === total) {
                logToConsole(`RawRead: ${done}/${total} bytes (${(done * 100 / total).toFixed(2)}%)`, 'info');
            }
        }

        if (writable) {
            await writable.close();
            logToConsole(`RawRead: saved '${fname}' (${total} bytes)`, 'info');
        } else {
            downloadU8AsFile(u8Concat(chunks), fname);
            logToConsole(`RawRead: downloaded '${fname}' (${total} bytes)`, 'info');
        }
    } catch (err) {
        try {
            if (writable) {
                await writable.close();
                logToConsole(`RawRead: saved partial '${fname}' (${done} bytes)`, 'info');
            } else if (chunks.length > 0) {
                downloadU8AsFile(u8Concat(chunks), fname);
                logToConsole(`RawRead: downloaded partial '${fname}' (${done} bytes)`, 'info');
            }
        } catch (e) {
            /* ignore */
        }
        throw err;
    }
}


function formatHexLine(u8) {
    let s = '';
    for (let i = 0; i < u8.length; i++) {
        s += u8[i].toString(16).padStart(2, '0').toUpperCase() + ' ';
    }
    return s.trim();
}

function formatBytes(n) {
    if (n < 1024) return `${n} B`;
    if (n < 1024 * 1024) return `${(n / 1024).toFixed(2)} KiB`;
    if (n < 1024 * 1024 * 1024) return `${(n / (1024 * 1024)).toFixed(2)} MiB`;
    return `${(n / (1024 * 1024 * 1024)).toFixed(2)} GiB`;
}

function fmtVerTuple(arr) {
    return `${arr[0]}.${arr[1]}.${arr[2]}.${arr[3]}`;
}

function fmtHexBytes(arr) {
    return Array.from(arr).map(b => b.toString(16).padStart(2, '0').toUpperCase()).join(':');
}

function onGetVersion(ctx, payload) {
    if (payload.length < 20) {
        logToConsole(`GetVersion: short payload (${payload.length}): ${formatHexLine(payload)}`, 'info');
        return;
    }

    const bootloader = payload.slice(0, 4);
    const nwp = payload.slice(4, 8);
    const mac = payload.slice(8, 12);
    const phy = payload.slice(12, 16);
    const chipType = payload.slice(16, 20);
    const tail = payload.slice(20);

    const isCc3200 = (chipType[0] & 0x10) !== 0;
    logToConsole(`GetVersion: bootloader=${fmtVerTuple(bootloader)} nwp=${fmtVerTuple(nwp)} phy=${fmtVerTuple(phy)} chip_type=${formatHexLine(chipType)} is_cc3200=${isCc3200}`, 'info');
    logToConsole(`GetVersion: mac=${fmtHexBytes(mac)} tail(${tail.length})=${formatHexLine(tail)}`, 'info');
}

function onGetStorageList(ctx, bytes) {
    const v = bytes[0];
    const flash = (v & 0x02) !== 0;
    const sflash = (v & 0x04) !== 0;
    const sram = (v & 0x80) !== 0;
    logToConsole(`GetStorageList: 0x${v.toString(16).padStart(2, '0')} (flash=${flash} sflash=${sflash} sram=${sram})`, 'info');
}

function onGetStorageInfo(ctx, payload) {
    if (payload.length < 4) {
        logToConsole(`GetStorageInfo: short payload (${payload.length})`, 'info');
        return;
    }
    const bsize = (payload[0] << 8) | payload[1];
    const bcount = (payload[2] << 8) | payload[3];
    const cap = bsize * bcount;
    const tail = payload.slice(4);
    logToConsole(`GetStorageInfo(storage_id=${ctx.storageId}): block_size=${bsize} block_count=${bcount} capacity=${formatBytes(cap)}`, 'info');
    if (tail.length > 0) {
        logToConsole(`GetStorageInfo: tail(${tail.length})=${formatHexLine(tail)}`, 'info');
    }
}

function parseStorageInfoPayload(payload) {
    if (!payload || payload.length < 4) {
        return null;
    }
    const blockSize = (payload[0] << 8) | payload[1];
    const blockCount = (payload[2] << 8) | payload[3];
    const capacity = blockSize * blockCount;
    return { blockSize, blockCount, capacity, tail: payload.slice(4) };
}

function setStorageStatus(text) {
    const el = document.getElementById('storageStatus');
    if (el) el.textContent = text;
}

function renderStoragePanes(storages, maskByte) {
    const panesEl = document.getElementById('storagePanes');
    if (!panesEl) return;

    const maskHex = `0x${(maskByte ?? 0).toString(16).padStart(2, '0')}`;
    if (!storages || storages.length === 0) {
        panesEl.innerHTML = `<div style="padding:10px; border:1px solid #374151; border-radius:6px; background: rgba(15, 23, 42, 0.6); color:#9ca3af;">No storages reported (mask=${maskHex}).</div>`;
        return;
    }

    panesEl.innerHTML = storages.map(s => {
        const cap = s.capacity ?? 0;
        const infoLine = s.error
            ? `<div style="color:#fca5a5;">Info error: ${escapeHtml(s.error)}</div>`
            : `<div>block_size=${s.blockSize} block_count=${s.blockCount} capacity=${formatBytes(cap)} (${cap} bytes)</div>`;

        const tailLine = (!s.error && s.tail && s.tail.length)
            ? `<div style="color:#9ca3af; font-size:12px;">tail(${s.tail.length})=${formatHexLine(s.tail)}</div>`
            : '';

        const readDisabled = (!!s.error || !Number.isFinite(cap) || cap <= 0 || cap > 0xFFFFFFFF);
        const readBtnStyle = readDisabled
            ? 'padding: 8px 14px; background: #374151; color: #9ca3af; border: none; border-radius: 6px; cursor: not-allowed; font-weight: 700;'
            : 'padding: 8px 14px; background: #2563eb; color: white; border: none; border-radius: 6px; cursor: pointer; font-weight: 700;';

        const readTitle = readDisabled
            ? (cap > 0xFFFFFFFF ? 'Capacity > 4GiB not supported' : 'Read disabled')
            : 'Read full storage to .bin';

        return `
            <div style="padding:10px; border:1px solid #374151; border-radius:6px; background: rgba(15, 23, 42, 0.6);">
                <div style="display:flex; justify-content:space-between; align-items:center; gap:10px; flex-wrap:wrap;">
                    <div style="font-weight:800;">${escapeHtml(s.name)} (storage_id=${s.storageId})</div>
                    <div style="display:flex; gap:10px; align-items:center;">
                        <button class="btnStorageRead" data-storage-id="${s.storageId}" data-storage-name="${escapeHtmlAttr(s.name)}" data-storage-size="${cap}" style="${readBtnStyle}" title="${escapeHtmlAttr(readTitle)}" ${readDisabled ? 'disabled' : ''}>Read</button>
                        <div class="storageProgress" style="width:160px; height:10px; border:1px solid #374151; border-radius:999px; overflow:hidden; background:#111827; display:none;">
                            <div class="storageProgressBar" style="height:100%; width:0%; background:#10b981;"></div>
                        </div>
                    </div>
                </div>
                <div style="margin-top:6px; font-family: 'Courier New', monospace; font-size: 13px; color:#e5e7eb;">
                    ${infoLine}
                    ${tailLine}
                </div>
            </div>
        `;
    }).join('');

    const btns = panesEl.querySelectorAll('button.btnStorageRead');
    for (const btn of btns) {
        btn.onclick = async () => {
            const storageId = parseInt(btn.getAttribute('data-storage-id') || '0', 10);
            const storageName = btn.getAttribute('data-storage-name') || 'storage';
            const size = parseInt(btn.getAttribute('data-storage-size') || '0', 10);

            const pane = btn.closest('div');
            const prog = pane ? pane.querySelector('div.storageProgress') : null;
            const bar = prog ? prog.querySelector('div.storageProgressBar') : null;
            await readFullStorageToFile(storageId, storageName, size, { buttonEl: btn, progressEl: prog, progressBarEl: bar });
        };
    }
}

function escapeHtml(s) {
    const t = String(s ?? '');
    return t
        .replaceAll('&', '&amp;')
        .replaceAll('<', '&lt;')
        .replaceAll('>', '&gt;')
        .replaceAll('"', '&quot;')
        .replaceAll("'", '&#39;');
}

function escapeHtmlAttr(s) {
    return escapeHtml(s).replaceAll('`', '&#96;');
}

async function ccStorageReadInfoAndRender() {
    if (!cc3200Serial || !espSerial) {
        logToConsole('Not connected', 'info');
        return;
    }

    setStorageStatus('Reading storage list...');
    try {
        const listRes = await cc3200Serial.sendCommandWithResponse(
            new Uint8Array([CC3200_OPCODES.GetStorageList]),
            { kind: 'bytes', expectedBytes: 1, timeoutMs: 1000, callback: null, ctx: { cmdId: 'StorageList' } }
        );

        const mask = listRes?.data?.[0] ?? 0;
        logToConsole(`Storage: list mask=0x${mask.toString(16).padStart(2, '0')}`, 'info');

        const present = CC3200_STORAGE_BITS.filter(d => (mask & d.bit) !== 0);
        if (present.length === 0) {
            renderStoragePanes([], mask);
            setStorageStatus('No storages present');
            return;
        }

        const storages = [];
        for (const def of present) {
            setStorageStatus(`Reading ${def.name} info...`);
            try {
                const infoCmd = new Uint8Array([
                    CC3200_OPCODES.GetStorageInfo,
                    (def.storageId >>> 24) & 0xFF,
                    (def.storageId >>> 16) & 0xFF,
                    (def.storageId >>> 8) & 0xFF,
                    def.storageId & 0xFF
                ]);
                const infoRes = await cc3200Serial.sendCommandWithResponse(
                    infoCmd,
                    { kind: 'packet', expectedLen: null, timeoutMs: 2000, callback: null, ctx: { cmdId: 'StorageInfo', storageId: def.storageId } }
                );

                const parsed = parseStorageInfoPayload(infoRes?.data);
                if (!parsed) {
                    storages.push({ name: def.name, storageId: def.storageId, error: 'short payload', blockSize: 0, blockCount: 0, capacity: 0, tail: new Uint8Array(0) });
                } else {
                    storages.push({ name: def.name, storageId: def.storageId, ...parsed });
                    logToConsole(`Storage: ${def.name} block_size=${parsed.blockSize} block_count=${parsed.blockCount} cap=${parsed.capacity}`, 'info');
                }
            } catch (err) {
                storages.push({ name: def.name, storageId: def.storageId, error: err?.message || String(err), blockSize: 0, blockCount: 0, capacity: 0, tail: new Uint8Array(0) });
                logToConsole(`Storage: ${def.name} info failed (${err?.message || err})`, 'info');
            }
        }

        renderStoragePanes(storages, mask);
        setStorageStatus(`Ready (mask=0x${mask.toString(16).padStart(2, '0')})`);
    } catch (err) {
        setStorageStatus(`Error: ${err?.message || err}`);
        logToConsole(`Storage: Read Info failed (${err?.message || err})`, 'info');
    }
}

async function readFullStorageToFile(storageId, storageName, size, ui = null) {
    if (!cc3200Serial || !espSerial) {
        logToConsole('Not connected', 'info');
        return;
    }
    if (!Number.isFinite(size) || size <= 0) {
        logToConsole(`Storage: invalid size for ${storageName}`, 'info');
        return;
    }
    if (size > 0xFFFFFFFF) {
        logToConsole(`Storage: ${storageName} too large (${size} bytes)`, 'info');
        return;
    }

    const sid = storageId >>> 0;
    const safeBase = String(storageName || 'storage').toLowerCase().replaceAll(/[^a-z0-9._-]+/g, '_');
    const suggestedName = `${safeBase}_sid${sid}_len${size}.bin`;

    const ctx = {
        cmdId: 'StorageRead',
        storageId: sid,
        offset: 0,
        size: size >>> 0,
        filename: suggestedName,
        fileHandle: null,
        onChunk: null
    };

    const setProgress = (frac) => {
        if (!ui || !ui.progressEl || !ui.progressBarEl) return;
        const v = Math.max(0, Math.min(1, frac || 0));
        ui.progressEl.style.display = 'block';
        ui.progressBarEl.style.width = `${(v * 100).toFixed(2)}%`;
    };

    if (ui && ui.buttonEl) {
        ui.buttonEl.disabled = true;
    }
    setProgress(0);

    ctx.onChunk = (info) => {
        const total = info?.total ?? size;
        const done = info?.done ?? 0;
        if (total > 0) {
            setProgress(done / total);
        }
    };

    if (window.showSaveFilePicker) {
        try {
            ctx.fileHandle = await window.showSaveFilePicker({
                suggestedName,
                types: [{
                    description: 'Binary',
                    accept: { 'application/octet-stream': ['.bin', '.dat'] }
                }]
            });
        } catch (err) {
            ctx.fileHandle = null;
        }
    }

    logToConsole(`Storage: reading ${storageName} (storage_id=${sid}) size=${size} -> ${suggestedName}`, 'info');
    try {
        await rawReadRangeToFile(ctx);
        setProgress(1);
    } catch (err) {
        logToConsole(`Storage: read failed (${err?.message || err})`, 'info');
    } finally {
        if (ui && ui.buttonEl) {
            ui.buttonEl.disabled = false;
        }
    }
}

function onRawRead(ctx, payload) {
    const expected = ctx?.size ?? null;
    if (typeof expected === 'number' && expected > 0 && payload.length !== expected) {
        logToConsole(`RawRead: size mismatch (got ${payload.length}, expected ${expected})`, 'info');
    }

    const sid = ctx?.storageId ?? CC3200_STORAGE.SRAM;
    const off = ctx?.offset ?? 0;
    const fname = ctx?.filename || `raw_read_sid${sid}_off0x${(off >>> 0).toString(16)}_len${payload.length}.bin`;
    logToConsole(`RawRead(storage_id=${sid}, offset=0x${(off >>> 0).toString(16)}, size=${expected}): received ${payload.length} bytes`, 'info');

    if (ctx?.fileHandle) {
        (async () => {
            try {
                const writable = await ctx.fileHandle.createWritable();
                await writable.write(payload);
                await writable.close();
                logToConsole(`RawRead: saved to '${fname}'`, 'info');
            } catch (err) {
                logToConsole(`RawRead: save failed (${err?.message || err}), falling back to download`, 'info');
                downloadU8AsFile(payload, fname);
            }
        })();
        return;
    }

    logToConsole(`RawRead: downloading '${fname}'`, 'info');
    downloadU8AsFile(payload, fname);
}

async function runCCCommandById(cmdId) {
    const cmd = CC_COMMANDS.find(c => c.id === cmdId);
    if (!cmd) return;

    if (!cc3200Serial || !espSerial) {
        logToConsole('Not connected', 'info');
        return;
    }

    const ctx = { cmdId };
    if (cmdId === 'GetStorageInfo') {
        ctx.storageId = getTabStorageId();
    }
    if (cmdId === 'RawRead') {
        const sidEl = document.getElementById('ccRawReadStorageSelect');
        const offsetEl = document.getElementById('ccRawReadOffset');
        const sizeEl = document.getElementById('ccRawReadSize');
        const fileEl = document.getElementById('ccRawReadFilename');

        const storageId = sidEl ? parseInt(sidEl.value, 10) : CC3200_STORAGE.SRAM;
        const offset = offsetEl ? parseIntAuto(offsetEl.value) : 0;
        const size = sizeEl ? parseIntAuto(sizeEl.value) : 0;
        const filename = fileEl ? fileEl.value : '';

        if (!Number.isFinite(offset) || offset < 0) {
            logToConsole('RawRead: invalid offset', 'info');
            return;
        }
        if (!Number.isFinite(size) || size <= 0) {
            logToConsole('RawRead: size must be > 0', 'info');
            return;
        }

        ctx.storageId = Number.isFinite(storageId) ? storageId : CC3200_STORAGE.SRAM;
        ctx.offset = offset >>> 0;
        ctx.size = size >>> 0;
        ctx.filename = filename;
        ctx.fileHandle = null;

        /* Prefer File System Access API to avoid async-download gesture restrictions */
        if (window.showSaveFilePicker) {
            try {
                const suggestedName = (filename && filename.trim()) ? filename.trim() : `raw_read_sid${ctx.storageId}_off0x${ctx.offset.toString(16)}_len${ctx.size}.bin`;
                ctx.fileHandle = await window.showSaveFilePicker({
                    suggestedName,
                    types: [{
                        description: 'Binary',
                        accept: { 'application/octet-stream': ['.bin', '.dat'] }
                    }]
                });
                ctx.filename = suggestedName;
            } catch (err) {
                /* user cancelled or not allowed; fallback to Blob download later */
                ctx.fileHandle = null;
            }
        }

        logToConsole(`CC3200 run: RawRead storage=${ctx.storageId} offset=0x${ctx.offset.toString(16)} size=${ctx.size}`, 'info');
        try {
            await rawReadRangeToFile(ctx);
        } catch (err) {
            logToConsole(`RawRead failed: ${err.message}`, 'info');
        }
        return;
    }

    const payload = cmd.build(ctx);
    const resp = { ...cmd.response, ctx };
    resp.callback = cmd.parse;

    if (cmdId === 'RawRead') {
        /* Very conservative: ~50 kB/s model + base latency */
        const estimatedMs = 1500 + Math.ceil((ctx.size ?? 0) / 50);
        resp.timeoutMs = Math.max(resp.timeoutMs ?? 0, estimatedMs);
    }

    logToConsole(`CC3200 run: ${cmdId}`, 'info');
    await cc3200Serial.sendCommandWithResponse(payload, resp);
}

