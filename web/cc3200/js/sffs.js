let sffsClient = null;

function setSffsStatus(text) {
    const el = document.getElementById('sffsStatus');
    if (el) el.textContent = text;
}

async function runSwitchToAppsSequence(delayTicks) {
    try {
        setSffsStatus('Switch to APPS: initializing...');
        logToConsole('Switch to APPS sequence starting...', 'info');

        /* Step 1: Send Switch UART to APPS command */
        const cmd = CC_COMMANDS.find(c => c.id === 'SwitchToApps');
        if (!cmd) {
            throw new Error('SwitchToApps command not found');
        }
        const payload = cmd.build({ delayTicks });
        logToConsole(`Sending Switch to APPS command (delay=${delayTicks} ticks)...`, 'info');
        const res = await cc3200Serial.sendCommandWithResponse(payload, {
            kind: 'bytes',
            expectedBytes: 0,
            timeoutMs: 2000
        });
        logToConsole('Switch to APPS: ACK received from main processor', 'info');

        /* Step 2: Wait for network processor to be ready (delay duration) */
        setSffsStatus(`Switch to APPS: waiting ${delayTicks} ticks for NWP reboot...`);
        const waitTime = Math.ceil(delayTicks / 26666667 * 1000); /* Convert ticks to approximate ms */
        logToConsole(`Waiting ${waitTime}ms for network processor to initialize...`, 'info');
        await new Promise(resolve => setTimeout(resolve, Math.max(1000, waitTime)));

        /* Step 3-5: Send break signals (up to 4 times) and wait for NWP ACK */
        let nwpReady = false;
        for (let i = 1; i <= 40; i++) {
            setSffsStatus(`Switch to APPS: sending break signal ${i}/4...`);
            logToConsole(`Break signal ${i}/4: asserting break on RX line...`, 'info');

            /* Set break (spacing condition) */
            if (espSerial && espSerial.port) {
                await espSerial.sendBreak();
            }

            /* Wait for ACK from network processor (they check during power-up) */
            try {
                logToConsole(`Break signal ${i}/4: listening for NWP ACK (100ms)...`, 'info');
                /* Give NWP 100ms to sense the break and respond */
                await new Promise(resolve => setTimeout(resolve, 500));

                /* Check if we got an ACK (0x00 0xCC) in the receive buffer */
                /* The processCC3200Response will handle this automatically */
                if (cc3200Serial.cc3200State === 'waitingForAck' || cc3200Serial.rxBuffer.length >= 2) {
                    const ackBytes = cc3200Serial.rxBuffer.slice(cc3200Serial.rxBuffer.length - 2);
                    if (ackBytes[0] === 0x00 && ackBytes[1] === 0xCC) {
                        nwpReady = true;
                        logToConsole(`Break signal ${i}/4: NWP ACK received!`, 'info');
                        /* Clear the buffer to avoid confusion */
                        cc3200Serial.rxBuffer = new Uint8Array(0);
                        break;
                    }
                }
            } catch (e) {
                logToConsole(`Break signal ${i}/4: timeout listening for ACK, continuing...`, 'info');
            }

            /* Deassert break before next iteration */
            logToConsole(`Break signal ${i}/4: deasserting break...`, 'info');
            /* Break is naturally deasserting after the condition ends */

            if (i < 4) {
                await new Promise(resolve => setTimeout(resolve, 100));
            }
        }

        if (nwpReady) {
            setSffsStatus('Switch to APPS: NWP bootloader ready!');
            logToConsole('Switch to APPS: Network processor bootloader is ready for communication', 'info');
        } else {
            setSffsStatus('Switch to APPS: break signal sent (check NWP response)');
            logToConsole('Switch to APPS: break signals sent, NWP bootloader should be initializing', 'info');
        }

    } catch (err) {
        setSffsStatus(`Switch to APPS failed: ${err.message}`);
        logToConsole(`Switch to APPS error: ${err.message}`, 'error');
    }
}

function ensureSffsClient() {
    /* Allow working with flash image only (no serial connection) */
    if (cc3200FlashImage && !cc3200Serial) {
        /* Create a mock CC3200Serial for flash image access */
        if (!sffsClient) {
            const mockCC3200 = {
                sendCommandWithResponse: async () => {
                    throw new Error('Cannot send commands when using flash image - use SparseImage directly');
                }
            };
            sffsClient = new SffsClient(mockCC3200);
        }
        return sffsClient;
    }

    /* Normal mode: require serial connection */
    if (!cc3200Serial || !espSerial) {
        throw new Error('Not connected');
    }
    if (!sffsClient) {
        sffsClient = new SffsClient(cc3200Serial);
    }
    return sffsClient;
}

function renderSffsList(result) {
    const el = document.getElementById('sffsList');
    if (!el) return;

    const files = result?.files ?? [];
    if (!files.length) {
        el.innerHTML = `<div style="color:#9ca3af;">No files found in FAT r${result?.fatCommitRevision ?? '?'}.</div>`;
        return;
    }

    const rows = files
        .slice()
        .sort((a, b) => a.index - b.index)
        .map(f => {
            const name = escapeHtml(f.fname);
            const ms = f.mirrored ? 'yes' : 'no';
            const fileKey = `sffs_${f.index}`;
            return `
                <tr>
                    <td style="padding:6px 8px; border-bottom:1px solid #374151;">${f.index}</td>
                    <td style="padding:6px 8px; border-bottom:1px solid #374151;">${f.startBlock}</td>
                    <td style="padding:6px 8px; border-bottom:1px solid #374151;">${f.sizeBlocks}</td>
                    <td style="padding:6px 8px; border-bottom:1px solid #374151;">${ms}</td>
                    <td style="padding:6px 8px; border-bottom:1px solid #374151;">0x${f.flags.toString(16)}</td>
                    <td style="padding:6px 8px; border-bottom:1px solid #374151; font-family: 'Courier New', monospace;">
                        <a href="#" class="sffsFileLink" data-filename="${escapeHtmlAttr(f.fname)}" style="color:#3b82f6; cursor:pointer; text-decoration:none; word-break:break-all;">${name}</a>
                        <button class="sffsDeleteBtn" data-filename="${escapeHtmlAttr(f.fname)}" style="margin-left:8px; padding:4px 8px; background:#ef4444; color:white; border:none; border-radius:4px; cursor:pointer; font-weight:600; font-size:12px;" title="Delete file">🗑️</button>
                    </td>
                </tr>
            `;
        }).join('');

    const hdr = `
        <div style="margin-bottom:10px; color:#9ca3af; font-size:12px;">
            FAT r${result.fatCommitRevision} (copy #${result.selectedFatIndex})
            ${result.otherFatCommitRevision !== null ? `| other r${result.otherFatCommitRevision}` : ''}
            | block_size=${result.storageInfo.blockSize} block_count=${result.storageInfo.blockCount}
        </div>
    `;

    el.innerHTML = hdr + `
        <table style="width:100%; border-collapse:collapse; font-size:13px;">
            <thead>
                <tr style="text-align:left; color:#9ca3af;">
                    <th style="padding:6px 8px; border-bottom:1px solid #374151;">idx</th>
                    <th style="padding:6px 8px; border-bottom:1px solid #374151;">start</th>
                    <th style="padding:6px 8px; border-bottom:1px solid #374151;">size(BLK)</th>
                    <th style="padding:6px 8px; border-bottom:1px solid #374151;">mirror</th>
                    <th style="padding:6px 8px; border-bottom:1px solid #374151;">flags</th>
                    <th style="padding:6px 8px; border-bottom:1px solid #374151;">name</th>
                </tr>
            </thead>
            <tbody>
                ${rows}
            </tbody>
        </table>
    `;

    /* Wire up file download links */
    const downloadLinks = el.querySelectorAll('a.sffsFileLink');
    for (const link of downloadLinks) {
        link.onclick = (e) => {
            e.preventDefault();
            const filename = link.getAttribute('data-filename');
            sffsReadAndDownloadFile(filename);
        };
    }

    /* Wire up delete buttons */
    const deleteButtons = el.querySelectorAll('button.sffsDeleteBtn');
    for (const btn of deleteButtons) {
        btn.onclick = async (e) => {
            e.preventDefault();
            const filename = btn.getAttribute('data-filename');
            if (confirm(`Delete file: ${filename}?`)) {
                await sffsDeleteFile(filename);
            }
        };
    }
}

async function sffsRefreshAndRender() {
    try {
        setSffsStatus('Reading FAT...');
        const inactiveEl = document.getElementById('sffsInactive');
        const inactive = !!inactiveEl?.checked;
        const client = ensureSffsClient();
        const res = await client.listFiles({ inactive });
        renderSffsList(res);
        setSffsStatus(`Ready (r${res.fatCommitRevision}, ${res.files.length} files)`);
        logToConsole(`SFFS: listed ${res.files.length} files (FAT r${res.fatCommitRevision})`, 'info');
    } catch (err) {
        setSffsStatus(`Error: ${err?.message || err}`);
        logToConsole(`SFFS: list failed (${err?.message || err})`, 'info');
    }
}

async function sffsReadAndDownload() {
    try {
        const client = ensureSffsClient();
        const nameEl = document.getElementById('sffsFilename');
        const filename = (nameEl?.value ?? '').trim();
        if (!filename) {
            throw new Error('SFFS: filename required');
        }

        setSffsStatus(`Reading ${filename}...`);
        const finfo = await client.getFileInfo(filename);
        if (!finfo.exists) {
            throw new Error(`SFFS: not found (${filename})`);
        }

        const res = await client.readFile(filename, {
            onProgress: (p) => {
                setSffsStatus(`Reading ${filename}: ${p.done}/${p.total} bytes (${(p.done * 100 / p.total).toFixed(2)}%)`);
            }
        });

        const base = filename.split('/').filter(Boolean).pop() || 'download.bin';
        const suggestedName = base;

        if (window.showSaveFilePicker) {
            try {
                const fileHandle = await window.showSaveFilePicker({
                    suggestedName,
                    types: [{
                        description: 'Binary',
                        accept: { 'application/octet-stream': ['.bin', '.dat', '.txt'] }
                    }]
                });
                const writable = await fileHandle.createWritable();
                await writable.write(res.data);
                await writable.close();
                setSffsStatus(`Saved ${suggestedName} (${res.size} bytes)`);
                logToConsole(`SFFS: saved '${suggestedName}' (${res.size} bytes)`, 'info');
                return;
            } catch (e) {
                /* fall back to download */
            }
        }

        downloadU8AsFile(res.data, suggestedName);
        setSffsStatus(`Downloaded ${suggestedName} (${res.size} bytes)`);
        logToConsole(`SFFS: downloaded '${suggestedName}' (${res.size} bytes)`, 'info');
    } catch (err) {
        setSffsStatus(`Error: ${err?.message || err}`);
        logToConsole(`SFFS: read failed (${err?.message || err})`, 'info');
    }
}

async function sffsWriteFromFileInput() {
    try {
        const client = ensureSffsClient();
        const nameEl = document.getElementById('sffsFilename');
        const filename = (nameEl?.value ?? '').trim();
        if (!filename) {
            throw new Error('SFFS: filename required');
        }

        const uploadEl = document.getElementById('sffsUpload');
        const file = uploadEl?.files?.[0];
        if (!file) {
            throw new Error('SFFS: select a local file first');
        }

        const commitEl = document.getElementById('sffsCommit');
        const commit = !!commitEl?.checked;

        const bytes = new Uint8Array(await file.arrayBuffer());
        setSffsStatus(`Uploading ${file.name} -> ${filename} (${bytes.length} bytes)...`);

        await client.writeFile(filename, bytes, {
            commit,
            eraseIfExists: true,
            onProgress: (p) => {
                setSffsStatus(`Uploading ${filename}: ${p.done}/${p.total} bytes (${(p.done * 100 / p.total).toFixed(2)}%)`);
            }
        });

        setSffsStatus(`Upload complete (${bytes.length} bytes)`);
        logToConsole(`SFFS: uploaded '${filename}' (${bytes.length} bytes) commit=${commit}`, 'info');
        await sffsRefreshAndRender();
    } catch (err) {
        setSffsStatus(`Error: ${err?.message || err}`);
        logToConsole(`SFFS: write failed (${err?.message || err})`, 'info');
    }
}

async function sffsErase() {
    try {
        const client = ensureSffsClient();
        const nameEl = document.getElementById('sffsFilename');
        const filename = (nameEl?.value ?? '').trim();
        if (!filename) {
            throw new Error('SFFS: filename required');
        }

        setSffsStatus(`Erasing ${filename}...`);
        await client.eraseFile(filename);
        setSffsStatus('Erase complete');
        logToConsole(`SFFS: erased '${filename}'`, 'info');
        await sffsRefreshAndRender();
    } catch (err) {
        setSffsStatus(`Error: ${err?.message || err}`);
        logToConsole(`SFFS: erase failed (${err?.message || err})`, 'info');
    }
}

async function sffsReadAndDownloadFile(filename) {
    try {
        const client = ensureSffsClient();
        setSffsStatus(`Reading ${filename}...`);

        /* Initialize SparseImage if needed */
        if (!cc3200FlashImage) {
            await initializeFlashImage();
        }

        /* Find file in FAT for block location */
        const listResult = await client.listFiles({ inactive: false });
        const fatEntry = listResult.files.find(f => f.fname === filename);

        if (!fatEntry) {
            throw new Error(`SFFS: file not found in FAT (${filename})`);
        }

        /* Calculate byte offset from FAT entry */
        const blockSize = fatEntry.blockSize;
        const headerOffset = fatEntry.startBlock * blockSize;

        /* First, read the 8-byte SFFS file header to get actual file size */
        logToConsole(`SFFS: reading file header at 0x${headerOffset.toString(16)}`, 'info');

        /* Use SparseImage to read header */
        await cc3200FlashImage.prefetch(headerOffset, 8);
        const header = await cc3200FlashImage.subarray_async(headerOffset, headerOffset + 8);

        /* Parse file size from header (first 4 bytes, little-endian) */
        const fileSize = le32From(header, 0) & 0xffffff;
        const headerHex = Array.from(header).map(b => b.toString(16).padStart(2, '0')).join(' ');

        logToConsole(`SFFS: file header [${headerHex}] fileSize=${fileSize} bytes`, 'info');
        logToConsole(`SFFS: reading '${filename}' startBlock=${fatEntry.startBlock} (0x${fatEntry.startBlock.toString(16)}) dataOffset=0x${(headerOffset + 8).toString(16)} fileSize=${fileSize}`, 'info');

        /* Read file data starting after the 8-byte header using SparseImage */
        const dataOffset = headerOffset + 8;

        /* Prefetch file data in chunks to show progress */
        const blockReadSize = 4096;
        let pos = 0;
        while (pos < fileSize) {
            const toRead = Math.min(blockReadSize, fileSize - pos);
            await cc3200FlashImage.prefetch(dataOffset + pos, toRead);
            pos += toRead;
            setSffsStatus(`Reading ${filename}: ${pos}/${fileSize} bytes (${(pos * 100 / fileSize).toFixed(2)}%)`);
        }

        /* Get the complete file data from SparseImage */
        const fileData = await cc3200FlashImage.subarray_async(dataOffset, dataOffset + fileSize);

        const base = filename.split('/').filter(Boolean).pop() || 'download.bin';
        const suggestedName = base;

        if (window.showSaveFilePicker) {
            try {
                const fileHandle = await window.showSaveFilePicker({
                    suggestedName,
                    types: [{
                        description: 'Binary',
                        accept: { 'application/octet-stream': ['.bin', '.dat', '.txt'] }
                    }]
                });
                const writable = await fileHandle.createWritable();
                await writable.write(fileData);
                await writable.close();
                setSffsStatus(`Saved ${suggestedName} (${fileSize} bytes)`);
                logToConsole(`SFFS: saved '${suggestedName}' (${fileSize} bytes)`, 'info');
                return;
            } catch (e) {
                /* fall back to download */
            }
        }

        downloadU8AsFile(fileData, suggestedName);
        setSffsStatus(`Downloaded ${suggestedName} (${fileSize} bytes)`);
        logToConsole(`SFFS: downloaded '${suggestedName}' (${fileSize} bytes)`, 'info');
    } catch (err) {
        setSffsStatus(`Error: ${err?.message || err}`);
        logToConsole(`SFFS: read failed (${err?.message || err})`, 'info');
    }
}

async function sffsDeleteFile(filename) {
    try {
        const client = ensureSffsClient();
        setSffsStatus(`Deleting ${filename}...`);
        await client.eraseFile(filename);
        setSffsStatus('Delete complete');
        logToConsole(`SFFS: deleted '${filename}'`, 'info');
        await sffsRefreshAndRender();
    } catch (err) {
        setSffsStatus(`Error: ${err?.message || err}`);
        logToConsole(`SFFS: delete failed (${err?.message || err})`, 'info');
    }
}

