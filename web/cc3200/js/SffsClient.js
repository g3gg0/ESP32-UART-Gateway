class SffsClient {
    constructor(cc) {
        this.cc = cc;
        this.storageId = CC3200_STORAGE.SFLASH;
        this.flashBlockSizes = [0x100, 0x400, 0x1000, 0x4000, 0x10000];
        this.slfsBlockSize = 4096;
        this.SLFS_FILE_OPEN_FLAG_COMMIT = 0x1;
        this.SLFS_MODE_OPEN_WRITE_CREATE_IF_NOT_EXIST = 3;
    }

    async getStorageInfo() {
        /* If using flash image offline (no serial connection), return default info */
        if (cc3200FlashImage && (!cc3200Serial || !espSerial)) {
            return {
                blockSize: 4096,
                numBlocks: Math.floor(cc3200FlashImage.size / 4096),
                storageId: this.storageId
            };
        }

        const cmd = u8Concat([
            new Uint8Array([CC3200_OPCODES.GetStorageInfo]),
            be32(this.storageId >>> 0)
        ]);
        const res = await this.cc.sendCommandWithResponse(cmd, {
            kind: 'packet',
            expectedLen: null,
            timeoutMs: 2000,
            callback: null,
            ctx: { cmdId: 'SffsGetStorageInfo' }
        });
        const parsed = parseStorageInfoPayload(res?.data);
        if (!parsed) {
            throw new Error('SFFS: GetStorageInfo returned short payload');
        }
        return parsed;
    }

    async readFatPair() {
        const info = await this.getStorageInfo();

        /* Read FAT directly from SparseImage if available */
        if (cc3200FlashImage) {
            await cc3200FlashImage.prefetch(0, 2 * info.blockSize);
            const bytes = await cc3200FlashImage.subarray_async(0, 2 * info.blockSize);

            if (bytes.length < 2 * info.blockSize) {
                throw new Error(`SFFS: FAT read short (got ${bytes.length}, expected ${2 * info.blockSize})`);
            }

            const fat0 = bytes.slice(0, info.blockSize);
            const fat1 = bytes.slice(info.blockSize, 2 * info.blockSize);
            return { info, fat0, fat1 };
        }

        /* Fall back to rawReadRangeToU8 for hardware mode */
        const bytes = await rawReadRangeToU8({
            storageId: this.storageId,
            offset: 0,
            size: (2 * info.blockSize) >>> 0
        });

        if (bytes.length < 2 * info.blockSize) {
            throw new Error(`SFFS: FAT read short (got ${bytes.length}, expected ${2 * info.blockSize})`);
        }

        const fat0 = bytes.slice(0, info.blockSize);
        const fat1 = bytes.slice(info.blockSize, 2 * info.blockSize);
        return { info, fat0, fat1 };
    }

    async listFiles({ inactive = false } = {}) {
        const { info, fat0, fat1 } = await this.readFatPair();

        const h0 = SffsFATParser.parseFatHeader(fat0);
        const h1 = SffsFATParser.parseFatHeader(fat1);
        const valids = [];
        if (h0.isValid) valids.push({ idx: 0, header: h0, fat: fat0 });
        if (h1.isValid) valids.push({ idx: 1, header: h1, fat: fat1 });
        if (valids.length === 0) {
            throw new Error('SFFS: no valid FAT copies found');
        }
        valids.sort((a, b) => b.header.fatCommitRevision - a.header.fatCommitRevision);

        let selected = valids[0];
        if (inactive) {
            if (valids.length < 2) {
                throw new Error('SFFS: no valid inactive FAT copy found');
            }
            selected = valids[1];
        }

        const files = SffsFATParser.parseFiles(selected.fat);

        /* Store the blockSize in each file entry for direct access */
        files.forEach(f => {
            f.blockSize = info.blockSize;
        });

        return {
            storageInfo: info,
            selectedFatIndex: selected.idx,
            fatCommitRevision: selected.header.fatCommitRevision,
            otherFatCommitRevision: (valids.length > 1) ? valids.find(v => v.idx !== selected.idx)?.header?.fatCommitRevision ?? null : null,
            files
        };
    }

    async getFileInfo(filename) {
        const nameBytes = new TextEncoder().encode(filename);
        const hexName = Array.from(nameBytes).map(b => b.toString(16).padStart(2, '0')).join('');
        logToConsole(`SFFS GetFileInfo: querying "${filename}" (${nameBytes.length} bytes: ${hexName})`, 'info');
        const cmd = u8Concat([
            new Uint8Array([CC3200_OPCODES.GetFileInfo]),
            be32(nameBytes.length >>> 0),
            nameBytes
        ]);
        const res = await this.cc.sendCommandWithResponse(cmd, {
            kind: 'packet',
            expectedLen: null,
            timeoutMs: 1500,
            callback: null,
            ctx: { cmdId: 'SffsGetFileInfo' }
        });
        const p = res?.data ?? new Uint8Array(0);
        if (p.length < 8) {
            logToConsole(`SFFS GetFileInfo response short: ${p.length} bytes`, 'info');
            return { exists: false, size: 0, raw: p };
        }
        const exists = p[0] === 0x01;
        const size = be32From(p, 4);
        return { exists, size, raw: p };
    }

    async getLastStatus() {
        const res = await this.cc.sendCommandWithResponse(new Uint8Array([CC3200_OPCODES.GetLastStatus]), {
            kind: 'packet',
            expectedLen: null,
            timeoutMs: 1500,
            callback: null,
            ctx: { cmdId: 'SffsGetLastStatus' }
        });
        const p = res?.data ?? new Uint8Array(0);
        const value = p.length > 0 ? p[0] : 0x00;
        return { value, isOk: value === 0x40, raw: p };
    }

    async eraseFile(filename) {
        const nameBytes = new TextEncoder().encode(filename);
        const cmd = u8Concat([
            new Uint8Array([CC3200_OPCODES.EraseFile]),
            be32(0),
            nameBytes,
            new Uint8Array([0x00])
        ]);
        await this.cc.sendCommandWaitAck(cmd, 3000);
        const st = await this.getLastStatus();
        if (!st.isOk) {
            throw new Error(`SFFS: erase failed (status=0x${st.value.toString(16).padStart(2, '0')})`);
        }
    }

    async openFile(filename, slfsFlags) {
        const nameBytes = new TextEncoder().encode(filename);
        const cmd = u8Concat([
            new Uint8Array([CC3200_OPCODES.StartUpload]),
            be32(slfsFlags >>> 0),
            be32(0),
            nameBytes,
            new Uint8Array([0x00, 0x00])
        ]);
        const res = await this.cc.sendCommandWithResponse(cmd, {
            kind: 'bytes',
            expectedBytes: 4,
            timeoutMs: 1500,
            callback: null,
            ctx: { cmdId: 'SffsOpenFile' }
        });
        return res?.data ?? new Uint8Array(0);
    }

    async closeFile(signatureBytes = null) {
        let sig = signatureBytes;
        if (!sig) {
            sig = new Uint8Array(256);
            sig.fill(0x46);
        }
        if (sig.length !== 256) {
            throw new Error('SFFS: signature must be 256 bytes');
        }
        const pad = new Uint8Array(63);
        const cmd = u8Concat([
            new Uint8Array([CC3200_OPCODES.FinishUpload]),
            pad,
            sig,
            new Uint8Array([0x00])
        ]);
        await this.cc.sendCommandWaitAck(cmd, 4000);
        const st = await this.getLastStatus();
        if (!st.isOk) {
            throw new Error(`SFFS: close failed (status=0x${st.value.toString(16).padStart(2, '0')})`);
        }
    }

    async readFile(filename, { onProgress } = {}) {
        const finfo = await this.getFileInfo(filename);
        if (!finfo.exists) {
            throw new Error(`SFFS: '${filename}' does not exist`);
        }

        await this.openFile(filename, 0);
        const chunks = [];
        let pos = 0;
        const size = finfo.size >>> 0;

        while (pos < size) {
            const toRead = Math.min(this.slfsBlockSize, size - pos);
            const cmd = u8Concat([
                new Uint8Array([CC3200_OPCODES.ReadFileChunk]),
                be32(pos >>> 0),
                be32(toRead >>> 0)
            ]);
            const timeoutMs = Math.max(2000, 1500 + Math.ceil(toRead / 50));
            const res = await this.cc.sendCommandWithResponse(cmd, {
                kind: 'packet',
                expectedLen: toRead,
                timeoutMs,
                callback: null,
                ctx: { cmdId: 'SffsReadFileChunk' }
            });
            if (!res?.data || res.data.length !== toRead) {
                throw new Error(`SFFS: read chunk failed (pos=${pos} got=${res?.data?.length ?? 0} expected=${toRead})`);
            }
            chunks.push(res.data);
            pos += toRead;
            if (onProgress) {
                try {
                    onProgress({ done: pos, total: size });
                } catch (e) {
                    /* ignore */
                }
            }
        }

        await this.closeFile();
        return { filename, size, data: u8Concat(chunks) };
    }

    _calcOpenFlagsForWrite(fileLen, commit) {
        let bsizeIdx = -1;
        let blocks = 0;

        for (let i = 0; i < this.flashBlockSizes.length; i++) {
            const bsize = this.flashBlockSizes[i];
            if ((bsize * 255) >= fileLen) {
                bsizeIdx = i;
                blocks = Math.ceil(fileLen / bsize);
                break;
            }
        }
        if (bsizeIdx < 0) {
            throw new Error('SFFS: file too big');
        }

        let flags = (((this.SLFS_MODE_OPEN_WRITE_CREATE_IF_NOT_EXIST & 0x0F) << 12) |
            ((bsizeIdx & 0x0F) << 8) |
            (blocks & 0xFF)) >>> 0;

        if (commit) {
            flags |= ((this.SLFS_FILE_OPEN_FLAG_COMMIT & 0xFF) << 16) >>> 0;
        }

        return flags >>> 0;
    }

    async writeFile(filename, bytes, { commit = false, eraseIfExists = true, onProgress } = {}) {
        if (!bytes || bytes.length === 0) {
            throw new Error('SFFS: will not upload empty file');
        }

        const finfo = await this.getFileInfo(filename);
        if (eraseIfExists && finfo.exists) {
            await this.eraseFile(filename);
        }

        const slfsFlags = this._calcOpenFlagsForWrite(bytes.length, commit);
        await this.openFile(filename, slfsFlags);

        let pos = 0;
        const total = bytes.length;
        while (pos < total) {
            const chunk = bytes.slice(pos, pos + this.slfsBlockSize);
            const cmd = u8Concat([
                new Uint8Array([CC3200_OPCODES.FileChunk]),
                be32(pos >>> 0),
                chunk
            ]);
            await this.cc.sendCommandWaitAck(cmd, 4000);
            const st = await this.getLastStatus();
            if (!st.isOk) {
                throw new Error(`SFFS: write failed at pos=${pos} (status=0x${st.value.toString(16).padStart(2, '0')})`);
            }
            pos += chunk.length;
            if (onProgress) {
                try {
                    onProgress({ done: pos, total });
                } catch (e) {
                    /* ignore */
                }
            }
        }

        await this.closeFile();
    }
}
