class SffsFATParser {
    static SFFS_HEADER_SIGNATURE = 0x534c;
    static SFFS_FAT_METADATA2_OFFSET = 0x774;
    static SFFS_FAT_FILE_NAME_ARRAY_OFFSET = 0x974;
    static SFFS_FAT_TOKEN_ARRAY_OFFSET = 0x204; // Tokens start after metadata1 (128 entries * 4 bytes = 0x204)

    static parseFatHeader(fatBytes) {
        if (!fatBytes || fatBytes.length < 4) {
            return { isValid: false, fatCommitRevision: 0, headerSign: 0 };
        }
        const fatCommitRevision = le16From(fatBytes, 0);
        const headerSign = le16From(fatBytes, 2);
        if (fatCommitRevision === 0xFFFF || headerSign === 0xFFFF) {
            return { isValid: false, fatCommitRevision, headerSign };
        }
        if (headerSign !== this.SFFS_HEADER_SIGNATURE) {
            return { isValid: false, fatCommitRevision, headerSign };
        }
        return { isValid: true, fatCommitRevision, headerSign };
    }

    static parseFiles(fatBytes) {
        const files = [];
        const decoder = new TextDecoder('utf-8', { fatal: false });

        /* Debug: Show token region (first 64 bytes of token array) */
        const tokenRegionStart = this.SFFS_FAT_TOKEN_ARRAY_OFFSET;
        const tokenRegionSample = fatBytes.slice(tokenRegionStart, tokenRegionStart + 64);
        const tokenHex = Array.from(tokenRegionSample).map(b => b.toString(16).padStart(2, '0').toUpperCase()).join(' ');
        logToConsole(`SFFS Token Region @0x${tokenRegionStart.toString(16)}: ${tokenHex}`, 'info');

        for (let i = 0; i < 128; i++) {
            const metaOff = (i + 1) * 4;
            if (metaOff + 4 > fatBytes.length) break;
            const b0 = fatBytes[metaOff + 0];
            const b1 = fatBytes[metaOff + 1];
            const b2 = fatBytes[metaOff + 2];
            const b3 = fatBytes[metaOff + 3];

            const isAllFF = (b0 === 0xFF && b1 === 0xFF && b2 === 0xFF && b3 === 0xFF);
            const isEmptyMid = (b0 === 0xFF && b1 === i && b2 === 0xFF && b3 === 0x7F);
            if (isAllFF || isEmptyMid) {
                continue;
            }

            const index = b0;
            const sizeBlocks = b1;
            const startBlockLsb = b2;
            const flagsSbMsb = b3;
            if (index !== i) {
                throw new Error(`SFFS FAT entry index mismatch (entry ${i} has index ${index})`);
            }

            const flags = (flagsSbMsb >>> 4) & 0x0F;
            const startBlockMsb = flagsSbMsb & 0x0F;
            const mirrored = (flags & 0x4) === 0;
            const startBlock = ((startBlockMsb << 8) | startBlockLsb) >>> 0;
            const totalBlocks = mirrored ? (sizeBlocks * 2) : sizeBlocks;

            const meta2Off = this.SFFS_FAT_METADATA2_OFFSET + i * 4;
            if (meta2Off + 4 > fatBytes.length) {
                continue;
            }
            const fnameOffset = le16From(fatBytes, meta2Off + 0);
            const fnameLen = le16From(fatBytes, meta2Off + 2);
            const fnameAbs = this.SFFS_FAT_FILE_NAME_ARRAY_OFFSET + fnameOffset;
            const fnameBytes = (fnameAbs + fnameLen <= fatBytes.length)
                ? fatBytes.slice(fnameAbs, fnameAbs + fnameLen)
                : new Uint8Array(0);

            const fname = decoder.decode(fnameBytes);

            /* Read filename token (32-bit hash at token array offset) */
            const tokenOff = this.SFFS_FAT_TOKEN_ARRAY_OFFSET + i * 4;
            const token = (tokenOff + 4 <= fatBytes.length)
                ? le32From(fatBytes, tokenOff)
                : 0xFFFFFFFF;

            /* Check if filename is missing (meta2 = FF FF FF FF) */
            const hasFilename = !(fnameOffset === 0xFFFF && fnameLen === 0xFFFF);

            /* DEBUG: Log parsed entry with offset details */
            const hexFname = Array.from(fnameBytes).map(b => b.toString(16).padStart(2, '0')).join('');
            const meta2Hex = Array.from(fatBytes.slice(meta2Off, meta2Off + 4)).map(b => b.toString(16).padStart(2, '0')).join(' ');
            const meta1Hex = `${b0.toString(16).padStart(2, '0')} ${b1.toString(16).padStart(2, '0')} ${b2.toString(16).padStart(2, '0')} ${b3.toString(16).padStart(2, '0')}`;
            const tokenHex = `0x${token.toString(16).padStart(8, '0')}`;
            const displayName = hasFilename ? `"${fname}"` : `[token ${tokenHex}]`;
            logToConsole(`SFFS FAT[${i}]: meta1=[${meta1Hex}] meta2@0x${meta2Off.toString(16)}=[${meta2Hex}] token=${tokenHex} fnameAbs=0x${fnameAbs.toString(16)} len=${fnameLen} name=${displayName}`, 'info');

            files.push({
                index,
                startBlock,
                sizeBlocks,
                mirrored,
                flags,
                totalBlocks,
                fname,
                token,
                hasFilename,
                fnameBytes
            });
        }

        return files;
    }
}
