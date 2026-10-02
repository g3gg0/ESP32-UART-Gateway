let cc3200FlashImage = null;
const FLASH_SIZE = 16 * 1024 * 1024; // 16MB default flash size

async function initializeFlashImage(sizeBytes = FLASH_SIZE) {
    /* Create SparseImage with RawRead callback */
    cc3200FlashImage = new SparseImage(sizeBytes, async (address, size) => {
        /* Read data from CC3200 via RawRead */
        const client = ensureSffsClient();
        const cmd = u8Concat([
            new Uint8Array([CC3200_OPCODES.RawRead]),
            be32(CC3200_STORAGE.SFLASH >>> 0),
            be32(address >>> 0),
            be32(size >>> 0)
        ]);
        const timeoutMs = Math.max(2000, 1500 + Math.ceil(size / 50));
        const res = await client.cc.sendCommandWithResponse(cmd, {
            kind: 'packet',
            expectedLen: size,
            timeoutMs,
            callback: null,
            ctx: { cmdId: 'SparseImageRead' }
        });
        if (!res?.data || res.data.length !== size) {
            throw new Error(`SparseImage: RawRead failed (address=0x${address.toString(16)} size=${size} got=${res?.data?.length ?? 0})`);
        }
        logToConsole(`SparseImage: read 0x${address.toString(16)} size=${size}`, 'debug');
        return { address, data: res.data };
    });
    logToConsole('SparseImage initialized for CC3200 flash access', 'info');
}


async function loadFlashImageFromFile() {
    try {
        const [fileHandle] = await window.showOpenFilePicker({
            types: [{
                description: 'Flash Image',
                accept: { 'application/octet-stream': ['.bin', '.img', '.flash'] }
            }],
            multiple: false
        });

        const file = await fileHandle.getFile();
        const arrayBuffer = await file.arrayBuffer();
        const flashData = new Uint8Array(arrayBuffer);

        logToConsole(`Loading flash image: ${file.name} (${flashData.length} bytes)`, 'info');

        /* Create SparseImage from the loaded file */
        cc3200FlashImage = SparseImage.fromBuffer(flashData);

        document.getElementById('statusIndicator').textContent = `Flash Image: ${file.name}`;
        document.getElementById('statusIndicator').classList.add('connected');

        logToConsole('Flash image loaded successfully - you can now browse SFFS files', 'info');

        /* Auto-refresh SFFS list if available */
        if (typeof sffsRefreshAndRender === 'function') {
            await sffsRefreshAndRender();
        }
    } catch (err) {
        if (err.name !== 'AbortError') {
            logToConsole(`Failed to load flash image: ${err.message}`, 'info');
        }
    }
}

