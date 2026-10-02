let flasher = null;
let flashConfig = null;
function applyFlashConfig(config, sourceLabel) {
    if (!config.files || !Array.isArray(config.files)) {
        throw new Error('Invalid JSON format: missing "files" array');
    }

    flashConfig = config;
    log(`Configuration loaded from ${sourceLabel}: ${flashConfig.files.length} file(s) to flash`, 'info');

    /* Display file list */
    const fileList = document.getElementById('fileList');
    fileList.innerHTML = '<strong>Files to flash:</strong>';
    flashConfig.files.forEach(item => {
        const div = document.createElement('div');
        div.className = 'file-item';
        let sizeInfo = '';
        if (EMBEDDED_BINARIES[item.file]) {
            const b64 = EMBEDDED_BINARIES[item.file];
            const bytes = Math.floor(b64.length * 3 / 4) - (b64.endsWith('==') ? 2 : b64.endsWith('=') ? 1 : 0);
            sizeInfo = ` (${bytes} bytes)`;
        }
        else {
            sizeInfo = ' (external file)';
        }

        div.innerHTML = `<code>${item.offset}</code> → ${item.file}${sizeInfo}`;
        fileList.appendChild(div);
    });
    fileList.style.display = 'block';

    const supported = updateWebSerialUiState();
    document.getElementById('connectBtn').disabled = !supported;
}

async function handleJsonSelect(event) {
    const file = event.target.files[0];
    if (!file) return;

    try {
        log('Reading JSON configuration...', 'info');
        const text = await file.text();
        const config = JSON.parse(text);
        applyFlashConfig(config, `file "${file.name}"`);
    } catch (error) {
        log(`Error reading JSON: ${error.message}`, 'error');
        flashConfig = null;
        document.getElementById('connectBtn').disabled = true;
    }
}

async function autoLoadConfig() {
    /* Built-in default configuration */
    const config = EMBEDDED_FLASH_CONFIG;

    try {
        applyFlashConfig(config, 'built-in default');
    } catch (error) {
        log(`Failed to apply default config: ${error.message}`, 'error');
    }
}

/* LOAD_BINARY_FILE_START */
async function loadBinaryFile(filename) {
    if (EMBEDDED_BINARIES[filename]) {
        log(`Using embedded binary: ${filename}`, 'info');
        return getEmbeddedBinary(filename);
    }
    throw new Error(`Embedded binary not found: ${filename}`);
}
/* LOAD_BINARY_FILE_END */

async function connectAndFlash() {
    if (!isWebSerialSupported()) {
        const msg = getWebSerialUnsupportedMessage();
        log(msg, 'error');
        const logSection = document.getElementById('logSection');
        if (logSection) logSection.style.display = 'block';
        return;
    }
    if (!flashConfig) {
        log('No configuration loaded', 'error');
        return;
    }

    try {
        document.getElementById('connectBtn').disabled = true;
        document.getElementById('disconnectBtn').disabled = false;
        document.querySelector('.progress-bar').style.display = 'block';

        // Create flasher instance
        log('Requesting serial port...', 'info');
        flasher = new ESPFlasher();
        flasher.logMessage = (msg) => log(msg, 'info');
        flasher.logError = (msg) => log(`[ERROR] ${msg}`, 'error');

        // Open port
        await flasher.openPort();
        log('Port opened successfully', 'info');

        // Reset to bootloader
        updateProgress(5, 'Resetting device...');
        await flasher.hardReset(true);
        log('Device reset to bootloader mode', 'info');

        // Sync with device
        updateProgress(10, 'Syncing...');
        await flasher.sync();
        log(`Device detected: ${flasher.current_chip}`, 'info');

        // Load stub for faster flashing
        updateProgress(15, 'Loading stub...');
        await flasher.downloadStub();
        log('Stub loader activated', 'info');

        // Flash each file
        const totalFiles = flashConfig.files.length;
        for (let i = 0; i < totalFiles; i++) {
            const item = flashConfig.files[i];
            const offset = parseInt(item.offset);

            log(`Flashing ${item.file} at ${item.offset}...`, 'info');

            // Load binary data
            const data = await loadBinaryFile(item.file);
            log(`Loaded ${data.length} bytes from ${item.file}`, 'info');

            // Calculate progress range for this file
            const baseProgress = 20 + (i * 70 / totalFiles);
            const progressRange = 70 / totalFiles;

            // Flash the data
            await flasher.writeFlash(offset, data, (written, total, status) => {
                const fileProgress = (written / total) * progressRange;
                const totalProgress = baseProgress + fileProgress;
                updateProgress(totalProgress, `${item.file}: ${Math.round((written / total) * 100)}%`);
            });

            log(`✓ ${item.file} flashed successfully`, 'info');
        }

        updateProgress(95, 'Resetting to application...');
        await flasher.hardReset(false);

        updateProgress(100, 'Complete!');
        log('All files flashed successfully! Device is rebooting.', 'info');

        // Flash green background on entire page
        const body = document.body;
        body.style.transition = 'background 0.5s ease';
        body.style.background = 'radial-gradient(circle at 20% 20%, #27ae60 0%, #1e8449 40%, #145a32 75%)';
        console.log('Setting green background');

        setTimeout(() => {
            body.style.background = 'radial-gradient(circle at 20% 20%, #1f2937 0%, #0b1220 40%, #070b15 75%)';
            console.log('Fading back to original background');
            setTimeout(() => {
                body.style.transition = '';
                body.style.background = '';
            }, 500);
        }, 1000);

        // Auto disconnect after a few seconds
        setTimeout(async () => {
            await disconnect();
        }, 2000);

    } catch (error) {
        log(`Flashing failed: ${error.message}`, 'error');
        document.getElementById('logSection').style.display = 'block';
        updateProgress(0, 'Failed');
        document.querySelector('.progress-bar').style.display = 'none';
        document.getElementById('connectBtn').disabled = !isWebSerialSupported();
        document.getElementById('disconnectBtn').disabled = true;
    }
}

async function disconnect() {
    if (flasher) {
        try {
            await flasher.disconnect();
            log('Disconnected', 'info');
        } catch (error) {
            log(`Disconnect error: ${error.message}`, 'error');
        }
        flasher = null;
    }
    document.getElementById('connectBtn').disabled = !isWebSerialSupported() ? true : (flashConfig ? false : true);
    document.getElementById('disconnectBtn').disabled = true;
    document.querySelector('.progress-bar').style.display = 'none';
    document.getElementById('logSection').style.display = 'none';
    updateProgress(0, '0%');
}

