function updateSendMode() {
    sendMode = document.querySelector('input[name="sendMode"]:checked').value;
}

function initCCTabs() {
    const header = document.getElementById('ccTabsHeader');
    const body = document.getElementById('ccTabsBody');
    if (!header || !body) return;

    header.innerHTML = '';
    body.innerHTML = '';

    for (const cmd of CC_COMMANDS) {
        const btn = document.createElement('button');
        btn.textContent = cmd.title;
        btn.setAttribute('data-tab', cmd.id);
        btn.style.padding = '8px 12px';
        btn.style.border = '1px solid #374151';
        btn.style.borderRadius = '6px';
        btn.style.cursor = 'pointer';
        btn.style.background = '#1f2937';
        btn.style.color = '#e5e7eb';
        btn.style.fontWeight = '700';
        btn.onclick = () => setActiveCCTab(cmd.id);
        header.appendChild(btn);

        const panel = document.createElement('div');
        panel.id = `ccTabPanel_${cmd.id}`;
        panel.style.display = 'none';
        panel.style.padding = '10px';
        panel.style.border = '1px solid #374151';
        panel.style.borderRadius = '6px';
        panel.style.background = 'rgba(15, 23, 42, 0.6)';
        if (cmd.render) {
            cmd.render(panel);
        }
        body.appendChild(panel);
    }

    if (CC_COMMANDS.length > 0) {
        setActiveCCTab(CC_COMMANDS[0].id);
    }

    wireCCTabButtons();
}

function setActiveCCTab(tabId) {
    const header = document.getElementById('ccTabsHeader');
    const body = document.getElementById('ccTabsBody');
    if (!header || !body) return;

    const buttons = header.querySelectorAll('button[data-tab]');
    for (const b of buttons) {
        const active = b.getAttribute('data-tab') === tabId;
        b.style.background = active ? '#2563eb' : '#1f2937';
        b.style.borderColor = active ? '#3b82f6' : '#374151';
    }

    for (const cmd of CC_COMMANDS) {
        const panel = document.getElementById(`ccTabPanel_${cmd.id}`);
        if (!panel) continue;
        panel.style.display = cmd.id === tabId ? 'block' : 'none';
    }
}

function getTabStorageId() {
    const el = document.getElementById('ccTabStorageSelect');
    if (!el) return CC3200_STORAGE.SRAM;
    const val = parseInt(el.value, 10);
    if (isNaN(val)) return CC3200_STORAGE.SRAM;
    return val;
}

function wireCCTabButtons() {
    const btnStorage = document.getElementById('btnStorageReadInfo');
    if (btnStorage) btnStorage.onclick = () => ccStorageReadInfoAndRender();

    const btnSffsRefresh = document.getElementById('btnSffsRefresh');
    if (btnSffsRefresh) btnSffsRefresh.onclick = () => sffsRefreshAndRender();

    const btnSffsRead = document.getElementById('btnSffsRead');
    if (btnSffsRead) btnSffsRead.onclick = () => sffsReadAndDownload();

    const btnSffsWrite = document.getElementById('btnSffsWrite');
    if (btnSffsWrite) btnSffsWrite.onclick = () => sffsWriteFromFileInput();

    const btnSffsErase = document.getElementById('btnSffsErase');
    if (btnSffsErase) btnSffsErase.onclick = () => sffsErase();

    const btnVer = document.getElementById('btnRun_GetVersion');
    if (btnVer) btnVer.onclick = () => runCCCommandById('GetVersion');

    const btnList = document.getElementById('btnRun_GetStorageList');
    if (btnList) btnList.onclick = () => runCCCommandById('GetStorageList');

    const btnInfo = document.getElementById('btnRun_GetStorageInfo');
    if (btnInfo) btnInfo.onclick = () => runCCCommandById('GetStorageInfo');

    const btnRawRead = document.getElementById('btnRun_RawRead');
    if (btnRawRead) btnRawRead.onclick = () => runCCCommandById('RawRead');

    const btnSwitchToApps = document.getElementById('btnRun_SwitchToApps');
    if (btnSwitchToApps) btnSwitchToApps.onclick = () => {
        const delayInput = document.getElementById('ccSwitchToAppsDelay');
        const delayTicks = parseInt(delayInput?.value || '26666667', 10);
        runSwitchToAppsSequence(delayTicks);
    };
}

