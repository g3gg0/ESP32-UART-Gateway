const CC_COMMANDS = [
    {
        id: 'Storage',
        title: 'Storage',
        description: 'Reads storage list + per-storage info, then allows full dumps to .bin',
        render: function (panelEl) {
            const template = document.getElementById('cc-view-Storage');
            panelEl.replaceChildren(template.content.cloneNode(true));
            panelEl.querySelectorAll('[data-command-description]').forEach(el => {
                el.textContent = this.description || '';
            });
        }
    },
    {
        id: 'SFFS',
        title: 'SFFS',
        description: 'List / read / write files in CC3200 SFFS (FAT in serial flash)',
        render: function (panelEl) {
            const template = document.getElementById('cc-view-SFFS');
            panelEl.replaceChildren(template.content.cloneNode(true));
            panelEl.querySelectorAll('[data-command-description]').forEach(el => {
                el.textContent = this.description || '';
            });
        }
    },
    {
        id: 'GetVersion',
        title: 'GetVersion',
        description: 'Opcode 0x2F, response: packet(28)',
        build: () => new Uint8Array([CC3200_OPCODES.GetVersion]),
        response: { kind: 'packet', expectedLen: 28, timeoutMs: 1500 },
        parse: onGetVersion,
        render: function (panelEl) {
            const template = document.getElementById('cc-view-GetVersion');
            panelEl.replaceChildren(template.content.cloneNode(true));
            panelEl.querySelectorAll('[data-command-description]').forEach(el => {
                el.textContent = this.description || '';
            });
        }
    },
    {
        id: 'GetStorageList',
        title: 'GetStorageList',
        description: 'Opcode 0x27, response: 1 byte',
        build: () => new Uint8Array([CC3200_OPCODES.GetStorageList]),
        response: { kind: 'bytes', expectedBytes: 1, timeoutMs: 700 },
        parse: onGetStorageList,
        render: function (panelEl) {
            const template = document.getElementById('cc-view-GetStorageList');
            panelEl.replaceChildren(template.content.cloneNode(true));
            panelEl.querySelectorAll('[data-command-description]').forEach(el => {
                el.textContent = this.description || '';
            });
        }
    },
    {
        id: 'GetStorageInfo',
        title: 'GetStorageInfo',
        description: 'Opcode 0x31 + be32(storage_id), response: packet',
        build: (ctx) => {
            const storageId = ctx?.storageId ?? 0;
            return new Uint8Array([
                CC3200_OPCODES.GetStorageInfo,
                (storageId >>> 24) & 0xFF,
                (storageId >>> 16) & 0xFF,
                (storageId >>> 8) & 0xFF,
                storageId & 0xFF
            ]);
        },
        response: { kind: 'packet', expectedLen: null, timeoutMs: 1500 },
        parse: onGetStorageInfo,
        render: function (panelEl) {
            const template = document.getElementById('cc-view-GetStorageInfo');
            panelEl.replaceChildren(template.content.cloneNode(true));
            panelEl.querySelectorAll('[data-command-description]').forEach(el => {
                el.textContent = this.description || '';
            });
        }
    }
    ,
    {
        id: 'RawRead',
        title: 'RawRead',
        description: 'Opcode 0x2C + be32(storage_id, offset, size), reads in 4096-byte blocks (downloads file)',
        build: (ctx) => {
            const storageId = ctx?.storageId ?? CC3200_STORAGE.SRAM;
            const offset = ctx?.offset ?? 0;
            const size = ctx?.size ?? 0;
            return u8Concat([
                new Uint8Array([CC3200_OPCODES.RawRead]),
                be32(storageId >>> 0),
                be32(offset >>> 0),
                be32(size >>> 0)
            ]);
        },
        response: { kind: 'packet', expectedLen: null, timeoutMs: 4000 },
        parse: onRawRead,
        render: function (panelEl) {
            const template = document.getElementById('cc-view-RawRead');
            panelEl.replaceChildren(template.content.cloneNode(true));
            panelEl.querySelectorAll('[data-command-description]').forEach(el => {
                el.textContent = this.description || '';
            });
        }
    }
    ,
    {
        id: 'SwitchToApps',
        title: 'Switch to APPS',
        description: 'Opcode 0x33 + be32(delay_ticks) - Switches UART from bootloader to CC3100 NWP (CC3200 only, requires ~1 sec delay = 26666667 ticks)',
        build: (ctx) => {
            const delayTicks = ctx?.delayTicks ?? 26666667;
            return u8Concat([
                new Uint8Array([CC3200_OPCODES.SwitchToApps]),
                be32(delayTicks >>> 0)
            ]);
        },
        response: { kind: 'bytes', expectedBytes: 0, timeoutMs: 2000 },
        parse: function (data) {
            logToConsole('Switch to APPS: command accepted, NWP should be initializing...', 'info');
            return data;
        },
        render: function (panelEl) {
            const template = document.getElementById('cc-view-SwitchToApps');
            panelEl.replaceChildren(template.content.cloneNode(true));
            panelEl.querySelectorAll('[data-command-description]').forEach(el => {
                el.textContent = this.description || '';
            });
        }
    }
    ,
    {
        id: 'Raw',
        title: 'Raw',
        description: 'Send raw bytes/text (Hex or CC3200 framed)',
        render: function (panelEl) {
            const template = document.getElementById('cc-view-Raw');
            panelEl.replaceChildren(template.content.cloneNode(true));
            panelEl.querySelectorAll('[data-command-description]').forEach(el => {
                el.textContent = this.description || '';
            });
        }
    }
];

