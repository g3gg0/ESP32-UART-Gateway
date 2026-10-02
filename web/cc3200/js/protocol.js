const CC3200_OPCODES = {
    GetVersion: 0x2F,
    GetStorageList: 0x27,
    GetStorageInfo: 0x31,
    RawRead: 0x2C,

    StartUpload: 0x21,
    FinishUpload: 0x22,
    GetLastStatus: 0x23,
    FileChunk: 0x24,
    GetFileInfo: 0x2A,
    ReadFileChunk: 0x2B,
    EraseFile: 0x2E,
    SwitchToApps: 0x33
};

const CC3200_STORAGE = {
    SRAM: 0,
    FLASH: 1,
    SFLASH: 2
};

/* Global SparseImage for flash access */
