/* SWD state, register definitions, diagnostics and UI. */

let swd = null;
let scanInProgress = false;

let detectLoopActive = false;
let detectLoopToken = 0;


let scannedAps = [];
let activeAp = null;

let lastMemAddrText = '0xE000ED00';
let lastMemWordsText = '16';
let lastMemUseBlockRead = true;
let lastMemWriteValueText = '0x00000000';

const apHexEditors = new Map();
const apSubTabState = new Map();

const coresightTreeCache = new Map();


const SWD_UART_MAGIC = 0xCAFE;
const SWD_UART_OP_DETECT_PINS = 0x01;
const SWD_UART_OP_DEINIT = 0x02;
const SWD_UART_OP_TRANSFER = 0x03;
const SWD_UART_OP_AP_READ = 0x10;
const SWD_UART_OP_AP_READ_SINGLE = 0x11;
const SWD_UART_OP_AP_WRITE = 0x12;

const SWD_UART_STATUS_OK = 0x00;
const SWD_UART_FLAG_VERBOSE_LOG = 0x01;

/* DP register address bits A[3:2] (a23 param for SWD_UART_OP_TRANSFER) */
const DP_A23_ABORT = 0x00;
const DP_A23_CTRLSTAT = 0x01;
const DP_A23_SELECT = 0x02;
const DP_A23_RDBUFF = 0x03;

/* DP CTRL/STAT power-up bits */
const CDBGPWRUPREQ = 0x10000000;
const CDBGPWRUPACK = 0x20000000;
const CSYSPWRUPREQ = 0x40000000;
const CSYSPWRUPACK = 0x80000000;

const AP_IDR = 0xFC;
const AP_BASE = 0xF8;
const MEMAP_CSW = 0x00;
const MEMAP_TAR = 0x04;
const MEMAP_DRW = 0x0C;

/* Cortex-M System Control Space (SCS) / CoreDebug registers */
const SCS_CPUID = 0xE000ED00;
const SCS_CPACR = 0xE000ED88;
const SCS_DHCSR = 0xE000EDF0;
const SCS_DCRSR = 0xE000EDF4;
const SCS_DCRDR = 0xE000EDF8;

const SCS_DHCSR_S_HALT = (1 << 17) >>> 0;
const SCS_DHCSR_S_REGRDY = (1 << 16) >>> 0;
const SCS_DHCSR_C_MASKINTS = (1 << 3) >>> 0;
const SCS_DHCSR_C_STEP = (1 << 2) >>> 0;
const SCS_DHCSR_C_HALT = (1 << 1) >>> 0;
const SCS_DHCSR_C_DEBUGEN = (1 << 0) >>> 0;
const SCS_DHCSR_KEY = 0xA05F0000 >>> 0;

const SCS_DCRSR_RD = 0x00000000;
const SCS_DCRSR_WR = 0x00010000;

/* CoreSight component ID offsets */
const CS_CIDR_OFFS = [0xFF0, 0xFF4, 0xFF8, 0xFFC];
const CS_PIDR_OFFS = [0xFE0, 0xFE4, 0xFE8, 0xFEC, 0xFD0, 0xFD4, 0xFD8, 0xFDC];

const CS_DEVARCH = 0xFBC;
const CS_DEVTYPE = 0xFCC;

const CIDR_CLASS_ROMTABLE = 0x01;
const CIDR_CLASS_CORESIGHT = 0x09;

const ARM_ID = 0x23B;

function archId(architect, archid) {
    return (((architect & 0x7FF) << 21) | (archid & 0xFFFF)) >>> 0;
}

const CORESIGHT_DEVARCH_DESC = new Map([
    /* Ported from orig/adi.c: class0x9_devarch[] (ARM IHI0029E) */
    [archId(ARM_ID, 0x0A00), 'RAS architecture'],
    [archId(ARM_ID, 0x1A01), 'Instrumentation Trace Macrocell (ITM) architecture'],
    [archId(ARM_ID, 0x1A02), 'DWT architecture'],
    [archId(ARM_ID, 0x1A03), 'Flash Patch and Breakpoint unit (FPB) architecture'],
    [archId(ARM_ID, 0x2A04), 'Processor debug architecture (ARMv8-M)'],
    [archId(ARM_ID, 0x6A05), 'Processor debug architecture (ARMv8-R)'],
    [archId(ARM_ID, 0x0A10), 'PC sample-based profiling'],
    [archId(ARM_ID, 0x4A13), 'Embedded Trace Macrocell (ETM) architecture'],
    [archId(ARM_ID, 0x1A14), 'Cross Trigger Interface (CTI) architecture'],
    [archId(ARM_ID, 0x6A15), 'Processor debug architecture (v8.0-A)'],
    [archId(ARM_ID, 0x7A15), 'Processor debug architecture (v8.1-A)'],
    [archId(ARM_ID, 0x8A15), 'Processor debug architecture (v8.2-A)'],
    [archId(ARM_ID, 0x2A16), 'Processor Performance Monitor (PMU) architecture'],
    [archId(ARM_ID, 0x0A17), 'Memory Access Port v2 architecture'],
    [archId(ARM_ID, 0x0A27), 'JTAG Access Port v2 architecture'],
    [archId(ARM_ID, 0x0A31), 'Basic trace router'],
    [archId(ARM_ID, 0x0A37), 'Power requestor'],
    [archId(ARM_ID, 0x0A47), 'Unknown Access Port v2 architecture'],
    [archId(ARM_ID, 0x0A50), 'HSSTP architecture'],
    [archId(ARM_ID, 0x0A63), 'System Trace Macrocell (STM) architecture'],
    [archId(ARM_ID, 0x0A75), 'CoreSight ELA architecture'],
    [archId(ARM_ID, 0x0AF7), 'CoreSight ROM architecture'],
]);

function coresightDevarchDesc(devarch) {
    const v = devarch >>> 0;
    if ((v & (1 << 20)) === 0) return 'not present';
    const id = (v & (0xFFE00000 | 0x0000FFFF)) >>> 0;
    return CORESIGHT_DEVARCH_DESC.get(id) || 'unknown';
}

const ADI_PARTNUM = new Map([
    /* Ported from orig/adi.c: dap_part_nums[] */
    ['23b:000', { type: 'Cortex-M3 SCS', full: '(System Control Space)' }],
    ['23b:001', { type: 'Cortex-M3 ITM', full: '(Instrumentation Trace Module)' }],
    ['23b:002', { type: 'Cortex-M3 DWT', full: '(Data Watchpoint and Trace)' }],
    ['23b:003', { type: 'Cortex-M3 FPB', full: '(Flash Patch and Breakpoint)' }],
    ['23b:008', { type: 'Cortex-M0 SCS', full: '(System Control Space)' }],
    ['23b:00a', { type: 'Cortex-M0 DWT', full: '(Data Watchpoint and Trace)' }],
    ['23b:00b', { type: 'Cortex-M0 BPU', full: '(Breakpoint Unit)' }],
    ['23b:00c', { type: 'Cortex-M4 SCS', full: '(System Control Space)' }],
    ['23b:00d', { type: 'CoreSight ETM11', full: '(Embedded Trace)' }],
    ['23b:00e', { type: 'Cortex-M7 FPB', full: '(Flash Patch and Breakpoint)' }],
    ['23b:193', { type: 'SoC-600 TSGEN', full: '(Timestamp Generator)' }],
    ['23b:470', { type: 'Cortex-M1 ROM', full: '(ROM Table)' }],
    ['23b:471', { type: 'Cortex-M0 ROM', full: '(ROM Table)' }],
    ['23b:490', { type: 'Cortex-A15 GIC', full: '(Generic Interrupt Controller)' }],
    ['23b:492', { type: 'Cortex-R52 GICD', full: '(Distributor)' }],
    ['23b:493', { type: 'Cortex-R52 GICR', full: '(Redistributor)' }],
    ['23b:4a1', { type: 'Cortex-A53 ROM', full: '(v8 Memory Map ROM Table)' }],
    ['23b:4a2', { type: 'Cortex-A57 ROM', full: '(ROM Table)' }],
    ['23b:4a3', { type: 'Cortex-A53 ROM', full: '(v7 Memory Map ROM Table)' }],
    ['23b:4a4', { type: 'Cortex-A72 ROM', full: '(ROM Table)' }],
    ['23b:4a9', { type: 'Cortex-A9 ROM', full: '(ROM Table)' }],
    ['23b:4aa', { type: 'Cortex-A35 ROM', full: '(v8 Memory Map ROM Table)' }],
    ['23b:4af', { type: 'Cortex-A15 ROM', full: '(ROM Table)' }],
    ['23b:4b5', { type: 'Cortex-R5 ROM', full: '(ROM Table)' }],
    ['23b:4b8', { type: 'Cortex-R52 ROM', full: '(ROM Table)' }],
    ['23b:4c0', { type: 'Cortex-M0+ ROM', full: '(ROM Table)' }],
    ['23b:4c3', { type: 'Cortex-M3 ROM', full: '(ROM Table)' }],
    ['23b:4c4', { type: 'Cortex-M4 ROM', full: '(ROM Table)' }],
    ['23b:4c7', { type: 'Cortex-M7 PPB ROM', full: '(Private Peripheral Bus ROM Table)' }],
    ['23b:4c8', { type: 'Cortex-M7 ROM', full: '(ROM Table)' }],
    ['23b:4e0', { type: 'Cortex-A35 ROM', full: '(v7 Memory Map ROM Table)' }],
    ['23b:4e4', { type: 'Cortex-A76 ROM', full: '(ROM Table)' }],
    ['23b:906', { type: 'CoreSight CTI', full: '(Cross Trigger)' }],
    ['23b:907', { type: 'CoreSight ETB', full: '(Trace Buffer)' }],
    ['23b:908', { type: 'CoreSight CSTF', full: '(Trace Funnel)' }],
    ['23b:909', { type: 'CoreSight ATBR', full: '(Advanced Trace Bus Replicator)' }],
    ['23b:910', { type: 'CoreSight ETM9', full: '(Embedded Trace)' }],
    ['23b:912', { type: 'CoreSight TPIU', full: '(Trace Port Interface Unit)' }],
    ['23b:913', { type: 'CoreSight ITM', full: '(Instrumentation Trace Macrocell)' }],
    ['23b:914', { type: 'CoreSight SWO', full: '(Single Wire Output)' }],
    ['23b:917', { type: 'CoreSight HTM', full: '(AHB Trace Macrocell)' }],
    ['23b:920', { type: 'CoreSight ETM11', full: '(Embedded Trace)' }],
    ['23b:921', { type: 'Cortex-A8 ETM', full: '(Embedded Trace)' }],
    ['23b:922', { type: 'Cortex-A8 CTI', full: '(Cross Trigger)' }],
    ['23b:923', { type: 'Cortex-M3 TPIU', full: '(Trace Port Interface Unit)' }],
    ['23b:924', { type: 'Cortex-M3 ETM', full: '(Embedded Trace)' }],
    ['23b:925', { type: 'Cortex-M4 ETM', full: '(Embedded Trace)' }],
    ['23b:930', { type: 'Cortex-R4 ETM', full: '(Embedded Trace)' }],
    ['23b:931', { type: 'Cortex-R5 ETM', full: '(Embedded Trace)' }],
    ['23b:932', { type: 'CoreSight MTB-M0+', full: '(Micro Trace Buffer)' }],
    ['23b:941', { type: 'CoreSight TPIU-Lite', full: '(Trace Port Interface Unit)' }],
    ['23b:950', { type: 'Cortex-A9 PTM', full: '(Program Trace Macrocell)' }],
    ['23b:955', { type: 'Cortex-A5 ETM', full: '(Embedded Trace)' }],
    ['23b:95a', { type: 'Cortex-A72 ETM', full: '(Embedded Trace)' }],
    ['23b:95b', { type: 'Cortex-A17 PTM', full: '(Program Trace Macrocell)' }],
    ['23b:95d', { type: 'Cortex-A53 ETM', full: '(Embedded Trace)' }],
    ['23b:95e', { type: 'Cortex-A57 ETM', full: '(Embedded Trace)' }],
    ['23b:95f', { type: 'Cortex-A15 PTM', full: '(Program Trace Macrocell)' }],
    ['23b:961', { type: 'CoreSight TMC', full: '(Trace Memory Controller)' }],
    ['23b:962', { type: 'CoreSight STM', full: '(System Trace Macrocell)' }],
    ['23b:975', { type: 'Cortex-M7 ETM', full: '(Embedded Trace)' }],
    ['23b:9a0', { type: 'CoreSight PMU', full: '(Performance Monitoring Unit)' }],
    ['23b:9a1', { type: 'Cortex-M4 TPIU', full: '(Trace Port Interface Unit)' }],
    ['23b:9a4', { type: 'CoreSight GPR', full: '(Granular Power Requester)' }],
    ['23b:9a5', { type: 'Cortex-A5 PMU', full: '(Performance Monitor Unit)' }],
    ['23b:9a7', { type: 'Cortex-A7 PMU', full: '(Performance Monitor Unit)' }],
    ['23b:9a8', { type: 'Cortex-A53 CTI', full: '(Cross Trigger)' }],
    ['23b:9a9', { type: 'Cortex-M7 TPIU', full: '(Trace Port Interface Unit)' }],
    ['23b:9ae', { type: 'Cortex-A17 PMU', full: '(Performance Monitor Unit)' }],
    ['23b:9af', { type: 'Cortex-A15 PMU', full: '(Performance Monitor Unit)' }],
    ['23b:9b6', { type: 'Cortex-R52 PMU/CTI/ETM', full: '(Performance Monitor Unit/Cross Trigger/ETM)' }],
    ['23b:9b7', { type: 'Cortex-R7 PMU', full: '(Performance Monitor Unit)' }],
    ['23b:9d3', { type: 'Cortex-A53 PMU', full: '(Performance Monitor Unit)' }],
    ['23b:9d7', { type: 'Cortex-A57 PMU', full: '(Performance Monitor Unit)' }],
    ['23b:9d8', { type: 'Cortex-A72 PMU', full: '(Performance Monitor Unit)' }],
    ['23b:9da', { type: 'Cortex-A35 PMU/CTI/ETM', full: '(Performance Monitor Unit/Cross Trigger/ETM)' }],
    ['23b:9e2', { type: 'SoC-600 APB-AP', full: '(APB4 Memory Access Port)' }],
    ['23b:9e3', { type: 'SoC-600 AHB-AP', full: '(AHB5 Memory Access Port)' }],
    ['23b:9e4', { type: 'SoC-600 AXI-AP', full: '(AXI Memory Access Port)' }],
    ['23b:9e5', { type: 'SoC-600 APv1 Adapter', full: '(Access Port v1 Adapter)' }],
    ['23b:9e6', { type: 'SoC-600 JTAG-AP', full: '(JTAG Access Port)' }],
    ['23b:9e7', { type: 'SoC-600 TPIU', full: '(Trace Port Interface Unit)' }],
    ['23b:9e8', { type: 'SoC-600 TMC ETR/ETS', full: '(Embedded Trace Router/Streamer)' }],
    ['23b:9e9', { type: 'SoC-600 TMC ETB', full: '(Embedded Trace Buffer)' }],
    ['23b:9ea', { type: 'SoC-600 TMC ETF', full: '(Embedded Trace FIFO)' }],
    ['23b:9eb', { type: 'SoC-600 ATB Funnel', full: '(Trace Funnel)' }],
    ['23b:9ec', { type: 'SoC-600 ATB Replicator', full: '(Trace Replicator)' }],
    ['23b:9ed', { type: 'SoC-600 CTI', full: '(Cross Trigger)' }],
    ['23b:9ee', { type: 'SoC-600 CATU', full: '(Address Translation Unit)' }],
    ['23b:c05', { type: 'Cortex-A5 Debug', full: '(Debug Unit)' }],
    ['23b:c07', { type: 'Cortex-A7 Debug', full: '(Debug Unit)' }],
    ['23b:c08', { type: 'Cortex-A8 Debug', full: '(Debug Unit)' }],
    ['23b:c09', { type: 'Cortex-A9 Debug', full: '(Debug Unit)' }],
    ['23b:c0e', { type: 'Cortex-A17 Debug', full: '(Debug Unit)' }],
    ['23b:c0f', { type: 'Cortex-A15 Debug', full: '(Debug Unit)' }],
    ['23b:c14', { type: 'Cortex-R4 Debug', full: '(Debug Unit)' }],
    ['23b:c15', { type: 'Cortex-R5 Debug', full: '(Debug Unit)' }],
    ['23b:c17', { type: 'Cortex-R7 Debug', full: '(Debug Unit)' }],
    ['23b:d03', { type: 'Cortex-A53 Debug', full: '(Debug Unit)' }],
    ['23b:d04', { type: 'Cortex-A35 Debug', full: '(Debug Unit)' }],
    ['23b:d07', { type: 'Cortex-A57 Debug', full: '(Debug Unit)' }],
    ['23b:d08', { type: 'Cortex-A72 Debug', full: '(Debug Unit)' }],
    ['23b:d0b', { type: 'Cortex-A76 Debug', full: '(Debug Unit)' }],
    ['23b:d0c', { type: 'Neoverse N1', full: '(Debug Unit)' }],
    ['23b:d13', { type: 'Cortex-R52 Debug', full: '(Debug Unit)' }],
    ['23b:d49', { type: 'Neoverse N2', full: '(Debug Unit)' }],
    ['17:120', { type: 'TI SDTI', full: '(System Debug Trace Interface)' }],
    ['17:343', { type: 'TI DAPCTL', full: '' }],
    ['17:9af', { type: 'MSP432 ROM', full: '(ROM Table)' }],
    ['1f:cd0', { type: 'Atmel CPU with DSU', full: '(CPU)' }],
    ['41:1db', { type: 'XMC4500 ROM', full: '(ROM Table)' }],
    ['41:1df', { type: 'XMC4700/4800 ROM', full: '(ROM Table)' }],
    ['41:1ed', { type: 'XMC1000 ROM', full: '(ROM Table)' }],
    ['65:000', { type: 'SHARC+/Blackfin+', full: '' }],
    ['70:440', { type: 'Qualcomm QDSS Component v1', full: '(Qualcomm Designed CoreSight Component v1)' }],
    ['bf:100', { type: 'Brahma-B53 Debug', full: '(Debug Unit)' }],
    ['bf:9d3', { type: 'Brahma-B53 PMU', full: '(Performance Monitor Unit)' }],
    ['bf:4a1', { type: 'Brahma-B53 ROM', full: '(ROM Table)' }],
    ['bf:721', { type: 'Brahma-B53 ROM', full: '(ROM Table)' }],
    ['1eb:181', { type: 'Tegra 186 ROM', full: '(ROM Table)' }],
    ['1eb:202', { type: 'Denver ETM', full: '(Denver Embedded Trace)' }],
    ['1eb:211', { type: 'Tegra 210 ROM', full: '(ROM Table)' }],
    ['1eb:302', { type: 'Denver Debug', full: '(Debug Unit)' }],
    ['1eb:402', { type: 'Denver PMU', full: '(Performance Monitor Unit)' }],
    ['20:410', { type: 'STM32F10 (med)', full: '(ROM Table)' }],
    ['20:411', { type: 'STM32F2', full: '(ROM Table)' }],
    ['20:412', { type: 'STM32F10 (low)', full: '(ROM Table)' }],
    ['20:413', { type: 'STM32F40/41', full: '(ROM Table)' }],
    ['20:414', { type: 'STM32F10 (high)', full: '(ROM Table)' }],
    ['20:415', { type: 'STM32L47/48', full: '(ROM Table)' }],
    ['20:416', { type: 'STM32L1xxx6/8/B', full: '(ROM Table)' }],
    ['20:417', { type: 'STM32L05/06', full: '(ROM Table)' }],
    ['20:418', { type: 'STM32F105xx/107', full: '(ROM Table)' }],
    ['20:419', { type: 'STM32F42/43', full: '(ROM Table)' }],
    ['20:420', { type: 'STM32F10 (med)', full: '(ROM Table)' }],
    ['20:421', { type: 'STM32F446xx', full: '(ROM Table)' }],
    ['20:422', { type: 'STM32FF358/02/03', full: '(ROM Table)' }],
    ['20:423', { type: 'STM32F401xB/C', full: '(ROM Table)' }],
    ['20:425', { type: 'STM32L031/41', full: '(ROM Table)' }],
    ['20:427', { type: 'STM32L1xxxC', full: '(ROM Table)' }],
    ['20:428', { type: 'STM32F10 (high)', full: '(ROM Table)' }],
    ['20:429', { type: 'STM32L1xxx6A/8A/BA', full: '(ROM Table)' }],
    ['20:430', { type: 'STM32F10 (xl)', full: '(ROM Table)' }],
    ['20:431', { type: 'STM32F411xx', full: '(ROM Table)' }],
    ['20:432', { type: 'STM32F373/8', full: '(ROM Table)' }],
    ['20:433', { type: 'STM32F401xD/E', full: '(ROM Table)' }],
    ['20:434', { type: 'STM32F469/79', full: '(ROM Table)' }],
    ['20:435', { type: 'STM32L43/44', full: '(ROM Table)' }],
    ['20:436', { type: 'STM32L1xxxD', full: '(ROM Table)' }],
    ['20:437', { type: 'STM32L1xxxE', full: '(ROM Table)' }],
    ['20:438', { type: 'STM32F303/34/28', full: '(ROM Table)' }],
    ['20:439', { type: 'STM32F301/02/18', full: '(ROM Table)' }],
    ['20:440', { type: 'STM32F03/5', full: '(ROM Table)' }],
    ['20:441', { type: 'STM32F412xx', full: '(ROM Table)' }],
    ['20:442', { type: 'STM32F03/9', full: '(ROM Table)' }],
    ['20:444', { type: 'STM32F03xx4', full: '(ROM Table)' }],
    ['20:445', { type: 'STM32F04/7', full: '(ROM Table)' }],
    ['20:446', { type: 'STM32F302/03/98', full: '(ROM Table)' }],
    ['20:447', { type: 'STM32L07/08', full: '(ROM Table)' }],
    ['20:448', { type: 'STM32F070/1/2', full: '(ROM Table)' }],
    ['20:449', { type: 'STM32F74/5', full: '(ROM Table)' }],
    ['20:450', { type: 'STM32H74/5', full: '(ROM Table)' }],
    ['20:451', { type: 'STM32F76/7', full: '(ROM Table)' }],
    ['20:452', { type: 'STM32F72/3', full: '(ROM Table)' }],
    ['20:457', { type: 'STM32L01/2', full: '(ROM Table)' }],
    ['20:458', { type: 'STM32F410xx', full: '(ROM Table)' }],
    ['20:460', { type: 'STM32G07/8', full: '(ROM Table)' }],
    ['20:461', { type: 'STM32L496/A6', full: '(ROM Table)' }],
    ['20:462', { type: 'STM32L45/46', full: '(ROM Table)' }],
    ['20:463', { type: 'STM32F413/23', full: '(ROM Table)' }],
    ['20:464', { type: 'STM32L412/22', full: '(ROM Table)' }],
    ['20:466', { type: 'STM32G03/04', full: '(ROM Table)' }],
    ['20:468', { type: 'STM32G431/41', full: '(ROM Table)' }],
    ['20:469', { type: 'STM32G47/48', full: '(ROM Table)' }],
    ['20:470', { type: 'STM32L4R/S', full: '(ROM Table)' }],
    ['20:471', { type: 'STM32L4P5/Q5', full: '(ROM Table)' }],
    ['20:479', { type: 'STM32G491xx', full: '(ROM Table)' }],
    ['20:480', { type: 'STM32H7A/B', full: '(ROM Table)' }],
    ['20:495', { type: 'STM32WB50/55', full: '(ROM Table)' }],
    ['20:497', { type: 'STM32WLE5xx', full: '(ROM Table)' }],
]);

function adiPartNumLookup(designer, part) {
    const key = `${(designer >>> 0).toString(16)}:${(part >>> 0).toString(16).padStart(3, '0')}`;
    return ADI_PARTNUM.get(key) || { type: 'Unrecognized', full: `D:${(designer >>> 0).toString(16)} P:${(part >>> 0).toString(16)}` };
}


async function stopSwdOperations(reason = '') {
    const wasDetecting = detectLoopActive;
    detectLoopActive = false;
    detectLoopToken++;

    if (wasDetecting) {
        const why = reason ? ` (${reason})` : '';
        logToConsole(`SWD detect loop stopped${why}`, 'info');
    }

    const btnDetect = document.getElementById('btnRunSWDTest');
    if (btnDetect) {
        btnDetect.textContent = 'Detect device';
        btnDetect.style.background = '#1f2a38';
    }

    if (swd && espSerial && espSerial.port) {
        try {
            await swd.deinit();
        } catch (e) {
            /* ignore */
        }
    }
}

async function stopSwd(reason = '', silent = false) {
    await stopSwdOperations(reason);
    if (espSerial && isDeviceConnected) {
        try {
            await espSerial.setGatewayMode('NONE');
        } catch (e) {
            if (!silent) logToConsole(`SWD stop failed: ${e.message}`, 'error');
        }
    }
}


async function ensureCapstoneReady() {
    if (!window.cs) throw new Error('Capstone not loaded (missing capstone.min.js)');
    try {
        if (cs.MCapstone && typeof cs.MCapstone.then === 'function') {
            await cs.MCapstone;
        }
    } catch (e) {
        throw new Error(`Capstone init failed: ${e && e.message ? e.message : e}`);
    }
    if (!cs.Capstone) throw new Error('Capstone API missing');
    return true;
}


function flashInput(el, ms = 420) {
    if (!el) return;
    try {
        const prev = el.dataset.flashTimerId;
        if (prev) {
            clearTimeout(parseInt(prev, 10));
        }
        el.classList.remove('flash-input');
        void el.offsetWidth;
        el.classList.add('flash-input');
        const tid = setTimeout(() => {
            try { el.classList.remove('flash-input'); } catch (e) { /* ignore */ }
            try { delete el.dataset.flashTimerId; } catch (e) { /* ignore */ }
        }, Math.max(80, ms | 0));
        el.dataset.flashTimerId = String(tid);
    } catch (e) {
        /* ignore */
    }
}

function isPrintableAscii(code) {
    return code >= 0x20 && code <= 0x7E;
}





function decodeApIdr(idr) {
    /* Matches firmware parsing in swd_apscan_test() and ADIv5 APIDR layout.
     * - REV:    bits 27:24
     * - JEP106: bits 26:17 (10-bit designer)
     * - CLASS:  bits 16:13
     * - VAR:    bits 7:4
     * - TYPE:   bits 3:0
     */
    const rev = (idr >>> 24) & 0x0F;
    const designer = (idr >>> 17) & 0x3FF;
    const ap_class = (idr >>> 13) & 0x0F;
    const variant = (idr >>> 4) & 0x0F;
    const type = (idr >>> 0) & 0x0F;
    return { rev, designer, ap_class, variant, type };
}

function apTypeName(info) {
    if (!info) return 'unknown';
    if (info.ap_class === 0x08) {
        if (info.type === 0x01) return 'MEM-AP (AHB)';
        if (info.type === 0x02) return 'MEM-AP (APB)';
        if (info.type === 0x04) return 'MEM-AP (AXI)';
        return `MEM-AP (type=${info.type})`;
    }
    return `class=0x${info.ap_class.toString(16)} type=0x${info.type.toString(16)}`;
}



/* ============= SWD Test Functions ============= */

function calculateMaskFromCheckboxes(containerId) {
    const container = document.getElementById(containerId);
    if (!container) return 0;

    let mask = 0;
    const checkboxes = container.querySelectorAll('input[type="checkbox"]:checked');
    for (const checkbox of checkboxes) {
        const bit = parseInt(checkbox.value, 10);
        mask = (mask | (1 << bit)) >>> 0;
    }
    return mask;
}

async function runSWDTest() {
    if (!espSerial || !espSerial.port) {
        logToConsole('Not connected', 'error');
        return;
    }

    const btnDetect = document.getElementById('btnRunSWDTest');
    const scanBtn = document.getElementById('btnScanAps');

    const setDetectBtn = (active) => {
        if (!btnDetect) return;
        if (active) {
            btnDetect.textContent = 'Stop';
            btnDetect.style.background = '#ef4444';
        } else {
            btnDetect.textContent = 'Detect device';
            btnDetect.style.background = '#1f2a38';
        }
    };

    const resetSwdUi = () => {
        setDpUiEnabled(false);
        const pinsEl = document.getElementById('detectedPins');
        const idsEl = document.getElementById('detectedIds');
        const apTabsEl = document.getElementById('apTabs');
        const apPanelEl = document.getElementById('apPanel');
        if (pinsEl) pinsEl.textContent = '-';
        if (idsEl) idsEl.textContent = '-';
        if (apTabsEl) apTabsEl.innerHTML = '';
        if (apPanelEl) apPanelEl.textContent = '(not scanned)';

        scannedAps = [];
        activeAp = null;
        apHexEditors.clear();
        apSubTabState.clear();

        /* New target detection should not reuse prior CoreSight scan trees. */
        coresightTreeCache.clear();
    };

    /* Toggle behavior: if cyclic detection is running, stop it (no logs). */
    if (detectLoopActive) {
        await stopSwdOperations('User stopped SWD detect');
        try {
            await espSerial.setGatewayMode('NONE');
        } catch (e) {
            logToConsole(`Failed to enter NONE mode: ${e.message}`, 'error');
        }
        return;
    }

    const scanMask = swdPins.masks().scan;
    if (!scanMask || !(scanMask & (scanMask - 1))) {
        logToConsole('Select at least two scan GPIOs for SWDIO and SWCLK', 'error');
        return;
    }
    try {
        await swdPins.apply();
    } catch (e) {
        logToConsole(`Failed to enter SWD mode: ${e.message}`, 'error');
        return;
    }

    /* Best-effort: prime audio from the user gesture. */
    primeTadaaAudio();

    const detectStartMs = performance.now();

    resetSwdUi();
    if (scanBtn) scanBtn.disabled = true;

    detectLoopActive = true;
    const myToken = ++detectLoopToken;
    setDetectBtn(true);

    const sleep = (ms) => new Promise(r => setTimeout(r, ms | 0));

    try {
        const verbose = !!(document.getElementById('chkVerboseSwd') && document.getElementById('chkVerboseSwd').checked);

        while (detectLoopActive && myToken === detectLoopToken) {
            const pinRevision = swdPins.revision;
            try {
                await swdPins.pending;
            } catch (error) {
                if (pinRevision !== swdPins.revision) continue;
                logToConsole(`Pin update failed: ${error.message}`, 'error');
                detectLoopActive = false;
                return;
            }
            if (!detectLoopActive || myToken !== detectLoopToken) return;
            if (pinRevision !== swdPins.revision) continue;
            const ioMask = swdPins.masks().scan;
            if (!ioMask || !(ioMask & (ioMask - 1))) {
                await sleep(250);
                continue;
            }
            if (!swd) swd = new Swd(espSerial, logToConsole);
            let det = null;
            try {
                /* Quiet cycling: don't ask firmware to be verbose. */
                det = await swd.detectPins(ioMask, false);
            } catch (e) {
                await sleep(250);
                continue;
            }

            if (!detectLoopActive || myToken !== detectLoopToken) return;
            if (pinRevision !== swdPins.revision) continue;

            if (!det || !det.detected_device || !det.dpidr_ok) {
                await sleep(250);
                continue;
            }

            /* Found a device: stop cycling. */
            detectLoopActive = false;
            setDetectBtn(false);

            if ((performance.now() - detectStartMs) >= 1000) {
                playTadaa();
            }

            // Keep the successful detection; changing log verbosity must
            // not issue another hardware scan or reset the live SW-DP.
            swd.lastVerbose = !!verbose;

            const pinsEl = document.getElementById('detectedPins');
            const idsEl = document.getElementById('detectedIds');
            if (pinsEl) pinsEl.textContent = `SWDIO=GPIO${det.swdio_gpio}  SWCLK=GPIO${det.swclk_gpio}`;
            if (idsEl) {
                const tid = det.targetid_ok ? u32ToHex(det.targetid) : '-';
                idsEl.textContent = `${u32ToHex(det.dpidr)} / ${tid}`;
            }

            if (scanBtn) scanBtn.disabled = false;
            setDpUiEnabled(true);
            const pinDetails = document.getElementById('swdPinDetails');
            if (pinDetails) pinDetails.open = false;

            try {
                logToConsole(`SWD Detect: io_mask=${u32ToHex(ioMask)}`, 'info');
                logToConsole('DP power-up: requesting CSYSPWRUPREQ + CDBGPWRUPREQ...', 'info');
                const pu = await swd.postDetectInit({ timeoutMs: 1200 });
                logToConsole(`DP power-up: ok=${pu.ok ? 1 : 0} CTRL/STAT=${u32ToHex(pu.ctrlstat)}`, pu.ok ? 'info' : 'error');
            } catch (e) {
                logToConsole(`DP power-up: ERROR: ${e.message}`, 'error');
                logToConsole('DP initialization failed; use DP diagnostics to inspect the link.', 'error');
                if (scanBtn) scanBtn.disabled = true;
                return;
            }

            logToConsole('SWD target detected; ready to scan access ports', 'info');
            return;
        }

        /* Stopped by user or token change: no logs. */
    } finally {
        if (myToken === detectLoopToken && !detectLoopActive) {
            setDetectBtn(false);
        }
    }
}

let dpMonitorTimer = null;
let dpMonitorBusy = false;
let dpMonitorGeneration = 0;

function renderDpStatus(status) {
    const valid = status && status.ack === 1 && !status.error;
    const value = valid ? status.value >>> 0 : 0;
    const valueEl = document.getElementById('dpCtrlstat');
    if (valueEl) valueEl.textContent = valid ? u32ToHex(value) : '-';
    for (const input of document.querySelectorAll('[data-dp-bit]')) {
        input.checked = valid && !!(value & (2 ** Number(input.dataset.dpBit)));
        input.indeterminate = !valid;
    }
    const lamp = (id, state, description) => {
        const element = document.getElementById(id);
        if (!element) return;
        element.dataset.state = valid ? state : 'unknown';
        element.title = valid ? description : 'No current status';
    };
    lamp('dpDebugPower', value & CDBGPWRUPACK ? 'on' : value & CDBGPWRUPREQ ? 'waiting' : 'off', value & CDBGPWRUPACK ? 'Acknowledged' : value & CDBGPWRUPREQ ? 'Requested, not acknowledged' : 'Not requested');
    lamp('dpSystemPower', value & CSYSPWRUPACK ? 'on' : value & CSYSPWRUPREQ ? 'waiting' : 'off', value & CSYSPWRUPACK ? 'Acknowledged' : value & CSYSPWRUPREQ ? 'Requested, not acknowledged' : 'Not requested');
    lamp('dpFaults', value & 0xB2 ? 'error' : 'on', value & 0xB2 ? 'Sticky flags set' : 'No sticky flags');
    const faults = document.getElementById('dpFaults');
    const clearButton = document.getElementById('dpClearStickyBtn');
    if (clearButton) clearButton.hidden = !valid || (value & 0xB2) === 0;
    const powerButton = document.getElementById('dpPowerUpBtn');
    const powerAcks = CDBGPWRUPACK | CSYSPWRUPACK;
    if (powerButton) powerButton.hidden = !valid || (value & powerAcks) === powerAcks;
    if (faults) faults.textContent = valid ? value & 0xB2 ? 'Sticky flags set' : 'No sticky flags' : 'Sticky flags';
    const warnings = [];
    if (status && !valid) warnings.push(status.error || `SWD ${swd.ackName(status.ack)}`);
    if (valid) {
        for (const [mask, name] of [[2, 'Sticky overrun'], [16, 'Sticky compare'], [32, 'Sticky error'], [128, 'Write-data error']]) {
            if (value & mask) warnings.push(name);
        }
        if ((value & CDBGPWRUPREQ) && !(value & CDBGPWRUPACK)) warnings.push('Debug power-up awaiting acknowledgement');
        if ((value & CSYSPWRUPREQ) && !(value & CSYSPWRUPACK)) warnings.push('System power-up awaiting acknowledgement');
    }
    const warningEl = document.getElementById('dpWarnings');
    if (warningEl) {
        const text = warnings.join('\n');
        if (warningEl.textContent !== text) warningEl.textContent = text;
        warningEl.hidden = warnings.length === 0;
    }
    const updated = document.getElementById('dpUpdateState');
    if (updated) updated.textContent = status ? `Updated ${new Date().toLocaleTimeString()}` : 'Not checked';
}

async function pollDpStatus() {
    if (dpMonitorBusy) return;
    const serial = espSerial;
    const target = swd;
    const updated = document.getElementById('dpUpdateState');
    if (!target || !serial?.port) {
        stopDpMonitor();
        renderDpStatus(null);
        return;
    }
    if (activeProtocolTab !== 'swd' || scanInProgress || detectLoopActive || document.getElementById('dpActionControls').disabled || document.getElementById('btnRunSWDTest')?.disabled) {
        if (updated) updated.textContent = 'Paused during SWD activity';
        return;
    }
    const generation = dpMonitorGeneration;
    const version = serial._swdActivityVersion;
    dpMonitorBusy = true;
    try {
        const args = new Uint8Array([0, 0, DP_A23_CTRLSTAT, 0, 0, 0, 0, 0]);
        const response = await serial.swdRequest(SWD_UART_OP_TRANSFER, args, { background: true, timeoutMs: 1500 });
        if (generation !== dpMonitorGeneration || target !== swd || serial !== espSerial || !serial.port || version !== serial._swdActivityVersion) return;
        if (!response) {
            if (updated) updated.textContent = serial._swdDpBank === 0 ? 'Paused during SWD activity' : 'Paused: DP bank 0 not selected';
            return;
        }
        if (response.ack === 1 && (response.status !== SWD_UART_STATUS_OK || response.data?.length < 4 || !response.data)) throw new Error('Invalid DP status response');
        const value = response.ack === 1 ? readU32LE(response.data, 0) : 0;
        renderDpStatus({ ack: response.ack, value });
    } catch (error) {
        if (generation === dpMonitorGeneration && target === swd && serial === espSerial && version === serial._swdActivityVersion) renderDpStatus({ error: error.message });
    } finally {
        dpMonitorBusy = false;
    }
}

function stopDpMonitor() {
    if (dpMonitorTimer !== null) clearInterval(dpMonitorTimer);
    dpMonitorTimer = null;
    dpMonitorGeneration++;
}

function setDpUiEnabled(enabled) {
    const controls = document.getElementById('dpActionControls');
    if (controls) controls.disabled = !enabled;
    const output = document.getElementById('dpStatusOutput');
    if (!enabled) {
        stopDpMonitor();
        renderDpStatus(null);
        if (output) output.textContent = 'No manual register read';
    } else if (dpMonitorTimer === null) {
        const details = document.getElementById('dpDiagnostics');
        if (details) details.open = true;
        dpMonitorTimer = setInterval(pollDpStatus, 500);
    }
}

async function dpDebug() {
    if (!swd) {
        logToConsole('SWD not initialized', 'error');
        return;
    }
    try {
        const lines = await swd.dpDebugOnce();
        const output = document.getElementById('dpStatusOutput');
        if (output) output.textContent = lines.join('\n');
        renderDpStatus(swd.dpStatus);
    } catch (err) {
        const output = document.getElementById('dpStatusOutput');
        if (output) output.textContent = `Status read failed: ${err.message}`;
        renderDpStatus({ error: err.message });
        logToConsole(`DP status error: ${err.message}`, 'error');
    }
}

async function dpAction(action) {
    const controls = document.getElementById('dpActionControls');
    if (!swd || !espSerial?.port || controls.disabled || document.getElementById('btnRunSWDTest')?.disabled || scanInProgress || detectLoopActive) return;
    const output = document.getElementById('dpStatusOutput');
    const target = swd;
    const buttons = [...document.querySelectorAll('#deviceInteraction button, #deviceInteraction input, #deviceInteraction select')];
    const disabled = buttons.map(button => button.disabled);
    buttons.forEach(button => { button.disabled = true; });
    controls.disabled = true;
    try {
        if (action === 'status') {
            await dpDebug();
        } else if (action === 'clear') {
            await swd.clearStickyErrors();
            await dpDebug();
        } else if (action === 'power') {
            await swd.ensurePowerUp(1200);
            await dpDebug();
        } else {
            const bank = parseHexOrDec(document.getElementById('dpBankInput').value);
            const register = Number(document.getElementById('dpRegisterInput').value);
            if (!Number.isInteger(bank) || bank < 0 || bank > 15) throw new Error('DP bank must be 0..15');
            if (action === 'write') {
                const value = parseHexOrDec(document.getElementById('dpValueInput').value) >>> 0;
                if (!confirm(`Write ${u32ToHex(value)} to DP register 0x${(register * 4).toString(16)}, bank ${bank}?`)) return;
                if (register === DP_A23_CTRLSTAT) await swd.dpWrite(DP_A23_SELECT, bank);
                await swd.dpWrite(register, value);
                output.textContent = `DP write completed: register 0x${(register * 4).toString(16)}, bank ${bank}, value ${u32ToHex(value)}`;
            } else {
                if (register === DP_A23_CTRLSTAT) await swd.dpWrite(DP_A23_SELECT, bank);
                const value = await swd.dpRead(register);
                output.textContent = `DP read: register 0x${(register * 4).toString(16)}, bank ${bank}, value ${u32ToHex(value)}`;
            }
        }
    } catch (error) {
        output.textContent = `DP action failed: ${error.message}`;
        logToConsole(output.textContent, 'error');
    } finally {
        if (swd === target && espSerial?.port) buttons.forEach((button, index) => { button.disabled = disabled[index]; });
        controls.disabled = swd !== target || !espSerial?.port;
    }
}

async function scanAps() {
    if (!swd) {
        logToConsole('SWD not initialized', 'error');
        return;
    }

    if (scanInProgress) {
        logToConsole('AP scan already running', 'info');
        return;
    }

    try {
        scanInProgress = true;
        const scanBtn = document.getElementById('btnScanAps');
        if (scanBtn) scanBtn.disabled = true;

        const maxEl = document.getElementById('apScanMaxInput');
        const fullEl = document.getElementById('chkFullApScan');
        const verboseEl = document.getElementById('chkVerboseSwd');
        let maxAps = 32;
        if (maxEl && maxEl.value) {
            const v = parseHexOrDec(maxEl.value);
            if (v > 0) maxAps = v;
        }
        maxAps = Math.max(1, Math.min(256, maxAps | 0));
        const fullScan = !!(fullEl && fullEl.checked);
        const trace = !!(verboseEl && verboseEl.checked);

        logToConsole(`AP scan: reading APIDR/APBASE... (max=${maxAps}${fullScan ? ', full' : ''})`, 'info');
        const res = await swd.scanAps(maxAps, { fullScan, trace });
        scannedAps = res.aps || [];

        renderApTabs(scannedAps, res.memAp);

        if (!scannedAps.length) {
            activeAp = null;
            setActiveAp(null);
            return;
        }

        const hasActive = (activeAp !== null && activeAp !== undefined) && scannedAps.some(a => a.ap === activeAp);
        if (!hasActive) {
            activeAp = (res.memAp !== null && res.memAp !== undefined) ? res.memAp : scannedAps[0].ap;
        }
        setActiveAp(activeAp);
    } catch (err) {
        logToConsole(`AP scan error: ${err.message}`, 'error');
    } finally {
        scanInProgress = false;
        const scanBtn = document.getElementById('btnScanAps');
        if (scanBtn) scanBtn.disabled = false;
    }
}

async function doRead32() {
    if (!swd) return;
    try {
        const addrEl = document.getElementById('memAddrInput');
        if (!addrEl) throw new Error('No memory address input (select a MEM-AP tab)');
        lastMemAddrText = addrEl.value;
        const addr = parseHexOrDec(addrEl.value);
        if (activeAp === null || activeAp === undefined) {
            throw new Error('No AP selected');
        }
        const apInfo = scannedAps.find(a => a.ap === activeAp);
        if (!apInfo || !apInfo.info || apInfo.info.ap_class !== 0x08) {
            throw new Error('Selected AP is not a MEM-AP');
        }
        const v = await swd.memRead32Via(activeAp, addr);
        const valEl = document.getElementById('memWriteValueInput');
        if (valEl) {
            valEl.value = u32ToHex(v);
            lastMemWriteValueText = valEl.value;
            flashInput(valEl, 420);
        }
    } catch (err) {
        logToConsole(`Read32 error: ${err.message}`, 'error');
        showToast(`Read32 failed: ${err.message}`, 'error');
    }
}

async function doWrite32() {
    if (!swd) return;
    try {
        const addrEl = document.getElementById('memAddrInput');
        const valEl = document.getElementById('memWriteValueInput');
        if (!addrEl || !valEl) throw new Error('No memory inputs (select a MEM-AP tab)');

        lastMemAddrText = addrEl.value;
        lastMemWriteValueText = valEl.value;

        const addr = parseHexOrDec(addrEl.value);
        const value = parseHexOrDec(valEl.value) >>> 0;

        if (activeAp === null || activeAp === undefined) {
            throw new Error('No AP selected');
        }
        const apInfo = scannedAps.find(a => a.ap === activeAp);
        if (!apInfo || !apInfo.info || apInfo.info.ap_class !== 0x08) {
            throw new Error('Selected AP is not a MEM-AP');
        }

        await swd.memWrite32Via(activeAp, addr, value);
    } catch (err) {
        logToConsole(`Write32 error: ${err.message}`, 'error');
        showToast(`Write32 failed: ${err.message}`, 'error');
    }
}

async function doReadBlock() {
    if (!swd) return;
    try {
        const addrEl = document.getElementById('memAddrInput');
        const wordsEl = document.getElementById('memWordsInput');
        const useBlkEl = document.getElementById('memUseBlockRead');
        if (!addrEl || !wordsEl) throw new Error('No memory inputs (select a MEM-AP tab)');
        lastMemAddrText = addrEl.value;
        lastMemWordsText = wordsEl.value;

        const addr = parseHexOrDec(addrEl.value);
        const words = Math.max(1, Math.min(1024, parseHexOrDec(wordsEl.value) || 16));
        const useBlock = !!(useBlkEl && useBlkEl.checked);
        lastMemUseBlockRead = useBlock;
        if (activeAp === null || activeAp === undefined) {
            throw new Error('No AP selected');
        }
        const apInfo = scannedAps.find(a => a.ap === activeAp);
        if (!apInfo || !apInfo.info || apInfo.info.ap_class !== 0x08) {
            throw new Error('Selected AP is not a MEM-AP');
        }

        let data;
        if (useBlock) {
            data = await swd.memReadBlock32Via(activeAp, addr, words);
        } else {
            data = [];
            for (let i = 0; i < words; i++) {
                const v = await swd.memRead32Via(activeAp, (addr + (i * 4)) >>> 0);
                data.push(v >>> 0);
            }
        }
        const bytes = new Uint8Array(words * 4);
        for (let i = 0; i < data.length; i++) {
            const v = data[i] >>> 0;
            bytes[i * 4 + 0] = v & 0xFF;
            bytes[i * 4 + 1] = (v >>> 8) & 0xFF;
            bytes[i * 4 + 2] = (v >>> 16) & 0xFF;
            bytes[i * 4 + 3] = (v >>> 24) & 0xFF;
        }
        const ed = apHexEditors.get(activeAp);
        if (ed) {
            ed.setData(bytes, addr);
        }
        apInfo.accessStatus = `Read OK at ${u32ToHex(addr)} (${bytes.length} bytes)`;
        const status = document.getElementById('apAccessStatus');
        if (status) status.textContent = apInfo.accessStatus;
        logToConsole(`AP${activeAp} ReadBlock @${u32ToHex(addr)} words=${words} mode=${useBlock ? 'block' : 'single'}`, 'info');
    } catch (err) {
        const apInfo = scannedAps.find(ap => ap.ap === activeAp);
        if (apInfo) {
            apInfo.accessStatus = `Read failed: ${err.message}`;
            const status = document.getElementById('apAccessStatus');
            if (status) status.textContent = apInfo.accessStatus;
        }
        logToConsole(`ReadBlock error: ${err.message}`, 'error');
        showToast(`Read block failed: ${err.message}`, 'error');
    }
}

function renderApTabs(aps, memAp) {
    const tabsEl = document.getElementById('apTabs');
    const panelEl = document.getElementById('apPanel');
    if (!tabsEl || !panelEl) return;

    tabsEl.innerHTML = '';
    if (!aps || !aps.length) {
        panelEl.textContent = '(no APs found)';
        return;
    }

    for (const a of aps) {
        const btn = document.createElement('button');
        btn.className = 'ap-tab' + ((activeAp === a.ap) ? ' active' : '');
        btn.dataset.ap = String(a.ap);
        const info = a.info;
        const t = apTypeName(info);
        btn.textContent = `AP${a.ap} \u00b7 ${t}`;
        btn.title = `${t}  IDR=${u32ToHex(a.idr)}`;
        btn.onclick = () => setActiveAp(a.ap);
        tabsEl.appendChild(btn);
    }
}

function setActiveAp(ap) {
    activeAp = ap;
    const tabsEl = document.getElementById('apTabs');
    const panelEl = document.getElementById('apPanel');
    if (!tabsEl || !panelEl) return;

    if (!scannedAps || !scannedAps.length) {
        panelEl.textContent = '(no APs found)';
        return;
    }

    if (ap === null || ap === undefined) {
        panelEl.textContent = '(no AP selected)';
        return;
    }

    const buttons = tabsEl.querySelectorAll('.ap-tab');
    for (const b of buttons) {
        b.classList.remove('active');
        if (b.dataset && b.dataset.ap === String(ap)) {
            b.classList.add('active');
        }
    }

    const a = scannedAps.find(x => x.ap === ap);
    if (!a) {
        panelEl.textContent = '(AP not found in scan results - rescan)';
        return;
    }

    const info = a.info;
    const t = apTypeName(info);
    const baseStr = (a.base === null || a.base === undefined) ? '-' : u32ToHex(a.base);

    const isMem = !!(info && info.ap_class === 0x08);

    panelEl.innerHTML = '';

    const subTabsEl = document.createElement('div');
    subTabsEl.className = 'ap-subtabs';
    panelEl.appendChild(subTabsEl);

    const pages = new Map();
    const mkPage = () => {
        const p = document.createElement('div');
        p.className = 'ap-subpanel';
        p.style.display = 'none';
        return p;
    };
    const pageInfo = mkPage();
    const pageHex = mkPage();
    const pageMemoryScan = mkPage();
    const pageCoresight = mkPage();
    const pageDebug = mkPage();
    pages.set('info', pageInfo);
    pages.set('hex', pageHex);
    pages.set('memory-scan', pageMemoryScan);
    pages.set('coresight', pageCoresight);
    pages.set('debug', pageDebug);

    panelEl.appendChild(pageInfo);
    panelEl.appendChild(pageHex);
    panelEl.appendChild(pageMemoryScan);
    panelEl.appendChild(pageCoresight);
    panelEl.appendChild(pageDebug);

    let selected = apSubTabState.get(a.ap) || 'info';
    if (!isMem && selected !== 'info') selected = 'info';

    const debugSupport = { state: 'unknown', cpuid: null, error: null };
    async function probeDebugSupport() {
        if (!isMem) {
            debugSupport.state = 'no-mem';
            return debugSupport;
        }
        if (debugSupport.state !== 'unknown') return debugSupport;
        try {
            const v = await swd.coreCpuidVia(a.ap);
            debugSupport.state = 'ok';
            debugSupport.cpuid = v >>> 0;
            if (!v || (v >>> 0) === 0xffffffff) {
                debugSupport.state = 'fail';
                debugSupport.error = `No CPU identified (CPUID ${u32ToHex(v)})`;
            }
        } catch (e) {
            debugSupport.state = 'fail';
            debugSupport.cpuid = null;
            debugSupport.error = e.message;
        }
        return debugSupport;
    }

    const subBtnById = new Map();
    const addSubTab = (id, title, enabled = true) => {
        const btn = document.createElement('button');
        btn.className = 'ap-subtab' + ((selected === id) ? ' active' : '');
        btn.textContent = title;
        btn.disabled = !enabled;
        btn.onclick = async () => {
            if (btn.disabled) return;
            await selectSubTab(id);
        };
        subTabsEl.appendChild(btn);
        subBtnById.set(id, btn);
    };

    addSubTab('info', 'Overview', true);
    addSubTab('hex', 'Hex dump', isMem);
    addSubTab('memory-scan', 'Memory scan', isMem);
    addSubTab('coresight', 'CoreSight', isMem);
    addSubTab('debug', 'Cortex-M debug', isMem);

    async function selectSubTab(id) {
        if (!isMem && id !== 'info') id = 'info';
        selected = id;
        apSubTabState.set(a.ap, id);
        for (const [k, p] of pages.entries()) {
            p.style.display = (k === id) ? 'block' : 'none';
        }
        for (const [k, b] of subBtnById.entries()) {
            if (k === id) b.classList.add('active');
            else b.classList.remove('active');
        }
        if (id === 'debug') {
            await probeDebugSupport();
            renderDebugSupportBanner();
        }
    }

    function fmtMaybeU32(v) {
        if (v === null || v === undefined) return '-';
        return u32ToHex(v >>> 0);
    }

    const summary = document.createElement('dl');
    summary.className = 'swd-values';
    for (const [label, value] of [
        ['Identification', 'Detected'],
        ['Interface', t],
        ['Memory access', a.accessStatus || 'Not checked'],
        ['CPU debug', 'Not checked'],
    ]) {
        const term = document.createElement('dt');
        term.textContent = label;
        const description = document.createElement('dd');
        description.textContent = label === 'Memory access' && !isMem ? 'No MEM-AP interface advertised' : value;
        if (label === 'Memory access') description.id = 'apAccessStatus';
        if (label === 'CPU debug') description.id = 'apCpuStatus';
        summary.append(term, description);
    }
    pageInfo.appendChild(summary);
    const registerDetails = document.createElement('details');
    registerDetails.className = 'collapsible-group swd-details';
    const registerSummary = document.createElement('summary');
    registerSummary.textContent = 'AP registers and identification';
    registerDetails.appendChild(registerSummary);
    const registerBody = document.createElement('div');
    registerBody.className = 'collapsible-body';
    registerDetails.appendChild(registerBody);
    pageInfo.appendChild(registerDetails);

    const apInfoBox = document.createElement('div');
    apInfoBox.style.whiteSpace = 'pre-wrap';
    apInfoBox.style.marginTop = '8px';
    apInfoBox.style.overflowWrap = 'anywhere';
    apInfoBox.textContent = '(not loaded)';

    async function refreshApInfo() {
        const lines = [];
        lines.push(`Scan IDR: ${u32ToHex(a.idr)}`);
        if (info) {
            lines.push(`  REV=${info.rev}  JEP106(designer)=0x${info.designer.toString(16)}  CLASS=0x${info.ap_class.toString(16)}  VAR=0x${info.variant.toString(16)}  TYPE=0x${info.type.toString(16)}`);
        }
        lines.push(`Scan BASE: ${baseStr}`);

        try {
            const idrNow = await swd.apRead(a.ap, AP_IDR, true);
            const baseNow = await swd.apRead(a.ap, AP_BASE, true);
            let cfgNow = null;
            try { cfgNow = await swd.apRead(a.ap, 0xF4, true); } catch (e) { cfgNow = null; }
            lines.push('');
            lines.push('Live AP regs:');
            lines.push(`  IDR (0xFC):  ${u32ToHex(idrNow)}`);
            lines.push(`  BASE(0xF8):  ${u32ToHex(baseNow)}`);
            lines.push(`  CFG (0xF4):  ${fmtMaybeU32(cfgNow)}`);
            if (isMem) {
                let cswNow = null;
                try { cswNow = await swd.apRead(a.ap, 0x00, true); } catch (e) { cswNow = null; }
                lines.push(`  CSW (0x00):  ${fmtMaybeU32(cswNow)}`);
            }
        } catch (e) {
            lines.push('');
            lines.push(`Live AP regs: read failed (${e.message})`);
        }

        apInfoBox.textContent = lines.join('\n');
    }

    const ops = document.createElement('div');
    ops.className = 'row';

    const btnRefreshInfo = document.createElement('button');
    btnRefreshInfo.textContent = 'Refresh info';
    btnRefreshInfo.onclick = async () => {
        try {
            await refreshApInfo();
        } catch (e) {
            apInfoBox.textContent = `Refresh failed: ${e.message}`;
        }
    };

    const regOff = document.createElement('input');
    regOff.id = 'apRegOffInput';
    regOff.value = '0xFC';
    regOff.style.width = '90px';

    const regBtn = document.createElement('button');
    regBtn.textContent = 'Read AP reg';
    regBtn.onclick = async () => {
        try {
            const off = parseHexOrDec(regOff.value) & 0xFF;
            const v = await swd.apRead(a.ap, off, true);
            logToConsole(`AP${a.ap} ReadReg off=0x${off.toString(16)} => ${u32ToHex(v)}`, 'info');
        } catch (e) {
            logToConsole(`AP${a.ap} ReadReg error: ${e.message}`, 'error');
        }
    };

    pageInfo.insertBefore(btnRefreshInfo, registerDetails);
    ops.appendChild(document.createTextNode('Reg off:'));
    ops.appendChild(regOff);
    ops.appendChild(regBtn);
    registerBody.appendChild(ops);
    registerBody.appendChild(apInfoBox);

    if (isMem) {
        const scanOptions = document.createElement('div');
        scanOptions.className = 'row';
        const blockLabel = document.createElement('label');
        blockLabel.textContent = 'Block size: ';
        const blockSelect = document.createElement('select');
        blockSelect.id = 'memoryScanBlockSize';
        for (const [value, text] of [[256, '256 B'], [1024, '1 KiB'], [4096, '4 KiB'], [65536, '64 KiB']]) {
            const option = document.createElement('option');
            option.value = String(value);
            option.textContent = text;
            blockSelect.appendChild(option);
        }
        blockSelect.value = '4096';
        blockLabel.appendChild(blockSelect);
        const scanStart = document.createElement('button');
        scanStart.textContent = 'Scan';
        const scanStop = document.createElement('button');
        scanStop.textContent = 'Stop';
        scanStop.disabled = true;
        scanOptions.append(blockLabel, scanStart, scanStop);
        const scanBreadcrumbs = document.createElement('div');
        scanBreadcrumbs.className = 'row';
        const scanStatus = document.createElement('div');
        scanStatus.setAttribute('role', 'status');
        const scanLegend = document.createElement('div');
        scanLegend.className = 'memory-scan-legend';
        for (const [text, color] of [['Data', '#3ac787'], ['00', '#ffffff'], ['FF', '#247adf'],
        ['Mixed 00/FF', 'linear-gradient(90deg, #ffffff 50%, #247adf 50%)'], ['No access', '#777777'],
        ['Unprobed', 'repeating-linear-gradient(135deg, transparent 0 5px, #77777755 5px 7px)']]) {
            const entry = document.createElement('span');
            entry.textContent = text;
            const swatch = document.createElement('span');
            swatch.style.cssText = 'display:inline-block;width:12px;height:12px;margin-right:6px;border:1px solid #303740;vertical-align:middle';
            swatch.style.background = color;
            entry.prepend(swatch);
            scanLegend.appendChild(entry);
        }
        const scanMap = document.createElement('div');
        scanMap.className = 'memory-scan-map';
        pageMemoryScan.append(scanOptions, scanBreadcrumbs, scanStatus, scanMap, scanLegend);
        const scanPath = [{ base: 0, size: 0x100000000 }];
        let scanTask = null;
        let openingMemoryRegion = false;
        const stateNames = { data: 'Data', zero: '00', ff: 'FF', mixed: 'Mixed 00/FF', fault: 'Access error' };
        const scanner = new MemoryScanner({
            read: async (address, words) => {
                try {
                    const data = await swd.memReadBlock32Via(a.ap, address, words);
                    a.accessStatus = `Read OK at ${u32ToHex(address)} (${words * 4} bytes)`;
                    return data;
                } catch (error) {
                    a.accessStatus = `Read failed at ${u32ToHex(address)}: ${error.message}`;
                    throw error;
                } finally {
                    if (activeAp === a.ap) document.getElementById('apAccessStatus').textContent = a.accessStatus;
                }
            },
            recover: () => swd.clearStickyErrors(true),
            onUpdate: () => renderMemoryMap()
        });
        function renderMemoryMap() {
            scanMap.replaceChildren();
            scanBreadcrumbs.replaceChildren();
            scanPath.forEach((range, index) => {
                const crumb = document.createElement('button');
                crumb.textContent = index === 0 ? '4 GiB' : u32ToHex(range.base);
                crumb.onclick = () => { scanPath.splice(index + 1); renderMemoryMap(); };
                scanBreadcrumbs.appendChild(crumb);
            });
            const scope = scanPath[scanPath.length - 1];
            for (const range of scanner.getMapView(scope.base, scope.size, 256, scope.parts || null)) {
                const address = range.base;
                const size = range.size;
                const result = range.result;
                const reading = range.state === 'reading';
                const merged = range.parts.length > 1;
                const cell = document.createElement('button');
                cell.dataset.context = String(!!range.context);
                cell.dataset.depth = String(range.depth);
                cell.dataset.state = range.state;
                const segment = document.createElement('span');
                segment.className = 'memory-scan-segment';
                segment.setAttribute('aria-hidden', 'true');
                const label = document.createElement('span');
                label.className = 'memory-scan-label';
                const status = reading ? 'Reading' : range.state === 'unknown' ? 'Unprobed' : range.containsData ? 'Data in subranges' : 'Sample: ' + stateNames[range.state];
                label.textContent = `${u32ToHex(address)} - ${u32ToHex(address + size - 1)}\n${status}${merged ? ' (' + range.parts.length + ' ranges)' : ''}${range.context ? ' | Overview' : ''}`;
                cell.append(segment, label);
                cell.title = merged ? `${range.parts.length} adjacent ranges; ${range.sampledBytes} bytes sampled${range.state === 'fault' ? '; Access errors' : ''}` : result ? `${result.state === 'fault' ? result.requestedBytes + ' bytes requested' : result.sampledBytes + ' bytes sampled'} at ${u32ToHex(address)}${result.error ? ': ' + result.error : ''}` : 'No retained sample';
                cell.onclick = async () => {
                    if (openingMemoryRegion) return;
                    openingMemoryRegion = true;
                    try {
                        if (scanner.busy) {
                            scanner.stop();
                            await scanTask;
                        }
                        await selectSubTab('hex');
                        document.getElementById('memAddrInput').value = u32ToHex(address);
                        document.getElementById('memWordsInput').value = String(Math.min(1024, size / 4));
                        await doReadBlock();
                    } catch (error) {
                        showToast(error.message, 'error');
                    } finally {
                        openingMemoryRegion = false;
                    }
                };
                scanMap.appendChild(cell);
            }
            const counts = scanner.counts;
            scanStatus.textContent = `${scanner.status} | ${scanner.probes} probes | Level: ${scanner.levelDone || 0}/${scanner.levelTotal || 16} | Data ${counts.data}, 00 ${counts.zero}, FF ${counts.ff}, mixed ${counts.mixed}, errors ${counts.fault}`;
        }
        async function runMemoryScan() {
            if (scanner.busy) return;
            const scope = scanPath[scanPath.length - 1];
            const controls = [...document.querySelectorAll('button, input, select')]
                .filter(control => !pageMemoryScan.contains(control) && control.id !== 'disconnectBtn');
            const disabled = controls.map(control => control.disabled);
            controls.forEach(control => { control.disabled = true; });
            scanStart.disabled = blockSelect.disabled = true;
            scanStop.disabled = false;
            try {
                scanTask = scanner.scan(scope.base, scope.size, Number(blockSelect.value));
                await scanTask;
            } catch (error) {
                logToConsole(`AP${a.ap} memory scan: ${error.message}`, 'error');
                showToast(error.message, 'error');
            } finally {
                scanTask = null;
                controls.forEach((control, index) => { control.disabled = disabled[index]; });
                scanStart.disabled = blockSelect.disabled = false;
                scanStop.disabled = true;
            }
        }
        scanStart.onclick = () => runMemoryScan();
        scanStop.onclick = () => scanner.stop();
        renderMemoryMap();
        /* ============ Hex dump tab ============ */
        const memHdr = document.createElement('div');
        memHdr.style.marginTop = '10px';
        memHdr.style.color = '#d4deea';
        memHdr.style.fontWeight = '700';
        memHdr.textContent = 'MEM-AP memory access';
        pageHex.appendChild(memHdr);

        const row1 = document.createElement('div');
        row1.className = 'row';
        row1.appendChild(document.createTextNode('Addr:'));

        const addr = document.createElement('input');
        addr.id = 'memAddrInput';
        addr.value = lastMemAddrText || '0xE000ED00';
        addr.style.width = '140px';

        const b32 = document.createElement('button');
        b32.id = 'btnRead32';
        b32.textContent = 'Read32';
        b32.onclick = doRead32;

        const bRecover = document.createElement('button');
        bRecover.id = 'btnRecoverDp';
        bRecover.textContent = 'Recover DP';
        bRecover.onclick = async () => {
            try {
                await swd.recoverDp('manual request');
            } catch (e) {
                logToConsole(`DP recovery error: ${e.message}`, 'error');
            }
        };

        row1.appendChild(addr);
        row1.appendChild(b32);
        row1.appendChild(bRecover);
        pageHex.appendChild(row1);

        const rowW = document.createElement('div');
        rowW.className = 'row';
        rowW.appendChild(document.createTextNode('Value:'));

        const wval = document.createElement('input');
        wval.id = 'memWriteValueInput';
        wval.value = lastMemWriteValueText || '0x00000000';
        wval.style.width = '140px';

        const bw = document.createElement('button');
        bw.id = 'btnWrite32';
        bw.textContent = 'Write32';
        bw.onclick = doWrite32;

        rowW.appendChild(wval);
        rowW.appendChild(bw);
        pageHex.appendChild(rowW);

        const row2 = document.createElement('div');
        row2.className = 'row';
        row2.appendChild(document.createTextNode('Words:'));

        const words = document.createElement('input');
        words.id = 'memWordsInput';
        words.value = lastMemWordsText || '16';
        words.style.width = '70px';

        const bblk = document.createElement('button');
        bblk.id = 'btnReadBlock';
        bblk.textContent = 'Read block';
        bblk.onclick = doReadBlock;

        const modeLbl = document.createElement('label');
        modeLbl.style.display = 'inline-flex';
        modeLbl.style.alignItems = 'center';
        modeLbl.style.gap = '8px';
        modeLbl.style.color = '#9ca3af';
        modeLbl.style.fontWeight = '700';

        const modeChk = document.createElement('input');
        modeChk.id = 'memUseBlockRead';
        modeChk.type = 'checkbox';
        modeChk.checked = !!lastMemUseBlockRead;
        modeChk.onchange = () => {
            lastMemUseBlockRead = !!modeChk.checked;
        };
        modeLbl.appendChild(modeChk);
        modeLbl.appendChild(document.createTextNode('Block read'));

        row2.appendChild(words);
        row2.appendChild(bblk);
        row2.appendChild(modeLbl);
        pageHex.appendChild(row2);

        const wbRow = document.createElement('div');
        wbRow.className = 'row';

        const dirtyLabel = document.createElement('span');
        dirtyLabel.style.color = '#9ca3af';
        dirtyLabel.textContent = 'Dirty: 0 words';

        const dirtyUnitName = (w) => {
            if ((w | 0) === 4) return 'words';
            if ((w | 0) === 2) return 'halfwords';
            return 'bytes';
        };

        const updateDirtyUi = () => {
            const editor = apHexEditors.get(a.ap);
            if (!editor) {
                dirtyLabel.textContent = 'Dirty: 0';
                return;
            }
            const width = editor.getEditSizeBytes();
            editor.setDirtyRegionSize(width);
            const n = editor.countDirtyRegions(width);
            dirtyLabel.textContent = `Dirty: ${n} ${dirtyUnitName(width)}`;
        };

        const wbBtn = document.createElement('button');
        wbBtn.textContent = 'Write back';
        wbBtn.onclick = async () => {
            try {
                const editor = apHexEditors.get(a.ap);
                if (!editor) throw new Error('No hex editor instance');
                if (!editor.isDirty()) {
                    logToConsole(`AP${a.ap} write-back: nothing to write`, 'info');
                    return;
                }
                const base = editor.baseAddr >>> 0;
                const bytes = editor.getData();

                const width = editor.getEditSizeBytes();
                editor.setDirtyRegionSize(width);

                if ((base & (width - 1)) !== 0) {
                    throw new Error(`Write-back base must be ${width * 8}-bit aligned (or pick a smaller width)`);
                }
                if ((bytes.length & (width - 1)) !== 0) {
                    throw new Error(`Write-back length must be a multiple of ${width} bytes (or pick a smaller width)`);
                }

                const ranges = editor.getDirtyRanges(width);
                logToConsole(`AP${a.ap} write-back: ${ranges.length} range(s), width=${width * 8}`, 'info');

                for (const rg of ranges) {
                    const addr = (base + (rg.start >>> 0)) >>> 0;
                    if ((addr & (width - 1)) !== 0) {
                        throw new Error(`Write-back range unaligned for ${width * 8}-bit writes @${u32ToHex(addr)} (pick smaller width)`);
                    }

                    if (width === 4) {
                        const wordsOut = [];
                        const n = (rg.length / 4) | 0;
                        for (let wi = 0; wi < n; wi++) {
                            const bo = (rg.start + (wi * 4)) | 0;
                            wordsOut.push(u32FromBytesLE(bytes, bo));
                        }
                        await swd.memWriteBlock32Via(a.ap, addr, wordsOut);
                        logToConsole(`AP${a.ap} write-back: wrote ${n} word(s) @${u32ToHex(addr)}`, 'info');
                    } else if (width === 2) {
                        const halfOut = [];
                        const n = (rg.length / 2) | 0;
                        for (let hi = 0; hi < n; hi++) {
                            const bo = (rg.start + (hi * 2)) | 0;
                            const v = (bytes[bo] | (bytes[bo + 1] << 8)) & 0xFFFF;
                            halfOut.push(v);
                        }
                        await swd.memWriteBlock16Via(a.ap, addr, halfOut);
                        logToConsole(`AP${a.ap} write-back: wrote ${n} halfword(s) @${u32ToHex(addr)}`, 'info');
                    } else {
                        const out = bytes.slice(rg.start, rg.start + rg.length);
                        await swd.memWriteBlock8Via(a.ap, addr, out);
                        logToConsole(`AP${a.ap} write-back: wrote ${out.length} byte(s) @${u32ToHex(addr)}`, 'info');
                    }
                }

                editor.clearDirty();
                updateDirtyUi();
            } catch (e) {
                logToConsole(`AP${a.ap} write-back error: ${e.message}`, 'error');
            }
        };

        wbRow.appendChild(wbBtn);
        wbRow.appendChild(dirtyLabel);
        pageHex.appendChild(wbRow);

        const hexContainer = document.createElement('div');
        pageHex.appendChild(hexContainer);

        let editor = apHexEditors.get(a.ap);
        if (!editor) {
            editor = new HexEditor({
                bytesPerRow: 16,
                onReadRange: async (address, length) => {
                    const words = await swd.memReadBlock32Via(a.ap, address, length / 4);
                    const bytes = new Uint8Array(length);
                    const values = new DataView(bytes.buffer);
                    words.forEach((value, index) => values.setUint32(index * 4, value, true));
                    return bytes;
                },
                onReadError: (error) => {
                    logToConsole(`AP${a.ap} hex scroll: ${error.message}`, 'error');
                    showToast(error.message, 'error');
                },
                onChange: (chg) => {
                    const start = (editor.baseAddr + chg.start) >>> 0;
                    updateDirtyUi();
                    logToConsole(`AP${a.ap} hex edit: ${chg.kind} @${u32ToHex(start)} len=${chg.length}`, 'info');
                },
                onModeChange: () => {
                    updateDirtyUi();
                },
            });
            apHexEditors.set(a.ap, editor);
        }
        editor.render(hexContainer);

        /* Update dirty label if we are re-rendering an existing editor. */
        updateDirtyUi();

        /* ============ Debug tab ============ */
        const coreHdr = document.createElement('div');
        coreHdr.style.marginTop = '10px';
        coreHdr.style.color = '#d4deea';
        coreHdr.style.fontWeight = '700';
        coreHdr.textContent = 'Core debug (Cortex-M)';
        pageDebug.appendChild(coreHdr);

        const dbgBanner = document.createElement('div');
        dbgBanner.style.marginTop = '8px';
        dbgBanner.style.color = '#9ca3af';
        dbgBanner.textContent = 'Select the Debugging tab to probe support.';
        pageDebug.appendChild(dbgBanner);

        function renderDebugSupportBanner() {
            if (debugSupport.state === 'unknown') {
                dbgBanner.textContent = 'Probing core debug support...';
                return;
            }
            if (debugSupport.state === 'ok') {
                dbgBanner.textContent = `CPUID read: ${u32ToHex(debugSupport.cpuid)}`;
                document.getElementById('apCpuStatus').textContent = dbgBanner.textContent;
                return;
            }
            dbgBanner.textContent = `CPUID read failed: ${debugSupport.error || 'Unknown error'}`;
            document.getElementById('apCpuStatus').textContent = dbgBanner.textContent;
        }

        const coreRow = document.createElement('div');
        coreRow.className = 'row';
        pageDebug.appendChild(coreRow);

        const regDefs = [
            { regsel: 0x00, name: 'R0' }, { regsel: 0x01, name: 'R1' }, { regsel: 0x02, name: 'R2' }, { regsel: 0x03, name: 'R3' },
            { regsel: 0x04, name: 'R4' }, { regsel: 0x05, name: 'R5' }, { regsel: 0x06, name: 'R6' }, { regsel: 0x07, name: 'R7' },
            { regsel: 0x08, name: 'R8' }, { regsel: 0x09, name: 'R9' }, { regsel: 0x0A, name: 'R10' }, { regsel: 0x0B, name: 'R11' },
            { regsel: 0x0C, name: 'R12' }, { regsel: 0x0D, name: 'SP' }, { regsel: 0x0E, name: 'LR' }, { regsel: 0x0F, name: 'PC' },
            { regsel: 0x10, name: 'xPSR' }, { regsel: 0x11, name: 'MSP' }, { regsel: 0x12, name: 'PSP' }, { regsel: 0x14, name: 'CONTROL' },
        ];

        const regGrid = document.createElement('div');
        regGrid.style.marginTop = '8px';
        regGrid.style.display = 'grid';
        regGrid.style.gridTemplateColumns = 'repeat(4, minmax(0, 1fr))';
        regGrid.style.gap = '6px';

        const regValueEls = new Map();
        let regEditOverlay = null;
        let lastDhcsr = 0;

        const endRegOverlay = () => {
            if (!regEditOverlay) return;
            try { regEditOverlay.remove(); } catch (e) { /* ignore */ }
            regEditOverlay = null;
        };

        const beginRegEdit = async (regsel, targetEl) => {
            endRegOverlay();

            if (!swd) {
                showToast('SWD not initialized', 'error');
                return;
            }

            /* Only allow editing when halted (required by DCRSR semantics). */
            if ((lastDhcsr & SCS_DHCSR_S_HALT) === 0) {
                showToast('Reg write requires core halted', 'error');
                return;
            }

            const rect = targetEl.getBoundingClientRect();
            const overlay = document.createElement('div');
            overlay.className = 'hex-input-overlay';
            overlay.style.left = `${Math.max(8, rect.left)}px`;
            overlay.style.top = `${Math.max(8, rect.top - 2)}px`;

            const input = document.createElement('input');
            input.style.width = '140px';
            input.placeholder = '0x00000000';

            const currentText = (targetEl.textContent || '').trim();
            input.value = (currentText && currentText !== '-') ? currentText : '0x00000000';
            input.maxLength = 10;

            const finish = async (commit) => {
                const text = (input.value || '').trim();
                endRegOverlay();
                if (!commit) return;
                let value;
                try {
                    value = parseHexOrDec(text) >>> 0;
                } catch (e) {
                    showToast('Bad value', 'error');
                    return;
                }
                try {
                    await swd.coreRegWriteVia(a.ap, regsel, value);
                    await refreshCoreRegs();
                } catch (e) {
                    showToast(`Reg write failed: ${e.message}`, 'error');
                }
            };

            input.addEventListener('keydown', (ev) => {
                if (ev.key === 'Enter') {
                    ev.preventDefault();
                    finish(true);
                } else if (ev.key === 'Escape') {
                    ev.preventDefault();
                    finish(false);
                }
            });
            input.addEventListener('blur', () => {
                finish(true);
            });

            overlay.appendChild(input);
            document.body.appendChild(overlay);
            regEditOverlay = overlay;
            input.focus();
            input.select();
        };

        /* Click elsewhere closes the overlay. */
        try {
            pageDebug.addEventListener('click', (ev) => {
                if (!regEditOverlay) return;
                const t = ev && ev.target ? ev.target : null;
                if (t && regEditOverlay.contains(t)) return;
                endRegOverlay();
            });
        } catch (e) {
            /* ignore */
        }
        for (const r of regDefs) {
            const cell = document.createElement('div');
            cell.style.border = '1px solid #374151';
            cell.style.borderRadius = '6px';
            cell.style.background = '#0b1220';
            cell.style.padding = '8px 10px';

            const label = document.createElement('div');
            label.style.color = '#9ca3af';
            label.style.fontSize = '12px';
            label.style.fontWeight = '700';
            label.textContent = r.name;

            const value = document.createElement('div');
            value.style.fontFamily = 'ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", "Courier New", monospace';
            value.style.fontSize = '13px';
            value.style.color = '#e5e7eb';
            value.textContent = '-';
            value.style.cursor = 'text';
            value.title = 'Double-click to edit (core must be halted)';

            value.ondblclick = async (ev) => {
                try {
                    ev.preventDefault();
                    ev.stopPropagation();
                } catch (e) { /* ignore */ }
                await beginRegEdit(r.regsel, value);
            };

            cell.appendChild(label);
            cell.appendChild(value);
            regGrid.appendChild(cell);
            regValueEls.set(r.regsel, value);
        }

        let regReadToken = 0;
        let lastPc = null;
        const setAllRegs = (text) => {
            for (const el of regValueEls.values()) el.textContent = text;
        };

        const disasmBox = document.createElement('div');
        disasmBox.className = 'disasm-box';
        disasmBox.style.display = 'none';

        const disasmHdr = document.createElement('div');
        disasmHdr.className = 'disasm-header';
        disasmHdr.textContent = 'Disassembly @ PC';

        const disasmView = document.createElement('div');
        disasmView.className = 'disasm-view';
        disasmView.textContent = '(click Disasm @PC)';

        disasmBox.appendChild(disasmHdr);
        disasmBox.appendChild(disasmView);

        async function refreshDisasmFromPc() {
            disasmBox.style.display = 'block';
            if (lastPc === null || lastPc === undefined) {
                disasmView.textContent = '(PC unknown - refresh regs while halted)';
                return;
            }

            const pcRaw = (lastPc >>> 0);
            const pc = (pcRaw & 1) ? (pcRaw & ~1) : pcRaw;

            disasmView.textContent = 'Disassembling...';
            try {
                await ensureCapstoneReady();

                const isZeroOkDisasmError = (err) => {
                    const msg = (err && err.message) ? String(err.message) : String(err);
                    return msg.includes('cs_disasm') && msg.includes('code 0') && (msg.includes('CS_ERR_OK') || msg.includes('OK (CS_ERR_OK)'));
                };

                /* Read a window around PC. Thumb is variable-length; over-read a bit. */
                const preBytes = 64;
                const totalBytes = 192;
                const start = (pc < preBytes) ? 0 : ((pc - preBytes) >>> 0);
                const startAligned = (start & ~3) >>> 0;
                const words = Math.max(1, Math.ceil(totalBytes / 4));

                const dataWords = await swd.memReadBlock32Via(a.ap, startAligned, words);
                const buf = new Uint8Array(words * 4);
                for (let i = 0; i < dataWords.length; i++) {
                    const v = dataWords[i] >>> 0;
                    buf[i * 4 + 0] = v & 0xFF;
                    buf[i * 4 + 1] = (v >>> 8) & 0xFF;
                    buf[i * 4 + 2] = (v >>> 16) & 0xFF;
                    buf[i * 4 + 3] = (v >>> 24) & 0xFF;
                }

                if (cs.ARCH_ARM === undefined || cs.MODE_THUMB === undefined) {
                    throw new Error('Capstone ARM/THUMB constants missing');
                }

                let mode = cs.MODE_THUMB;
                if (cs.MODE_MCLASS !== undefined) mode |= cs.MODE_MCLASS;
                if (cs.MODE_LITTLE_ENDIAN !== undefined) mode |= cs.MODE_LITTLE_ENDIAN;

                const d = new cs.Capstone(cs.ARCH_ARM, mode);

                let ins = null;
                let disErr = null;
                for (const input of [buf, Array.from(buf)]) {
                    try {
                        ins = d.disasm(input, startAligned >>> 0);
                        disErr = null;
                        break;
                    } catch (e) {
                        if (isZeroOkDisasmError(e)) {
                            ins = [];
                            disErr = null;
                            break;
                        }
                        disErr = e;
                    }
                }
                try { d.close(); } catch (e) { /* ignore */ }

                if (disErr) throw disErr;

                if (!ins) ins = [];

                if (!ins.length) {
                    disasmView.textContent = '(no instructions decoded)';
                    return;
                }

                let active = 0;
                for (let i = 0; i < ins.length; i++) {
                    const a0 = (ins[i].address >>> 0);
                    const a1 = (i + 1 < ins.length) ? (ins[i + 1].address >>> 0) : ((a0 + 4) >>> 0);
                    if (pc === a0) { active = i; break; }
                    if (pc > a0 && pc < a1) { active = i; break; }
                    if (pc < a0) { active = Math.max(0, i - 1); break; }
                    active = i;
                }

                const startIdx = Math.max(0, active - 8);
                const endIdx = Math.min(ins.length, active + 9);

                disasmView.innerHTML = '';
                for (let i = startIdx; i < endIdx; i++) {
                    const it = ins[i];
                    const line = document.createElement('span');
                    line.className = 'disasm-line' + (i === active ? ' active' : '');
                    const addr = u32ToHex(it.address >>> 0);
                    const mnem = (it.mnemonic || '').toString();
                    const ops = (it.op_str || '').toString();
                    line.textContent = `${addr}:\t${mnem}\t${ops}`;
                    disasmView.appendChild(line);
                }
            } catch (e) {
                disasmView.textContent = `Disasm failed: ${e.message || e}`;
            }
        }

        async function refreshCoreRegs() {
            const token = ++regReadToken;
            setAllRegs('-');
            lastPc = null;
            lastDhcsr = 0;

            let dhcsr = 0;
            try {
                dhcsr = await swd.memRead32Via(a.ap, SCS_DHCSR);
            } catch (e) {
                return;
            }

            lastDhcsr = dhcsr >>> 0;

            if ((dhcsr & SCS_DHCSR_S_HALT) === 0) {
                /* Reading core regs via DCRSR/DCRDR requires the core to be halted.
                 * Show '-' when forbidden.
                 */
                return;
            }

            for (const r of regDefs) {
                if (token !== regReadToken) return;
                const el = regValueEls.get(r.regsel);
                if (!el) continue;
                try {
                    const v = await swd.coreRegReadVia(a.ap, r.regsel);
                    el.textContent = u32ToHex(v);
                    if (r.regsel === 0x0F) lastPc = v >>> 0;
                } catch (e) {
                    el.textContent = '-';
                }
            }

            /* Best-effort auto-refresh disassembly when regs are available. */
            try {
                await refreshDisasmFromPc();
            } catch (e) {
                /* ignore */
            }
        }

        const btnCpuid = document.createElement('button');
        btnCpuid.textContent = 'CPUID';
        btnCpuid.onclick = async () => {
            try {
                const v = await swd.coreCpuidVia(a.ap);
                logToConsole(`AP${a.ap} CPUID @${u32ToHex(SCS_CPUID)} => ${u32ToHex(v)}`, 'info');
            } catch (e) {
                logToConsole(`AP${a.ap} CPUID error: ${e.message}`, 'error');
            }
        };

        const btnHalt = document.createElement('button');
        btnHalt.textContent = 'Halt';
        btnHalt.onclick = async () => {
            try {
                const r = await swd.coreHaltVia(a.ap);
                logToConsole(`AP${a.ap} core halt: halted=${r.halted ? 1 : 0} DHCSR=${u32ToHex(r.dhcsr)}`, r.halted ? 'info' : 'error');
                await refreshCoreRegs();
            } catch (e) {
                logToConsole(`AP${a.ap} core halt error: ${e.message}`, 'error');
                setAllRegs('-');
            }
        };

        const btnCont = document.createElement('button');
        btnCont.textContent = 'Continue';
        btnCont.onclick = async () => {
            try {
                const r = await swd.coreContinueVia(a.ap);
                if (!r.continued) {
                    logToConsole(`AP${a.ap} core continue: ${r.reason || 'failed'} DHCSR=${u32ToHex(r.dhcsr)}`, 'error');
                } else {
                    logToConsole(`AP${a.ap} core continued: DHCSR=${u32ToHex(r.dhcsr)}`, 'info');

                    /* When running, registers/PC/disasm are no longer valid. */
                    try { endRegOverlay(); } catch (e) { /* ignore */ }
                    setAllRegs('-');
                    lastPc = null;
                    lastDhcsr = 0;
                    disasmBox.style.display = 'block';
                    disasmView.textContent = '-';
                }
            } catch (e) {
                logToConsole(`AP${a.ap} core continue error: ${e.message}`, 'error');
            }
        };

        const btnStep = document.createElement('button');
        btnStep.textContent = 'Step';
        btnStep.onclick = async () => {
            try {
                const r = await swd.coreStepVia(a.ap);
                if (!r.stepped) {
                    logToConsole(`AP${a.ap} core step: ${r.reason || 'failed'} DHCSR=${u32ToHex(r.dhcsr)}`, 'error');
                } else {
                    logToConsole(`AP${a.ap} core stepped: DHCSR=${u32ToHex(r.dhcsr)}`, 'info');
                }
                await refreshCoreRegs();
            } catch (e) {
                logToConsole(`AP${a.ap} core step error: ${e.message}`, 'error');
                setAllRegs('-');
            }
        };

        const btnRegs = document.createElement('button');
        btnRegs.textContent = 'Refresh regs';
        btnRegs.onclick = async () => {
            try {
                await refreshCoreRegs();
            } catch (e) {
                setAllRegs('-');
            }
        };

        const btnDisasm = document.createElement('button');
        btnDisasm.textContent = 'Disasm @PC';
        btnDisasm.onclick = async () => {
            try {
                if (lastPc === null || lastPc === undefined) {
                    await refreshCoreRegs();
                } else {
                    await refreshDisasmFromPc();
                }
            } catch (e) {
                disasmBox.style.display = 'block';
                disasmView.textContent = `Disasm failed: ${e.message || e}`;
            }
        };

        coreRow.appendChild(btnCpuid);
        coreRow.appendChild(btnHalt);
        coreRow.appendChild(btnCont);
        coreRow.appendChild(btnStep);
        coreRow.appendChild(btnRegs);
        coreRow.appendChild(btnDisasm);
        pageDebug.appendChild(regGrid);
        pageDebug.appendChild(disasmBox);

        /* ============ CoreSight tab ============ */
        const csHdr = document.createElement('div');
        csHdr.style.marginTop = '10px';
        csHdr.style.color = '#d4deea';
        csHdr.style.fontWeight = '700';
        csHdr.textContent = 'CoreSight ROM table';
        pageCoresight.appendChild(csHdr);

        const csRow = document.createElement('div');
        csRow.className = 'row';

        const csBaseInput = document.createElement('input');
        csBaseInput.value = (() => {
            if (a.base === null || a.base === undefined) return '0xE00FF000';
            const base = a.base >>> 0;
            const present = (base & 1) !== 0;
            const addr = (base & 0xFFFFF000) >>> 0;
            return present ? u32ToHex(addr) : '0xE00FF000';
        })();
        csBaseInput.style.width = '140px';

        const csPath = document.createElement('span');
        csPath.style.color = '#9ca3af';
        csPath.textContent = '';

        const csOut = document.createElement('div');
        csOut.style.marginTop = '8px';
        csOut.style.padding = '8px';
        csOut.style.border = '1px solid #374151';
        csOut.style.borderRadius = '6px';
        csOut.style.background = '#0b1220';
        csOut.textContent = '(not scanned)';

        let csTree = null;
        const csKeyOf = (base) => `${a.ap}:${((base >>> 0) & 0xFFFFF000) >>> 0}`;

        async function csBuildTreeOnce(rootBase) {
            const base0 = ((rootBase >>> 0) & 0xFFFFF000) >>> 0;
            const key = csKeyOf(base0);
            const cached = coresightTreeCache.get(key);
            if (cached) return cached;

            const maxDepth = 7;
            const maxNodes = 256;
            const visited = new Set();

            async function buildAt(baseAddr, depth) {
                const base = ((baseAddr >>> 0) & 0xFFFFF000) >>> 0;
                const node = {
                    base,
                    depth,
                    cls: null,
                    pidr: null,
                    part: null,
                    devarch: null,
                    devtype: null,
                    isRomTable: false,
                    children: [],
                    error: null,
                    expanded: (depth === 0)
                };

                if (visited.has(base)) {
                    node.error = 'loop';
                    return node;
                }
                if (visited.size >= maxNodes) {
                    node.error = 'node limit';
                    return node;
                }
                visited.add(base);

                if (depth > maxDepth) {
                    node.error = 'depth limit';
                    return node;
                }

                try {
                    const cls = await swd.adiGetClassVia(a.ap, base);
                    node.cls = cls;
                    if (cls === null) {
                        node.error = 'bad CIDR';
                        return node;
                    }

                    const pidr = await swd.adiGetPidrVia(a.ap, base);
                    node.pidr = pidr;
                    node.part = adiPartNumLookup(pidr.designer, pidr.part);

                    if (cls === CIDR_CLASS_CORESIGHT) {
                        try {
                            node.devarch = await swd.memRead32Via(a.ap, (base + CS_DEVARCH) >>> 0);
                            node.devtype = await swd.memRead32Via(a.ap, (base + CS_DEVTYPE) >>> 0);
                        } catch (e) {
                            /* ignore */
                        }
                    }

                    if (cls !== CIDR_CLASS_ROMTABLE) {
                        return node;
                    }

                    node.isRomTable = true;
                    const count = await swd.adiRomtableEntryCountVia(a.ap, base);
                    for (let i = 0; i < count; i++) {
                        const childBase = await swd.adiRomtableGetVia(a.ap, base, i);
                        const child = await buildAt(childBase, depth + 1);
                        child.index = i;
                        node.children.push(child);
                    }
                } catch (e) {
                    node.error = e.message;
                }

                return node;
            }

            const tree = await buildAt(base0, 0);
            coresightTreeCache.set(key, tree);
            return tree;
        }

        function csRenderTree() {
            csOut.innerHTML = '';
            if (!csTree) {
                csOut.textContent = '(not scanned)';
                return;
            }

            const header = document.createElement('div');
            header.style.color = '#9ca3af';
            header.style.marginBottom = '8px';
            header.textContent = 'Click nodes to expand/collapse (cached; no re-scan on click).';
            csOut.appendChild(header);

            const list = document.createElement('div');
            list.style.display = 'flex';
            list.style.flexDirection = 'column';
            list.style.gap = '4px';

            const renderNode = (node, ancestorsExpanded) => {
                const visible = ancestorsExpanded;
                const row = document.createElement('div');
                row.style.display = visible ? 'flex' : 'none';
                row.style.alignItems = 'center';
                row.style.gap = '8px';
                row.style.padding = '6px 8px';
                row.style.border = '1px solid #1f2937';
                row.style.borderRadius = '6px';
                row.style.background = '#0f172a';
                row.style.cursor = (node.children && node.children.length) ? 'pointer' : 'default';
                row.style.paddingLeft = `${8 + (node.depth * 14)}px`;

                const twist = document.createElement('div');
                twist.style.width = '16px';
                twist.style.color = '#9ca3af';
                const hasKids = (node.children && node.children.length);
                twist.textContent = hasKids ? (node.expanded ? '▼' : '▶') : '•';

                const title = document.createElement('div');
                title.style.flex = '1';
                title.style.whiteSpace = 'nowrap';
                title.style.overflow = 'hidden';
                title.style.textOverflow = 'ellipsis';

                const clsTxt = (node.cls === null) ? '?' : `0x${node.cls.toString(16)}`;
                const clsName = (node.cls === CIDR_CLASS_ROMTABLE) ? 'ROM' : (node.cls === CIDR_CLASS_CORESIGHT ? 'CS' : '');
                const pn = node.part ? node.part.type : 'Unrecognized';
                const err = node.error ? `  (${node.error})` : '';
                const idx = (node.index !== undefined) ? `#${node.index} ` : '';
                title.textContent = `${idx}${u32ToHex(node.base)}  class=${clsTxt}${clsName ? ' ' + clsName : ''}  ${pn}${err}`;

                row.appendChild(twist);
                row.appendChild(title);

                if (hasKids) {
                    row.onclick = () => {
                        node.expanded = !node.expanded;
                        csRenderTree();
                    };
                }

                list.appendChild(row);

                const kidsVisible = visible && (!!node.expanded);
                if (hasKids) {
                    for (const ch of node.children) {
                        renderNode(ch, kidsVisible);
                    }
                }
            };

            renderNode(csTree, true);
            csOut.appendChild(list);
        }

        const csBtnScan = document.createElement('button');
        csBtnScan.textContent = 'Scan';
        csBtnScan.onclick = async () => {
            try {
                const base = parseHexOrDec(csBaseInput.value);
                /* Clear any old view and force a fresh scan for this base. */
                csTree = null;
                coresightTreeCache.delete(csKeyOf(base));
                csOut.textContent = 'Scanning...';
                csTree = await csBuildTreeOnce(base);
                csRenderTree();
            } catch (e) {
                logToConsole(`AP${a.ap} CoreSight scan error: ${e.message}`, 'error');
            }
        };

        csRow.appendChild(document.createTextNode('Base:'));
        csRow.appendChild(csBaseInput);
        csRow.appendChild(csBtnScan);
        pageCoresight.appendChild(csRow);
        pageCoresight.appendChild(csOut);

        /* Show cached tree if present for current base value. */
        try {
            const b0 = parseHexOrDec(csBaseInput.value);
            const cached = coresightTreeCache.get(csKeyOf(b0));
            if (cached) {
                csTree = cached;
                csRenderTree();
            }
        } catch (e) {
            /* ignore */
        }
    } else {
        const memHint = document.createElement('div');
        memHint.style.marginTop = '8px';
        memHint.style.color = '#9ca3af';
        memHint.textContent = 'Non MEM-AP: memory access / CoreSight / debugging tabs are unavailable.';
        pageHex.appendChild(memHint.cloneNode(true));
        pageCoresight.appendChild(memHint.cloneNode(true));
        pageDebug.appendChild(memHint.cloneNode(true));
    }

    /* Initial render */
    selectSubTab(selected);
    refreshApInfo();
}


let swdPins;
function initializeGpioCheckboxes() {
    swdPins = new SwdPinControls(document.getElementById('ioGpioCheckboxes'), () => espSerial, async () => {
        if (scanInProgress) throw new Error('Wait for the AP scan to finish');
        if (!detectLoopActive) await stopSwdOperations('GPIO configuration changed');
        swd = null;
        setDpUiEnabled(false);
        document.getElementById('btnScanAps').disabled = true;
        document.getElementById('detectedPins').textContent = '-';
        document.getElementById('detectedIds').textContent = '-';
        document.getElementById('apTabs').innerHTML = '';
        document.getElementById('apPanel').textContent = '(not scanned)';
        scannedAps = [];
        activeAp = null;
    });
}


function connectSwdUi() {
    /* Wire up SWD Test button */
    const testBtn = document.getElementById('btnRunSWDTest');
    if (testBtn) testBtn.onclick = runSWDTest;

    const scanBtn = document.getElementById('btnScanAps');
    if (scanBtn) scanBtn.onclick = scanAps;

    swd = new Swd(espSerial, logToConsole);
}

function resetSwdUi() {
    /* Stop any active cyclic detection loop. */
    detectLoopActive = false;
    detectLoopToken++;

    swd = null;
    const scanBtn = document.getElementById('btnScanAps');
    if (scanBtn) scanBtn.disabled = true;
    setDpUiEnabled(false);

    const pinsEl = document.getElementById('detectedPins');
    const idsEl = document.getElementById('detectedIds');
    const apTabsEl = document.getElementById('apTabs');
    const apPanelEl = document.getElementById('apPanel');
    if (pinsEl) pinsEl.textContent = '-';
    if (idsEl) idsEl.textContent = '-';
    if (apTabsEl) apTabsEl.innerHTML = '';
    if (apPanelEl) apPanelEl.textContent = '(not scanned)';

    scannedAps = [];
    activeAp = null;
    apHexEditors.clear();

}
