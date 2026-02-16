#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "uart_gateway.h"

/* short debug message */
#define DBGS(format) send_message("%s: " format, __FUNCTION__)
/* formatted debug message */
#define DBG(format, ...) send_message("%s: " format, __FUNCTION__, __VA_ARGS__)
/* log message*/
#define LOG(...) send_message(__VA_ARGS__)


typedef struct {
    uint32_t ctrlstat;
    bool ctrlstat_ok;
    uint32_t dlcr;
    bool dlcr_ok;
    uint32_t dlpidr;
    bool dlpidr_ok;
    uint32_t dpidr;
    bool dpidr_ok;
    uint32_t eventstat;
    bool eventstat_ok;
    uint32_t select;
    bool select_ok;
    uint32_t targetid;
    bool targetid_ok;
} swd_dpreg_t;

typedef struct {
    bool ok;
    bool tested;
    uint8_t revision;
    uint16_t designer;
    uint8_t class;
    uint8_t variant;
    uint8_t type;
    uint32_t base;
} swd_apidr_info_t;

typedef struct {
    uint8_t revision;
    uint8_t partno;
    uint8_t version;
    uint16_t designer;
} swd_dpidr_info_t;

typedef struct {
    uint8_t revision;
    uint16_t partno;
    uint16_t designer;
} swd_targetid_info_t;


typedef struct {
    SemaphoreHandle_t swd_mutex;

    swd_targetid_info_t targetid_info;
    swd_dpidr_info_t dpidr_info;
    swd_dpreg_t dp_regs;
    swd_apidr_info_t apidr_info[256];

    uint8_t timeout_overdue;
    uint32_t loop_count;
    uint8_t current_mask_id;
    uint32_t current_mask;
    uint32_t io_swc;
    uint32_t io_swd;
    uint32_t io_selected;
    uint8_t io_num_swc;
    uint8_t io_num_swd;
    int32_t detected_timeout;
    uint32_t swd_clock_delay;
    uint32_t swd_idle_bits;
    bool detected;
    bool detected_device;
    bool detected_notified;
    uint32_t mode_page;
    uint8_t ap_pos;
    uint8_t ap_scanned;

    uint32_t coresight_pos[16];
    uint32_t coresight_count[16];
    uint8_t coresight_level;
    uint32_t coresight_bases[16];

    uint32_t hex_addr;
    uint8_t hex_select;
    uint8_t hex_buffer[32];
    uint8_t hex_buffer_valid[8];

    char state_string[64];
} AppFSM;

void swd_main_loop(AppFSM* ctx);
void swd_main_loop_scan(AppFSM* ctx);
void swd_main_loop_apscan(AppFSM* ctx);
void swd_main_loop_hexdump(AppFSM* ctx);
void swd_main_loop_idle(AppFSM* ctx);

#define COUNT(x) ((size_t)(sizeof(x) / sizeof((x)[0])))
#define ARRAY_SIZE(x) COUNT(x)

#define SWD_DELAY_US 0
#define TIMER_HZ 25
#define TIMEOUT 3
#define QUEUE_SIZE 8
#define IDLE_BITS 8
#define CLOCK_DELAY 0

#define CDBGPWRUPREQ (1 << 28)
#define CDBGPWRUPACK (1 << 29)
#define CSYSPWRUPREQ (1 << 30)
#define CSYSPWRUPACK (1 << 31)
#define WDATAERR (1 << 7)
#define STICKYORUN (1 << 1)
#define STICKYERR (1 << 5)
#define STICKYCMP (1 << 4)
#define STAT_ERROR_FLAGS (WDATAERR | STICKYERR | STICKYORUN | STICKYCMP)

#define REG_IDCODE 0x00
#define REG_CTRLSTAT 0x01
#define REG_CTRLSTAT_BANK 0x00
#define REG_DLCR 0x01
#define REG_DLCR_BANK 0x01
#define REG_TARGETID 0x01
#define REG_TARGETID_BANK 0x02
#define REG_DLPIDR 0x01
#define REG_DLPIDR_BANK 0x03
#define REG_EVENTSTAT 0x01
#define REG_EVENTSTAT_BANK 0x04

#define REG_SELECT 0x02

#define MEMAP_CSW 0x00
#define MEMAP_TAR 0x04
#define MEMAP_DRW 0x0C
#define AP_IDR 0xFC
#define AP_BASE 0xF8

#define SCS_CPUID 0xE000ED00u
#define SCS_CPACR 0xE000ED88u
#define SCS_DHCSR 0xE000EDF0u
#define SCS_DHCSR_S_HALT (1u << 17)
#define SCS_DHCSR_C_MASKINTS (1u << 3)
#define SCS_DHCSR_C_STEP (1u << 2)
#define SCS_DHCSR_C_HALT (1u << 1)
#define SCS_DHCSR_C_DEBUGEN (1u << 0)
#define SCS_DHCSR_KEY 0xA05F0000u
#define SCS_DCRSR 0xE000EDF4u
#define SCS_DCRSR_RD 0x00000000u
#define SCS_DCRSR_WR 0x00010000u
#define SCS_DCRDR 0xE000EDF8u
#define SCS_DEMCR 0xE000EDFCu


uint8_t swd_read_memory(AppFSM* const ctx, uint8_t ap, uint32_t address, uint32_t* data);

void swd_init(AppFSM* const ctx, uint32_t io_mask);
void swd_deinit(AppFSM* const ctx);
void swd_do_scan(AppFSM* ctx);

/* UART-facing low-level SWD accessors (binary protocol helpers)
 *
 * These intentionally expose only basic IO routines; higher-level logic
 * (AP scan, MEM-AP memory reads, etc.) should live on the host side.
 */
void swd_get_active_pins(const AppFSM *ctx, uint8_t *swdio_gpio, uint8_t *swclk_gpio);

/* Raw SWD transfer primitive.
 *
 * ap: 0=DP, 1=AP
 * write: 0=read, 1=write
 * a23: register address bits A[2:3] (0..3)
 * data: in/out (write: contains WDATA; read: filled with RDATA)
 *
 * Return value: SWD ACK (1 OK, 2 WAIT, 4 FAULT) or 8 on parity mismatch.
 */
uint8_t swd_uart_transfer(AppFSM *ctx, bool ap, bool write, uint8_t a23, uint32_t *data);

/* Convenience helpers for AP register accesses.
 * ap_off is the byte offset (e.g. 0xFC for AP_IDR).
 */
uint8_t swd_uart_read_ap(AppFSM *ctx, uint8_t ap, uint8_t ap_off, uint32_t *data);
uint8_t swd_uart_read_ap_single(AppFSM *ctx, uint8_t ap, uint8_t ap_off, uint32_t *data);
uint8_t swd_uart_write_ap(AppFSM *ctx, uint8_t ap, uint8_t ap_off, uint32_t data);
