
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
#include <assert.h>

#include "swd.h"
#include "uart_gateway.h"

#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const uint32_t gpio_legal_mask = 0x3007FF;

/* bit set: clock, else data */
static const uint32_t gpio_direction_mask[10] = {
    /* 1-bit stripes */
    0xAAAAAAAA, 0x55555555,
    /* 2-bit stripes */
    0xCCCCCCCC, 0x33333333,
    /* 4-bit stripes */
    0xF0F0F0F0, 0x0F0F0F0F,
    /* 8-bit stripes */
    0xFF00FF00, 0x00FF00FF,
    /* 16-bit stripes */
    0xFFFF0000, 0x0000FFFF};

static bool has_multiple_bits(uint32_t x)
{
    return (x & (x - 1)) != 0;
}

static uint8_t get_bit_num(uint32_t x)
{
    return (uint8_t)__builtin_ctz(x);
}

static void swd_configure_pins(AppFSM *const ctx, bool output)
{
    if (ctx->io_num_swc < 32 && ctx->io_num_swd < 32)
    {
        gpio_set_direction(ctx->io_num_swc, GPIO_MODE_OUTPUT);
        gpio_set_pull_mode(ctx->io_num_swc, GPIO_FLOATING);
        if (!output)
        {
            gpio_set_direction(ctx->io_num_swd, GPIO_MODE_INPUT);
            gpio_set_pull_mode(ctx->io_num_swd, GPIO_PULLUP_ONLY);
        }
        else
        {
            gpio_set_direction(ctx->io_num_swd, GPIO_MODE_OUTPUT_OD);
            gpio_set_pull_mode(ctx->io_num_swd, GPIO_PULLUP_ONLY);
        }
        return;
    }

    for (int io = 0; io < 32; io++)
    {
        uint32_t bitmask = 1U << io;

        /* dont touch non-selected */
        if ((ctx->io_selected & bitmask) == 0)
        {
            continue;
        }

        /* if neither candidate for SWC nor SWD then skip */
        if (!(ctx->io_swc & bitmask) && !(ctx->io_swd & bitmask))
        {
            gpio_set_direction(io, GPIO_MODE_INPUT);
            gpio_set_pull_mode(io, GPIO_PULLUP_ONLY);
            continue;
        }

        if (ctx->current_mask & bitmask)
        {
            /* set for clock */
            gpio_set_direction(io, GPIO_MODE_OUTPUT);
        }
        else
        {
            /* set for data */
            if (!output)
            {
                gpio_set_direction(io, GPIO_MODE_INPUT);
                gpio_set_pull_mode(io, GPIO_PULLUP_ONLY);
            }
            else
            {
                gpio_set_direction(io, GPIO_MODE_OUTPUT);
            }
        }
    }
}

static void swd_set_clock(AppFSM *const ctx, const uint8_t level)
{
    if (ctx->io_num_swc < 32)
    {
        gpio_set_level(ctx->io_num_swc, level);
        return;
    }

    for (int io = 0; io < 32; io++)
    {
        uint32_t bitmask = 1U << io;

        /* if no candidate for SWC then skip */
        if (!(ctx->io_swc & bitmask))
        {
            continue;
        }

        if (ctx->current_mask & bitmask)
        {
            gpio_set_level(io, level);
        }
    }
}

static void swd_set_data(AppFSM *const ctx, const uint8_t level)
{
    if (ctx->io_num_swd < 32)
    {
        gpio_set_level(ctx->io_num_swd, level);
        return;
    }

    for (int io = 0; io < 32; io++)
    {
        uint32_t bitmask = 1U << io;

        /* if no candidate for SWD then skip */
        if (!(ctx->io_swd & bitmask))
        {
            continue;
        }

        if (!(ctx->current_mask & bitmask))
        {
            gpio_set_level(io, level);
        }
    }
}

static uint32_t swd_get_data(AppFSM *const ctx)
{
    if (ctx->io_num_swd < 32)
    {
        return gpio_get_level(ctx->io_num_swd);
    }

    uint32_t bits = 0;
    for (int io = 0; io < 32; io++)
    {
        uint32_t bitmask = 1U << io;

        /* if no candidate for SWD then skip */
        if (!(ctx->io_swd & bitmask))
        {
            continue;
        }
        bits |= gpio_get_level(io) ? bitmask : 0;
    }
    return bits;
}

static void swd_clock_delay(AppFSM *const ctx)
{
    if (ctx->swd_clock_delay)
    {
        esp_rom_delay_us(ctx->swd_clock_delay);
    }
}

static void swd_write_bit(AppFSM *const ctx, bool level)
{
    swd_set_clock(ctx, 0);
    swd_set_data(ctx, level);
    swd_clock_delay(ctx);
    swd_set_clock(ctx, 1);
    swd_clock_delay(ctx);
    swd_set_clock(ctx, 0);
}

static uint32_t swd_read_bit(AppFSM *const ctx)
{
    swd_set_clock(ctx, 1);
    swd_clock_delay(ctx);
    swd_set_clock(ctx, 0);
    uint32_t bits = swd_get_data(ctx);
    swd_clock_delay(ctx);
    swd_set_clock(ctx, 1);

    return bits;
}

/* send a byte or less LSB-first */
static void swd_write_byte(AppFSM *const ctx, const uint8_t data, size_t bits)
{
    for (size_t pos = 0; pos < bits; pos++)
    {
        swd_write_bit(ctx, data & (1 << pos));
    }
}

/* send a sequence of bytes LSB-first */
static void swd_write(AppFSM *const ctx, const uint8_t *data, size_t bits)
{
    size_t byte_pos = 0;
    while (bits > 0)
    {
        size_t remain = (bits > 8) ? 8 : bits;
        swd_write_byte(ctx, data[byte_pos++], remain);
        bits -= remain;
    }
}

static uint8_t swd_transfer(AppFSM *const ctx, bool ap, bool write, uint8_t a23, uint32_t *data)
{
    // notification_message(ctx->notification, &sequence_set_blue_255);
    // notification_message(ctx->notification, &sequence_reset_red);

    swd_set_data(ctx, false);
    swd_configure_pins(ctx, true);

    uint32_t idle = 0;
    swd_write(ctx, (uint8_t *)&idle, ctx->swd_idle_bits);

    uint8_t request[] = {0};

    request[0] |= 0x01;                                    /* start bit*/
    request[0] |= ap ? 0x02 : 0;                           /* APnDP */
    request[0] |= write ? 0 : 0x04;                        /* operation */
    request[0] |= (a23 & 0x01) ? 0x08 : 0;                 /* A[2:3] */
    request[0] |= (a23 & 0x02) ? 0x10 : 0;                 /* A[2:3] */
    request[0] |= 0x80;                                    /* park bit */
    request[0] |= __builtin_parity(request[0]) ? 0x20 : 0; /* parity */

    swd_write(ctx, request, sizeof(request) * 8);

    /* turnaround cycle */
    swd_configure_pins(ctx, false);

    uint32_t ack = 0;

    /* receive 3 ACK bits */
    for (int pos = 0; pos < 3; pos++)
    {
        ack >>= 1;
        ack |= swd_read_bit(ctx) ? 0x04 : 0;
    }

    /* Force ABORT writes to always "work" so we can recover from bad states.
     * Do NOT mask read ACKs here (otherwise IDCODE/DPIDR reads look successful even if the bus is broken).
     */
    if (!ap && write && a23 == 0)
    {
        ack = 1;
    }

    if (ack != 0x01)
    {
        // notification_message(ctx->notification, &sequence_reset_blue);
        // notification_message(ctx->notification, &sequence_set_red_255);
        return ack;
    }

    if (write)
    {
        swd_write_bit(ctx, 0);
        swd_configure_pins(ctx, true);

        /* send 32 WDATA bits */
        for (int pos = 0; pos < 32; pos++)
        {
            swd_write_bit(ctx, *data & (1 << pos));
        }

        /* send parity bit */
        swd_write_bit(ctx, __builtin_parity(*data));
    }
    else
    {
        *data = 0;
        /* receive 32 RDATA bits */
        for (int pos = 0; pos < 32; pos++)
        {
            *data >>= 1;
            *data |= swd_read_bit(ctx) ? 0x80000000 : 0;
        }

        /* receive parity bit */
        bool parity = swd_read_bit(ctx);

        if (parity != __builtin_parity(*data))
        {
            // notification_message(ctx->notification, &sequence_reset_blue);
            // notification_message(ctx->notification, &sequence_set_red_255);
            return 8;
        }
    }
    swd_set_data(ctx, false);
    swd_configure_pins(ctx, true);
    // notification_message(ctx->notification, &sequence_reset_blue);

    return ack;
}

void swd_get_active_pins(const AppFSM *ctx, uint8_t *swdio_gpio, uint8_t *swclk_gpio)
{
    if (swdio_gpio)
    {
        *swdio_gpio = ctx ? ctx->io_num_swd : 0xFF;
    }
    if (swclk_gpio)
    {
        *swclk_gpio = ctx ? ctx->io_num_swc : 0xFF;
    }
}

uint8_t swd_uart_transfer(AppFSM *ctx, bool ap, bool write, uint8_t a23, uint32_t *data)
{
    if (!ctx || !ctx->swd_mutex)
    {
        return 0;
    }

    xSemaphoreTake(ctx->swd_mutex, portMAX_DELAY);
    uint8_t ack = swd_transfer(ctx, ap, write, a23, data);

    if (!ap && write && a23 == REG_SELECT)
    {
        /* Keep DP SELECT cache coherent with raw host writes.
         * If the host changes SELECT behind the firmware's back, subsequent AP/DP banked
         * accesses can silently hit the wrong bank and return bogus data with ACK=OK.
         */
        ctx->dp_regs.select_ok = false;
        if (ack == 1 && data)
        {
            ctx->dp_regs.select = *data;
            ctx->dp_regs.select_ok = true;
        }
    }

    xSemaphoreGive(ctx->swd_mutex);
    return ack;
}

/* A line reset is achieved by holding the data signal HIGH for at least 50 clock cycles, followed by at least two idle cycles. */
static void swd_line_reset(AppFSM *const ctx)
{
    // notification_message(ctx->notification, &sequence_set_red_255);
    for (int bitcount = 0; bitcount < 50; bitcount += 8)
    {
        swd_write_byte(ctx, 0xFF, 8);
    }
    swd_write_byte(ctx, 0, 8);
    ctx->dp_regs.select_ok = false;
    // notification_message(ctx->notification, &sequence_reset_red);
}

static void swd_abort(AppFSM *const ctx)
{
    uint32_t dpidr;

    /* first reset the line */
    swd_line_reset(ctx);
    swd_transfer(ctx, false, false, 0, &dpidr);
    /* Clear sticky compare/error/write-data-error and overrun.
     * (ORUNERRCLR is bit 4; previous 0x0E missed that and could leave the DP stuck in FAULT.)
     */
    uint32_t abort = 0x1E;
    swd_transfer(ctx, false, true, 0, &abort);
}

static void swd_abort_simple(AppFSM *const ctx)
{
    uint32_t abort = 0x1E;
    swd_transfer(ctx, false, true, 0, &abort);

    uint32_t dpidr;
    if (swd_transfer(ctx, false, false, 0, &dpidr) != 1)
    {
        swd_abort(ctx);
    }
}

static uint8_t swd_select(AppFSM *const ctx, uint8_t ap_sel, uint8_t ap_bank, uint8_t dp_bank)
{
    uint32_t bank_reg = (ap_sel << 24) | ((ap_bank & 0x0F) << 4) | (dp_bank & 0x0F);

    if (ctx->dp_regs.select_ok && bank_reg == ctx->dp_regs.select)
    {
        return 1;
    }

    uint8_t ret = swd_transfer(ctx, false, true, REG_SELECT, &bank_reg);
    if (ret != 1)
    {
        ctx->dp_regs.select_ok = false;
        DBG("failed: %d", ret);
        return ret;
    }

    ctx->dp_regs.select = bank_reg;
    ctx->dp_regs.select_ok = true;
    return ret;
}

static uint8_t
swd_read_dpbank(AppFSM *const ctx, uint8_t dp_off, uint8_t dp_bank, uint32_t *data)
{
    uint8_t ret = 0;

    /* select target bank */
    if (dp_bank < 0x10)
    {
        uint8_t ret = swd_select(ctx, 0, 0, dp_bank);
        if (ret != 1)
        {
            DBGS("swd_select failed");
            return ret;
        }
    }

    /* read data from it */
    *data = 0;
    ret = swd_transfer(ctx, false, false, dp_off, data);
    if (ret != 1)
    {
        DBG("failed: %d", ret);
        return ret;
    }
    return ret;
}
static uint8_t swd_read_ap(AppFSM *const ctx, uint8_t ap, uint8_t ap_off, uint32_t *data)
{
    /* select target bank */
    uint8_t ret = swd_select(ctx, ap, (ap_off >> 4) & 0x0F, 0);
    if (ret != 1)
    {
        DBGS("swd_select failed");
        return ret;
    }

    /* AP reads are posted: the actual data must be read from DP RDBUFF (A[3:2]=3). */
    uint32_t posted = 0;
    ret = swd_transfer(ctx, true, false, (ap_off >> 2) & 3, &posted);
    if (ret != 1)
    {
        DBG("failed: %d", ret);
        if (ret == 4)
        {
            swd_abort_simple(ctx);
        }
        return ret;
    }

    uint32_t rdbuff = 0;
    ret = swd_transfer(ctx, false, false, 3, &rdbuff);
    if (ret != 1)
    {
        DBG("rdbuff failed: %d", ret);
        if (ret == 4)
        {
            swd_abort_simple(ctx);
        }
        return ret;
    }

    *data = rdbuff;
    return 1;
}

uint8_t swd_uart_read_ap(AppFSM *ctx, uint8_t ap, uint8_t ap_off, uint32_t *data)
{
    if (!ctx || !ctx->swd_mutex)
    {
        return 0;
    }

    xSemaphoreTake(ctx->swd_mutex, portMAX_DELAY);
    uint8_t ack = swd_read_ap(ctx, ap, ap_off, data);
    xSemaphoreGive(ctx->swd_mutex);
    return ack;
}

static uint8_t swd_read_ap_single(AppFSM *const ctx, uint8_t ap, uint8_t ap_off, uint32_t *data)
{
    uint8_t ret = swd_select(ctx, ap, (ap_off >> 4) & 0x0F, 0);
    if (ret != 1)
    {
        DBGS("swd_select failed");
        return ret;
    }

    /* Single AP reads must also consume the posted result via DP RDBUFF,
     * otherwise the next AP access can trigger STICKYORUN and the DP gets stuck returning FAULT.
     */
    uint32_t posted = 0;
    ret = swd_transfer(ctx, true, false, (ap_off >> 2) & 3, &posted);
    if (ret != 1)
    {
        DBG("failed: %d", ret);
        if (ret == 4)
        {
            swd_abort_simple(ctx);
        }
        return ret;
    }

    uint32_t rdbuff = 0;
    ret = swd_transfer(ctx, false, false, 3, &rdbuff);
    if (ret != 1)
    {
        DBG("rdbuff failed: %d", ret);
        if (ret == 4)
        {
            swd_abort_simple(ctx);
        }
        return ret;
    }

    *data = rdbuff;
    return 1;
}

uint8_t swd_uart_read_ap_single(AppFSM *ctx, uint8_t ap, uint8_t ap_off, uint32_t *data)
{
    if (!ctx || !ctx->swd_mutex)
    {
        return 0;
    }

    xSemaphoreTake(ctx->swd_mutex, portMAX_DELAY);
    uint8_t ack = swd_read_ap_single(ctx, ap, ap_off, data);
    xSemaphoreGive(ctx->swd_mutex);
    return ack;
}

static uint8_t swd_write_ap(AppFSM *const ctx, uint8_t ap, uint8_t ap_off, uint32_t data)
{
    uint8_t ret = swd_select(ctx, ap, (ap_off >> 4) & 0x0F, 0);
    if (ret != 1)
    {
        DBGS("swd_select failed");
        return ret;
    }
    ret = swd_transfer(ctx, true, true, (ap_off >> 2) & 3, &data);
    if (ret != 1)
    {
        DBG("failed: %d", ret);
        return ret;
    }
    return ret;
}

uint8_t swd_uart_write_ap(AppFSM *ctx, uint8_t ap, uint8_t ap_off, uint32_t data)
{
    if (!ctx || !ctx->swd_mutex)
    {
        return 0;
    }

    xSemaphoreTake(ctx->swd_mutex, portMAX_DELAY);
    uint8_t ack = swd_write_ap(ctx, ap, ap_off, data);
    xSemaphoreGive(ctx->swd_mutex);
    return ack;
}


static uint32_t swd_detect(AppFSM *const ctx)
{
    swd_set_data(ctx, false);
    swd_configure_pins(ctx, true);

    uint8_t data[] = {0xA5};
    swd_write(ctx, data, sizeof(data) * 8);

    /* turnaround cycle */
    swd_configure_pins(ctx, false);

    uint32_t ack_bits[3];
    uint32_t rdata[32];

    /* receive 3 ACK bits */
    for (int pos = 0; pos < COUNT(ack_bits); pos++)
    {
        ack_bits[pos] = swd_read_bit(ctx);
    }

    /* receive 32 RDATA bits */
    for (int pos = 0; pos < COUNT(rdata); pos++)
    {
        rdata[pos] = swd_read_bit(ctx);
    }

    /* receive parity bit */
    uint32_t parity = swd_read_bit(ctx);

    for (int io = 0; io < 32; io++)
    {
        uint32_t bitmask = 1 << io;

        /* skip if it's a clock */
        if (ctx->current_mask & bitmask)
        {
            continue;
        }

        uint8_t ack = 0;
        for (int pos = 0; pos < COUNT(ack_bits); pos++)
        {
            ack >>= 1;
            ack |= (ack_bits[pos] & bitmask) ? 4 : 0;
        }

        uint32_t dpidr = 0;
        for (int pos = 0; pos < COUNT(rdata); pos++)
        {
            dpidr >>= 1;
            dpidr |= (rdata[pos] & bitmask) ? 0x80000000 : 0;
        }

        if (ack == 1 && dpidr != 0 && dpidr != 0xFFFFFFFF)
        {
            bool received_parity = (parity & bitmask);
            if (__builtin_parity(dpidr) == received_parity)
            {
                ctx->dp_regs.dpidr = dpidr;
                ctx->dp_regs.dpidr_ok = true;
                ctx->detected = true;
                ctx->io_swd = bitmask;
                ctx->io_swc &= ctx->current_mask;
                LOG("swd_detect: data: %08lX, io_swd %08lX, io_swc %08lX",
                    dpidr,
                    ctx->io_swd,
                    ctx->io_swc);

                if (!has_multiple_bits(ctx->io_swc))
                {
                    ctx->io_num_swd = get_bit_num(ctx->io_swd);
                    ctx->io_num_swc = get_bit_num(ctx->io_swc);
                }
            }
        }
    }
    swd_set_data(ctx, false);
    swd_configure_pins(ctx, true);

    return 0;
}

static void swd_scan(AppFSM *const ctx)
{
    /* To switch SWJ-DP from JTAG to SWD operation:
        1. Send at least 50 SWCLKTCK cycles with SWDIOTMS HIGH. This ensures that the current interface is in its reset state. The JTAG interface only detects the 16-bit JTAG-to-SWD sequence starting from the Test-Logic-Reset state.
        2. Send the 16-bit JTAG-to-SWD select sequence 0x79e7 on SWDIOTMS.
        3. Send at least 50 SWCLKTCK cycles with SWDIOTMS HIGH. This ensures that if SWJ-DP was already in SWD operation before sending the select sequence, the SWD interface enters line reset state.
    */
    swd_configure_pins(ctx, true);

    /* reset JTAG interface */
    for (int bitcount = 0; bitcount < 50; bitcount += 8)
    {
        swd_write_byte(ctx, 0xFF, 8);
    }

    /* Send the 16-bit JTAG-to-SWD select sequence */
    swd_write_byte(ctx, 0x9E, 8);
    swd_write_byte(ctx, 0xE7, 8);

    /* resynchronize SWD */
    swd_line_reset(ctx);

    swd_detect(ctx);
}

void swd_do_scan(AppFSM *ctx)
{
    assert(ctx);

    for (int num = 0; num < COUNT(gpio_direction_mask); num++)
    {
        ctx->current_mask_id = num;

        /* reset after timeout */
        if (ctx->detected_timeout > 0)
        {
            ctx->detected_timeout--;
        }
        else
        {
            DBGS("Reset detected flag");
            ctx->detected_device = false;
            ctx->io_swd = ctx->io_selected;
            ctx->io_swc = ctx->io_selected;
            ctx->io_num_swd = 0xFF;
            ctx->io_num_swc = 0xFF;
            ctx->ap_scanned = 0;
            memset(&ctx->dp_regs, 0x00, sizeof(ctx->dp_regs));
            memset(&ctx->targetid_info, 0x00, sizeof(ctx->targetid_info));
            memset(&ctx->apidr_info, 0x00, sizeof(ctx->apidr_info));
        }

        ctx->detected = false;
        ctx->current_mask = gpio_direction_mask[ctx->current_mask_id];

        /* when SWD was already detected, set it to data pin regardless of the mask */
        if (ctx->detected_device)
        {
            ctx->current_mask &= ~ctx->io_swd;
        }

        /* do the scan */
        xSemaphoreTake(ctx->swd_mutex, portMAX_DELAY);
        swd_scan(ctx);
        xSemaphoreGive(ctx->swd_mutex);

        /* now when detected a device, set the timeout */
        if (ctx->detected)
        {
            DBGS("Set detected flag");
            ctx->detected_device = true;
            ctx->detected_timeout = TIMER_HZ * TIMEOUT;

            /* update DPIDR fields */
            ctx->dpidr_info.revision = (ctx->dp_regs.dpidr >> 28) & 0x0F;
            ctx->dpidr_info.partno = (ctx->dp_regs.dpidr >> 20) & 0xFF;
            ctx->dpidr_info.version = (ctx->dp_regs.dpidr >> 12) & 0x0F;
            ctx->dpidr_info.designer = (ctx->dp_regs.dpidr >> 1) & 0x3FF;

            if (!has_multiple_bits(ctx->io_swc))
            {
                DBGS(" - Detected pins");
                DBGS(" - Resetting error");

                xSemaphoreTake(ctx->swd_mutex, portMAX_DELAY);
                /* reset error */
                /* first make sure we have the correct bank by invalidating the current select cache */
                ctx->dp_regs.select_ok = false;
                uint8_t ack =
                    swd_read_dpbank(ctx, REG_CTRLSTAT, REG_CTRLSTAT_BANK, &ctx->dp_regs.ctrlstat);

                if (ack != 1 || (ctx->dp_regs.ctrlstat & STAT_ERROR_FLAGS))
                {
                    DBGS(" - send ABORT");
                    swd_abort(ctx);
                }
                DBGS(" - Fetch CTRL/STAT");
                ctx->dp_regs.ctrlstat_ok =
                    swd_read_dpbank(
                        ctx, REG_CTRLSTAT, REG_CTRLSTAT_BANK, &ctx->dp_regs.ctrlstat) == 1;
                DBG("     %08lX %s",
                    ctx->dp_regs.ctrlstat,
                    ctx->dp_regs.ctrlstat_ok ? "OK" : "FAIL");

                if (ctx->dpidr_info.version >= 1)
                {
                    DBGS(" - DAPv1, read DLCR");
                    ctx->dp_regs.dlcr_ok =
                        swd_read_dpbank(ctx, REG_DLCR, REG_DLCR_BANK, &ctx->dp_regs.dlcr) == 1;
                    DBG("     %08lX %s", ctx->dp_regs.dlcr, ctx->dp_regs.dlcr_ok ? "OK" : "FAIL");
                }

                if (ctx->dpidr_info.version >= 2)
                {
                    DBGS(" - DAPv2, read TARGETID");
                    ctx->dp_regs.targetid_ok =
                        swd_read_dpbank(
                            ctx, REG_TARGETID, REG_TARGETID_BANK, &ctx->dp_regs.targetid) == 1;
                    DBG("     %08lX %s",
                        ctx->dp_regs.targetid,
                        ctx->dp_regs.targetid_ok ? "OK" : "FAIL");
                    DBGS(" - DAPv2, read EVENTSTAT");
                    ctx->dp_regs.eventstat_ok =
                        swd_read_dpbank(
                            ctx, REG_EVENTSTAT, REG_EVENTSTAT_BANK, &ctx->dp_regs.eventstat) == 1;
                    DBG("     %08lX %s",
                        ctx->dp_regs.eventstat,
                        ctx->dp_regs.eventstat_ok ? "OK" : "FAIL");
                    DBGS(" - DAPv2, read DLPIDR");
                    ctx->dp_regs.dlpidr_ok =
                        swd_read_dpbank(ctx, REG_DLPIDR, REG_DLPIDR_BANK, &ctx->dp_regs.dlpidr) ==
                        1;
                    DBG("     %08lX %s",
                        ctx->dp_regs.dlpidr,
                        ctx->dp_regs.dlpidr_ok ? "OK" : "FAIL");
                }

                if (ctx->dp_regs.targetid_ok)
                {
                    ctx->targetid_info.revision = (ctx->dp_regs.targetid >> 28) & 0x0F;
                    ctx->targetid_info.partno = (ctx->dp_regs.targetid >> 12) & 0xFFFF;
                    ctx->targetid_info.designer = (ctx->dp_regs.targetid >> 1) & 0x3FF;
                }

                xSemaphoreGive(ctx->swd_mutex);
            }

            /* Only finish once SWCLK is uniquely determined.
             * Otherwise keep scanning with additional direction masks until
             * ctx->io_swc collapses to a single bit (and ctx->io_num_* get set).
             */
            if (!has_multiple_bits(ctx->io_swc) &&
                ctx->io_num_swd != 0xFF &&
                ctx->io_num_swc != 0xFF)
            {
                return;
            }
        }
        else
        {
            if (!has_multiple_bits(ctx->io_swc))
            {
                DBGS(" - Lost device");
            }
        }
        DBGS("next mask");
    }
}


void swd_init(AppFSM *const ctx, uint32_t io_mask)
{
    assert(ctx);

    io_mask &= gpio_legal_mask;

    ctx->loop_count = 0;
    ctx->detected_timeout = 0;
    ctx->detected = false;
    ctx->detected_device = false;
    ctx->detected_notified = false;
    ctx->ap_scanned = 0;
    ctx->current_mask_id = 0;
    ctx->current_mask = gpio_direction_mask[ctx->current_mask_id];
    ctx->io_selected = io_mask;
    ctx->io_swd = io_mask;
    ctx->io_swc = io_mask;
    ctx->io_num_swd = 0xFF;
    ctx->io_num_swc = 0xFF;
    memset(&ctx->dp_regs, 0x00, sizeof(ctx->dp_regs));
    memset(&ctx->targetid_info, 0x00, sizeof(ctx->targetid_info));
    memset(&ctx->apidr_info, 0x00, sizeof(ctx->apidr_info));
    ctx->hex_addr = 0x40002800;
    ctx->hex_addr = 0xE000EDF0;
    ctx->swd_clock_delay = CLOCK_DELAY;
    ctx->swd_idle_bits = IDLE_BITS;
    ctx->swd_mutex = xSemaphoreCreateRecursiveMutex();
    xSemaphoreGive(ctx->swd_mutex);

    strcpy(ctx->state_string, "none");
}

void swd_deinit(AppFSM *const ctx)
{
    assert(ctx);
    vSemaphoreDelete(ctx->swd_mutex);

    strcpy(ctx->state_string, "exiting");
}
