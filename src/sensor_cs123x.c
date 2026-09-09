// Support for CS1237 ADC chip
//
// Copyright (C) 2026 James Turton <james.turton@gmx.com>
// Original CS1237 driver Copyright (C) 2024 ELEGOO
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/gpio.h"        // gpio_out_write
#include "board/irq.h"         // irq_poll
#include "board/misc.h"        // timer_read_time
#include "basecmd.h"           // oid_alloc
#include "command.h"           // DECL_COMMAND
#include "sched.h"             // sched_add_timer
#include "sensor_bulk.h"       // sensor_bulk_report
#include "load_cell_probe.h"   // load_cell_probe_report_sample
#include <stdbool.h>
#include <stdint.h>

struct cs123x_adc
{
    struct timer timer;
    uint8_t flags;
    uint32_t rest_ticks;
    uint32_t last_error;
    struct gpio_in dout;  // pin used to receive data from the cs123x
    struct gpio_out din;  // pin used to send data from the host
    struct gpio_out sclk; // pin used to generate clock for the cs123x
    struct sensor_bulk sb;
    struct load_cell_probe *lce;
};

enum
{
    CS_PENDING = 1 << 0,
    CS_OVERFLOW = 1 << 1,
};

#define BYTES_PER_SAMPLE 4
#define SAMPLE_ERROR_DESYNC 1L << 31
#define SAMPLE_ERROR_READ_TOO_LONG 1L << 30

#define CS123X_WRITE_CONFIG 0x65
#define CS123X_READ_CONFIG 0x56

static struct task_wake wake_cs123x;


/****************************************************************
 * Low-level bit-banging
 ****************************************************************/

#define MIN_PULSE_TIME nsecs_to_ticks(500)

static uint32_t
nsecs_to_ticks(uint32_t ns)
{
    return timer_from_us(ns * 1000) / 1000000;
}

static void
cs123x_delay_noirq(void)
{
    uint32_t end = timer_read_time() + MIN_PULSE_TIME;
    while (timer_is_before(timer_read_time(), end))
        ;
}

static void
cs123x_delay(void)
{
    uint32_t end = timer_read_time() + MIN_PULSE_TIME;
    while (timer_is_before(timer_read_time(), end))
        irq_poll();
}

static void cs123x_generate_clk(struct gpio_out sclk, uint8_t count)
{
    for (int i = 0; i < count; i++) {
        irq_disable();
        gpio_out_toggle_noirq(sclk);
        cs123x_delay_noirq();
        gpio_out_toggle_noirq(sclk);
        irq_enable();
        cs123x_delay();
    }
}

static uint32_t
cs123x_raw_read(struct gpio_in dout, struct gpio_out sclk)
{
    uint32_t bits_read = 0;
    for (int i = 0; i < 24; i++) {
        irq_disable();
        gpio_out_toggle_noirq(sclk);
        cs123x_delay_noirq();
        gpio_out_toggle_noirq(sclk);
        uint_fast8_t bit = gpio_in_read(dout);
        irq_enable();
        cs123x_delay();
        bits_read = (bits_read << 1) | bit;
    }
    cs123x_generate_clk(sclk, 3);
    return bits_read;
}

static uint_fast8_t
cs123x_config(struct gpio_in dout, struct gpio_out din, struct gpio_out sclk,
              uint_fast8_t cmd, uint_fast8_t val)
{
    uint_fast8_t config = 0;
    uint_fast8_t write_config = (cmd == CS123X_WRITE_CONFIG);
    uint32_t timeout = timer_read_time() + timer_from_us(200000);

    // Exit low-power mode
    gpio_out_write(sclk, 0);
    // Waiting for data to be ready and has pulled the pin down
    while (gpio_in_read(dout))
    {
        if (timer_is_before(timeout, timer_read_time()))
            return 0XFF;
    }

    // 1- 27
    cs123x_generate_clk(sclk, 27);

    // 28-29
    // Set the I/O pin to output mode and force it to a high level
    gpio_out_reset(din, 1);
    cs123x_generate_clk(sclk, 2);

    // 30-36
    // Send read and write commands
    for (int i = 0; i < 7; i++)
    {
        irq_disable();
        gpio_out_write(din, cmd & 0x40);
        cmd <<= 1;
        gpio_out_toggle_noirq(sclk);
        cs123x_delay_noirq();
        gpio_out_toggle_noirq(sclk);
        irq_enable();
        cs123x_delay();
    }

    // 37
    // Read the register to set the I/O as an input
    if (!write_config)
        gpio_in_reset(dout, 1);
    cs123x_generate_clk(sclk, 1);

    // 38-45
    // Read register
    if (!write_config)
    {
        for (int i = 0; i < 8; i++)
        {
            irq_disable();
            gpio_out_toggle_noirq(sclk);
            cs123x_delay_noirq();
            gpio_out_toggle_noirq(sclk);
            uint_fast8_t bit = gpio_in_read(dout);
            irq_enable();
            cs123x_delay();
            config = (config << 1) | bit;
        }
    }
    // Write to register
    else
    {
        for (int i = 0; i < 8; i++)
        {
            irq_disable();
            gpio_out_write(din, val & 0x80);
            val <<= 1;
            gpio_out_toggle_noirq(sclk);
            cs123x_delay_noirq();
            gpio_out_toggle_noirq(sclk);
            irq_enable();
            cs123x_delay();
        }
    }

    // 46
    // Write to the register to set the I/O as an input
    if (write_config)
        gpio_in_reset(dout, 1);
    cs123x_generate_clk(sclk, 1);

    // Enter low-power mode
    gpio_out_write(sclk, 1);
    return config;
}

/****************************************************************
 * CS1237 Sensor Support
 ****************************************************************/

// Check if data is ready
static uint_fast8_t
cs123x_is_data_ready(struct cs123x_adc *cs123x)
{
    return !gpio_in_read(cs123x->dout);
}

// Event handler that wakes wake_cs123x() periodically
static uint_fast8_t
cs123x_event(struct timer *timer)
{
    struct cs123x_adc *cs123x = container_of(timer, struct cs123x_adc, timer);
    uint32_t rest_ticks = cs123x->rest_ticks;
    uint8_t flags = cs123x->flags;
    if (flags & CS_PENDING) {
        cs123x->sb.possible_overflows++;
        cs123x->flags = CS_PENDING | CS_OVERFLOW;
        rest_ticks *= 4;
    } else if (cs123x_is_data_ready(cs123x)) {
        // New sample pending
        cs123x->flags = CS_PENDING;
        sched_wake_task(&wake_cs123x);
        rest_ticks *= 8;
    }
    cs123x->timer.waketime += rest_ticks;
    return SF_RESCHEDULE;
}

static void
add_sample(struct cs123x_adc *cs123x, uint8_t oid, uint32_t counts,
           uint8_t force_flush) {
    // Add measurement to buffer
    cs123x->sb.data[cs123x->sb.data_count] = counts;
    cs123x->sb.data[cs123x->sb.data_count + 1] = counts >> 8;
    cs123x->sb.data[cs123x->sb.data_count + 2] = counts >> 16;
    cs123x->sb.data[cs123x->sb.data_count + 3] = counts >> 24;
    cs123x->sb.data_count += BYTES_PER_SAMPLE;

    if (cs123x->sb.data_count + BYTES_PER_SAMPLE > ARRAY_SIZE(cs123x->sb.data)
        || force_flush)
        sensor_bulk_report(&cs123x->sb, oid);
}

// cs123x ADC query
static void
cs123x_read_adc(struct cs123x_adc *cs123x, uint8_t oid)
{
    // Read from sensor
    uint32_t adc = cs123x_raw_read(cs123x->dout, cs123x->sclk);

    // Clear pending flag (and note if an overflow occurred)
    irq_disable();
    uint8_t flags = cs123x->flags;
    cs123x->flags = 0;
    irq_enable();

    // Extract report from raw data
    if (adc & 0x800000)
        adc |= 0xFF000000;

    // Transfer took too long
    if (flags & CS_OVERFLOW)
        cs123x->last_error = SAMPLE_ERROR_READ_TOO_LONG;

    // forever send errors until reset
    if (cs123x->last_error != 0)
        adc = cs123x->last_error;

    // probe is optional, report if enabled
    if (cs123x->last_error == 0 && cs123x->lce) {
        load_cell_probe_report_sample(cs123x->lce, adc);
    }

    // Add measurement to buffer
    add_sample(cs123x, oid, adc, false);
}

// Create a cs123x sensor
void
command_config_cs123x(uint32_t *args)
{
    struct cs123x_adc *cs123x = oid_alloc(args[0], command_config_cs123x,
                                          sizeof(*cs123x));
    cs123x->timer.func = cs123x_event;
    cs123x->din = gpio_out_setup(args[1], 1);
    cs123x->dout = gpio_in_setup(args[1], 1);
    cs123x->sclk = gpio_out_setup(args[2], 0);
    gpio_out_write(cs123x->sclk, 1); // put chip in power down state
}
DECL_COMMAND(command_config_cs123x, "config_cs123x oid=%c"
                                    " dout_pin=%u sclk_pin=%u");

void
command_cs123x_write(uint32_t *args)
{
    struct cs123x_adc *cs123x = oid_lookup(args[0], command_config_cs123x);
    cs123x_config(cs123x->dout, cs123x->din, cs123x->sclk, CS123X_WRITE_CONFIG,
                  args[1]);
    output("command_cs123x_write oid %u config %u", args[0], args[1]);
}
DECL_COMMAND(command_cs123x_write, "cs123x_write oid=%c config=%c");

void
command_cs123x_read(uint32_t *args)
{
    struct cs123x_adc *cs123x = oid_lookup(args[0], command_config_cs123x);
    uint8_t config = cs123x_config(cs123x->dout, cs123x->din, cs123x->sclk,
                                   CS123X_READ_CONFIG, 0xff);
    sendf("cs123x_read_response oid=%c config=%c", args[0], config);
    output("command_cs123x_read oid %u config %u", args[0], config);
}
DECL_COMMAND(command_cs123x_read, "cs123x_read oid=%c");

void
cs123x_attach_load_cell_probe(uint32_t *args) {
    uint8_t oid = args[0];
    struct cs123x_adc *cs123x = oid_lookup(oid, command_config_cs123x);
    cs123x->lce = load_cell_probe_oid_lookup(args[1]);
}
DECL_COMMAND(cs123x_attach_load_cell_probe, "cs123x_attach_load_cell_probe"
    " oid=%c load_cell_probe_oid=%c");

// start/stop capturing ADC data
void
command_query_cs123x(uint32_t *args)
{
    uint8_t oid = args[0];
    struct cs123x_adc *cs123x = oid_lookup(oid, command_config_cs123x);
    sched_del_timer(&cs123x->timer);
    cs123x->flags = 0;
    cs123x->last_error = 0;
    cs123x->rest_ticks = args[1];
    if (!cs123x->rest_ticks) {
        // End measurements
        gpio_out_write(cs123x->sclk, 1); // put chip in power down state
        return;
    }
    // Start new measurements
    gpio_out_write(cs123x->sclk, 0); // wake chip from power down
    sensor_bulk_reset(&cs123x->sb);
    irq_disable();
    cs123x->timer.waketime = timer_read_time() + cs123x->rest_ticks;
    sched_add_timer(&cs123x->timer);
    irq_enable();
}
DECL_COMMAND(command_query_cs123x, "query_cs123x oid=%c rest_ticks=%u");

void
command_query_cs123x_status(const uint32_t *args)
{
    uint8_t oid = args[0];
    struct cs123x_adc *cs123x = oid_lookup(oid, command_config_cs123x);
    irq_disable();
    const uint32_t start_t = timer_read_time();
    uint8_t is_data_ready = cs123x_is_data_ready(cs123x);
    irq_enable();
    uint8_t pending_bytes = is_data_ready ? BYTES_PER_SAMPLE : 0;
    sensor_bulk_status(&cs123x->sb, oid, start_t, 0, pending_bytes);
}
DECL_COMMAND(command_query_cs123x_status, "query_cs123x_status oid=%c");

// Background task that performs measurements
void
cs123x_capture_task(void)
{
    if (!sched_check_wake(&wake_cs123x))
        return;
    uint8_t oid;
    struct cs123x_adc *cs123x;
    foreach_oid(oid, cs123x, command_config_cs123x) {
        if (cs123x->flags)
            cs123x_read_adc(cs123x, oid);
    }
}
DECL_TASK(cs123x_capture_task);
