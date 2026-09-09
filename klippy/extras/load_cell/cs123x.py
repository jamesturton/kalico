# CS1237 Support
#
# Copyright (C) 2026 James Turton <james.turton@gmx.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

from klippy.mcu import MCU

from klippy.extras import bulk_sensor
from klippy.extras.load_cell.interfaces import (
    BulkAdcData,
    BulkAdcDataCallback,
    LoadCellSensor,
)

#
# Constants
#
UPDATE_INTERVAL = 0.10
SAMPLE_ERROR_DESYNC = -0x80000000
SAMPLE_ERROR_LONG_READ = 0x40000000

SPEED_SEL = {10: 0x0, 40: 0x1, 640: 0x2, 1280: 0x3}


# Implementation of CS1237
class CS123X(LoadCellSensor):
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.last_error_count = 0
        self.consecutive_fails = 0
        # Chip options
        dout_pin_name = config.get("dout_pin")
        sclk_pin_name = config.get("sclk_pin")
        ppins = printer.lookup_object("pins")
        dout_ppin = ppins.lookup_pin(dout_pin_name)
        sclk_ppin = ppins.lookup_pin(sclk_pin_name)
        mcu: MCU = dout_ppin["chip"]
        self.mcu: MCU = mcu
        self.oid = mcu.create_oid()
        if sclk_ppin["chip"] is not mcu:
            raise config.error(
                "%s config error: All pins must be "
                "connected to the same MCU" % (self.name,)
            )
        self.dout_pin = dout_ppin["pin"]
        self.sclk_pin = sclk_ppin["pin"]
        # Samples per second choices
        self.sps_options = {
            "10": 10,
            "40": 40,
            "640": 640,
            "1280": 1280,
        }
        self.sps = config.getchoice(
            "sample_rate", self.sps_options, default="640"
        )
        # Gain choices
        self.gain_options = {
            "1": 0x0,
            "2": 0x1,
            "64": 0x2,
            "128": 0x3,
        }
        self.pga_sel = config.getchoice(
            "gain", self.gain_options, default="128"
        )
        ## Bulk Sensor Setup
        # Clock tracking
        chip_smooth = self.sps * UPDATE_INTERVAL * 2
        self.ffreader = bulk_sensor.FixedFreqReader(mcu, chip_smooth, "<i")
        # Process messages in batches
        self.batch_bulk = bulk_sensor.BatchBulkHelper(
            self.printer,
            self._process_batch,
            self._start_measurements,
            self._finish_measurements,
            UPDATE_INTERVAL,
        )
        # Command Configuration
        self.query_cs123x_cmd = None
        self.read_cs123x_cmd = None
        self.write_cs123x_cmd = None
        self.attach_probe_cmd = None
        mcu.add_config_cmd(
            "config_cs123x oid=%d dout_pin=%s sclk_pin=%s"
            % (self.oid, self.dout_pin, self.sclk_pin)
        )
        mcu.add_config_cmd(
            "query_cs123x oid=%d rest_ticks=0" % (self.oid,), on_restart=True
        )
        mcu.register_config_callback(self._build_config)

    def _build_config(self):
        # All commands share one queue so that a config write is guaranteed to
        # be processed before the read that verifies it
        cmdqueue = self.mcu.alloc_command_queue()
        self.query_cs123x_cmd = self.mcu.lookup_command(
            "query_cs123x oid=%c rest_ticks=%u", cq=cmdqueue
        )
        self.write_cs123x_cmd = self.mcu.lookup_command(
            "cs123x_write oid=%c config=%c", cq=cmdqueue
        )
        self.read_cs123x_cmd = self.mcu.lookup_query_command(
            "cs123x_read oid=%c",
            "cs123x_read_response oid=%c config=%c",
            oid=self.oid,
            cq=cmdqueue,
        )
        self.attach_probe_cmd = self.mcu.lookup_command(
            "cs123x_attach_load_cell_probe oid=%c load_cell_probe_oid=%c",
            cq=cmdqueue,
        )
        self.ffreader.setup_query_command(
            "query_cs123x_status oid=%c", oid=self.oid, cq=cmdqueue
        )

    def get_mcu(self) -> MCU:
        return self.mcu

    def get_samples_per_second(self) -> int:
        return self.sps

    # returns a tuple of the minimum and maximum value of the sensor, used to
    # detect if a data value is saturated
    def get_range(self) -> tuple[int, int]:
        return -0x800000, 0x7FFFFF

    def get_channel_count(self) -> int:
        return 1

    # add_client interface, direct pass through to bulk_sensor API
    def add_client(self, callback: BulkAdcDataCallback):
        self.batch_bulk.add_client(callback)

    def attach_load_cell_probe(self, load_cell_probe_oid: int):
        self.attach_probe_cmd.send([self.oid, load_cell_probe_oid])

    # Chip configuration register layout (reset value 0x0C):
    #   [7]    reserved, must be written as 0
    #   [6]    REFO_OFF   1 = disable the internal reference output
    #   [5:4]  SPEED_SEL  ADC output rate
    #   [3:2]  PGA_SEL    gain
    #   [1:0]  CH_SEL     input channel
    def _config_register(self):
        return ((SPEED_SEL[self.sps] << 4) | (self.pga_sel << 2))

    # Bring the Config register to the requested state. The register is read
    # before writing so that an already configured chip - the common case on a
    # klippy restart - is left alone, and so that a chip still settling out of
    # power down gets that first attempt as free settle time.
    def setup_chip(self):
        want = self._config_register()
        reactor = self.printer.get_reactor()
        got = CONFIG_NOT_READY
        for _ in range(CONFIG_ATTEMPTS):
            got = self.read_cs123x_cmd.send([self.oid])["config"]
            if got == want:
                return
            self.write_cs123x_cmd.send([self.oid, want])
            reactor.pause(reactor.monotonic() + CONFIG_RETRY_DELAY)
        if got == CONFIG_NOT_READY:
            raise self.printer.command_error(
                "CS1237 '%s' never signalled data-ready.\n"
                "This is generally indicative of connection problems (e.g."
                " faulty wiring), the chip being unpowered, or a faulty"
                " CS1237 chip." % (self.name,)
            )
        raise self.printer.command_error(
            "Failed to set CS1237 '%s' config register to 0x%02x: got 0x%02x."
            " This may be a connection problem (e.g. faulty wiring)"
            % (self.name, want, got)
        )

    # Measurement decoding
    def _convert_samples(self, samples):
        adc_factor = 1.0 / (1 << 23)
        count = 0
        for ptime, val in samples:
            if val == SAMPLE_ERROR_DESYNC or val == SAMPLE_ERROR_LONG_READ:
                self.last_error_count += 1
                break  # additional errors are duplicates
            samples[count] = (round(ptime, 6), val, round(val * adc_factor, 9))
            count += 1
        del samples[count:]

    # Start, stop, and process message batches
    def _start_measurements(self):
        self.consecutive_fails = 0
        self.last_error_count = 0
        # Configure the chip before the capture timer starts driving SCLK
        self.setup_chip()
        # Start bulk reading
        rest_ticks = self.mcu.seconds_to_clock(1.0 / (10.0 * self.sps))
        self.query_cs123x_cmd.send([self.oid, rest_ticks])
        logging.info("CS1237 starting '%s' measurements", self.name)
        # Initialize clock tracking
        self.ffreader.note_start()

    def _finish_measurements(self):
        # don't use serial connection after shutdown
        if self.printer.is_shutdown():
            return
        # Halt bulk reading
        self.query_cs123x_cmd.send_wait_ack([self.oid, 0])
        self.ffreader.note_end()
        logging.info("CS1237 finished '%s' measurements", self.name)

    def _process_batch(self, eventtime) -> BulkAdcData:
        prev_overflows = self.ffreader.get_last_overflows()
        prev_error_count = self.last_error_count
        samples = self.ffreader.pull_samples()
        self._convert_samples(samples)
        overflows = self.ffreader.get_last_overflows() - prev_overflows
        errors = self.last_error_count - prev_error_count
        if errors > 0:
            logging.error("%s: Forced sensor restart due to error", self.name)
            self._finish_measurements()
            self._start_measurements()
        elif overflows > 0:
            self.consecutive_fails += 1
            if self.consecutive_fails > 4:
                logging.error(
                    "%s: Forced sensor restart due to overflows", self.name
                )
                self._finish_measurements()
                self._start_measurements()
        else:
            self.consecutive_fails = 0
        return {
            "data": samples,
            "errors": self.last_error_count,
            "overflows": self.ffreader.get_last_overflows(),
        }


CS123X_SENSOR_TYPE = {"cs1237": CS123X}
