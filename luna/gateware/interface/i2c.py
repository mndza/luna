# amaranth: UnusedElaboratable=no
#
# This file is part of LUNA.
#
# Copyright (c) 2020 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause

""" I2C interfacing hardware. """

import unittest

from amaranth       import Signal, Module, Elaboratable, Record
from amaranth.hdl.rec import Record, DIR_FANIN, DIR_FANOUT

from ..test         import LunaGatewareTestCase, sync_test_case

from .i2c_ll import I2CInitiator


class I2CDeviceInterface(Elaboratable):
    """ Gateware interface that handles I2C register reads and writes.

    I/O ports:

        # Controller signals:
        O: busy              -- indicates when the interface is busy processing a transaction
        I: address[8]        -- the address of the register to work with
        O: done              -- strobe that indicates when a register request is complete

        I: read_request      -- strobe that requests a register read
        O: read_data[8]      -- data read from the relevant register read

        I: write_request     -- strobe that indicates a register write
        I: write_data[8]     -- data to be written during a register write

    """

    def __init__(self, pads, period_cyc, address):

        self.pads          = pads
        self.period_cyc    = period_cyc
        self.dev_address   = address

        # I/O ports

        self.busy          = Signal()
        self.address       = Signal(8)
        self.done          = Signal()

        self.read_request  = Signal()
        self.read_data     = Signal(8)

        self.write_request = Signal()
        self.write_data    = Signal(8)


    def elaborate(self, platform):
        m = Module()

        current_address = Signal.like(self.address)
        current_write   = Signal.like(self.write_data)

        # I2C initiator (low level manager) and default signal values
        m.submodules.i2c = i2c = I2CInitiator(pads=self.pads, period_cyc=self.period_cyc, clk_stretch=True)
        m.d.comb += [
            i2c.start .eq(0),
            i2c.write .eq(0),
            i2c.read  .eq(0),
            i2c.stop  .eq(0),

            self.read_data.eq(i2c.data_o),
        ]

        with m.FSM() as fsm:

            # We're busy whenever we're not IDLE; indicate so.
            m.d.comb += self.busy.eq(~fsm.ongoing('IDLE'))

            # IDLE: wait for a request to be made
            with m.State('IDLE'):
                with m.If(self.read_request):
                    m.d.sync += current_address.eq(self.address)
                    m.next = 'RD_START'
                with m.If(self.write_request):
                    m.d.sync += current_address.eq(self.address)
                    m.d.sync += current_write.eq(self.write_data)
                    m.next = 'WR_START'

            # Write handling.
            
            with m.State('WR_START'):
                with m.If(~i2c.busy):
                    m.d.comb += i2c.start.eq(1)
                    m.next = 'WR_SEND_DEV_ADDRESS'

            with m.State("WR_SEND_DEV_ADDRESS"):
                with m.If(~i2c.busy):
                    m.d.comb += i2c.data_i.eq((self.dev_address << 1) | 0)
                    m.d.comb += i2c.write.eq(1)
                with m.Elif(i2c.ack_o):
                    m.next = "WR_SEND_REG_ADDRESS"

            with m.State("WR_SEND_REG_ADDRESS"):
                with m.If(~i2c.busy):
                    m.d.comb += i2c.data_i.eq(current_address)
                    m.d.comb += i2c.write.eq(1)
                with m.Elif(i2c.ack_o):
                    m.next = "WR_SEND_VALUE"

            with m.State("WR_SEND_VALUE"):
                with m.If(~i2c.busy):
                    m.d.comb += i2c.data_i.eq(current_write)
                    m.d.comb += i2c.write.eq(1)
                with m.Elif(i2c.ack_o):
                    m.next = "WR_FINISH"

            with m.State("WR_FINISH"):
                with m.If(~i2c.busy):
                    m.d.comb += i2c.stop.eq(1)
                    m.d.comb += self.done.eq(1)
                    m.next = "IDLE"

            # Read handling.
            # Read is actually divided into two phases:
            # - First phase (RD0) performs a write with the register address
            # - Second phase (RD1) performs the actual read

            with m.State('RD_START'):
                with m.If(~i2c.busy):
                    m.d.comb += i2c.start.eq(1)
                    m.next = 'RD0_SEND_DEV_ADDRESS'

            with m.State("RD0_SEND_DEV_ADDRESS"):
                with m.If(~i2c.busy):
                    m.d.comb += i2c.data_i.eq((self.dev_address << 1) | 0)
                    m.d.comb += i2c.write.eq(1)
                with m.Elif(i2c.ack_o):  # dev address asserted
                    m.next = "RD0_SEND_REG_ADDRESS"

            with m.State("RD0_SEND_REG_ADDRESS"):
                with m.If(~i2c.busy):
                    m.d.comb += i2c.data_i.eq(current_address)
                    m.d.comb += i2c.write.eq(1)
                with m.Elif(i2c.ack_o):  # reg address asserted
                    m.next = "RD0_FINISH"

            with m.State("RD0_FINISH"):
                with m.If(~i2c.busy):
                    m.d.comb += i2c.stop.eq(1)
                    m.next = "RD1_START"

            with m.State('RD1_START'):
                with m.If(~i2c.busy):
                    m.d.comb += i2c.start.eq(1)
                    m.next = 'RD1_SEND_DEV_ADDRESS'

            with m.State("RD1_SEND_DEV_ADDRESS"):
                with m.If(~i2c.busy):
                    m.d.comb += i2c.data_i.eq((self.dev_address << 1) | 1)
                    m.d.comb += i2c.write.eq(1)
                with m.Elif(i2c.ack_o):  # dev address asserted
                    m.next = "RD1_RECV_VALUE"

            with m.State("RD1_RECV_VALUE"):
                with m.If(~i2c.busy):
                    m.d.comb += i2c.ack_i.eq(0)  # 0 in last read byte
                    m.d.comb += i2c.read.eq(1)
                    m.next = "RD1_FINISH"

            with m.State("RD1_FINISH"):
                with m.If(~i2c.busy):
                    m.d.comb += i2c.stop.eq(1)
                    m.d.comb += self.done.eq(1)
                    m.next = "IDLE"

        return m


class I2CPads(Record):
    """ Record representing an I2C bus. """

    def __init__(self):
        super().__init__([
            ('scl', [('i', 1, DIR_FANIN), ('o', 1, DIR_FANOUT), ('oe', 1, DIR_FANOUT)]),
            ('sda', [('i', 1, DIR_FANIN), ('o', 1, DIR_FANOUT), ('oe', 1, DIR_FANOUT)]),
        ])


class TestI2CDeviceInterface(LunaGatewareTestCase):
    pads = I2CPads()
    FRAGMENT_UNDER_TEST = I2CDeviceInterface
    FRAGMENT_ARGUMENTS = {
        "address": 0b1110010,
        "pads": pads,
        "period_cyc": 300,  # 400 kHz
    }

    def initialize_signals(self):
        yield self.dut.read_request.eq(0)
        yield self.dut.write_request.eq(0)

    @sync_test_case
    def test_idle_behavior(self):
        self.assertEqual((yield self.dut.busy), 0)

    @sync_test_case
    def test_register_read(self):
        pass        

    @sync_test_case
    def test_register_write(self):
        pass

if __name__ == "__main__":
    unittest.main()