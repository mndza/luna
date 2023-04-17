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

from .i2c_ll import I2CBus, I2CInitiator


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

    def __init__(self, pads, *, period_cyc, address, clk_stretch=False, data_bytes=1):

        self.pads          = pads
        self.period_cyc    = period_cyc
        self.dev_address   = address
        self.clk_stretch   = clk_stretch

        # I/O ports

        self.busy          = Signal()
        self.address       = Signal(8)
        self.size          = Signal(range(data_bytes+1))
        self.done          = Signal()

        self.read_request  = Signal()
        self.read_data     = Signal(8 * data_bytes)

        self.write_request = Signal()
        self.write_data    = Signal(8 * data_bytes)



    def elaborate(self, platform):
        m = Module()

        current_address = Signal.like(self.address)
        current_write   = Signal.like(self.write_data)
        current_read    = Signal.like(self.read_data - 8)
        rem_bytes       = Signal.like(self.size)

        # I2C initiator (low level manager) and default signal values
        m.submodules.i2c = i2c = I2CInitiator(pads=self.pads, period_cyc=self.period_cyc, clk_stretch=self.clk_stretch)
        m.d.comb += [
            i2c.start .eq(0),
            i2c.write .eq(0),
            i2c.read  .eq(0),
            i2c.stop  .eq(0),
        ]

        with m.FSM() as fsm:

            # We're busy whenever we're not IDLE; indicate so.
            m.d.comb += self.busy.eq(~fsm.ongoing('IDLE'))

            # IDLE: wait for a request to be made
            with m.State('IDLE'):
                with m.If(self.read_request):
                    m.d.sync += current_address.eq(self.address)
                    m.d.sync += current_read.eq(0)
                    m.d.sync += rem_bytes.eq(self.size)
                    m.next = 'RD_START'
                with m.If(self.write_request):
                    m.d.sync += current_address.eq(self.address)
                    m.d.sync += current_write.eq(self.write_data)
                    m.d.sync += rem_bytes.eq(self.size)
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
                    m.next = "WR_ACK_DEV_ADDRESS"

            with m.State("WR_ACK_DEV_ADDRESS"):
                with m.If(~i2c.busy):
                    with m.If(i2c.ack_o):  # dev address asserted
                        m.next = "WR_SEND_REG_ADDRESS"
                    with m.Else():
                        m.next = "ABORT"

            with m.State("WR_SEND_REG_ADDRESS"):
                with m.If(~i2c.busy):
                    m.d.comb += i2c.data_i.eq(current_address)
                    m.d.comb += i2c.write.eq(1)
                    m.next = "WR_ACK_REG_ADDRESS"

            with m.State("WR_ACK_REG_ADDRESS"):
                with m.If(~i2c.busy):
                    with m.If(i2c.ack_o):  # reg address asserted
                        with m.If(rem_bytes != 0):
                            m.next = "WR_SEND_VALUE"
                        with m.Else():
                            m.next = "FINISH"  # 0-byte write
                    with m.Else():
                        m.next = "ABORT"

            with m.State("WR_SEND_VALUE"):
                with m.If(~i2c.busy):
                    m.d.comb += i2c.data_i.eq(current_write[-8:])
                    m.d.comb += i2c.write.eq(1)
                    # prepare next byte too
                    m.d.sync += current_write.eq(current_write << 8)
                    m.d.sync += rem_bytes.eq(rem_bytes - 1)
                    m.next = "WR_ACK_VALUE"
            
            with m.State("WR_ACK_VALUE"):
                with m.If(~i2c.busy):
                    with m.If(i2c.ack_o):
                        with m.If(rem_bytes != 0):
                            m.next = "WR_SEND_VALUE"
                        with m.Else():
                            m.next = "FINISH"
                    with m.Else():
                        m.next = "ABORT"

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
                    m.next = "RD0_ACK_DEV_ADDRESS"

            with m.State("RD0_ACK_DEV_ADDRESS"):
                with m.If(~i2c.busy):
                    with m.If(i2c.ack_o):  # dev address asserted
                        m.next = "RD0_SEND_REG_ADDRESS"
                    with m.Else():
                        m.next = "ABORT"

            with m.State("RD0_SEND_REG_ADDRESS"):
                with m.If(~i2c.busy):
                    m.d.comb += i2c.data_i.eq(current_address)
                    m.d.comb += i2c.write.eq(1)
                    m.next = "RD0_ACK_REG_ADDRESS"

            with m.State("RD0_ACK_REG_ADDRESS"):
                with m.If(~i2c.busy):
                    with m.If(i2c.ack_o):  # reg address asserted
                        with m.If(rem_bytes != 0):
                            m.next = "RD1_START"
                        with m.Else():
                            m.next = "FINISH"  # 0-byte read
                    with m.Else():
                        m.next = "ABORT"
            
            with m.State('RD1_START'):
                with m.If(~i2c.busy):
                    m.d.comb += i2c.start.eq(1)
                    m.next = 'RD1_SEND_DEV_ADDRESS'

            with m.State("RD1_SEND_DEV_ADDRESS"):
                with m.If(~i2c.busy):
                    m.d.comb += i2c.data_i.eq((self.dev_address << 1) | 1)
                    m.d.comb += i2c.write.eq(1)
                    m.next = "RD1_ACK_DEV_ADDRESS"

            with m.State("RD1_ACK_DEV_ADDRESS"):
                with m.If(~i2c.busy):
                    with m.If(i2c.ack_o):  # dev address asserted
                        m.next = "RD1_RECV_VALUE"
                    with m.Else():
                        m.next = "ABORT"

            with m.State("RD1_RECV_VALUE"):
                with m.If(~i2c.busy):
                    m.d.comb += i2c.ack_i.eq(~(rem_bytes == 1))  # 0 in last read byte
                    m.d.comb += i2c.read.eq(1)
                    m.d.sync += rem_bytes.eq(rem_bytes - 1)
                    m.next = "RD1_WAIT_VALUE"

            with m.State("RD1_WAIT_VALUE"):
                with m.If(~i2c.busy):
                    m.d.sync += current_read.eq((current_read << 8) | i2c.data_o)
                    with m.If(rem_bytes != 0):
                        m.next = "RD1_RECV_VALUE"
                    with m.Else():
                        m.d.sync += self.read_data.eq((current_read << 8) | i2c.data_o)
                        m.next = "FINISH"

            # Proper finish asserting "done"
            with m.State("FINISH"):
                with m.If(~i2c.busy):
                    m.d.comb += i2c.stop.eq(1)
                    m.d.comb += self.done.eq(1)
                    m.next = "IDLE"
            
            # Aborting returns to IDLE after freeing the bus
            with m.State("ABORT"):
                with m.If(~i2c.busy):
                    m.d.comb += i2c.stop.eq(1)
                    m.next = "IDLE"

        return m


class I2CDeviceInterfaceTestbench(I2CDeviceInterface):
    def __init__(self, pads, period_cyc, address):
        super().__init__(pads, period_cyc, address)
        self.scl_o = Signal(reset=1)  # used to override values from testbench
        self.sda_o = Signal(reset=1)  

    def elaborate(self, platform):
        m = super().elaborate(platform)
        m.d.comb += [
            self.pads.scl.i.eq((self.pads.scl.o | ~self.pads.scl.oe) & self.scl_o),
            self.pads.sda.i.eq((self.pads.sda.o | ~self.pads.sda.oe) & self.sda_o),
        ]
        return m

class TestI2CDeviceInterface(LunaGatewareTestCase):
    FRAGMENT_UNDER_TEST = I2CDeviceInterfaceTestbench
    FRAGMENT_ARGUMENTS = {
        "address": 0b1110010,
        "pads": I2CBus(),
        "period_cyc": 4,  # 400 kHz
    }

    def initialize_signals(self):
        yield self.dut.read_request.eq(0)
        yield self.dut.write_request.eq(0)

    @sync_test_case
    def test_idle_behavior(self):
        self.assertEqual((yield self.dut.busy), 0)

    @sync_test_case
    def test_register_read(self):

        # Set up a read request.
        yield self.dut.address.eq(0x12)
        yield from self.pulse(self.dut.read_request)

        # Starting the request should make us busy.
        self.assertEqual((yield self.dut.busy), 1)

        #yield from self.wait_until(self.dut.done, timeout=1000)


if __name__ == "__main__":
    unittest.main()