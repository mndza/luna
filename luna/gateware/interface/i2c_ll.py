# Based on I2C code from Glasgow, ported to Amaranth HDL
# I2C reference: https://www.nxp.com/docs/en/user-guide/UM10204.pdf

from amaranth import Elaboratable, Module, Signal, Cat, C
from amaranth.lib.cdc import FFSynchronizer
from amaranth.hdl.rec import Record, DIR_FANIN, DIR_FANOUT, DIR_NONE
from contextlib import contextmanager

import unittest
from ..test import LunaGatewareTestCase, sync_test_case

__all__ = ["I2CBus", "I2CInitiator"]

class I2CBus(Record):
    """ Record representing an I2C bus. """
    def __init__(self):
        super().__init__([
            ('scl', [('i', 1, DIR_FANIN), ('o', 1, DIR_FANOUT), ('oe', 1, DIR_FANOUT)]),
            ('sda', [('i', 1, DIR_FANIN), ('o', 1, DIR_FANOUT), ('oe', 1, DIR_FANOUT)]),
        ])

class I2CBusDriver(Elaboratable):
    def __init__(self, pads):
        self.scl_t  = pads.scl_t if hasattr(pads, "scl_t") else pads.scl
        self.sda_t  = pads.sda_t if hasattr(pads, "sda_t") else pads.sda

        self.scl_i  = Signal()
        self.scl_o  = Signal(reset=1)
        self.sda_i  = Signal()
        self.sda_o  = Signal(reset=1)

        self.sample = Signal(name="bus_sample")
        self.setup  = Signal(name="bus_setup")
        self.start  = Signal(name="bus_start")
        self.stop   = Signal(name="bus_stop")

    def elaborate(self, platform):
        m = Module()

        scl_r = Signal(reset=1)
        sda_r = Signal(reset=1)

        m.d.comb += [
            # Only drive SDA/SCL when =0 (pull-up by default)
            self.scl_t.o  .eq(0),
            self.scl_t.oe .eq(~self.scl_o),
            self.sda_t.o  .eq(0),
            self.sda_t.oe .eq(~self.sda_o),

            self.sample .eq(~scl_r & self.scl_i),  # SCL rising edge
            self.setup  .eq(scl_r & ~self.scl_i),  # SCL falling edge
            self.start  .eq(self.scl_i & sda_r & ~self.sda_i),  # SDA fall, SCL high
            self.stop   .eq(self.scl_i & ~sda_r & self.sda_i),  # SDA rise, SCL high
        ]
        m.d.sync += [
            scl_r.eq(self.scl_i),
            sda_r.eq(self.sda_i),
        ]
        m.submodules += [
            FFSynchronizer(self.scl_t.i, self.scl_i, reset=1),
            FFSynchronizer(self.sda_t.i, self.sda_i, reset=1),
        ]
        
        return m


class I2CInitiator(Elaboratable):
    """
    Simple I2C transaction initiator.

    Generates start and stop conditions, and transmits and receives octets.
    Clock stretching is supported.

    :param period_cyc:
        Bus clock period, as a multiple of system clock period.
    :type period_cyc: int
    :param clk_stretch:
        If true, SCL will be monitored for devices stretching the clock. Otherwise,
        only internally generated SCL is considered.
    :type clk_stretch: bool

    :attr busy:
        Busy flag. Low if the state machine is idle, high otherwise.
    :attr start:
        Start strobe. When ``busy`` is low, asserting ``start`` for one cycle generates
        a start or repeated start condition on the bus. Ignored when ``busy`` is high.
    :attr stop:
        Stop strobe. When ``busy`` is low, asserting ``stop`` for one cycle generates
        a stop condition on the bus. Ignored when ``busy`` is high.
    :attr write:
        Write strobe. When ``busy`` is low, asserting ``write`` for one cycle receives
        an octet on the bus and latches it to ``data_o``, after which the acknowledge bit
        is asserted if ``ack_i`` is high. Ignored when ``busy`` is high.
    :attr data_i:
        Data octet to be transmitted. Latched immediately after ``write`` is asserted.
    :attr ack_o:
        Received acknowledge bit.
    :attr read:
        Read strobe. When ``busy`` is low, asserting ``read`` for one cycle latches
        ``data_i`` and transmits it on the bus, after which the acknowledge bit
        from the bus is latched to ``ack_o``. Ignored when ``busy`` is high.
    :attr data_o:
        Received data octet.
    :attr ack_i:
        Acknowledge bit to be transmitted. Latched immediately after ``read`` is asserted.
    """
    def __init__(self, pads, period_cyc, clk_stretch=True):
        self.busy   = Signal(reset=1)
        self.start  = Signal()
        self.stop   = Signal()
        self.read   = Signal()
        self.data_i = Signal(8)
        self.ack_o  = Signal()
        self.write  = Signal()
        self.data_o = Signal(8)
        self.ack_i  = Signal()

        self.bus         = I2CBusDriver(pads)
        self.period_cyc  = int(period_cyc)
        self.clk_stretch = clk_stretch

    def elaborate(self, platform):
        m = Module()

        m.submodules.bus = bus = self.bus

        timer = Signal(range(self.period_cyc))
        stb   = Signal()

        with m.If((timer == 0) | ~self.busy):
            m.d.sync += timer.eq(self.period_cyc // 4)
        with m.Elif((not self.clk_stretch) | (bus.scl_o == bus.scl_i)):
            m.d.sync += timer.eq(timer - 1)

        m.d.comb += stb.eq(timer == 0)

        bitno   = Signal(range(8))
        r_shreg = Signal(8)
        w_shreg = Signal(8)
        r_ack   = Signal()

        # SCL low phase context
        @contextmanager
        def scl_l(state, next_state):
            with m.State(state) as _state:
                with m.If(stb) as _stb:
                   m.d.sync += bus.scl_o.eq(0)
                   m.next = next_state
                   yield (_state, _stb)

        # SCL high phase context
        @contextmanager
        def scl_h(state, next_state):
            with m.State(state) as _state:
                with m.If(stb):
                    m.d.sync += bus.scl_o.eq(1)
                with m.Elif(bus.scl_o == 1) as _scl_o:
                    with m.If((not self.clk_stretch) | (bus.scl_i == 1)):
                        m.next = next_state
                        yield (_state, _scl_o)

        # State transition context
        @contextmanager
        def stb_x(state, next_state):
            with m.State(state) as _state:
                with m.If(stb) as _stb:
                    m.next = next_state
                    yield (_state, _stb)
        
        with m.FSM() as fsm:
            with m.State("IDLE"):
                m.d.sync += self.busy.eq(1)
                with m.If(self.start):
                    with m.If(bus.scl_i & bus.sda_i):
                        m.next = "START-SDA-L"
                    with m.Elif(~bus.scl_i):
                        m.next = "START-SCL-H"
                    with m.Elif(bus.scl_i):
                        m.next = "START-SCL-L"
                with m.Elif(self.stop):
                    with m.If(bus.scl_i & ~bus.sda_o):
                        m.next = "STOP-SDA-H"
                    with m.Elif(~bus.scl_i):
                        m.next = "STOP-SCL-H"
                    with m.Elif(bus.scl_i):
                        m.next = "STOP-SCL-L"
                with m.Elif(self.write):
                    m.d.sync += w_shreg.eq(self.data_i)
                    m.next = "WRITE-DATA-SCL-L"
                with m.Elif(self.read):
                    m.d.sync += r_ack.eq(self.ack_i)
                    m.next = "READ-DATA-SCL-L"
                with m.Else():
                    m.d.sync += self.busy.eq(0)
            
            # start
            with scl_l("START-SCL-L", "START-SDA-H"): pass
            with stb_x("START-SDA-H", "START-SCL-H"):
                m.d.sync += bus.sda_o.eq(1)
            with scl_h("START-SCL-H", "START-SDA-L"): pass
            with stb_x("START-SDA-L", "IDLE"):
                m.d.sync += bus.sda_o.eq(0)
            
            # stop
            with scl_l("STOP-SCL-L",  "STOP-SDA-L"): pass
            with stb_x("STOP-SDA-L",  "STOP-SCL-H"):
                m.d.sync += bus.sda_o.eq(0)
            with scl_h("STOP-SCL-H",  "STOP-SDA-H"): pass
            with stb_x("STOP-SDA-H",  "IDLE"):
                m.d.sync += bus.sda_o.eq(1)
            
            # write data
            with scl_l("WRITE-DATA-SCL-L", "WRITE-DATA-SDA-X"): pass
            with stb_x("WRITE-DATA-SDA-X", "WRITE-DATA-SCL-H"):
                m.d.sync += bus.sda_o.eq(w_shreg[7])
            with scl_h("WRITE-DATA-SCL-H", "WRITE-DATA-SDA-N"):
                m.d.sync += w_shreg.eq(Cat(C(0, 1), w_shreg[0:7]))
            with stb_x("WRITE-DATA-SDA-N", "WRITE-DATA-SCL-L"):
                m.d.sync += bitno.eq(bitno + 1)
                with m.If(bitno == 7):
                    m.next = "WRITE-ACK-SCL-L"

            # write ack
            with scl_l("WRITE-ACK-SCL-L", "WRITE-ACK-SDA-H"): pass
            with stb_x("WRITE-ACK-SDA-H", "WRITE-ACK-SCL-H"):
                m.d.sync += bus.sda_o.eq(1)
            with scl_h("WRITE-ACK-SCL-H", "WRITE-ACK-SDA-N"):
                m.d.sync += self.ack_o.eq(~bus.sda_i)
            with stb_x("WRITE-ACK-SDA-N", "IDLE"): pass
            
            # read data
            with scl_l("READ-DATA-SCL-L", "READ-DATA-SDA-H"): pass
            with stb_x("READ-DATA-SDA-H", "READ-DATA-SCL-H"):
                m.d.sync += bus.sda_o.eq(1)
            with scl_h("READ-DATA-SCL-H", "READ-DATA-SDA-N"):
                m.d.sync += r_shreg.eq(Cat(bus.sda_i, r_shreg[0:7]))
            with stb_x("READ-DATA-SDA-N", "READ-DATA-SCL-L"):
                m.d.sync += bitno.eq(bitno + 1)
                with m.If(bitno == 7):
                    m.next = "READ-ACK-SCL-L"

            # read ack
            with scl_l("READ-ACK-SCL-L", "READ-ACK-SDA-X"): pass
            with stb_x("READ-ACK-SDA-X", "READ-ACK-SCL-H"):
                m.d.sync += bus.sda_o.eq(~r_ack)
            with scl_h("READ-ACK-SCL-H", "READ-ACK-SDA-N"):
                m.d.sync += self.data_o.eq(r_shreg)
            with stb_x("READ-ACK-SDA-N", "IDLE"): pass

        return m


class I2CInitiatorTestbench(I2CInitiator):
    def __init__(self, pads, period_cyc, clk_stretch=True):
        super().__init__(pads, period_cyc, clk_stretch)
        self.scl_o = Signal(reset=1)  # used to override values from testbench
        self.sda_o = Signal(reset=1)  

    def elaborate(self, platform):
        m = super().elaborate(platform)
        m.d.comb += [
            self.bus.scl_t.i.eq((self.bus.scl_t.o | ~self.bus.scl_t.oe) & self.scl_o),
            self.bus.sda_t.i.eq((self.bus.sda_t.o | ~self.bus.sda_t.oe) & self.sda_o),
        ]
        return m

class TestI2CInitiator(LunaGatewareTestCase):
    FRAGMENT_UNDER_TEST = I2CInitiatorTestbench
    FRAGMENT_ARGUMENTS = { "pads": I2CBus(), "period_cyc": 16 }

    def wait_condition(self, strobe):
        yield from self.wait_until(strobe, timeout=3*self.dut.period_cyc)

    def start(self):
        yield from self.pulse(self.dut.start)
        yield from self.wait_condition(self.dut.bus.start)

    def stop(self):
        yield from self.pulse(self.dut.stop)
        yield from self.wait_condition(self.dut.bus.stop)
    
    @sync_test_case
    def test_start(self):
        yield from self.start()
        self.assertEqual((yield self.dut.busy), 0)

    @sync_test_case
    def test_repeated_start(self):
        yield self.dut.bus.sda_o.eq(0)
        yield
        yield
        yield from self.start()
        yield from self.wait_condition(self.dut.bus.start)
        self.assertEqual((yield self.dut.busy), 0)

    @sync_test_case
    def test_stop(self):
        yield self.dut.bus.sda_o.eq(0)
        yield
        yield
        yield from self.stop()
        self.assertEqual((yield self.dut.busy), 0)

    def write(self, data, bits, ack):
        yield self.dut.data_i.eq(data)
        yield from self.pulse(self.dut.write)
        for n, bit in enumerate(bits):
            yield
            yield
            yield from self.wait_condition(self.dut.bus.scl_i == 0)
            yield from self.wait_condition(self.dut.bus.scl_i == 1)
            self.assertEqual((yield self.dut.bus.sda_i), bit)
            yield
        yield from self.advance_cycles(self.dut.period_cyc // 2)
        yield
        yield
        yield from self.wait_condition(self.dut.bus.scl_i == 0)
        yield self.dut.sda_o.eq(not ack)
        yield from self.wait_condition(self.dut.bus.scl_i == 1)
        yield self.dut.sda_o.eq(1)
        self.assertEqual((yield self.dut.busy), 1)
        yield from self.advance_cycles(self.dut.period_cyc // 2)
        yield
        yield
        yield
        yield
        self.assertEqual((yield self.dut.busy), 0)
        self.assertEqual((yield self.dut.ack_o), ack)

    @sync_test_case
    def test_write_ack(self):
        yield self.dut.bus.sda_o.eq(0)
        yield
        yield
        yield from self.write(0xA5, [1, 0, 1, 0, 0, 1, 0, 1], 1)

    @sync_test_case
    def test_write_nak(self):
        yield self.dut.bus.sda_o.eq(0)
        yield
        yield
        yield from self.write(0x5A, [0, 1, 0, 1, 1, 0, 1, 0], 0)

    @sync_test_case
    def test_write_tx(self):
        yield from self.start()
        yield from self.write(0x55, [0, 1, 0, 1, 0, 1, 0, 1], 1)
        yield from self.write(0x33, [0, 0, 1, 1, 0, 0, 1, 1], 0)
        yield from self.stop()
        yield
        yield
        self.assertEqual((yield self.dut.bus.sda_i), 1)
        self.assertEqual((yield self.dut.bus.scl_i), 1)

    def read(self, data, bits, ack):
        yield self.dut.ack_i.eq(ack)
        yield from self.pulse(self.dut.read)
        for n, bit in enumerate(bits):
            yield
            yield
            yield from self.wait_condition(self.dut.bus.scl_i == 0)
            yield self.dut.sda_o.eq(bit)
            yield from self.wait_condition(self.dut.bus.scl_i == 1)
            yield
        yield self.dut.sda_o.eq(1)
        yield from self.advance_cycles(self.dut.period_cyc // 2)
        yield
        yield
        yield from self.wait_condition(self.dut.bus.scl_i == 0)
        yield from self.wait_condition(self.dut.bus.scl_i == 1)
        self.assertEqual((yield self.dut.bus.sda_i), not ack)
        self.assertEqual((yield self.dut.busy), 1)
        yield from self.advance_cycles(self.dut.period_cyc // 2)
        yield
        yield
        yield
        yield
        self.assertEqual((yield self.dut.busy), 0)
        self.assertEqual((yield self.dut.data_o), data)

    @sync_test_case
    def test_read_ack(self):
        yield self.dut.bus.sda_o.eq(0)
        yield
        yield
        yield from self.read(0xA5, [1, 0, 1, 0, 0, 1, 0, 1], 1)

    @sync_test_case
    def test_read_nak(self):
        yield self.dut.bus.sda_o.eq(0)
        yield
        yield
        yield from self.read(0x5A, [0, 1, 0, 1, 1, 0, 1, 0], 0)

    @sync_test_case
    def test_read_tx(self):
        yield from self.start()
        yield from self.read(0x55, [0, 1, 0, 1, 0, 1, 0, 1], 1)
        yield from self.read(0x33, [0, 0, 1, 1, 0, 0, 1, 1], 0)
        yield from self.stop()
        yield
        yield
        self.assertEqual((yield self.dut.bus.sda_i), 1)
        self.assertEqual((yield self.dut.bus.scl_i), 1)


if __name__ == "__main__":
    unittest.main()
