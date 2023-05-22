from luna.gateware.applets.analyzer import USBAnalyzerApplet
from luna.gateware.applets.speed_test import USBInSpeedTestDevice
from luna.gateware.platform import CynthionPlatformRev0D7
from amaranth import *

class EMCTestDevice(Elaboratable):

    def elaborate(self, platform):
        m = Module()
        m.submodules += platform.clock_domain_generator()
        m.submodules += USBAnalyzerApplet(generate_clocks=False)
        m.submodules += USBInSpeedTestDevice(generate_clocks=False,
                                             phy=platform.request('control_phy'),
                                             pid=0x0f3b)
        m.submodules += USBInSpeedTestDevice(generate_clocks=False,
                                             phy=platform.request('aux_phy'),
                                             pid=0x0f3c)
        return m
    
platform = CynthionPlatformRev0D7()
device = EMCTestDevice()

platform.build(device)
