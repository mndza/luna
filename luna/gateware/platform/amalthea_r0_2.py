#
# This file is part of LUNA.
#
# Copyright (c) 2020-2023 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause

import os

from amaranth.build import *
from amaranth_boards.resources import *

from .cynthion_r1 import CynthionPlatformRev1

__all__ = ["AmaltheaPlatformRev0D2"]

class AmaltheaPlatformRev0D2(CynthionPlatformRev1):
    """ Board description for Amalthea r0.2. """

    name        = "Amalthea r0.2"

    #
    # I/O resources.
    #
    resources   = CynthionPlatformRev1.resources + [
        # Radio.
        Resource("radio", 0,
            Subsignal("rst",   PinsN("A12", dir="o")),

            Subsignal("clk",   Pins( "A9", dir="o")),
            Subsignal("sel",   PinsN("B10", dir="o")),
            Subsignal("copi",  Pins( "A11", dir="o")),  # TODO: rename to PICO, POCI
            Subsignal("cipo",  Pins( "B12", dir="i")),

            Subsignal("irq",   Pins( "B9", dir="i")),

            Subsignal("rxclk", DiffPairs("E14", "F14", dir="i"), Attrs(IO_TYPE="LVDS", DIFFRESISTOR="100", PULLMODE="UP"), Clock(64e6)),
            Subsignal("rxd09", DiffPairs("G12", "G13", dir="i"), Attrs(IO_TYPE="LVDS", DIFFRESISTOR="100")),
            Subsignal("rxd24", DiffPairs("C14", "D14", dir="i"), Attrs(IO_TYPE="LVDS", DIFFRESISTOR="100")),

            #Subsignal("txclk", DiffPairs("C16", "C15", dir="o"), Attrs(IO_TYPE="LVCMOS33D", DRIVE="4")),
            #Subsignal("txd",   DiffPairs("B16", "B15", dir="o"), Attrs(IO_TYPE="LVCMOS33D", DRIVE="4")),
            Attrs(IO_TYPE="LVCMOS33"),
        ),
    ]
