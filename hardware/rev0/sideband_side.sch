EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 9
Title "LUNA: Sideband USB PHY"
Date "2020-12-22"
Rev "r0"
Comp "Great Scott Gadgets"
Comment1 "Katherine J. Temkin"
Comment2 ""
Comment3 "Licensed under the CERN-OHL-P v2"
Comment4 ""
$EndDescr
$Comp
L usb:USB3343 U8
U 1 1 5DCDAEF5
P 6100 2350
F 0 "U8" H 6700 2513 50  0000 C CNN
F 1 "USB3343" H 6700 2423 50  0000 C CNN
F 2 "Package_DFN_QFN:VQFN-24-1EP_4x4mm_P0.5mm_EP2.45x2.45mm" H 6100 2350 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/334x.pdf" H 6100 2350 50  0001 C CNN
F 4 "IC TRANSCEIVER 1/1 24QFN" H 6100 2350 50  0001 C CNN "Description"
F 5 "Microchip" H 6100 2350 50  0001 C CNN "Manufacturer"
F 6 "USB3343-CP" H 6100 2350 50  0001 C CNN "Part Number"
	1    6100 2350
	1    0    0    -1  
$EndComp
$Comp
L fpgas_and_processors:ECP5-BGA256 IC1
U 6 1 5DCE10A7
P 1550 1900
F 0 "IC1" H 1520 -167 50  0000 R CNN
F 1 "ECP5-BGA256" H 1520 -257 50  0000 R CNN
F 2 "luna:lattice_cabga256" H -1650 5350 50  0001 L CNN
F 3 "" H -2100 6300 50  0001 L CNN
F 4 "FPGA - Field Programmable Gate Array ECP5; 12k LUTs; 1.1V" H -2100 6200 50  0001 L CNN "Description"
F 5 "Lattice" H -2050 7150 50  0001 L CNN "Manufacturer"
F 6 "LFE5U-12F-6BG256C" H -2050 7050 50  0001 L CNN "Part Number"
	6    1550 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 1700 1700 1550
Wire Wire Line
	1700 1550 1750 1550
Wire Wire Line
	1800 1550 1800 1700
$Comp
L power:+3V3 #PWR057
U 1 1 5DD028F3
P 1750 1450
F 0 "#PWR057" H 1750 1300 50  0001 C CNN
F 1 "+3V3" H 1764 1623 50  0000 C CNN
F 2 "" H 1750 1450 50  0001 C CNN
F 3 "" H 1750 1450 50  0001 C CNN
	1    1750 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 1450 1750 1550
Connection ~ 1750 1550
Wire Wire Line
	1750 1550 1800 1550
Text Label 2650 4500 0    50   ~ 0
SIDEBAND_NXT
Text Label 2700 5350 0    50   ~ 0
SIDEBAND_STP
Text Label 2650 4600 0    50   ~ 0
SIDEBAND_DIR
$Comp
L power:+3V3 #PWR058
U 1 1 5DD09124
P 5950 3650
F 0 "#PWR058" H 5950 3500 50  0001 C CNN
F 1 "+3V3" V 5965 3778 50  0000 L CNN
F 2 "" H 5950 3650 50  0001 C CNN
F 3 "" H 5950 3650 50  0001 C CNN
	1    5950 3650
	0    -1   -1   0   
$EndComp
Text Label 5150 2450 0    50   ~ 0
SIDEBAND_DATA0
Text Label 5150 2550 0    50   ~ 0
SIDEBAND_DATA1
Text Label 5150 2650 0    50   ~ 0
SIDEBAND_DATA2
Text Label 5150 2750 0    50   ~ 0
SIDEBAND_DATA3
NoConn ~ 2500 2900
NoConn ~ 2500 3000
NoConn ~ 2500 3100
NoConn ~ 2500 3200
Wire Wire Line
	7400 3700 8300 3700
Text Label 8300 3700 2    50   ~ 0
SIDEBAND_PHY_CLK
Text Label 5150 2850 0    50   ~ 0
SIDEBAND_DATA4
Text Label 5150 2950 0    50   ~ 0
SIDEBAND_DATA5
Text Label 5150 3050 0    50   ~ 0
SIDEBAND_DATA6
Text Label 5150 3150 0    50   ~ 0
SIDEBAND_DATA7
Text Label 3850 5650 0    50   ~ 0
~SIDEBAND_PHY_RESET
Wire Wire Line
	6100 4400 5850 4400
Wire Wire Line
	5850 4400 5850 3900
Wire Wire Line
	5850 3900 6000 3900
Text Label 6100 4400 0    50   ~ 0
~SIDEBAND_PHY_RESET
Text HLabel 8700 3950 2    50   Output ~ 0
SIDEBAND_PHY_1V8
$Comp
L Device:C C36
U 1 1 5DD2517F
P 7800 4200
F 0 "C36" H 7915 4245 50  0000 L CNN
F 1 "1uF" H 7915 4155 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7838 4050 50  0001 C CNN
F 3 "~" H 7800 4200 50  0001 C CNN
F 4 "GENERIC-CAP-0603-1uF" H 7800 4200 50  0001 C CNN "Part Number"
	1    7800 4200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR062
U 1 1 5DD25634
P 7800 4500
F 0 "#PWR062" H 7800 4250 50  0001 C CNN
F 1 "GND" H 7804 4328 50  0000 C CNN
F 2 "" H 7800 4500 50  0001 C CNN
F 3 "" H 7800 4500 50  0001 C CNN
	1    7800 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	7800 4500 7800 4350
$Comp
L Device:C C37
U 1 1 5DD280CB
P 8250 4200
F 0 "C37" H 8365 4245 50  0000 L CNN
F 1 "1uF" H 8365 4155 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 8288 4050 50  0001 C CNN
F 3 "~" H 8250 4200 50  0001 C CNN
F 4 "GENERIC-CAP-0603-1uF" H 8250 4200 50  0001 C CNN "Part Number"
	1    8250 4200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR064
U 1 1 5DD280D5
P 8250 4500
F 0 "#PWR064" H 8250 4250 50  0001 C CNN
F 1 "GND" H 8254 4328 50  0000 C CNN
F 2 "" H 8250 4500 50  0001 C CNN
F 3 "" H 8250 4500 50  0001 C CNN
	1    8250 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	8250 4500 8250 4350
Wire Wire Line
	7400 3950 8250 3950
Wire Wire Line
	8250 4050 8250 3950
Connection ~ 8250 3950
Text HLabel 8100 2850 2    50   Input ~ 0
SIDEBAND_VBUS
Wire Wire Line
	7400 2950 8100 2950
Text HLabel 8100 2950 2    50   BiDi ~ 0
SIDEBAND_D-
Text HLabel 8100 3050 2    50   BiDi ~ 0
SIDEBAND_D+
Wire Wire Line
	7400 3050 8100 3050
Wire Wire Line
	7400 2650 7550 2650
$Comp
L power:GND #PWR059
U 1 1 5DD345C7
P 7550 2650
F 0 "#PWR059" H 7550 2400 50  0001 C CNN
F 1 "GND" V 7555 2522 50  0000 R CNN
F 2 "" H 7550 2650 50  0001 C CNN
F 3 "" H 7550 2650 50  0001 C CNN
	1    7550 2650
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7400 2450 7600 2450
Wire Wire Line
	7600 2450 7600 2200
$Comp
L power:+5V #PWR060
U 1 1 5DD35DC7
P 7600 2200
F 0 "#PWR060" H 7600 2050 50  0001 C CNN
F 1 "+5V" H 7614 2373 50  0000 C CNN
F 2 "" H 7600 2200 50  0001 C CNN
F 3 "" H 7600 2200 50  0001 C CNN
	1    7600 2200
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR061
U 1 1 5DD36515
P 7800 2200
F 0 "#PWR061" H 7800 2050 50  0001 C CNN
F 1 "+3V3" H 7814 2373 50  0000 C CNN
F 2 "" H 7800 2200 50  0001 C CNN
F 3 "" H 7800 2200 50  0001 C CNN
	1    7800 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	7800 2200 7800 2550
Wire Wire Line
	7800 2550 7400 2550
$Comp
L Device:R R16
U 1 1 5DD37F86
P 7700 3400
F 0 "R16" V 7650 3200 50  0000 C CNN
F 1 "8.06k+1%" V 7600 3550 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 7630 3400 50  0001 C CNN
F 3 "~" H 7700 3400 50  0001 C CNN
F 4 "RES SMD 8.06K OHM 1% 1/10W 0402" H 7700 3400 50  0001 C CNN "Description"
F 5 "Panasonic" H 7700 3400 50  0001 C CNN "Manufacturer"
F 6 "ERJ-2RKF8061X" H 7700 3400 50  0001 C CNN "Part Number"
	1    7700 3400
	0    1    1    0   
$EndComp
Wire Wire Line
	7550 3400 7400 3400
$Comp
L power:GND #PWR063
U 1 1 5DD39A7C
P 8050 3400
F 0 "#PWR063" H 8050 3150 50  0001 C CNN
F 1 "GND" V 8055 3272 50  0000 R CNN
F 2 "" H 8050 3400 50  0001 C CNN
F 3 "" H 8050 3400 50  0001 C CNN
	1    8050 3400
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8050 3400 7850 3400
$Comp
L Device:R R17
U 1 1 5DD3B600
P 7750 2850
F 0 "R17" V 7700 2650 50  0000 C CNN
F 1 "20K" V 7750 2850 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 7680 2850 50  0001 C CNN
F 3 "~" H 7750 2850 50  0001 C CNN
F 4 "RES SMD 20K OHM 5% 1/16W 0402" H 7750 2850 50  0001 C CNN "Description"
F 5 "Yageo" H 7750 2850 50  0001 C CNN "Manufacturer"
F 6 "RC0402JR-0720KL" H 7750 2850 50  0001 C CNN "Part Number"
	1    7750 2850
	0    1    1    0   
$EndComp
Wire Wire Line
	7600 2850 7400 2850
Wire Wire Line
	7900 2850 8100 2850
$Comp
L Device:C C38
U 1 1 5DD3F3AE
P 9250 1550
F 0 "C38" H 9365 1595 50  0000 L CNN
F 1 "0.1uF" H 9365 1505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 9288 1400 50  0001 C CNN
F 3 "~" H 9250 1550 50  0001 C CNN
F 4 "GENERIC-CAP-0402-0.1uF" H 9250 1550 50  0001 C CNN "Part Number"
	1    9250 1550
	1    0    0    -1  
$EndComp
$Comp
L Device:C C39
U 1 1 5DD3FB40
P 9750 1550
F 0 "C39" H 9865 1595 50  0000 L CNN
F 1 "0.1uF" H 9865 1505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 9788 1400 50  0001 C CNN
F 3 "~" H 9750 1550 50  0001 C CNN
F 4 "GENERIC-CAP-0402-0.1uF" H 9750 1550 50  0001 C CNN "Part Number"
	1    9750 1550
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR065
U 1 1 5DD40529
P 9250 1300
F 0 "#PWR065" H 9250 1150 50  0001 C CNN
F 1 "+5V" H 9264 1473 50  0000 C CNN
F 2 "" H 9250 1300 50  0001 C CNN
F 3 "" H 9250 1300 50  0001 C CNN
	1    9250 1300
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR067
U 1 1 5DD40800
P 9750 1300
F 0 "#PWR067" H 9750 1150 50  0001 C CNN
F 1 "+3V3" H 9764 1473 50  0000 C CNN
F 2 "" H 9750 1300 50  0001 C CNN
F 3 "" H 9750 1300 50  0001 C CNN
	1    9750 1300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR066
U 1 1 5DD40C56
P 9250 1800
F 0 "#PWR066" H 9250 1550 50  0001 C CNN
F 1 "GND" H 9254 1628 50  0000 C CNN
F 2 "" H 9250 1800 50  0001 C CNN
F 3 "" H 9250 1800 50  0001 C CNN
	1    9250 1800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR068
U 1 1 5DD410FE
P 9750 1800
F 0 "#PWR068" H 9750 1550 50  0001 C CNN
F 1 "GND" H 9754 1628 50  0000 C CNN
F 2 "" H 9750 1800 50  0001 C CNN
F 3 "" H 9750 1800 50  0001 C CNN
	1    9750 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	9750 1300 9750 1400
Wire Wire Line
	9250 1300 9250 1400
Wire Wire Line
	9250 1700 9250 1800
Wire Wire Line
	9750 1700 9750 1800
NoConn ~ 2500 3650
NoConn ~ 2500 4000
NoConn ~ 2500 4100
NoConn ~ 2500 4300
NoConn ~ 2500 5000
NoConn ~ 2500 5100
NoConn ~ 2500 5700
NoConn ~ 2500 5800
NoConn ~ 2500 5900
NoConn ~ 2500 6000
Wire Wire Line
	5850 4400 5850 4850
Wire Wire Line
	5850 5300 6200 5300
Connection ~ 5850 4400
$Comp
L Device:R R15
U 1 1 5E110842
P 2950 5650
F 0 "R15" V 2850 5650 50  0000 C CNN
F 1 "1K" V 2950 5650 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 2880 5650 50  0001 C CNN
F 3 "~" H 2950 5650 50  0001 C CNN
F 4 "GENERIC-RES-0402-1K" H 2950 5650 50  0001 C CNN "Part Number"
	1    2950 5650
	0    1    1    0   
$EndComp
Wire Wire Line
	2500 5350 4750 5350
Wire Wire Line
	3100 5650 3850 5650
Wire Wire Line
	6100 3650 5950 3650
Text HLabel 6200 5300 2    50   Input ~ 0
UC_USB_INHIBIT
$Comp
L Device:R R?
U 1 1 5E1591E2
P 5550 5100
AR Path="/5DD754D4/5E1591E2" Ref="R?"  Part="1" 
AR Path="/5DCD9772/5E1591E2" Ref="R22"  Part="1" 
F 0 "R22" V 5650 5100 50  0000 C CNN
F 1 "15K" V 5550 5100 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 5480 5100 50  0001 C CNN
F 3 "~" H 5550 5100 50  0001 C CNN
F 4 "GENERIC-RES-0402-15K" H 5550 5100 50  0001 C CNN "Part Number"
	1    5550 5100
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 4850 5850 4850
Connection ~ 5850 4850
Wire Wire Line
	5850 4850 5850 5300
NoConn ~ 7400 3600
Wire Wire Line
	2500 4400 4750 4400
Entry Wire Line
	4750 4400 4850 4300
Entry Wire Line
	4850 3450 4950 3350
Wire Wire Line
	4950 3350 6100 3350
Wire Wire Line
	4950 3450 6100 3450
Entry Wire Line
	4950 3450 4850 3550
Wire Wire Line
	4950 3550 6100 3550
Entry Wire Line
	4950 3550 4850 3650
Text Label 5150 3350 0    50   ~ 0
SIDEBAND_STP
Text Label 5150 3450 0    50   ~ 0
SIDEBAND_NXT
Text Label 5150 3550 0    50   ~ 0
SIDEBAND_DIR
Wire Wire Line
	6100 2450 4950 2450
Entry Wire Line
	4950 2450 4850 2550
Wire Wire Line
	6100 2550 4950 2550
Wire Wire Line
	6100 2650 4950 2650
Wire Wire Line
	6100 2750 4950 2750
Wire Wire Line
	6100 2850 4950 2850
Wire Wire Line
	6100 2950 4950 2950
Wire Wire Line
	6100 3050 4950 3050
Wire Wire Line
	6100 3150 4950 3150
Entry Wire Line
	4850 2650 4950 2550
Entry Wire Line
	4850 2750 4950 2650
Entry Wire Line
	4850 2850 4950 2750
Entry Wire Line
	4850 2950 4950 2850
Entry Wire Line
	4850 3050 4950 2950
Entry Wire Line
	4850 3150 4950 3050
Entry Wire Line
	4850 3250 4950 3150
Wire Wire Line
	2500 3800 4750 3800
Entry Wire Line
	4750 3800 4850 3700
Wire Wire Line
	2500 3900 4750 3900
Entry Wire Line
	4750 3900 4850 3800
Wire Wire Line
	2500 3450 4750 3450
Wire Wire Line
	2500 3350 4750 3350
Entry Wire Line
	4750 3350 4850 3250
Entry Wire Line
	4750 3450 4850 3350
Wire Wire Line
	2500 2750 4750 2750
Wire Wire Line
	2500 2650 4750 2650
Wire Wire Line
	2500 2550 4750 2550
Entry Wire Line
	4750 2550 4850 2450
Entry Wire Line
	4750 2650 4850 2550
Entry Wire Line
	4750 2750 4850 2650
Entry Wire Line
	4750 4500 4850 4400
Wire Wire Line
	2500 4500 4750 4500
Text Label 2650 3900 0    50   ~ 0
SIDEBAND_DATA0
Text Label 2650 4400 0    50   ~ 0
SIDEBAND_DATA1
Text Label 3850 5250 0    50   ~ 0
SIDEBAND_PHY_CLK
Text Label 2650 3350 0    50   ~ 0
SIDEBAND_DATA4
Text Label 2650 2750 0    50   ~ 0
SIDEBAND_DATA5
Text Label 2650 2650 0    50   ~ 0
SIDEBAND_DATA6
Text Label 2650 2550 0    50   ~ 0
SIDEBAND_DATA7
Wire Wire Line
	8250 3950 8700 3950
Wire Wire Line
	7400 4050 7800 4050
$Comp
L power:GND #PWR0101
U 1 1 5ECCE017
P 5550 5350
F 0 "#PWR0101" H 5550 5100 50  0001 C CNN
F 1 "GND" V 5555 5222 50  0000 R CNN
F 2 "" H 5550 5350 50  0001 C CNN
F 3 "" H 5550 5350 50  0001 C CNN
	1    5550 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 5250 5550 5350
Wire Wire Line
	5550 4850 5550 4950
Text HLabel 2600 4800 2    50   Output ~ 0
~FPGA_SELF_PROGRAM
Wire Wire Line
	2600 4800 2500 4800
$Comp
L power:+3V3 #PWR0126
U 1 1 6083CB46
P 7500 3150
AR Path="/5DCD9772/6083CB46" Ref="#PWR0126"  Part="1" 
AR Path="/5DD754D4/6083CB46" Ref="#PWR?"  Part="1" 
AR Path="/5DDDB747/6083CB46" Ref="#PWR?"  Part="1" 
F 0 "#PWR0126" H 7500 3000 50  0001 C CNN
F 1 "+3V3" V 7500 3350 50  0000 C CNN
F 2 "" H 7500 3150 50  0001 C CNN
F 3 "" H 7500 3150 50  0001 C CNN
	1    7500 3150
	0    1    1    0   
$EndComp
Wire Wire Line
	7500 3150 7400 3150
$Comp
L Device:R R?
U 1 1 60996F81
P 3550 5900
AR Path="/60996F81" Ref="R?"  Part="1" 
AR Path="/5DD754D4/60996F81" Ref="R?"  Part="1" 
AR Path="/5DCD9772/60996F81" Ref="R34"  Part="1" 
F 0 "R34" V 3650 5900 50  0000 C CNN
F 1 "5.1K" V 3550 5900 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 3480 5900 50  0001 C CNN
F 3 "~" H 3550 5900 50  0001 C CNN
F 4 "GENERIC-RES-0402-5.1K" H 3550 5900 50  0001 C CNN "Part Number"
	1    3550 5900
	1    0    0    1   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60996F87
P 3550 6250
AR Path="/60996F87" Ref="#PWR?"  Part="1" 
AR Path="/5DD754D4/60996F87" Ref="#PWR?"  Part="1" 
AR Path="/5DCD9772/60996F87" Ref="#PWR010"  Part="1" 
F 0 "#PWR010" H 3550 6000 50  0001 C CNN
F 1 "GND" H 3554 6078 50  0000 C CNN
F 2 "" H 3550 6250 50  0001 C CNN
F 3 "" H 3550 6250 50  0001 C CNN
	1    3550 6250
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 60996F8E
P 3750 5900
AR Path="/60996F8E" Ref="R?"  Part="1" 
AR Path="/5DD754D4/60996F8E" Ref="R?"  Part="1" 
AR Path="/5DCD9772/60996F8E" Ref="R35"  Part="1" 
F 0 "R35" V 3850 5900 50  0000 C CNN
F 1 "5.1K" V 3750 5900 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 3680 5900 50  0001 C CNN
F 3 "~" H 3750 5900 50  0001 C CNN
F 4 "GENERIC-RES-0402-5.1K" H 3750 5900 50  0001 C CNN "Part Number"
	1    3750 5900
	1    0    0    1   
$EndComp
Wire Wire Line
	3550 6050 3550 6150
Wire Wire Line
	3550 6150 3750 6150
Wire Wire Line
	3750 6150 3750 6050
Connection ~ 3550 6150
Wire Wire Line
	3550 6150 3550 6250
Text HLabel 3850 3550 2    50   BiDi ~ 0
SIDEBAND_CC2
Text Label 2650 3450 0    50   ~ 0
SIDEBAND_DATA3
Text Label 2650 3800 0    50   ~ 0
SIDEBAND_DATA2
Entry Wire Line
	4850 4500 4750 4600
Wire Wire Line
	2500 4600 4750 4600
Entry Wire Line
	4750 5350 4850 5250
Wire Wire Line
	2800 5650 2700 5650
Wire Wire Line
	2700 5650 2700 5550
Wire Wire Line
	2700 5550 2500 5550
Wire Wire Line
	3850 5250 2500 5250
Text HLabel 3850 5450 2    50   BiDi ~ 0
SIDEBAND_SBU2
Connection ~ 2500 5250
Wire Wire Line
	2500 5250 2350 5250
Text HLabel 3850 4900 2    50   BiDi ~ 0
SIDEBAND_SBU1
Wire Wire Line
	3850 4900 2500 4900
Wire Wire Line
	3850 5450 2500 5450
Wire Wire Line
	2500 3550 3750 3550
Text HLabel 3850 2450 2    50   BiDi ~ 0
SIDEBAND_CC1
Wire Wire Line
	2500 2450 3550 2450
Wire Wire Line
	3550 5750 3550 2450
Connection ~ 3550 2450
Wire Wire Line
	3550 2450 3850 2450
Wire Wire Line
	3750 5750 3750 3550
Connection ~ 3750 3550
Wire Wire Line
	3750 3550 3850 3550
Wire Bus Line
	4850 2350 4850 5250
$EndSCHEMATC
