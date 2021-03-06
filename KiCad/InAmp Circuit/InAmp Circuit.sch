EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Device:C_Small C1
U 1 1 60DDFA8E
P 2000 1350
F 0 "C1" H 2092 1396 50  0000 L CNN
F 1 "4.7uF" H 2092 1305 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 2000 1350 50  0001 C CNN
F 3 "~" H 2000 1350 50  0001 C CNN
	1    2000 1350
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small_US R1
U 1 1 60DE14E0
P 2000 1750
F 0 "R1" H 2068 1796 50  0000 L CNN
F 1 "1k Ohm" H 2068 1705 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 2000 1750 50  0001 C CNN
F 3 "~" H 2000 1750 50  0001 C CNN
	1    2000 1750
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x12 J1
U 1 1 6207018B
P 900 1550
F 0 "J1" H 818 725 50  0000 C CNN
F 1 "Conn_01x12" H 818 816 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x12_P2.54mm_Vertical" H 900 1550 50  0001 C CNN
F 3 "~" H 900 1550 50  0001 C CNN
	1    900  1550
	-1   0    0    1   
$EndComp
Wire Wire Line
	1100 1550 1200 1550
Wire Wire Line
	2000 1450 2000 1650
$Comp
L INA122U:INA122U INA1
U 1 1 620745AC
P 5350 1550
F 0 "INA1" H 5350 2220 50  0000 C CNN
F 1 "INA122U" H 5350 2129 50  0000 C CNN
F 2 "KICAD Third Party:SOIC127P599X175-8N" H 5350 1550 50  0001 L BNN
F 3 "" H 5350 1550 50  0001 L BNN
	1    5350 1550
	1    0    0    -1  
$EndComp
Connection ~ 2000 1450
Wire Wire Line
	4650 1450 4250 1450
$Comp
L Device:R_Small_US Ref1
U 1 1 62081531
P 4150 1450
F 0 "Ref1" H 4082 1404 50  0000 R CNN
F 1 "200 Ohm" H 4082 1495 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 4150 1450 50  0001 C CNN
F 3 "~" H 4150 1450 50  0001 C CNN
	1    4150 1450
	-1   0    0    1   
$EndComp
Wire Wire Line
	4250 1450 4250 1350
Wire Wire Line
	4250 1350 4150 1350
Wire Wire Line
	4250 1450 4250 1550
Wire Wire Line
	4250 1550 4150 1550
Connection ~ 4250 1450
Wire Wire Line
	4650 1550 4400 1550
Wire Wire Line
	4400 1550 4400 1850
Wire Wire Line
	4650 1650 4500 1650
Wire Wire Line
	4650 1350 4500 1350
Text GLabel 2100 1250 2    50   Input ~ 0
1
Text GLabel 2100 1450 2    50   Input ~ 0
2
Text GLabel 6150 1150 0    50   Input ~ 0
V
Text GLabel 6150 1350 0    50   Input ~ 0
O1
Text GLabel 6150 1950 0    50   Input ~ 0
GND
$Comp
L Device:C_Small C2
U 1 1 62086F7B
P 2000 2600
F 0 "C2" H 2092 2646 50  0000 L CNN
F 1 "4.7uF" H 2092 2555 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 2000 2600 50  0001 C CNN
F 3 "~" H 2000 2600 50  0001 C CNN
	1    2000 2600
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small_US R2
U 1 1 62086F81
P 2000 3000
F 0 "R2" H 2068 3046 50  0000 L CNN
F 1 "1k Ohm" H 2068 2955 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 2000 3000 50  0001 C CNN
F 3 "~" H 2000 3000 50  0001 C CNN
	1    2000 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 2700 2000 2900
Wire Wire Line
	2000 2500 2000 2350
Wire Wire Line
	2000 3100 2000 3150
$Comp
L INA122U:INA122U INA2
U 1 1 62086F8A
P 5350 2800
F 0 "INA2" H 5350 3470 50  0000 C CNN
F 1 "INA122U" H 5350 3379 50  0000 C CNN
F 2 "KICAD Third Party:SOIC127P599X175-8N" H 5350 2800 50  0001 L BNN
F 3 "" H 5350 2800 50  0001 L BNN
	1    5350 2800
	1    0    0    -1  
$EndComp
Connection ~ 2000 2500
Connection ~ 2000 2700
Wire Wire Line
	4650 2700 4250 2700
$Comp
L Device:R_Small_US Ref2
U 1 1 62086F99
P 4150 2700
F 0 "Ref2" H 4082 2654 50  0000 R CNN
F 1 "200 Ohm" H 4082 2745 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 4150 2700 50  0001 C CNN
F 3 "~" H 4150 2700 50  0001 C CNN
	1    4150 2700
	-1   0    0    1   
$EndComp
Wire Wire Line
	4250 2700 4250 2600
Wire Wire Line
	4250 2600 4150 2600
Wire Wire Line
	4250 2700 4250 2800
Wire Wire Line
	4250 2800 4150 2800
Connection ~ 4250 2700
Wire Wire Line
	4650 2800 4400 2800
Wire Wire Line
	4400 2800 4400 3100
Wire Wire Line
	4400 3100 4500 3100
Wire Wire Line
	4650 2900 4500 2900
Wire Wire Line
	4500 2900 4500 3000
Wire Wire Line
	4650 2600 4500 2600
Wire Wire Line
	4500 2600 4500 2450
Text GLabel 2100 2500 2    50   Input ~ 0
3
Text GLabel 2100 2700 2    50   Input ~ 0
4
Text GLabel 6150 2400 0    50   Input ~ 0
V
Text GLabel 6150 2600 0    50   Input ~ 0
O2
Text GLabel 6150 3200 0    50   Input ~ 0
GND
$Comp
L Device:C_Small C3
U 1 1 62089DE4
P 2000 3850
F 0 "C3" H 2092 3896 50  0000 L CNN
F 1 "4.7uF" H 2092 3805 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 2000 3850 50  0001 C CNN
F 3 "~" H 2000 3850 50  0001 C CNN
	1    2000 3850
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small_US R3
U 1 1 62089DEA
P 2000 4250
F 0 "R3" H 2068 4296 50  0000 L CNN
F 1 "1k Ohm" H 2068 4205 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 2000 4250 50  0001 C CNN
F 3 "~" H 2000 4250 50  0001 C CNN
	1    2000 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 3950 2000 4150
Wire Wire Line
	2000 3750 2000 3600
Wire Wire Line
	2000 4350 2000 4400
$Comp
L INA122U:INA122U INA3
U 1 1 62089DF3
P 5350 4050
F 0 "INA3" H 5350 4720 50  0000 C CNN
F 1 "INA122U" H 5350 4629 50  0000 C CNN
F 2 "KICAD Third Party:SOIC127P599X175-8N" H 5350 4050 50  0001 L BNN
F 3 "" H 5350 4050 50  0001 L BNN
	1    5350 4050
	1    0    0    -1  
$EndComp
Connection ~ 2000 3750
Connection ~ 2000 3950
Wire Wire Line
	4650 3950 4250 3950
$Comp
L Device:R_Small_US Ref3
U 1 1 62089E02
P 4150 3950
F 0 "Ref3" H 4082 3904 50  0000 R CNN
F 1 "200 Ohm" H 4082 3995 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 4150 3950 50  0001 C CNN
F 3 "~" H 4150 3950 50  0001 C CNN
	1    4150 3950
	-1   0    0    1   
$EndComp
Wire Wire Line
	4250 3950 4250 3850
Wire Wire Line
	4250 3850 4150 3850
Wire Wire Line
	4250 3950 4250 4050
Wire Wire Line
	4250 4050 4150 4050
Connection ~ 4250 3950
Wire Wire Line
	4650 4050 4400 4050
Wire Wire Line
	4400 4050 4400 4350
Wire Wire Line
	4400 4350 4500 4350
Wire Wire Line
	4650 4150 4500 4150
Wire Wire Line
	4500 4150 4500 4250
Wire Wire Line
	4650 3850 4500 3850
Wire Wire Line
	4500 3850 4500 3700
Text GLabel 2100 3750 2    50   Input ~ 0
5
Text GLabel 2100 3950 2    50   Input ~ 0
6
Text GLabel 6150 3650 0    50   Input ~ 0
V
Text GLabel 6150 3850 0    50   Input ~ 0
O3
Text GLabel 6150 4450 0    50   Input ~ 0
GND
$Comp
L Device:C_Small C4
U 1 1 6209000F
P 2000 5100
F 0 "C4" H 2092 5146 50  0000 L CNN
F 1 "4.7uF" H 2092 5055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 2000 5100 50  0001 C CNN
F 3 "~" H 2000 5100 50  0001 C CNN
	1    2000 5100
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small_US R4
U 1 1 62090015
P 2000 5500
F 0 "R4" H 2068 5546 50  0000 L CNN
F 1 "1k Ohm" H 2068 5455 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 2000 5500 50  0001 C CNN
F 3 "~" H 2000 5500 50  0001 C CNN
	1    2000 5500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 5200 2000 5400
Wire Wire Line
	2000 5000 2000 4850
Wire Wire Line
	2000 5600 2000 5650
$Comp
L INA122U:INA122U INA4
U 1 1 6209001E
P 5350 5300
F 0 "INA4" H 5350 5970 50  0000 C CNN
F 1 "INA122U" H 5350 5879 50  0000 C CNN
F 2 "KICAD Third Party:SOIC127P599X175-8N" H 5350 5300 50  0001 L BNN
F 3 "" H 5350 5300 50  0001 L BNN
	1    5350 5300
	1    0    0    -1  
$EndComp
Connection ~ 2000 5000
Connection ~ 2000 5200
Wire Wire Line
	4650 5200 4250 5200
$Comp
L Device:R_Small_US Ref4
U 1 1 6209002D
P 4150 5200
F 0 "Ref4" H 4082 5154 50  0000 R CNN
F 1 "200 Ohm" H 4082 5245 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 4150 5200 50  0001 C CNN
F 3 "~" H 4150 5200 50  0001 C CNN
	1    4150 5200
	-1   0    0    1   
$EndComp
Wire Wire Line
	4250 5200 4250 5100
Wire Wire Line
	4250 5100 4150 5100
Wire Wire Line
	4250 5200 4250 5300
Wire Wire Line
	4250 5300 4150 5300
Connection ~ 4250 5200
Wire Wire Line
	4650 5300 4400 5300
Wire Wire Line
	4400 5300 4400 5600
Wire Wire Line
	4400 5600 4500 5600
Wire Wire Line
	4650 5400 4500 5400
Wire Wire Line
	4500 5400 4500 5500
Wire Wire Line
	4650 5100 4500 5100
Wire Wire Line
	4500 5100 4500 4950
Text GLabel 2100 5000 2    50   Input ~ 0
7
Text GLabel 2100 5200 2    50   Input ~ 0
8
Text GLabel 6150 4900 0    50   Input ~ 0
V
Text GLabel 6150 5100 0    50   Input ~ 0
O4
Text GLabel 6150 5700 0    50   Input ~ 0
GND
$Comp
L Device:C_Small C5
U 1 1 620A5A8A
P 2000 6350
F 0 "C5" H 2092 6396 50  0000 L CNN
F 1 "4.7uF" H 2092 6305 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 2000 6350 50  0001 C CNN
F 3 "~" H 2000 6350 50  0001 C CNN
	1    2000 6350
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small_US R5
U 1 1 620A5A90
P 2000 6750
F 0 "R5" H 2068 6796 50  0000 L CNN
F 1 "1k Ohm" H 2068 6705 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 2000 6750 50  0001 C CNN
F 3 "~" H 2000 6750 50  0001 C CNN
	1    2000 6750
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 6450 2000 6650
Wire Wire Line
	2000 6250 2000 6100
Wire Wire Line
	2000 6850 2000 6900
Connection ~ 2000 6250
Connection ~ 2000 6450
Wire Wire Line
	4650 6450 4250 6450
$Comp
L Device:R_Small_US Ref5
U 1 1 620A5AA8
P 4150 6450
F 0 "Ref5" H 4082 6404 50  0000 R CNN
F 1 "200 Ohm" H 4082 6495 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 4150 6450 50  0001 C CNN
F 3 "~" H 4150 6450 50  0001 C CNN
	1    4150 6450
	-1   0    0    1   
$EndComp
Wire Wire Line
	4250 6450 4250 6350
Wire Wire Line
	4250 6350 4150 6350
Wire Wire Line
	4250 6450 4250 6550
Wire Wire Line
	4250 6550 4150 6550
Connection ~ 4250 6450
Wire Wire Line
	4650 6550 4400 6550
Wire Wire Line
	4400 6550 4400 6850
Wire Wire Line
	4400 6850 4500 6850
Wire Wire Line
	4650 6650 4500 6650
Wire Wire Line
	4500 6650 4500 6750
Wire Wire Line
	4650 6350 4500 6350
Wire Wire Line
	4500 6350 4500 6200
Text GLabel 2100 6250 2    50   Input ~ 0
9
Text GLabel 2100 6450 2    50   Input ~ 0
10
Text GLabel 6150 6150 0    50   Input ~ 0
V
Text GLabel 6150 6350 0    50   Input ~ 0
O5
Text GLabel 6150 6950 0    50   Input ~ 0
GND
$Comp
L INA122U:INA122U INA5
U 1 1 620A5A99
P 5350 6550
F 0 "INA5" H 5350 7220 50  0000 C CNN
F 1 "INA122U" H 5350 7129 50  0000 C CNN
F 2 "KICAD Third Party:SOIC127P599X175-8N" H 5350 6550 50  0001 L BNN
F 3 "" H 5350 6550 50  0001 L BNN
	1    5350 6550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4500 1350 4500 1200
Wire Wire Line
	6050 6950 6150 6950
Wire Wire Line
	6050 5700 6150 5700
Wire Wire Line
	6050 4450 6150 4450
Wire Wire Line
	6050 3200 6150 3200
Wire Wire Line
	6050 1950 6150 1950
Wire Wire Line
	6050 6350 6150 6350
Wire Wire Line
	6050 5100 6150 5100
Wire Wire Line
	6050 3850 6150 3850
Wire Wire Line
	6050 2600 6150 2600
Wire Wire Line
	6050 1350 6150 1350
Wire Wire Line
	2000 6450 2100 6450
Wire Wire Line
	2000 6250 2100 6250
Wire Wire Line
	1100 1950 1200 1950
Wire Wire Line
	1100 1850 1200 1850
Wire Wire Line
	2000 5200 2100 5200
Text GLabel 1200 2400 2    50   Input ~ 0
O5
Text GLabel 1200 2500 2    50   Input ~ 0
O4
Text GLabel 1200 2600 2    50   Input ~ 0
O3
Text GLabel 1200 2700 2    50   Input ~ 0
O2
Text GLabel 1200 2800 2    50   Input ~ 0
O1
Wire Wire Line
	1100 2800 1200 2800
Wire Wire Line
	1100 2700 1200 2700
Wire Wire Line
	1100 2600 1200 2600
Wire Wire Line
	1100 2500 1200 2500
Wire Wire Line
	1100 2400 1200 2400
$Comp
L Connector_Generic:Conn_01x05 J2
U 1 1 620763AA
P 900 2600
F 0 "J2" H 818 2175 50  0000 C CNN
F 1 "Conn_01x05" H 818 2266 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical" H 900 2600 50  0001 C CNN
F 3 "~" H 900 2600 50  0001 C CNN
	1    900  2600
	-1   0    0    1   
$EndComp
Wire Wire Line
	1100 1750 1200 1750
Wire Wire Line
	2000 5000 2100 5000
Wire Wire Line
	2000 3950 2100 3950
Wire Wire Line
	1100 1650 1200 1650
Wire Wire Line
	2000 3750 2100 3750
Wire Wire Line
	2000 2700 2100 2700
Wire Wire Line
	1100 1450 1200 1450
Wire Wire Line
	2000 2500 2100 2500
Wire Wire Line
	1100 1350 1200 1350
Wire Wire Line
	2000 1450 2100 1450
Wire Wire Line
	1100 1250 1200 1250
Wire Wire Line
	2000 1250 2100 1250
Wire Wire Line
	1100 1150 1200 1150
Wire Wire Line
	4400 1850 4500 1850
Wire Wire Line
	1100 1050 1200 1050
Wire Wire Line
	2000 1850 2000 1900
Text GLabel 2100 1900 2    50   Input ~ 0
GND
Wire Wire Line
	2000 1900 2100 1900
Text GLabel 2100 3150 2    50   Input ~ 0
GND
Wire Wire Line
	2000 3150 2100 3150
Text GLabel 2100 4400 2    50   Input ~ 0
GND
Wire Wire Line
	2000 4400 2100 4400
Text GLabel 2100 5650 2    50   Input ~ 0
GND
Wire Wire Line
	2000 5650 2100 5650
Text GLabel 2100 6900 2    50   Input ~ 0
GND
Wire Wire Line
	2000 6900 2100 6900
Wire Wire Line
	1100 2050 1200 2050
Connection ~ 2000 1250
Wire Wire Line
	2000 1250 2000 1100
Text GLabel 2100 1100 2    50   Input ~ 0
V
Wire Wire Line
	2000 1100 2100 1100
Text GLabel 2100 2350 2    50   Input ~ 0
V
Wire Wire Line
	2000 2350 2100 2350
Text GLabel 2100 3600 2    50   Input ~ 0
V
Wire Wire Line
	2000 3600 2100 3600
Text GLabel 2100 4850 2    50   Input ~ 0
V
Wire Wire Line
	2000 4850 2100 4850
Text GLabel 2100 6100 2    50   Input ~ 0
V
Wire Wire Line
	2000 6100 2100 6100
Wire Wire Line
	6050 2400 6150 2400
Wire Wire Line
	6050 3650 6150 3650
Wire Wire Line
	6050 4900 6150 4900
Wire Wire Line
	6050 6150 6150 6150
Wire Wire Line
	6050 1150 6150 1150
Entry Wire Line
	2100 1100 2200 1200
Entry Wire Line
	2100 1250 2200 1350
Entry Wire Line
	2100 1450 2200 1550
Entry Wire Line
	2100 1900 2200 2000
Entry Wire Line
	2100 2350 2200 2450
Entry Wire Line
	2100 2500 2200 2600
Entry Wire Line
	2100 2700 2200 2800
Entry Wire Line
	2100 3150 2200 3250
Entry Wire Line
	2100 3600 2200 3700
Entry Wire Line
	2100 3750 2200 3850
Entry Wire Line
	2100 3950 2200 4050
Entry Wire Line
	2100 4400 2200 4500
Entry Wire Line
	2100 4850 2200 4950
Entry Wire Line
	2100 5000 2200 5100
Entry Wire Line
	2100 5200 2200 5300
Entry Wire Line
	2100 5650 2200 5750
Entry Wire Line
	2100 6100 2200 6200
Entry Wire Line
	2100 6250 2200 6350
Entry Wire Line
	2100 6450 2200 6550
Entry Wire Line
	2100 6900 2200 7000
Text GLabel 4400 5500 0    50   Input ~ 0
7
Text GLabel 4500 5400 0    50   Input ~ 0
8
Entry Wire Line
	4500 5600 4600 5700
Entry Wire Line
	4500 5500 4600 5600
Text GLabel 4500 1300 0    50   Input ~ 0
GND
Text GLabel 4400 3000 0    50   Input ~ 0
3
Text GLabel 4500 2900 0    50   Input ~ 0
4
Entry Wire Line
	4500 3100 4600 3200
Entry Wire Line
	4500 3000 4600 3100
Text GLabel 4500 2550 0    50   Input ~ 0
GND
Entry Wire Line
	4500 2450 4600 2550
Text GLabel 4500 3800 0    50   Input ~ 0
GND
Entry Wire Line
	4500 3700 4600 3800
Text GLabel 4400 4250 0    50   Input ~ 0
5
Text GLabel 4500 4150 0    50   Input ~ 0
6
Entry Wire Line
	4500 4350 4600 4450
Entry Wire Line
	4500 4250 4600 4350
Text GLabel 4500 5050 0    50   Input ~ 0
GND
Entry Wire Line
	4500 4950 4600 5050
Text GLabel 4500 6300 0    50   Input ~ 0
GND
Entry Wire Line
	4500 6200 4600 6300
Text GLabel 4400 6800 0    50   Input ~ 0
9
Text GLabel 4500 6650 0    50   Input ~ 0
10
Entry Wire Line
	4500 6850 4600 6950
Entry Wire Line
	4500 6750 4600 6850
Entry Wire Line
	6150 6950 6250 7050
Entry Wire Line
	6150 6350 6250 6450
Entry Wire Line
	6150 6150 6250 6250
Entry Wire Line
	6150 5700 6250 5800
Entry Wire Line
	6150 5100 6250 5200
Entry Wire Line
	6150 4900 6250 5000
Entry Wire Line
	6150 4450 6250 4550
Entry Wire Line
	6150 3850 6250 3950
Entry Wire Line
	6150 3650 6250 3750
Entry Wire Line
	6150 3200 6250 3300
Entry Wire Line
	6150 2600 6250 2700
Entry Wire Line
	6150 2400 6250 2500
Entry Wire Line
	6150 1950 6250 2050
Entry Wire Line
	6150 1350 6250 1450
Entry Wire Line
	6150 1150 6250 1250
Text GLabel 4400 1850 0    50   Input ~ 0
1
Wire Wire Line
	4500 1650 4500 1750
Entry Wire Line
	4500 1200 4600 1300
Wire Bus Line
	4600 800  6250 800 
Entry Wire Line
	1200 2800 1300 2900
Entry Wire Line
	1200 2700 1300 2800
Entry Wire Line
	1200 2600 1300 2700
Entry Wire Line
	1200 2500 1300 2600
Entry Wire Line
	1200 2400 1300 2500
Entry Wire Line
	1200 1050 1300 1150
Entry Wire Line
	1200 1150 1300 1250
Entry Wire Line
	1200 1250 1300 1350
Entry Wire Line
	1200 1350 1300 1450
Entry Wire Line
	1200 1450 1300 1550
Entry Wire Line
	1200 1550 1300 1650
Entry Wire Line
	1200 1650 1300 1750
Entry Wire Line
	1200 1750 1300 1850
Entry Wire Line
	1200 1850 1300 1950
Entry Wire Line
	1200 1950 1300 2050
Text GLabel 1200 1150 2    50   Input ~ 0
10
Text GLabel 1200 1050 2    50   Input ~ 0
9
Text GLabel 1200 1350 2    50   Input ~ 0
8
Text GLabel 1200 1250 2    50   Input ~ 0
7
Text GLabel 1200 1550 2    50   Input ~ 0
6
Text GLabel 1200 1450 2    50   Input ~ 0
5
Text GLabel 1200 1750 2    50   Input ~ 0
4
Text GLabel 1200 1650 2    50   Input ~ 0
3
Text GLabel 1200 1950 2    50   Input ~ 0
2
Text GLabel 1200 1850 2    50   Input ~ 0
1
Entry Wire Line
	1200 2050 1300 2150
Text GLabel 1200 2050 2    50   Input ~ 0
GND
Connection ~ 4600 800 
Wire Bus Line
	4600 800  2200 800 
Connection ~ 2200 800 
Wire Bus Line
	1300 800  2200 800 
Entry Wire Line
	4500 1850 4600 1950
Text GLabel 4500 1650 0    50   Input ~ 0
2
Entry Wire Line
	4500 1750 4600 1850
Entry Wire Line
	1200 950  1300 1050
Wire Wire Line
	1100 950  1200 950 
Text GLabel 1200 950  2    50   Input ~ 0
V
Wire Bus Line
	4600 800  4600 6950
Wire Bus Line
	6250 800  6250 7050
Wire Bus Line
	2200 800  2200 7000
Wire Bus Line
	1300 800  1300 2900
$EndSCHEMATC
