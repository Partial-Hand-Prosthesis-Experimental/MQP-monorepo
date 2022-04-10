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
L Connector_Generic:Conn_01x22 J?
U 1 1 624C6ACA
P 4200 3250
F 0 "J?" H 4280 3242 50  0000 L CNN
F 1 "Conn_01x22" H 4280 3151 50  0000 L CNN
F 2 "" H 4200 3250 50  0001 C CNN
F 3 "~" H 4200 3250 50  0001 C CNN
	1    4200 3250
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x22 J?
U 1 1 624C7C10
P 9250 3400
F 0 "J?" H 9330 3392 50  0000 L CNN
F 1 "Conn_01x22" H 9330 3301 50  0000 L CNN
F 2 "" H 9250 3400 50  0001 C CNN
F 3 "~" H 9250 3400 50  0001 C CNN
	1    9250 3400
	1    0    0    -1  
$EndComp
Text Label 4000 2250 2    50   ~ 0
BAT
Text Label 4000 2350 2    50   ~ 0
Servo
Text Label 4000 2450 2    50   ~ 0
IO19
Text Label 4000 2550 2    50   ~ 0
H0+
Text Label 4000 2650 2    50   ~ 0
H0-
Text Label 4000 2750 2    50   ~ 0
H1+
Text Label 4000 2850 2    50   ~ 0
H1-
Text Label 4000 2950 2    50   ~ 0
H2+
Text Label 4000 3050 2    50   ~ 0
H2-
Text Label 4000 3150 2    50   ~ 0
H3+
Text Label 4000 3250 2    50   ~ 0
H3-
Text Label 4000 3350 2    50   ~ 0
H4+
Text Label 4000 3450 2    50   ~ 0
H4-
Text Label 4000 3550 2    50   ~ 0
A0
Text Label 4000 3650 2    50   ~ 0
A1
Text Label 4000 3750 2    50   ~ 0
A2
Text Label 4000 3850 2    50   ~ 0
A3
Text Label 4000 3950 2    50   ~ 0
A4
Text Label 4000 4050 2    50   ~ 0
A5
Text Label 4000 4150 2    50   ~ 0
A6
Text Label 4000 4250 2    50   ~ 0
A7
Text Label 4000 4350 2    50   ~ 0
GND
Text Label 9050 2400 2    50   ~ 0
GND
Text Label 9050 3500 2    50   ~ 0
SDA
Text Label 9050 3600 2    50   ~ 0
SCL
Text Label 9050 3700 2    50   ~ 0
IO32
Text Label 9050 3800 2    50   ~ 0
IO33
Text Label 9050 3900 2    50   ~ 0
IO39
Text Label 9050 4000 2    50   ~ 0
IO37
Text Label 9050 4100 2    50   ~ 0
IO36
Text Label 9050 4200 2    50   ~ 0
IO35
Text Label 9050 4300 2    50   ~ 0
VDD3.3
Text Label 9050 4400 2    50   ~ 0
VDD3.3
Text Label 9050 4500 2    50   ~ 0
VDD3.3
$Comp
L Device:Battery_Cell BT?
U 1 1 624D0E3A
P 4600 1250
F 0 "BT?" V 4855 1300 50  0000 C CNN
F 1 "Battery_Cell" V 4764 1300 50  0000 C CNN
F 2 "" V 4600 1310 50  0001 C CNN
F 3 "~" V 4600 1310 50  0001 C CNN
	1    4600 1250
	0    -1   -1   0   
$EndComp
$Comp
L Device:Battery_Cell BT?
U 1 1 624D10AE
P 5100 1250
F 0 "BT?" V 5355 1300 50  0000 C CNN
F 1 "Battery_Cell" V 5264 1300 50  0000 C CNN
F 2 "" V 5100 1310 50  0001 C CNN
F 3 "~" V 5100 1310 50  0001 C CNN
	1    5100 1250
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4900 1250 4700 1250
Wire Wire Line
	4400 1250 4000 1250
Wire Wire Line
	4000 1250 4000 1600
$Comp
L Motor:Motor_DC M?
U 1 1 624D43D2
P 1250 2600
F 0 "M?" H 1408 2596 50  0000 L CNN
F 1 "Motor_DC" H 1408 2505 50  0000 L CNN
F 2 "" H 1250 2510 50  0001 C CNN
F 3 "~" H 1250 2510 50  0001 C CNN
	1    1250 2600
	1    0    0    -1  
$EndComp
$Comp
L Motor:Motor_DC M?
U 1 1 624D4EE2
P 1250 3250
F 0 "M?" H 1408 3246 50  0000 L CNN
F 1 "Motor_DC" H 1408 3155 50  0000 L CNN
F 2 "" H 1250 3160 50  0001 C CNN
F 3 "~" H 1250 3160 50  0001 C CNN
	1    1250 3250
	1    0    0    -1  
$EndComp
$Comp
L Motor:Motor_DC M?
U 1 1 624D5251
P 1250 3950
F 0 "M?" H 1408 3946 50  0000 L CNN
F 1 "Motor_DC" H 1408 3855 50  0000 L CNN
F 2 "" H 1250 3860 50  0001 C CNN
F 3 "~" H 1250 3860 50  0001 C CNN
	1    1250 3950
	1    0    0    -1  
$EndComp
$Comp
L Motor:Motor_DC M?
U 1 1 624D5379
P 1250 4600
F 0 "M?" H 1408 4596 50  0000 L CNN
F 1 "Motor_DC" H 1408 4505 50  0000 L CNN
F 2 "" H 1250 4510 50  0001 C CNN
F 3 "~" H 1250 4510 50  0001 C CNN
	1    1250 4600
	1    0    0    -1  
$EndComp
$Comp
L Motor:Motor_DC M?
U 1 1 624D558F
P 1250 5300
F 0 "M?" H 1408 5296 50  0000 L CNN
F 1 "Motor_DC" H 1408 5205 50  0000 L CNN
F 2 "" H 1250 5210 50  0001 C CNN
F 3 "~" H 1250 5210 50  0001 C CNN
	1    1250 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 2550 1900 2550
Wire Wire Line
	1900 2550 1900 2400
Wire Wire Line
	1900 2400 1250 2400
Wire Wire Line
	1250 2900 1900 2900
Wire Wire Line
	1900 2900 1900 2650
Wire Wire Line
	1900 2650 4000 2650
Wire Wire Line
	2000 2750 2000 3050
Wire Wire Line
	2000 3050 1250 3050
Wire Wire Line
	1250 3550 2050 3550
Wire Wire Line
	2050 3550 2050 2850
Wire Wire Line
	2050 2850 4000 2850
Wire Wire Line
	2000 2750 4000 2750
Wire Wire Line
	4000 2950 2100 2950
Wire Wire Line
	2100 2950 2100 3750
Wire Wire Line
	2100 3750 1250 3750
Wire Wire Line
	1250 4250 2150 4250
Wire Wire Line
	2150 4250 2150 3050
Wire Wire Line
	2150 3050 4000 3050
Wire Wire Line
	4000 3150 2250 3150
Wire Wire Line
	2250 3150 2250 4400
Wire Wire Line
	2250 4400 1250 4400
Wire Wire Line
	1250 4900 2300 4900
Wire Wire Line
	2300 4900 2300 3250
Wire Wire Line
	2300 3250 4000 3250
Wire Wire Line
	4000 3350 2350 3350
Wire Wire Line
	2350 3350 2350 5100
Wire Wire Line
	2350 5100 1250 5100
Wire Wire Line
	1250 5600 2400 5600
Wire Wire Line
	2400 5600 2400 3450
Wire Wire Line
	2400 3450 4000 3450
$Comp
L Motor:Motor_Servo M?
U 1 1 624D9F15
P 2300 1600
F 0 "M?" H 2294 1293 50  0000 C CNN
F 1 "Motor_Servo" H 2294 1384 50  0000 C CNN
F 2 "" H 2300 1410 50  0001 C CNN
F 3 "http://forums.parallax.com/uploads/attachments/46831/74481.png" H 2300 1410 50  0001 C CNN
	1    2300 1600
	-1   0    0    1   
$EndComp
Wire Wire Line
	4000 2350 3700 2350
Wire Wire Line
	3700 2350 3700 1700
Wire Wire Line
	3700 1700 2600 1700
Wire Wire Line
	2600 1600 4000 1600
Connection ~ 4000 1600
Wire Wire Line
	4000 1600 4000 2250
Wire Wire Line
	2600 1500 3800 1500
Wire Wire Line
	3800 1500 3800 900 
Wire Wire Line
	3800 900  5200 900 
Wire Wire Line
	5200 900  5200 1250
Connection ~ 5200 1250
$Comp
L Sensor_Current:A1363xKTTN-1 U?
U 1 1 624DFF4F
P 3350 6300
F 0 "U?" V 3075 6300 50  0000 C CNN
F 1 "A1363xKTTN-1" V 2984 6300 50  0000 C CNN
F 2 "Sensor_Current:Allegro_SIP-4" H 3700 6200 50  0001 L CIN
F 3 "http://www.allegromicro.com/~/media/Files/Datasheets/A1363-Datasheet.ashx?la=en" H 3350 6300 50  0001 C CNN
	1    3350 6300
	0    -1   -1   0   
$EndComp
$Comp
L Sensor_Current:A1363xKTTN-1 U?
U 1 1 624E22FD
P 4200 6300
F 0 "U?" V 3925 6300 50  0000 C CNN
F 1 "A1363xKTTN-1" V 3834 6300 50  0000 C CNN
F 2 "Sensor_Current:Allegro_SIP-4" H 4550 6200 50  0001 L CIN
F 3 "http://www.allegromicro.com/~/media/Files/Datasheets/A1363-Datasheet.ashx?la=en" H 4200 6300 50  0001 C CNN
	1    4200 6300
	0    -1   -1   0   
$EndComp
$Comp
L Sensor_Current:A1363xKTTN-1 U?
U 1 1 624E3230
P 5000 6300
F 0 "U?" V 4725 6300 50  0000 C CNN
F 1 "A1363xKTTN-1" V 4634 6300 50  0000 C CNN
F 2 "Sensor_Current:Allegro_SIP-4" H 5350 6200 50  0001 L CIN
F 3 "http://www.allegromicro.com/~/media/Files/Datasheets/A1363-Datasheet.ashx?la=en" H 5000 6300 50  0001 C CNN
	1    5000 6300
	0    -1   -1   0   
$EndComp
$Comp
L Sensor_Current:A1363xKTTN-1 U?
U 1 1 624E409B
P 2550 6300
F 0 "U?" V 2275 6300 50  0000 C CNN
F 1 "A1363xKTTN-1" V 2184 6300 50  0000 C CNN
F 2 "Sensor_Current:Allegro_SIP-4" H 2900 6200 50  0001 L CIN
F 3 "http://www.allegromicro.com/~/media/Files/Datasheets/A1363-Datasheet.ashx?la=en" H 2550 6300 50  0001 C CNN
	1    2550 6300
	0    -1   -1   0   
$EndComp
$Comp
L Sensor_Current:A1363xKTTN-1 U?
U 1 1 624E4EDA
P 5900 6300
F 0 "U?" V 5625 6300 50  0000 C CNN
F 1 "A1363xKTTN-1" V 5534 6300 50  0000 C CNN
F 2 "Sensor_Current:Allegro_SIP-4" H 6250 6200 50  0001 L CIN
F 3 "http://www.allegromicro.com/~/media/Files/Datasheets/A1363-Datasheet.ashx?la=en" H 5900 6300 50  0001 C CNN
	1    5900 6300
	0    -1   -1   0   
$EndComp
$Comp
L Sensor_Current:A1363xKTTN-1 U?
U 1 1 624E6A8B
P 1750 6300
F 0 "U?" V 1475 6300 50  0000 C CNN
F 1 "A1363xKTTN-1" V 1384 6300 50  0000 C CNN
F 2 "Sensor_Current:Allegro_SIP-4" H 2100 6200 50  0001 L CIN
F 3 "http://www.allegromicro.com/~/media/Files/Datasheets/A1363-Datasheet.ashx?la=en" H 1750 6300 50  0001 C CNN
	1    1750 6300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4000 3550 2450 3550
Wire Wire Line
	2450 3550 2450 5700
Wire Wire Line
	2450 5700 1750 5700
Wire Wire Line
	1750 5700 1750 5900
Wire Wire Line
	2550 5900 2550 3650
Wire Wire Line
	2550 3650 4000 3650
Wire Wire Line
	3350 5900 3350 3750
Wire Wire Line
	3350 3750 4000 3750
Wire Wire Line
	4000 3850 3450 3850
Wire Wire Line
	3450 3850 3450 5800
Wire Wire Line
	3450 5800 4200 5800
Wire Wire Line
	4200 5800 4200 5900
Wire Wire Line
	4000 3950 3550 3950
Wire Wire Line
	3550 3950 3550 5700
Wire Wire Line
	3550 5700 5000 5700
Wire Wire Line
	5000 5700 5000 5900
Wire Wire Line
	5900 5900 5900 5600
Wire Wire Line
	5900 5600 3650 5600
Wire Wire Line
	3650 5600 3650 4050
Wire Wire Line
	3650 4050 4000 4050
Wire Wire Line
	4000 4350 4000 5450
Wire Wire Line
	4000 5450 6200 5450
Wire Wire Line
	6200 5450 6200 6300
Wire Wire Line
	6200 7350 5300 7350
Wire Wire Line
	2050 7350 2050 6300
Wire Wire Line
	6200 6300 6200 7350
Connection ~ 6200 6300
Wire Wire Line
	2850 6300 2850 7350
Connection ~ 2850 7350
Wire Wire Line
	2850 7350 2050 7350
Wire Wire Line
	3650 6300 3650 7350
Connection ~ 3650 7350
Wire Wire Line
	3650 7350 2850 7350
Wire Wire Line
	4500 6300 4500 7350
Connection ~ 4500 7350
Wire Wire Line
	4500 7350 3650 7350
Wire Wire Line
	5300 6300 5300 7350
Connection ~ 5300 7350
Wire Wire Line
	5300 7350 4500 7350
Wire Wire Line
	9050 4300 9050 4400
Connection ~ 9050 4400
Wire Wire Line
	9050 4400 9050 4500
Wire Wire Line
	5600 6300 5600 6900
Wire Wire Line
	5600 6900 4700 6900
Wire Wire Line
	1450 6900 1450 6300
Wire Wire Line
	2250 6300 2250 6900
Connection ~ 2250 6900
Wire Wire Line
	2250 6900 1450 6900
Wire Wire Line
	3050 6900 3050 6300
Connection ~ 3050 6900
Wire Wire Line
	3050 6900 2250 6900
Wire Wire Line
	3900 6300 3900 6900
Connection ~ 3900 6900
Wire Wire Line
	3900 6900 3050 6900
Wire Wire Line
	4700 6300 4700 6900
Connection ~ 4700 6900
Wire Wire Line
	4700 6900 3900 6900
Wire Wire Line
	5200 1250 5200 2400
Wire Wire Line
	5200 2400 8800 2400
$Comp
L Device:R_Variable_US R?
U 1 1 6251429C
P 7200 3500
F 0 "R?" V 6955 3500 50  0000 C CNN
F 1 "R_Variable_US" V 7046 3500 50  0000 C CNN
F 2 "" V 7130 3500 50  0001 C CNN
F 3 "~" H 7200 3500 50  0001 C CNN
	1    7200 3500
	0    1    1    0   
$EndComp
$Comp
L Device:R_Variable_US R?
U 1 1 62514E0D
P 7200 3850
F 0 "R?" V 6955 3850 50  0000 C CNN
F 1 "R_Variable_US" V 7046 3850 50  0000 C CNN
F 2 "" V 7130 3850 50  0001 C CNN
F 3 "~" H 7200 3850 50  0001 C CNN
	1    7200 3850
	0    1    1    0   
$EndComp
$Comp
L Device:R_Variable_US R?
U 1 1 6251577C
P 7200 4200
F 0 "R?" V 6955 4200 50  0000 C CNN
F 1 "R_Variable_US" V 7046 4200 50  0000 C CNN
F 2 "" V 7130 4200 50  0001 C CNN
F 3 "~" H 7200 4200 50  0001 C CNN
	1    7200 4200
	0    1    1    0   
$EndComp
$Comp
L Device:R_Variable_US R?
U 1 1 62517A4A
P 7200 4550
F 0 "R?" V 6955 4550 50  0000 C CNN
F 1 "R_Variable_US" V 7046 4550 50  0000 C CNN
F 2 "" V 7130 4550 50  0001 C CNN
F 3 "~" H 7200 4550 50  0001 C CNN
	1    7200 4550
	0    1    1    0   
$EndComp
$Comp
L Device:R_Variable_US R?
U 1 1 62519C75
P 7200 4900
F 0 "R?" V 6955 4900 50  0000 C CNN
F 1 "R_Variable_US" V 7046 4900 50  0000 C CNN
F 2 "" V 7130 4900 50  0001 C CNN
F 3 "~" H 7200 4900 50  0001 C CNN
	1    7200 4900
	0    1    1    0   
$EndComp
Wire Wire Line
	9050 4500 9050 4650
Wire Wire Line
	9050 6400 7050 6400
Wire Wire Line
	6600 6400 6600 6900
Wire Wire Line
	6600 6900 5600 6900
Connection ~ 9050 4500
Connection ~ 5600 6900
Wire Wire Line
	7050 3500 7050 3850
Connection ~ 7050 6400
Wire Wire Line
	7050 6400 6600 6400
Connection ~ 7050 3850
Wire Wire Line
	7050 3850 7050 4200
Connection ~ 7050 4200
Wire Wire Line
	7050 4200 7050 4550
Connection ~ 7050 4550
Wire Wire Line
	7050 4550 7050 4900
Connection ~ 7050 4900
Wire Wire Line
	7050 4900 7050 5200
Wire Wire Line
	9050 3700 7550 3700
Wire Wire Line
	7550 3700 7550 3500
Wire Wire Line
	7550 3500 7350 3500
Wire Wire Line
	7350 3850 7550 3850
Wire Wire Line
	7550 3850 7550 3800
Wire Wire Line
	7550 3800 9050 3800
Wire Wire Line
	9050 3900 7550 3900
Wire Wire Line
	7550 3900 7550 4200
Wire Wire Line
	7550 4200 7350 4200
Wire Wire Line
	7350 4550 7600 4550
Wire Wire Line
	7600 4550 7600 4000
Wire Wire Line
	7600 4000 9050 4000
Wire Wire Line
	9050 4100 7650 4100
Wire Wire Line
	7650 4100 7650 4900
Wire Wire Line
	7650 4900 7350 4900
$Comp
L Switch:SW_Push SW?
U 1 1 6254A01B
P 4950 3000
F 0 "SW?" V 4904 3148 50  0000 L CNN
F 1 "SW_Push" V 4995 3148 50  0000 L CNN
F 2 "" H 4950 3200 50  0001 C CNN
F 3 "~" H 4950 3200 50  0001 C CNN
	1    4950 3000
	0    1    1    0   
$EndComp
Wire Wire Line
	4000 2450 3750 2450
Wire Wire Line
	3750 2450 3750 2100
Wire Wire Line
	3750 2100 4950 2100
Wire Wire Line
	4950 2100 4950 2800
Wire Wire Line
	4950 3200 4950 5200
Wire Wire Line
	4950 5200 7050 5200
Connection ~ 7050 5200
Wire Wire Line
	7050 5200 7050 6400
$Comp
L Display_Character:EA_T123X-I2C U?
U 1 1 624D4CB4
P 8000 1650
F 0 "U?" V 8567 1650 50  0000 C CNN
F 1 "EA_T123X-I2C" V 8476 1650 50  0000 C CNN
F 2 "Display:EA_T123X-I2C" H 8000 1050 50  0001 C CNN
F 3 "http://www.lcd-module.de/pdf/doma/t123-i2c.pdf" H 8000 1150 50  0001 C CNN
	1    8000 1650
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7500 1650 7500 1000
Wire Wire Line
	7500 1000 9500 1000
Wire Wire Line
	9500 1000 9500 4650
Wire Wire Line
	9500 4650 9050 4650
Connection ~ 9050 4650
Wire Wire Line
	8500 1650 8800 1650
Wire Wire Line
	8800 1650 8800 2400
Connection ~ 8800 2400
Wire Wire Line
	8800 2400 9050 2400
Wire Wire Line
	7800 2150 7800 3500
Wire Wire Line
	7800 3500 9050 3500
Wire Wire Line
	9050 3600 7700 3600
Wire Wire Line
	7700 3600 7700 2150
Text Notes 900  2250 0    118  ~ 0
Haptic Motors
Text Notes 2050 1150 0    118  ~ 0
Servo
Text Notes 4500 750  0    118  ~ 0
Battery
Text Notes 7550 800  0    118  ~ 0
Display
Text Notes 6650 3150 0    118  ~ 0
Velostat
Text Notes 3000 7600 0    118  ~ 0
Hall Effect Sensors
Text Notes 4350 2900 0    118  ~ 0
Left
Text Notes 8300 2900 0    118  ~ 0
Right
Wire Wire Line
	9050 4650 9050 6400
$EndSCHEMATC
