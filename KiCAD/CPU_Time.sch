EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 3 5
Title "Nixie Tube Clock Host Board"
Date "2021-04-20"
Rev "v01.1"
Comp "University of Alberta (AlbertaSat)"
Comment1 "License: CC BY 4.0 (https://creativecommons.org/licenses/by/4.0/)"
Comment2 "Steven Knudsen (ECE dept., Faculty of Engineering, University of Alberta)."
Comment3 "This schematic was designed with the assistance of "
Comment4 "Author: Pundeep Hundal                   Co-author: Sichun Xu"
$EndDescr
$Comp
L power:GND #PWR?
U 1 1 6096D806
P 2150 3000
AR Path="/6096D806" Ref="#PWR?"  Part="1" 
AR Path="/6091BF76/6096D806" Ref="#PWR08"  Part="1" 
F 0 "#PWR08" H 2150 2750 50  0001 C CNN
F 1 "GND" H 2155 2827 50  0000 C CNN
F 2 "" H 2150 3000 50  0001 C CNN
F 3 "" H 2150 3000 50  0001 C CNN
	1    2150 3000
	1    0    0    -1  
$EndComp
Text GLabel 2150 2200 1    50   Input ~ 0
RESET
Text GLabel 5050 4650 2    50   Input ~ 0
SCL_DS3231
Text GLabel 5050 4750 2    50   Input ~ 0
SDA_DS3231
Text GLabel 5050 2850 2    50   Input ~ 0
SCK
Text GLabel 5050 2950 2    50   Input ~ 0
MISO
Text GLabel 5050 3050 2    50   Input ~ 0
MOSI
Text GLabel 5050 5250 2    50   Input ~ 0
EXIXE14_5
Text GLabel 5050 5350 2    50   Input ~ 0
EXIXE14_4
Text GLabel 5050 5450 2    50   Input ~ 0
EXIXE14_3
Text GLabel 5050 3550 2    50   Input ~ 0
EXIXE14_0
Text GLabel 5050 3450 2    50   Input ~ 0
EXIXE14_1
Text GLabel 5050 5150 2    50   Input ~ 0
EXIXE14_2
$Comp
L Device:C_Small C?
U 1 1 6096D84C
P 2550 2800
AR Path="/6096D84C" Ref="C?"  Part="1" 
AR Path="/6091BF76/6096D84C" Ref="C4"  Part="1" 
F 0 "C4" H 2400 2850 50  0000 L CNN
F 1 "100nF" H 2300 2750 50  0000 L CNN
F 2 "" H 2550 2800 50  0001 C CNN
F 3 "~" H 2550 2800 50  0001 C CNN
	1    2550 2800
	1    0    0    -1  
$EndComp
Text GLabel 2850 4950 0    50   Input ~ 0
INS-1
$Comp
L Switch:SW_Push SW?
U 1 1 6096D859
P 2150 2750
AR Path="/6096D859" Ref="SW?"  Part="1" 
AR Path="/6091BF76/6096D859" Ref="SW1"  Part="1" 
F 0 "SW1" V 2100 2550 50  0000 L CNN
F 1 "Reset" V 2200 2500 50  0000 L CNN
F 2 "" H 2150 2950 50  0001 C CNN
F 3 "~" H 2150 2950 50  0001 C CNN
	1    2150 2750
	0    1    1    0   
$EndComp
Wire Wire Line
	2150 2200 2150 2450
Connection ~ 2150 2450
Wire Wire Line
	2150 2450 2150 2550
Wire Wire Line
	2550 2700 2550 2450
Wire Wire Line
	2550 2450 2150 2450
$Comp
L power:GND #PWR?
U 1 1 6096D866
P 2550 3000
AR Path="/6096D866" Ref="#PWR?"  Part="1" 
AR Path="/6091BF76/6096D866" Ref="#PWR09"  Part="1" 
F 0 "#PWR09" H 2550 2750 50  0001 C CNN
F 1 "GND" H 2555 2827 50  0000 C CNN
F 2 "" H 2550 3000 50  0001 C CNN
F 3 "" H 2550 3000 50  0001 C CNN
	1    2550 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 3000 2550 2900
Wire Wire Line
	2150 3000 2150 2950
NoConn ~ 8900 2050
NoConn ~ 8900 1750
NoConn ~ 8900 1650
Wire Wire Line
	8650 2250 8900 2250
Wire Wire Line
	8650 2450 8650 2250
$Comp
L power:+3.3V #PWR?
U 1 1 6096D873
P 8650 2450
AR Path="/6096D873" Ref="#PWR?"  Part="1" 
AR Path="/6091BF76/6096D873" Ref="#PWR023"  Part="1" 
F 0 "#PWR023" H 8650 2300 50  0001 C CNN
F 1 "+3.3V" H 8665 2623 50  0000 C CNN
F 2 "" H 8650 2450 50  0001 C CNN
F 3 "" H 8650 2450 50  0001 C CNN
	1    8650 2450
	-1   0    0    1   
$EndComp
Wire Wire Line
	8350 2150 8900 2150
Wire Wire Line
	8350 2250 8350 2150
$Comp
L power:GND #PWR?
U 1 1 6096D87B
P 8350 2250
AR Path="/6096D87B" Ref="#PWR?"  Part="1" 
AR Path="/6091BF76/6096D87B" Ref="#PWR020"  Part="1" 
F 0 "#PWR020" H 8350 2000 50  0001 C CNN
F 1 "GND" H 8355 2077 50  0000 C CNN
F 2 "" H 8350 2250 50  0001 C CNN
F 3 "" H 8350 2250 50  0001 C CNN
	1    8350 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	8100 1850 8900 1850
Wire Wire Line
	8100 1850 8100 1950
Text GLabel 8900 1950 0    50   Input ~ 0
MAS6180C_ACR
$Comp
L power:GND #PWR?
U 1 1 6096D884
P 8100 1950
AR Path="/6096D884" Ref="#PWR?"  Part="1" 
AR Path="/6091BF76/6096D884" Ref="#PWR017"  Part="1" 
F 0 "#PWR017" H 8100 1700 50  0001 C CNN
F 1 "GND" H 8105 1777 50  0000 C CNN
F 2 "" H 8100 1950 50  0001 C CNN
F 3 "" H 8100 1950 50  0001 C CNN
	1    8100 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	8650 1550 8900 1550
Wire Wire Line
	8650 1400 8650 1550
$Comp
L power:+3.3V #PWR?
U 1 1 6096D88C
P 8650 1400
AR Path="/6096D88C" Ref="#PWR?"  Part="1" 
AR Path="/6091BF76/6096D88C" Ref="#PWR022"  Part="1" 
F 0 "#PWR022" H 8650 1250 50  0001 C CNN
F 1 "+3.3V" H 8665 1573 50  0000 C CNN
F 2 "" H 8650 1400 50  0001 C CNN
F 3 "" H 8650 1400 50  0001 C CNN
	1    8650 1400
	1    0    0    -1  
$EndComp
$Comp
L Nixie_KiCAD_Library:MAS6180 U?
U 1 1 6096D892
P 9250 1800
AR Path="/6096D892" Ref="U?"  Part="1" 
AR Path="/6091BF76/6096D892" Ref="U3"  Part="1" 
F 0 "U3" H 9200 2250 50  0000 L CNN
F 1 "MAS6180" H 9100 2150 50  0000 L CNN
F 2 "" H 9250 1800 50  0001 C CNN
F 3 "" H 9250 1800 50  0001 C CNN
	1    9250 1800
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 6096D898
P 7600 4300
AR Path="/6096D898" Ref="C?"  Part="1" 
AR Path="/6091BF76/6096D898" Ref="C6"  Part="1" 
F 0 "C6" H 7450 4350 50  0000 L CNN
F 1 "100nF" H 7350 4250 50  0000 L CNN
F 2 "" H 7600 4300 50  0001 C CNN
F 3 "~" H 7600 4300 50  0001 C CNN
	1    7600 4300
	1    0    0    -1  
$EndComp
NoConn ~ 8950 4750
Text GLabel 8050 4850 0    50   Input ~ 0
RESET
Wire Wire Line
	7600 4400 7600 4500
Wire Wire Line
	7600 4100 7600 4200
$Comp
L power:GND #PWR?
U 1 1 6096D8A2
P 7600 4500
AR Path="/6096D8A2" Ref="#PWR?"  Part="1" 
AR Path="/6091BF76/6096D8A2" Ref="#PWR015"  Part="1" 
F 0 "#PWR015" H 7600 4250 50  0001 C CNN
F 1 "GND" H 7605 4327 50  0000 C CNN
F 2 "" H 7600 4500 50  0001 C CNN
F 3 "" H 7600 4500 50  0001 C CNN
	1    7600 4500
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 6096D8A8
P 7600 4100
AR Path="/6096D8A8" Ref="#PWR?"  Part="1" 
AR Path="/6091BF76/6096D8A8" Ref="#PWR014"  Part="1" 
F 0 "#PWR014" H 7600 3950 50  0001 C CNN
F 1 "+3.3V" H 7615 4273 50  0000 C CNN
F 2 "" H 7600 4100 50  0001 C CNN
F 3 "" H 7600 4100 50  0001 C CNN
	1    7600 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	7800 4650 8050 4650
Wire Wire Line
	7800 4300 7800 4650
$Comp
L power:+3.3V #PWR?
U 1 1 6096D8B0
P 7800 4300
AR Path="/6096D8B0" Ref="#PWR?"  Part="1" 
AR Path="/6091BF76/6096D8B0" Ref="#PWR016"  Part="1" 
F 0 "#PWR016" H 7800 4150 50  0001 C CNN
F 1 "+3.3V" H 7815 4473 50  0000 C CNN
F 2 "" H 7800 4300 50  0001 C CNN
F 3 "" H 7800 4300 50  0001 C CNN
	1    7800 4300
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 6096D8B6
P 9800 4800
AR Path="/6096D8B6" Ref="#PWR?"  Part="1" 
AR Path="/6091BF76/6096D8B6" Ref="#PWR024"  Part="1" 
F 0 "#PWR024" H 9800 4650 50  0001 C CNN
F 1 "+3.3V" H 9815 4973 50  0000 C CNN
F 2 "" H 9800 4800 50  0001 C CNN
F 3 "" H 9800 4800 50  0001 C CNN
	1    9800 4800
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R?
U 1 1 6096D8BC
P 9800 5000
AR Path="/6096D8BC" Ref="R?"  Part="1" 
AR Path="/6091BF76/6096D8BC" Ref="R2"  Part="1" 
F 0 "R2" H 9650 4950 50  0000 L CNN
F 1 "4.7k" H 9600 5050 50  0000 L CNN
F 2 "" H 9800 5000 50  0001 C CNN
F 3 "~" H 9800 5000 50  0001 C CNN
	1    9800 5000
	-1   0    0    1   
$EndComp
$Comp
L Device:R_Small R?
U 1 1 6096D8C2
P 9550 5000
AR Path="/6096D8C2" Ref="R?"  Part="1" 
AR Path="/6091BF76/6096D8C2" Ref="R1"  Part="1" 
F 0 "R1" H 9600 4950 50  0000 L CNN
F 1 "4.7k" H 9600 5050 50  0000 L CNN
F 2 "" H 9550 5000 50  0001 C CNN
F 3 "~" H 9550 5000 50  0001 C CNN
	1    9550 5000
	-1   0    0    1   
$EndComp
Text GLabel 10000 5250 2    50   Input ~ 0
SDA_DS3231
Text GLabel 10000 5150 2    50   Input ~ 0
SCL_DS3231
NoConn ~ 8050 4750
NoConn ~ 8050 4550
Wire Wire Line
	9050 5250 9050 5400
Connection ~ 9050 5250
Wire Wire Line
	8950 5250 9050 5250
Wire Wire Line
	9050 5150 9050 5250
Connection ~ 9050 5150
Wire Wire Line
	8950 5150 9050 5150
Wire Wire Line
	9050 5050 9050 5150
Connection ~ 9050 5050
Wire Wire Line
	8950 5050 9050 5050
Wire Wire Line
	9050 4950 9050 5050
Connection ~ 9050 4950
Wire Wire Line
	8950 4950 9050 4950
Wire Wire Line
	8500 5400 9050 5400
Wire Wire Line
	8950 4850 9050 4850
Wire Wire Line
	9050 4850 9050 4950
Wire Wire Line
	7950 5250 7950 5400
Connection ~ 7950 5250
Wire Wire Line
	8050 5250 7950 5250
Wire Wire Line
	7950 5150 7950 5250
Connection ~ 7950 5150
Wire Wire Line
	8050 5150 7950 5150
Wire Wire Line
	7950 5050 7950 5150
Connection ~ 7950 5050
Wire Wire Line
	8050 5050 7950 5050
Connection ~ 8500 5400
Wire Wire Line
	8500 5400 8500 5450
Wire Wire Line
	7950 5400 8500 5400
Wire Wire Line
	7950 4950 7950 5050
Wire Wire Line
	8050 4950 7950 4950
$Comp
L power:GND #PWR?
U 1 1 6096D8E9
P 8500 5450
AR Path="/6096D8E9" Ref="#PWR?"  Part="1" 
AR Path="/6091BF76/6096D8E9" Ref="#PWR021"  Part="1" 
F 0 "#PWR021" H 8500 5200 50  0001 C CNN
F 1 "GND" H 8505 5277 50  0000 C CNN
F 2 "" H 8500 5450 50  0001 C CNN
F 3 "" H 8500 5450 50  0001 C CNN
	1    8500 5450
	1    0    0    -1  
$EndComp
$Comp
L Nixie_KiCAD_Library:DS3231 U?
U 1 1 6096D8EF
P 8500 4900
AR Path="/6096D8EF" Ref="U?"  Part="1" 
AR Path="/6091BF76/6096D8EF" Ref="U2"  Part="1" 
F 0 "U2" H 8500 5475 50  0000 C CNN
F 1 "DS3231" H 8500 5384 50  0000 C CNN
F 2 "" H 8500 4900 50  0001 C CNN
F 3 "" H 8500 4900 50  0001 C CNN
	1    8500 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	8300 1250 8300 1300
Wire Wire Line
	8300 1550 8300 1500
$Comp
L power:GND #PWR?
U 1 1 6096D8F7
P 8300 1550
AR Path="/6096D8F7" Ref="#PWR?"  Part="1" 
AR Path="/6091BF76/6096D8F7" Ref="#PWR019"  Part="1" 
F 0 "#PWR019" H 8300 1300 50  0001 C CNN
F 1 "GND" H 8305 1377 50  0000 C CNN
F 2 "" H 8300 1550 50  0001 C CNN
F 3 "" H 8300 1550 50  0001 C CNN
	1    8300 1550
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 6096D8FD
P 8300 1250
AR Path="/6096D8FD" Ref="#PWR?"  Part="1" 
AR Path="/6091BF76/6096D8FD" Ref="#PWR018"  Part="1" 
F 0 "#PWR018" H 8300 1100 50  0001 C CNN
F 1 "+3.3V" H 8315 1423 50  0000 C CNN
F 2 "" H 8300 1250 50  0001 C CNN
F 3 "" H 8300 1250 50  0001 C CNN
	1    8300 1250
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 6096D903
P 8300 1400
AR Path="/6096D903" Ref="C?"  Part="1" 
AR Path="/6091BF76/6096D903" Ref="C7"  Part="1" 
F 0 "C7" H 8392 1446 50  0000 L CNN
F 1 "100nF" H 8392 1355 50  0000 L CNN
F 2 "" H 8300 1400 50  0001 C CNN
F 3 "~" H 8300 1400 50  0001 C CNN
	1    8300 1400
	1    0    0    -1  
$EndComp
$Comp
L Nixie_KiCAD_Library:JST_connector P?
U 1 1 6096D911
P 1550 6450
AR Path="/6096D911" Ref="P?"  Part="1" 
AR Path="/6091BF76/6096D911" Ref="P1"  Part="1" 
F 0 "P1" H 1458 6915 50  0000 C CNN
F 1 "JST_connector" H 1458 6824 50  0000 C CNN
F 2 "" H 1550 6450 50  0001 C CNN
F 3 "" H 1550 6450 50  0001 C CNN
	1    1550 6450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 6096D927
P 1650 6850
AR Path="/6096D927" Ref="#PWR?"  Part="1" 
AR Path="/6091BF76/6096D927" Ref="#PWR07"  Part="1" 
F 0 "#PWR07" H 1650 6600 50  0001 C CNN
F 1 "GND" H 1655 6677 50  0000 C CNN
F 2 "" H 1650 6850 50  0001 C CNN
F 3 "" H 1650 6850 50  0001 C CNN
	1    1650 6850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 6850 1650 6650
Wire Wire Line
	1650 6650 1550 6650
Wire Wire Line
	9300 5150 9550 5150
Wire Wire Line
	9550 5150 9550 5100
Connection ~ 9550 5150
Wire Wire Line
	9550 5150 10000 5150
Wire Wire Line
	9550 4900 9550 4850
Wire Wire Line
	9550 4850 9800 4850
Wire Wire Line
	9800 4850 9800 4800
Wire Wire Line
	9800 4900 9800 4850
Connection ~ 9800 4850
Wire Wire Line
	9800 5100 9800 5250
Connection ~ 9800 5250
Wire Wire Line
	9800 5250 10000 5250
Wire Wire Line
	9200 5250 9800 5250
Wire Wire Line
	9300 5150 9300 4550
Wire Wire Line
	9300 4550 8950 4550
Wire Wire Line
	9200 5250 9200 4650
Wire Wire Line
	9200 4650 8950 4650
Text GLabel 2850 5050 0    50   Input ~ 0
BN_CLK0
Text GLabel 2850 3750 0    50   Input ~ 0
BN_CLK1
Text GLabel 5050 2350 2    50   Input ~ 0
BN_CLK2
Text GLabel 5050 2450 2    50   Input ~ 0
BN_CLK3
Text GLabel 5050 2750 2    50   Input ~ 0
BN_CLK4
Text GLabel 5050 4050 2    50   Input ~ 0
BN_CLK5
Text GLabel 2850 4050 0    50   Input ~ 0
BN_CLK6
Text GLabel 2850 3950 0    50   Input ~ 0
BN_CLK7
$Comp
L Device:C_Small C?
U 1 1 6089E69D
P 3000 1400
AR Path="/6089E69D" Ref="C?"  Part="1" 
AR Path="/6091BF76/6089E69D" Ref="C5"  Part="1" 
F 0 "C5" H 3092 1446 50  0000 L CNN
F 1 "100nF" H 3092 1355 50  0000 L CNN
F 2 "" H 3000 1400 50  0001 C CNN
F 3 "~" H 3000 1400 50  0001 C CNN
	1    3000 1400
	1    0    0    -1  
$EndComp
$Comp
L MCU_Module:NUCLEO64-F411RE U1
U 1 1 608AA55B
P 3950 3850
F 0 "U1" H 3050 5750 50  0000 C CNN
F 1 "NUCLEO64-F411RE" H 4900 1950 50  0000 C CNN
F 2 "Module:ST_Morpho_Connector_144_STLink" H 4500 1950 50  0001 L CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/data_brief/DM00105918.pdf" H 3050 2450 50  0001 C CNN
	1    3950 3850
	1    0    0    -1  
$EndComp
NoConn ~ 3650 1850
NoConn ~ 3750 1850
NoConn ~ 3950 1850
NoConn ~ 4450 1850
NoConn ~ 3850 1850
NoConn ~ 3550 1850
NoConn ~ 3350 1850
NoConn ~ 4050 1850
$Comp
L power:+5V #PWR010
U 1 1 608D652A
P 3000 1200
F 0 "#PWR010" H 3000 1050 50  0001 C CNN
F 1 "+5V" H 3015 1373 50  0000 C CNN
F 2 "" H 3000 1200 50  0001 C CNN
F 3 "" H 3000 1200 50  0001 C CNN
	1    3000 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 1200 3000 1300
$Comp
L power:GND #PWR?
U 1 1 608D9339
P 3000 1600
AR Path="/608D9339" Ref="#PWR?"  Part="1" 
AR Path="/6091BF76/608D9339" Ref="#PWR011"  Part="1" 
F 0 "#PWR011" H 3000 1350 50  0001 C CNN
F 1 "GND" H 3005 1427 50  0000 C CNN
F 2 "" H 3000 1600 50  0001 C CNN
F 3 "" H 3000 1600 50  0001 C CNN
	1    3000 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 1600 3000 1500
NoConn ~ 4450 5850
$Comp
L power:GND #PWR?
U 1 1 608ECA7C
P 4250 6050
AR Path="/608ECA7C" Ref="#PWR?"  Part="1" 
AR Path="/6091BF76/608ECA7C" Ref="#PWR013"  Part="1" 
F 0 "#PWR013" H 4250 5800 50  0001 C CNN
F 1 "GND" H 4255 5877 50  0000 C CNN
F 2 "" H 4250 6050 50  0001 C CNN
F 3 "" H 4250 6050 50  0001 C CNN
	1    4250 6050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3350 5850 3350 5950
Wire Wire Line
	4250 5950 4250 5850
Wire Wire Line
	4250 6050 4250 5950
Wire Wire Line
	3350 5950 3450 5950
Connection ~ 3850 5950
Wire Wire Line
	3850 5950 3950 5950
Wire Wire Line
	3450 5850 3450 5950
Connection ~ 3450 5950
Wire Wire Line
	3450 5950 3550 5950
Wire Wire Line
	3550 5850 3550 5950
Connection ~ 3550 5950
Wire Wire Line
	3650 5850 3650 5950
Wire Wire Line
	3550 5950 3650 5950
Connection ~ 3650 5950
Wire Wire Line
	3650 5950 3750 5950
Wire Wire Line
	3750 5850 3750 5950
Connection ~ 3750 5950
Wire Wire Line
	3850 5850 3850 5950
Wire Wire Line
	3950 5850 3950 5950
Connection ~ 3950 5950
Wire Wire Line
	3950 5950 4050 5950
Wire Wire Line
	4050 5850 4050 5950
Connection ~ 4050 5950
Wire Wire Line
	4050 5950 4150 5950
Wire Wire Line
	4150 5850 4150 5950
Connection ~ 4150 5950
Wire Wire Line
	4150 5950 4250 5950
Wire Wire Line
	3750 5950 3850 5950
Connection ~ 4250 5950
Wire Wire Line
	2850 2450 2550 2450
Connection ~ 2550 2450
Text GLabel 2850 3450 0    50   Input ~ 0
LED_Launch
Text GLabel 2850 3550 0    50   Input ~ 0
LED_Timer
Text GLabel 2850 4150 0    50   Input ~ 0
LED_Clock
Text GLabel 2850 4250 0    50   Input ~ 0
LED_Stopwatch
Wire Wire Line
	1550 6550 5650 6550
Wire Wire Line
	5650 6550 5650 4850
Wire Wire Line
	5650 4850 5050 4850
Wire Wire Line
	1550 6450 2400 6450
Wire Wire Line
	2400 6450 2400 4850
Wire Wire Line
	2400 4850 2850 4850
Wire Wire Line
	1550 6350 2250 6350
Wire Wire Line
	2250 6350 2250 4750
Wire Wire Line
	2250 4750 2850 4750
Wire Wire Line
	1550 6250 2100 6250
Wire Wire Line
	2100 6250 2100 4650
Wire Wire Line
	2100 4650 2850 4650
Wire Notes Line
	1000 500  11050 500 
Wire Notes Line
	11050 500  11050 6000
Wire Notes Line
	11050 6000 6500 6000
Wire Notes Line
	6500 7500 1000 7500
Wire Notes Line
	1000 7500 1000 500 
Wire Notes Line
	6500 500  6500 7500
Wire Notes Line
	6500 3300 11050 3300
Text Notes 3350 700  2    98   ~ 20
STM64-F401RE Nucleo Board
Text Notes 8750 800  2    79   ~ 0
MAS6180 AM-Receiver with Antenna\n(or WWVB atomic clock receiver)
Text Notes 8550 3500 2    79   ~ 0
DS3231 RTC (with 32k EEPROM)
NoConn ~ 2850 4350
NoConn ~ 2850 4450
NoConn ~ 2850 4550
NoConn ~ 2850 5150
NoConn ~ 2850 5250
NoConn ~ 2850 5350
NoConn ~ 2850 5450
NoConn ~ 5050 2550
NoConn ~ 5050 2650
NoConn ~ 5050 3150
NoConn ~ 5050 3250
NoConn ~ 5050 3650
NoConn ~ 5050 3750
NoConn ~ 5050 3850
NoConn ~ 5050 4150
NoConn ~ 5050 4250
NoConn ~ 5050 4350
NoConn ~ 5050 4450
NoConn ~ 5050 4550
NoConn ~ 5050 4950
NoConn ~ 5050 5050
NoConn ~ 2850 2350
Text GLabel 5050 3350 2    50   Input ~ 0
MAS6180C_ACR
$Comp
L power:+5V #PWR0101
U 1 1 60A5E9F1
P 3450 1600
F 0 "#PWR0101" H 3450 1450 50  0001 C CNN
F 1 "+5V" H 3465 1773 50  0000 C CNN
F 2 "" H 3450 1600 50  0001 C CNN
F 3 "" H 3450 1600 50  0001 C CNN
	1    3450 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 1600 3450 1650
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 60A61E45
P 3750 1500
F 0 "#FLG0101" H 3750 1575 50  0001 C CNN
F 1 "PWR_FLAG" H 3750 1673 50  0000 C CNN
F 2 "" H 3750 1500 50  0001 C CNN
F 3 "~" H 3750 1500 50  0001 C CNN
	1    3750 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 1500 3750 1650
Wire Wire Line
	3750 1650 3450 1650
Connection ~ 3450 1650
Wire Wire Line
	3450 1650 3450 1850
$EndSCHEMATC
