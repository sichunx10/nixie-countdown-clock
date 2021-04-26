EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 5
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
L Connector_Generic:Conn_01x01 J1
U 1 1 60925BA8
P 2500 3050
F 0 "J1" V 2600 3100 50  0000 R CNN
F 1 "170V" V 2500 3300 50  0000 R CNN
F 2 "" H 2500 3050 50  0001 C CNN
F 3 "~" H 2500 3050 50  0001 C CNN
	1    2500 3050
	0    -1   -1   0   
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J2
U 1 1 60926684
P 5500 3050
F 0 "J2" V 5600 3100 50  0000 R CNN
F 1 "5V" V 5500 3200 50  0000 R CNN
F 2 "" H 5500 3050 50  0001 C CNN
F 3 "~" H 5500 3050 50  0001 C CNN
	1    5500 3050
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR01
U 1 1 6092B0BF
P 2500 3900
F 0 "#PWR01" H 2500 3650 50  0001 C CNN
F 1 "GND" H 2505 3727 50  0000 C CNN
F 2 "" H 2500 3900 50  0001 C CNN
F 3 "" H 2500 3900 50  0001 C CNN
	1    2500 3900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR03
U 1 1 6092B471
P 5500 3900
F 0 "#PWR03" H 5500 3650 50  0001 C CNN
F 1 "GND" H 5505 3727 50  0000 C CNN
F 2 "" H 5500 3900 50  0001 C CNN
F 3 "" H 5500 3900 50  0001 C CNN
	1    5500 3900
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C1
U 1 1 6092E1DA
P 2500 3600
F 0 "C1" H 2592 3646 50  0000 L CNN
F 1 "10uF" H 2592 3555 50  0000 L CNN
F 2 "" H 2500 3600 50  0001 C CNN
F 3 "~" H 2500 3600 50  0001 C CNN
	1    2500 3600
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C2
U 1 1 60939809
P 5500 3650
F 0 "C2" H 5592 3696 50  0000 L CNN
F 1 "1uF" H 5592 3605 50  0000 L CNN
F 2 "" H 5500 3650 50  0001 C CNN
F 3 "~" H 5500 3650 50  0001 C CNN
	1    5500 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	5500 3750 5500 3900
Wire Wire Line
	5500 3250 5500 3350
$Comp
L power:+5V #PWR04
U 1 1 6093A014
P 5850 2950
F 0 "#PWR04" H 5850 2800 50  0001 C CNN
F 1 "+5V" H 5865 3123 50  0000 C CNN
F 2 "" H 5850 2950 50  0001 C CNN
F 3 "" H 5850 2950 50  0001 C CNN
	1    5850 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	5850 2950 5850 3350
Wire Wire Line
	5850 3350 5500 3350
Connection ~ 5500 3350
Wire Wire Line
	5500 3350 5500 3550
Wire Wire Line
	2500 3700 2500 3900
$Comp
L power:VDD #PWR02
U 1 1 60941447
P 2900 2950
F 0 "#PWR02" H 2900 2800 50  0001 C CNN
F 1 "VDD" H 2915 3123 50  0000 C CNN
F 2 "" H 2900 2950 50  0001 C CNN
F 3 "" H 2900 2950 50  0001 C CNN
	1    2900 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	2900 3350 2500 3350
Connection ~ 2500 3350
Wire Wire Line
	2500 3350 2500 3500
Wire Wire Line
	2500 3250 2500 3350
Wire Wire Line
	2900 2950 2900 3350
Wire Wire Line
	8500 3700 8500 3850
Wire Wire Line
	8500 3300 8500 3500
Connection ~ 8500 3300
Wire Wire Line
	8850 3300 8500 3300
Wire Wire Line
	8850 2900 8850 3300
$Comp
L power:+3.3V #PWR06
U 1 1 60931EFE
P 8850 2900
F 0 "#PWR06" H 8850 2750 50  0001 C CNN
F 1 "+3.3V" H 8865 3073 50  0000 C CNN
F 2 "" H 8850 2900 50  0001 C CNN
F 3 "" H 8850 2900 50  0001 C CNN
	1    8850 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	8500 3200 8500 3300
$Comp
L Device:C_Small C3
U 1 1 6093035A
P 8500 3600
F 0 "C3" H 8592 3646 50  0000 L CNN
F 1 "1uF" H 8592 3555 50  0000 L CNN
F 2 "" H 8500 3600 50  0001 C CNN
F 3 "~" H 8500 3600 50  0001 C CNN
	1    8500 3600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR05
U 1 1 6092B73F
P 8500 3850
F 0 "#PWR05" H 8500 3600 50  0001 C CNN
F 1 "GND" H 8505 3677 50  0000 C CNN
F 2 "" H 8500 3850 50  0001 C CNN
F 3 "" H 8500 3850 50  0001 C CNN
	1    8500 3850
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J3
U 1 1 60926D85
P 8500 3000
F 0 "J3" V 8600 3050 50  0000 R CNN
F 1 "3.3V" V 8500 3250 50  0000 R CNN
F 2 "" H 8500 3000 50  0001 C CNN
F 3 "~" H 8500 3000 50  0001 C CNN
	1    8500 3000
	0    -1   -1   0   
$EndComp
Wire Notes Line
	10400 1500 10400 5000
Wire Notes Line
	1500 5000 1500 1500
Wire Notes Line
	4500 1500 4500 5000
Wire Notes Line
	7500 1500 7500 5000
Wire Notes Line
	1500 1500 10400 1500
Wire Notes Line
	1500 5000 10400 5000
Text Notes 1550 1750 0    79   ~ 0
External high voltage supply for \nExixe driver (IN-14) and INS-1.
Text Notes 4500 1650 0    79   ~ 0
5V supply for microcontroller
Text Notes 7550 1650 0    79   ~ 0
3.3V supply for RTC and atomic clock\n
Text Notes 1500 1450 0    98   ~ 20
All three power inputs are derived from High Voltage Power Supply Unit.
$EndSCHEMATC