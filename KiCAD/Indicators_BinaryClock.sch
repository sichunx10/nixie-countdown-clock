EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 5 5
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
L Device:R_Small R?
U 1 1 60A0A440
P 3250 1100
AR Path="/60A0A440" Ref="R?"  Part="1" 
AR Path="/6091E188/60A0A440" Ref="R14"  Part="1" 
F 0 "R14" H 3309 1146 50  0000 L CNN
F 1 "220k" H 3309 1055 50  0000 L CNN
F 2 "" H 3250 1100 50  0001 C CNN
F 3 "~" H 3250 1100 50  0001 C CNN
	1    3250 1100
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R?
U 1 1 60A0A446
P 3250 2250
AR Path="/60A0A446" Ref="R?"  Part="1" 
AR Path="/6091E188/60A0A446" Ref="R15"  Part="1" 
F 0 "R15" H 3309 2296 50  0000 L CNN
F 1 "220k" H 3309 2205 50  0000 L CNN
F 2 "" H 3250 2250 50  0001 C CNN
F 3 "~" H 3250 2250 50  0001 C CNN
	1    3250 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 2050 3250 2150
Wire Wire Line
	3250 2350 3250 2400
Wire Wire Line
	3250 2750 3250 2850
$Comp
L Device:R_Small R?
U 1 1 60A0A44F
P 4450 1100
AR Path="/60A0A44F" Ref="R?"  Part="1" 
AR Path="/6091E188/60A0A44F" Ref="R16"  Part="1" 
F 0 "R16" H 4509 1146 50  0000 L CNN
F 1 "220k" H 4509 1055 50  0000 L CNN
F 2 "" H 4450 1100 50  0001 C CNN
F 3 "~" H 4450 1100 50  0001 C CNN
	1    4450 1100
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R?
U 1 1 60A0A455
P 4450 2250
AR Path="/60A0A455" Ref="R?"  Part="1" 
AR Path="/6091E188/60A0A455" Ref="R17"  Part="1" 
F 0 "R17" H 4509 2296 50  0000 L CNN
F 1 "220k" H 4509 2205 50  0000 L CNN
F 2 "" H 4450 2250 50  0001 C CNN
F 3 "~" H 4450 2250 50  0001 C CNN
	1    4450 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 2050 4450 2100
Wire Wire Line
	4450 2350 4450 2400
Wire Wire Line
	3250 950  3250 1000
$Comp
L power:GND #PWR?
U 1 1 60A0A45E
P 2900 3500
AR Path="/60A0A45E" Ref="#PWR?"  Part="1" 
AR Path="/6091E188/60A0A45E" Ref="#PWR050"  Part="1" 
F 0 "#PWR050" H 2900 3250 50  0001 C CNN
F 1 "GND" H 2905 3327 50  0000 C CNN
F 2 "" H 2900 3500 50  0001 C CNN
F 3 "" H 2900 3500 50  0001 C CNN
	1    2900 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2900 3500 2900 3400
Wire Wire Line
	2900 3000 2900 2850
Wire Wire Line
	4450 2750 4450 2850
Text GLabel 2150 3200 0    50   Input ~ 0
INS-1
Wire Wire Line
	2900 1750 3250 1750
Wire Wire Line
	2900 2850 3250 2850
Wire Wire Line
	2600 3200 2500 3200
$Comp
L Device:R_Small R?
U 1 1 60A0A471
P 2400 3200
AR Path="/60A0A471" Ref="R?"  Part="1" 
AR Path="/6091E188/60A0A471" Ref="R9"  Part="1" 
F 0 "R9" V 2300 3150 50  0000 L CNN
F 1 "130k" V 2500 3100 50  0000 L CNN
F 2 "" H 2400 3200 50  0001 C CNN
F 3 "~" H 2400 3200 50  0001 C CNN
	1    2400 3200
	0    1    1    0   
$EndComp
Wire Wire Line
	2150 3200 2300 3200
$Comp
L power:VDD #PWR?
U 1 1 60A0A478
P 3250 950
AR Path="/60A0A478" Ref="#PWR?"  Part="1" 
AR Path="/6091E188/60A0A478" Ref="#PWR055"  Part="1" 
F 0 "#PWR055" H 3250 800 50  0001 C CNN
F 1 "VDD" H 3265 1123 50  0000 C CNN
F 2 "" H 3250 950 50  0001 C CNN
F 3 "" H 3250 950 50  0001 C CNN
	1    3250 950 
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR?
U 1 1 60A0A47E
P 4450 900
AR Path="/60A0A47E" Ref="#PWR?"  Part="1" 
AR Path="/6091E188/60A0A47E" Ref="#PWR057"  Part="1" 
F 0 "#PWR057" H 4450 750 50  0001 C CNN
F 1 "VDD" H 4465 1073 50  0000 C CNN
F 2 "" H 4450 900 50  0001 C CNN
F 3 "" H 4450 900 50  0001 C CNN
	1    4450 900 
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR?
U 1 1 60A0A484
P 3250 2050
AR Path="/60A0A484" Ref="#PWR?"  Part="1" 
AR Path="/6091E188/60A0A484" Ref="#PWR056"  Part="1" 
F 0 "#PWR056" H 3250 1900 50  0001 C CNN
F 1 "VDD" H 3265 2223 50  0000 C CNN
F 2 "" H 3250 2050 50  0001 C CNN
F 3 "" H 3250 2050 50  0001 C CNN
	1    3250 2050
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR?
U 1 1 60A0A48A
P 4450 2050
AR Path="/60A0A48A" Ref="#PWR?"  Part="1" 
AR Path="/6091E188/60A0A48A" Ref="#PWR058"  Part="1" 
F 0 "#PWR058" H 4450 1900 50  0001 C CNN
F 1 "VDD" H 4465 2223 50  0000 C CNN
F 2 "" H 4450 2050 50  0001 C CNN
F 3 "" H 4450 2050 50  0001 C CNN
	1    4450 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 1200 3250 1250
$Comp
L power:PWR_FLAG #FLG?
U 1 1 60A0A491
P 3650 1100
AR Path="/60A0A491" Ref="#FLG?"  Part="1" 
AR Path="/6091E188/60A0A491" Ref="#FLG08"  Part="1" 
F 0 "#FLG08" H 3650 1175 50  0001 C CNN
F 1 "PWR_FLAG" H 3650 1273 50  0000 C CNN
F 2 "" H 3650 1100 50  0001 C CNN
F 3 "~" H 3650 1100 50  0001 C CNN
	1    3650 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 1100 3650 1250
Wire Wire Line
	3650 1250 3250 1250
Connection ~ 3250 1250
Wire Wire Line
	3250 1250 3250 1300
$Comp
L power:PWR_FLAG #FLG?
U 1 1 60A0A49B
P 4100 1100
AR Path="/60A0A49B" Ref="#FLG?"  Part="1" 
AR Path="/6091E188/60A0A49B" Ref="#FLG010"  Part="1" 
F 0 "#FLG010" H 4100 1175 50  0001 C CNN
F 1 "PWR_FLAG" H 4100 1273 50  0000 C CNN
F 2 "" H 4100 1100 50  0001 C CNN
F 3 "~" H 4100 1100 50  0001 C CNN
	1    4100 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 1200 4450 1250
Wire Wire Line
	4100 1100 4100 1250
Wire Wire Line
	4100 1250 4450 1250
Connection ~ 4450 1250
Wire Wire Line
	4450 1250 4450 1350
Wire Wire Line
	4450 1650 4450 1750
$Comp
L power:PWR_FLAG #FLG?
U 1 1 60A0A4A7
P 4100 2200
AR Path="/60A0A4A7" Ref="#FLG?"  Part="1" 
AR Path="/6091E188/60A0A4A7" Ref="#FLG011"  Part="1" 
F 0 "#FLG011" H 4100 2275 50  0001 C CNN
F 1 "PWR_FLAG" H 4100 2373 50  0000 C CNN
F 2 "" H 4100 2200 50  0001 C CNN
F 3 "~" H 4100 2200 50  0001 C CNN
	1    4100 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 2200 4100 2400
Wire Wire Line
	4100 2400 4450 2400
Connection ~ 4450 2400
Wire Wire Line
	4450 2400 4450 2450
$Comp
L power:PWR_FLAG #FLG?
U 1 1 60A0A4B1
P 3650 2200
AR Path="/60A0A4B1" Ref="#FLG?"  Part="1" 
AR Path="/6091E188/60A0A4B1" Ref="#FLG09"  Part="1" 
F 0 "#FLG09" H 3650 2275 50  0001 C CNN
F 1 "PWR_FLAG" H 3650 2373 50  0000 C CNN
F 2 "" H 3650 2200 50  0001 C CNN
F 3 "~" H 3650 2200 50  0001 C CNN
	1    3650 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 2200 3650 2400
Wire Wire Line
	3650 2400 3250 2400
Connection ~ 3250 2400
Wire Wire Line
	3250 2400 3250 2450
Wire Wire Line
	4450 900  4450 1000
Wire Wire Line
	2900 1750 2900 2850
Connection ~ 2900 2850
Wire Wire Line
	3250 1600 3250 1750
Wire Wire Line
	3250 1750 4450 1750
Connection ~ 3250 1750
Wire Wire Line
	3250 2850 4450 2850
Connection ~ 3250 2850
$Comp
L Nixie_KiCAD_Library:INS-1 D?
U 1 1 60A0A4C3
P 3250 2600
AR Path="/60A0A4C3" Ref="D?"  Part="1" 
AR Path="/6091E188/60A0A4C3" Ref="D2"  Part="1" 
F 0 "D2" H 3328 2646 50  0000 L CNN
F 1 "INS-1" H 3328 2555 50  0000 L CNN
F 2 "" H 3250 2600 50  0001 C CNN
F 3 "" H 3250 2600 50  0001 C CNN
	1    3250 2600
	1    0    0    -1  
$EndComp
$Comp
L Nixie_KiCAD_Library:INS-1 D?
U 1 1 60A0A4C9
P 3250 1450
AR Path="/60A0A4C9" Ref="D?"  Part="1" 
AR Path="/6091E188/60A0A4C9" Ref="D1"  Part="1" 
F 0 "D1" H 3328 1496 50  0000 L CNN
F 1 "INS-1" H 3328 1405 50  0000 L CNN
F 2 "" H 3250 1450 50  0001 C CNN
F 3 "" H 3250 1450 50  0001 C CNN
	1    3250 1450
	1    0    0    -1  
$EndComp
$Comp
L Nixie_KiCAD_Library:INS-1 D?
U 1 1 60A0A4CF
P 4450 1500
AR Path="/60A0A4CF" Ref="D?"  Part="1" 
AR Path="/6091E188/60A0A4CF" Ref="D7"  Part="1" 
F 0 "D7" H 4528 1546 50  0000 L CNN
F 1 "INS-1" H 4528 1455 50  0000 L CNN
F 2 "" H 4450 1500 50  0001 C CNN
F 3 "" H 4450 1500 50  0001 C CNN
	1    4450 1500
	1    0    0    -1  
$EndComp
$Comp
L Nixie_KiCAD_Library:INS-1 D?
U 1 1 60A0A4D5
P 4450 2600
AR Path="/60A0A4D5" Ref="D?"  Part="1" 
AR Path="/6091E188/60A0A4D5" Ref="D8"  Part="1" 
F 0 "D8" H 4528 2646 50  0000 L CNN
F 1 "INS-1" H 4528 2555 50  0000 L CNN
F 2 "" H 4450 2600 50  0001 C CNN
F 3 "" H 4450 2600 50  0001 C CNN
	1    4450 2600
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG?
U 1 1 60A0A4DB
P 2900 1600
AR Path="/60A0A4DB" Ref="#FLG?"  Part="1" 
AR Path="/6091E188/60A0A4DB" Ref="#FLG07"  Part="1" 
F 0 "#FLG07" H 2900 1675 50  0001 C CNN
F 1 "PWR_FLAG" H 2900 1773 50  0000 C CNN
F 2 "" H 2900 1600 50  0001 C CNN
F 3 "~" H 2900 1600 50  0001 C CNN
	1    2900 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2900 1600 2900 1750
Connection ~ 2900 1750
$Comp
L Nixie_KiCAD_Library:INS-1 D?
U 1 1 60A0A4E3
P 9200 2250
AR Path="/60A0A4E3" Ref="D?"  Part="1" 
AR Path="/6091E188/60A0A4E3" Ref="D13"  Part="1" 
F 0 "D13" V 9100 2200 50  0000 L CNN
F 1 "INS-1" V 9300 2200 50  0000 L CNN
F 2 "" H 9200 2250 50  0001 C CNN
F 3 "" H 9200 2250 50  0001 C CNN
	1    9200 2250
	0    1    1    0   
$EndComp
$Comp
L Nixie_KiCAD_Library:INS-1 D?
U 1 1 60A0A4E9
P 7550 2250
AR Path="/60A0A4E9" Ref="D?"  Part="1" 
AR Path="/6091E188/60A0A4E9" Ref="D9"  Part="1" 
F 0 "D9" V 7450 2200 50  0000 L CNN
F 1 "INS-1" V 7650 2200 50  0000 L CNN
F 2 "" H 7550 2250 50  0001 C CNN
F 3 "" H 7550 2250 50  0001 C CNN
	1    7550 2250
	0    1    1    0   
$EndComp
$Comp
L Nixie_KiCAD_Library:INS-1 D?
U 1 1 60A0A4EF
P 7550 4750
AR Path="/60A0A4EF" Ref="D?"  Part="1" 
AR Path="/6091E188/60A0A4EF" Ref="D12"  Part="1" 
F 0 "D12" V 7450 4700 50  0000 L CNN
F 1 "INS-1" V 7650 4700 50  0000 L CNN
F 2 "" H 7550 4750 50  0001 C CNN
F 3 "" H 7550 4750 50  0001 C CNN
	1    7550 4750
	0    1    1    0   
$EndComp
$Comp
L Nixie_KiCAD_Library:INS-1 D?
U 1 1 60A0A4F5
P 7550 3950
AR Path="/60A0A4F5" Ref="D?"  Part="1" 
AR Path="/6091E188/60A0A4F5" Ref="D11"  Part="1" 
F 0 "D11" V 7450 3900 50  0000 L CNN
F 1 "INS-1" V 7650 3900 50  0000 L CNN
F 2 "" H 7550 3950 50  0001 C CNN
F 3 "" H 7550 3950 50  0001 C CNN
	1    7550 3950
	0    1    1    0   
$EndComp
$Comp
L Nixie_KiCAD_Library:INS-1 D?
U 1 1 60A0A4FB
P 9200 3050
AR Path="/60A0A4FB" Ref="D?"  Part="1" 
AR Path="/6091E188/60A0A4FB" Ref="D14"  Part="1" 
F 0 "D14" V 9100 3000 50  0000 L CNN
F 1 "INS-1" V 9300 3000 50  0000 L CNN
F 2 "" H 9200 3050 50  0001 C CNN
F 3 "" H 9200 3050 50  0001 C CNN
	1    9200 3050
	0    1    1    0   
$EndComp
$Comp
L Nixie_KiCAD_Library:INS-1 D?
U 1 1 60A0A501
P 7550 3050
AR Path="/60A0A501" Ref="D?"  Part="1" 
AR Path="/6091E188/60A0A501" Ref="D10"  Part="1" 
F 0 "D10" V 7450 3000 50  0000 L CNN
F 1 "INS-1" V 7650 3000 50  0000 L CNN
F 2 "" H 7550 3050 50  0001 C CNN
F 3 "" H 7550 3050 50  0001 C CNN
	1    7550 3050
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R?
U 1 1 60A0A50D
P 8550 2500
AR Path="/60A0A50D" Ref="R?"  Part="1" 
AR Path="/6091E188/60A0A50D" Ref="R26"  Part="1" 
F 0 "R26" V 8450 2450 50  0000 L CNN
F 1 "130k" V 8650 2400 50  0000 L CNN
F 2 "" H 8550 2500 50  0001 C CNN
F 3 "~" H 8550 2500 50  0001 C CNN
	1    8550 2500
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60A0A513
P 9000 2750
AR Path="/60A0A513" Ref="#PWR?"  Part="1" 
AR Path="/6091E188/60A0A513" Ref="#PWR064"  Part="1" 
F 0 "#PWR064" H 9000 2500 50  0001 C CNN
F 1 "GND" H 9005 2577 50  0000 C CNN
F 2 "" H 9000 2750 50  0001 C CNN
F 3 "" H 9000 2750 50  0001 C CNN
	1    9000 2750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60A0A51F
P 9000 3550
AR Path="/60A0A51F" Ref="#PWR?"  Part="1" 
AR Path="/6091E188/60A0A51F" Ref="#PWR065"  Part="1" 
F 0 "#PWR065" H 9000 3300 50  0001 C CNN
F 1 "GND" H 9005 3377 50  0000 C CNN
F 2 "" H 9000 3550 50  0001 C CNN
F 3 "" H 9000 3550 50  0001 C CNN
	1    9000 3550
	1    0    0    -1  
$EndComp
$Comp
L Nixie_KiCAD_Library:INS-1 D?
U 1 1 60A0A525
P 9200 3950
AR Path="/60A0A525" Ref="D?"  Part="1" 
AR Path="/6091E188/60A0A525" Ref="D15"  Part="1" 
F 0 "D15" V 9100 3900 50  0000 L CNN
F 1 "INS-1" V 9300 3900 50  0000 L CNN
F 2 "" H 9200 3950 50  0001 C CNN
F 3 "" H 9200 3950 50  0001 C CNN
	1    9200 3950
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60A0A531
P 9000 4450
AR Path="/60A0A531" Ref="#PWR?"  Part="1" 
AR Path="/6091E188/60A0A531" Ref="#PWR066"  Part="1" 
F 0 "#PWR066" H 9000 4200 50  0001 C CNN
F 1 "GND" H 9005 4277 50  0000 C CNN
F 2 "" H 9000 4450 50  0001 C CNN
F 3 "" H 9000 4450 50  0001 C CNN
	1    9000 4450
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R?
U 1 1 60A0A537
P 8550 3300
AR Path="/60A0A537" Ref="R?"  Part="1" 
AR Path="/6091E188/60A0A537" Ref="R27"  Part="1" 
F 0 "R27" V 8450 3250 50  0000 L CNN
F 1 "130k" V 8650 3200 50  0000 L CNN
F 2 "" H 8550 3300 50  0001 C CNN
F 3 "~" H 8550 3300 50  0001 C CNN
	1    8550 3300
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R?
U 1 1 60A0A53D
P 8550 4200
AR Path="/60A0A53D" Ref="R?"  Part="1" 
AR Path="/6091E188/60A0A53D" Ref="R28"  Part="1" 
F 0 "R28" V 8450 4150 50  0000 L CNN
F 1 "130k" V 8650 4100 50  0000 L CNN
F 2 "" H 8550 4200 50  0001 C CNN
F 3 "~" H 8550 4200 50  0001 C CNN
	1    8550 4200
	0    1    1    0   
$EndComp
$Comp
L Nixie_KiCAD_Library:INS-1 D?
U 1 1 60A0A543
P 9200 4750
AR Path="/60A0A543" Ref="D?"  Part="1" 
AR Path="/6091E188/60A0A543" Ref="D16"  Part="1" 
F 0 "D16" V 9100 4700 50  0000 L CNN
F 1 "INS-1" V 9300 4700 50  0000 L CNN
F 2 "" H 9200 4750 50  0001 C CNN
F 3 "" H 9200 4750 50  0001 C CNN
	1    9200 4750
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60A0A54F
P 9000 5250
AR Path="/60A0A54F" Ref="#PWR?"  Part="1" 
AR Path="/6091E188/60A0A54F" Ref="#PWR067"  Part="1" 
F 0 "#PWR067" H 9000 5000 50  0001 C CNN
F 1 "GND" H 9005 5077 50  0000 C CNN
F 2 "" H 9000 5250 50  0001 C CNN
F 3 "" H 9000 5250 50  0001 C CNN
	1    9000 5250
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R?
U 1 1 60A0A555
P 8550 5000
AR Path="/60A0A555" Ref="R?"  Part="1" 
AR Path="/6091E188/60A0A555" Ref="R29"  Part="1" 
F 0 "R29" V 8450 4950 50  0000 L CNN
F 1 "130k" V 8650 4900 50  0000 L CNN
F 2 "" H 8550 5000 50  0001 C CNN
F 3 "~" H 8550 5000 50  0001 C CNN
	1    8550 5000
	0    1    1    0   
$EndComp
Wire Wire Line
	9000 2300 9000 2250
Wire Wire Line
	9000 2250 9050 2250
Wire Wire Line
	8700 2500 8650 2500
Wire Wire Line
	9000 2700 9000 2750
Wire Wire Line
	9000 3100 9000 3050
Wire Wire Line
	9000 3050 9050 3050
Wire Wire Line
	8700 3300 8650 3300
Wire Wire Line
	9000 3500 9000 3550
Wire Wire Line
	9050 3950 9000 3950
Wire Wire Line
	9000 3950 9000 4000
Wire Wire Line
	8700 4200 8650 4200
Wire Wire Line
	9000 4400 9000 4450
Wire Wire Line
	9050 4750 9000 4750
Wire Wire Line
	9000 4750 9000 4800
Wire Wire Line
	8700 5000 8650 5000
Wire Wire Line
	9000 5200 9000 5250
$Comp
L power:GND #PWR?
U 1 1 60A0A571
P 7350 5250
AR Path="/60A0A571" Ref="#PWR?"  Part="1" 
AR Path="/6091E188/60A0A571" Ref="#PWR063"  Part="1" 
F 0 "#PWR063" H 7350 5000 50  0001 C CNN
F 1 "GND" H 7355 5077 50  0000 C CNN
F 2 "" H 7350 5250 50  0001 C CNN
F 3 "" H 7350 5250 50  0001 C CNN
	1    7350 5250
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R?
U 1 1 60A0A577
P 6900 5000
AR Path="/60A0A577" Ref="R?"  Part="1" 
AR Path="/6091E188/60A0A577" Ref="R21"  Part="1" 
F 0 "R21" V 6800 4950 50  0000 L CNN
F 1 "130k" V 7000 4900 50  0000 L CNN
F 2 "" H 6900 5000 50  0001 C CNN
F 3 "~" H 6900 5000 50  0001 C CNN
	1    6900 5000
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R?
U 1 1 60A0A58F
P 6900 4200
AR Path="/60A0A58F" Ref="R?"  Part="1" 
AR Path="/6091E188/60A0A58F" Ref="R20"  Part="1" 
F 0 "R20" V 6800 4150 50  0000 L CNN
F 1 "130k" V 7000 4100 50  0000 L CNN
F 2 "" H 6900 4200 50  0001 C CNN
F 3 "~" H 6900 4200 50  0001 C CNN
	1    6900 4200
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R?
U 1 1 60A0A595
P 6900 3300
AR Path="/60A0A595" Ref="R?"  Part="1" 
AR Path="/6091E188/60A0A595" Ref="R19"  Part="1" 
F 0 "R19" V 6800 3250 50  0000 L CNN
F 1 "130k" V 7000 3200 50  0000 L CNN
F 2 "" H 6900 3300 50  0001 C CNN
F 3 "~" H 6900 3300 50  0001 C CNN
	1    6900 3300
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R?
U 1 1 60A0A59B
P 6900 2500
AR Path="/60A0A59B" Ref="R?"  Part="1" 
AR Path="/6091E188/60A0A59B" Ref="R18"  Part="1" 
F 0 "R18" V 6800 2450 50  0000 L CNN
F 1 "130k" V 7000 2400 50  0000 L CNN
F 2 "" H 6900 2500 50  0001 C CNN
F 3 "~" H 6900 2500 50  0001 C CNN
	1    6900 2500
	0    1    1    0   
$EndComp
Wire Wire Line
	7400 2250 7350 2250
Wire Wire Line
	7350 2250 7350 2300
Wire Wire Line
	7050 2500 7000 2500
$Comp
L power:GND #PWR?
U 1 1 60A0A5A4
P 7350 2750
AR Path="/60A0A5A4" Ref="#PWR?"  Part="1" 
AR Path="/6091E188/60A0A5A4" Ref="#PWR060"  Part="1" 
F 0 "#PWR060" H 7350 2500 50  0001 C CNN
F 1 "GND" H 7355 2577 50  0000 C CNN
F 2 "" H 7350 2750 50  0001 C CNN
F 3 "" H 7350 2750 50  0001 C CNN
	1    7350 2750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60A0A5AA
P 7350 3550
AR Path="/60A0A5AA" Ref="#PWR?"  Part="1" 
AR Path="/6091E188/60A0A5AA" Ref="#PWR061"  Part="1" 
F 0 "#PWR061" H 7350 3300 50  0001 C CNN
F 1 "GND" H 7355 3377 50  0000 C CNN
F 2 "" H 7350 3550 50  0001 C CNN
F 3 "" H 7350 3550 50  0001 C CNN
	1    7350 3550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60A0A5B0
P 7350 4450
AR Path="/60A0A5B0" Ref="#PWR?"  Part="1" 
AR Path="/6091E188/60A0A5B0" Ref="#PWR062"  Part="1" 
F 0 "#PWR062" H 7350 4200 50  0001 C CNN
F 1 "GND" H 7355 4277 50  0000 C CNN
F 2 "" H 7350 4450 50  0001 C CNN
F 3 "" H 7350 4450 50  0001 C CNN
	1    7350 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	7350 2700 7350 2750
Wire Wire Line
	7400 3050 7350 3050
Wire Wire Line
	7350 3050 7350 3100
Wire Wire Line
	7050 3300 7000 3300
Wire Wire Line
	7350 3500 7350 3550
Wire Wire Line
	7400 3950 7350 3950
Wire Wire Line
	7350 3950 7350 4000
Wire Wire Line
	7050 4200 7000 4200
Wire Wire Line
	7350 5200 7350 5250
$Comp
L power:VDD #PWR?
U 1 1 60A0A5BF
P 10050 3500
AR Path="/60A0A5BF" Ref="#PWR?"  Part="1" 
AR Path="/6091E188/60A0A5BF" Ref="#PWR068"  Part="1" 
F 0 "#PWR068" H 10050 3350 50  0001 C CNN
F 1 "VDD" H 10065 3673 50  0000 C CNN
F 2 "" H 10050 3500 50  0001 C CNN
F 3 "" H 10050 3500 50  0001 C CNN
	1    10050 3500
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R?
U 1 1 60A0A5C5
P 9700 2250
AR Path="/60A0A5C5" Ref="R?"  Part="1" 
AR Path="/6091E188/60A0A5C5" Ref="R30"  Part="1" 
F 0 "R30" V 9600 2150 50  0000 L CNN
F 1 "220k" V 9800 2150 50  0000 L CNN
F 2 "" H 9700 2250 50  0001 C CNN
F 3 "~" H 9700 2250 50  0001 C CNN
	1    9700 2250
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R?
U 1 1 60A0A5CB
P 9700 3050
AR Path="/60A0A5CB" Ref="R?"  Part="1" 
AR Path="/6091E188/60A0A5CB" Ref="R31"  Part="1" 
F 0 "R31" V 9600 2950 50  0000 L CNN
F 1 "220k" V 9800 2950 50  0000 L CNN
F 2 "" H 9700 3050 50  0001 C CNN
F 3 "~" H 9700 3050 50  0001 C CNN
	1    9700 3050
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R?
U 1 1 60A0A5D1
P 9700 3950
AR Path="/60A0A5D1" Ref="R?"  Part="1" 
AR Path="/6091E188/60A0A5D1" Ref="R32"  Part="1" 
F 0 "R32" V 9600 3850 50  0000 L CNN
F 1 "220k" V 9800 3850 50  0000 L CNN
F 2 "" H 9700 3950 50  0001 C CNN
F 3 "~" H 9700 3950 50  0001 C CNN
	1    9700 3950
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R?
U 1 1 60A0A5D7
P 9700 4750
AR Path="/60A0A5D7" Ref="R?"  Part="1" 
AR Path="/6091E188/60A0A5D7" Ref="R33"  Part="1" 
F 0 "R33" V 9600 4650 50  0000 L CNN
F 1 "220k" V 9800 4650 50  0000 L CNN
F 2 "" H 9700 4750 50  0001 C CNN
F 3 "~" H 9700 4750 50  0001 C CNN
	1    9700 4750
	0    1    1    0   
$EndComp
Wire Wire Line
	9800 4750 9850 4750
Wire Wire Line
	9850 4750 9850 3950
Wire Wire Line
	9800 3950 9850 3950
Connection ~ 9850 3950
Wire Wire Line
	9800 3050 9850 3050
Wire Wire Line
	9850 3050 9850 3500
Wire Wire Line
	9800 2250 9850 2250
Wire Wire Line
	9850 2250 9850 3050
Wire Wire Line
	9850 3500 10050 3500
Connection ~ 9850 3050
Connection ~ 9850 3500
Wire Wire Line
	9600 4750 9550 4750
Wire Wire Line
	9600 3950 9550 3950
Wire Wire Line
	9600 3050 9550 3050
Wire Wire Line
	9600 2250 9550 2250
$Comp
L Device:R_Small R?
U 1 1 60A0A5EC
P 7900 2250
AR Path="/60A0A5EC" Ref="R?"  Part="1" 
AR Path="/6091E188/60A0A5EC" Ref="R22"  Part="1" 
F 0 "R22" V 7800 2150 50  0000 L CNN
F 1 "220k" V 8000 2150 50  0000 L CNN
F 2 "" H 7900 2250 50  0001 C CNN
F 3 "~" H 7900 2250 50  0001 C CNN
	1    7900 2250
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R?
U 1 1 60A0A5F2
P 7900 3050
AR Path="/60A0A5F2" Ref="R?"  Part="1" 
AR Path="/6091E188/60A0A5F2" Ref="R23"  Part="1" 
F 0 "R23" V 7800 2950 50  0000 L CNN
F 1 "220k" V 8000 2950 50  0000 L CNN
F 2 "" H 7900 3050 50  0001 C CNN
F 3 "~" H 7900 3050 50  0001 C CNN
	1    7900 3050
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R?
U 1 1 60A0A5F8
P 7900 3950
AR Path="/60A0A5F8" Ref="R?"  Part="1" 
AR Path="/6091E188/60A0A5F8" Ref="R24"  Part="1" 
F 0 "R24" V 7800 3850 50  0000 L CNN
F 1 "220k" V 8000 3850 50  0000 L CNN
F 2 "" H 7900 3950 50  0001 C CNN
F 3 "~" H 7900 3950 50  0001 C CNN
	1    7900 3950
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R?
U 1 1 60A0A5FE
P 7900 4750
AR Path="/60A0A5FE" Ref="R?"  Part="1" 
AR Path="/6091E188/60A0A5FE" Ref="R25"  Part="1" 
F 0 "R25" V 7800 4650 50  0000 L CNN
F 1 "220k" V 8000 4650 50  0000 L CNN
F 2 "" H 7900 4750 50  0001 C CNN
F 3 "~" H 7900 4750 50  0001 C CNN
	1    7900 4750
	0    1    1    0   
$EndComp
Wire Wire Line
	7050 5000 7000 5000
Wire Wire Line
	7350 4800 7350 4750
Wire Wire Line
	7350 4750 7400 4750
Wire Wire Line
	7700 4750 7750 4750
Wire Wire Line
	7700 3950 7750 3950
Wire Wire Line
	7700 2250 7750 2250
Wire Wire Line
	8000 2250 8050 2250
Wire Wire Line
	8050 2250 8050 3050
Wire Wire Line
	8050 3050 8000 3050
Wire Wire Line
	8000 3950 8050 3950
Connection ~ 8050 3050
Wire Wire Line
	8000 4750 8050 4750
Wire Wire Line
	8050 4750 8050 3950
Connection ~ 8050 3950
Wire Wire Line
	9850 3500 9850 3950
Wire Wire Line
	8050 3050 8050 3800
Wire Wire Line
	9850 3500 9300 3500
Wire Wire Line
	9300 3500 9300 3800
Wire Wire Line
	9300 3800 8050 3800
Connection ~ 8050 3800
Wire Wire Line
	8050 3800 8050 3950
Text GLabel 8450 2500 0    50   Input ~ 0
BN_CLK0
Text GLabel 8450 3300 0    50   Input ~ 0
BN_CLK1
Text GLabel 8450 4200 0    50   Input ~ 0
BN_CLK2
Text GLabel 6800 2500 0    50   Input ~ 0
BN_CLK4
Text GLabel 8450 5000 0    50   Input ~ 0
BN_CLK3
Text GLabel 6800 3300 0    50   Input ~ 0
BN_CLK5
Text GLabel 6800 4200 0    50   Input ~ 0
BN_CLK6
Text GLabel 6800 5000 0    50   Input ~ 0
BN_CLK7
$Comp
L power:PWR_FLAG #FLG?
U 1 1 60A0A621
P 9550 2450
AR Path="/60A0A621" Ref="#FLG?"  Part="1" 
AR Path="/6091E188/60A0A621" Ref="#FLG024"  Part="1" 
F 0 "#FLG024" H 9550 2525 50  0001 C CNN
F 1 "PWR_FLAG" H 9550 2623 50  0000 C CNN
F 2 "" H 9550 2450 50  0001 C CNN
F 3 "~" H 9550 2450 50  0001 C CNN
	1    9550 2450
	-1   0    0    1   
$EndComp
$Comp
L power:PWR_FLAG #FLG?
U 1 1 60A0A627
P 9550 2900
AR Path="/60A0A627" Ref="#FLG?"  Part="1" 
AR Path="/6091E188/60A0A627" Ref="#FLG025"  Part="1" 
F 0 "#FLG025" H 9550 2975 50  0001 C CNN
F 1 "PWR_FLAG" H 9550 3073 50  0000 C CNN
F 2 "" H 9550 2900 50  0001 C CNN
F 3 "~" H 9550 2900 50  0001 C CNN
	1    9550 2900
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG?
U 1 1 60A0A62D
P 9550 4150
AR Path="/60A0A62D" Ref="#FLG?"  Part="1" 
AR Path="/6091E188/60A0A62D" Ref="#FLG026"  Part="1" 
F 0 "#FLG026" H 9550 4225 50  0001 C CNN
F 1 "PWR_FLAG" H 9550 4323 50  0000 C CNN
F 2 "" H 9550 4150 50  0001 C CNN
F 3 "~" H 9550 4150 50  0001 C CNN
	1    9550 4150
	-1   0    0    1   
$EndComp
$Comp
L power:PWR_FLAG #FLG?
U 1 1 60A0A633
P 9550 4600
AR Path="/60A0A633" Ref="#FLG?"  Part="1" 
AR Path="/6091E188/60A0A633" Ref="#FLG027"  Part="1" 
F 0 "#FLG027" H 9550 4675 50  0001 C CNN
F 1 "PWR_FLAG" H 9550 4773 50  0000 C CNN
F 2 "" H 9550 4600 50  0001 C CNN
F 3 "~" H 9550 4600 50  0001 C CNN
	1    9550 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	9550 2450 9550 2250
Connection ~ 9550 2250
Wire Wire Line
	9550 2250 9350 2250
Wire Wire Line
	9550 2900 9550 3050
Connection ~ 9550 3050
Wire Wire Line
	9550 3050 9350 3050
Wire Wire Line
	9550 4150 9550 3950
Connection ~ 9550 3950
Wire Wire Line
	9550 3950 9350 3950
Wire Wire Line
	9550 4600 9550 4750
Connection ~ 9550 4750
Wire Wire Line
	9550 4750 9350 4750
$Comp
L power:PWR_FLAG #FLG?
U 1 1 60A0A645
P 8300 2650
AR Path="/60A0A645" Ref="#FLG?"  Part="1" 
AR Path="/6091E188/60A0A645" Ref="#FLG018"  Part="1" 
F 0 "#FLG018" H 8300 2725 50  0001 C CNN
F 1 "PWR_FLAG" H 8300 2823 50  0000 C CNN
F 2 "" H 8300 2650 50  0001 C CNN
F 3 "~" H 8300 2650 50  0001 C CNN
	1    8300 2650
	-1   0    0    1   
$EndComp
$Comp
L power:PWR_FLAG #FLG?
U 1 1 60A0A64B
P 7750 2900
AR Path="/60A0A64B" Ref="#FLG?"  Part="1" 
AR Path="/6091E188/60A0A64B" Ref="#FLG016"  Part="1" 
F 0 "#FLG016" H 7750 2975 50  0001 C CNN
F 1 "PWR_FLAG" H 7750 3073 50  0000 C CNN
F 2 "" H 7750 2900 50  0001 C CNN
F 3 "~" H 7750 2900 50  0001 C CNN
	1    7750 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	7750 2250 7750 2600
Wire Wire Line
	7750 2600 8300 2600
Wire Wire Line
	8300 2600 8300 2650
Connection ~ 7750 2250
Wire Wire Line
	7750 2250 7800 2250
$Comp
L power:PWR_FLAG #FLG?
U 1 1 60A0A656
P 8300 4350
AR Path="/60A0A656" Ref="#FLG?"  Part="1" 
AR Path="/6091E188/60A0A656" Ref="#FLG019"  Part="1" 
F 0 "#FLG019" H 8300 4425 50  0001 C CNN
F 1 "PWR_FLAG" H 8300 4523 50  0000 C CNN
F 2 "" H 8300 4350 50  0001 C CNN
F 3 "~" H 8300 4350 50  0001 C CNN
	1    8300 4350
	-1   0    0    1   
$EndComp
Wire Wire Line
	7700 3050 7750 3050
Wire Wire Line
	7750 3050 7750 2900
Connection ~ 7750 3050
Wire Wire Line
	7750 3050 7800 3050
$Comp
L power:PWR_FLAG #FLG?
U 1 1 60A0A660
P 7750 4550
AR Path="/60A0A660" Ref="#FLG?"  Part="1" 
AR Path="/6091E188/60A0A660" Ref="#FLG017"  Part="1" 
F 0 "#FLG017" H 7750 4625 50  0001 C CNN
F 1 "PWR_FLAG" H 7750 4723 50  0000 C CNN
F 2 "" H 7750 4550 50  0001 C CNN
F 3 "~" H 7750 4550 50  0001 C CNN
	1    7750 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	7750 4550 7750 4750
Connection ~ 7750 4750
Wire Wire Line
	7750 4750 7800 4750
Wire Wire Line
	8300 4350 8300 4300
Wire Wire Line
	8300 4300 7750 4300
Wire Wire Line
	7750 4300 7750 3950
Connection ~ 7750 3950
Wire Wire Line
	7750 3950 7800 3950
$Comp
L power:PWR_FLAG #FLG?
U 1 1 60A0A66E
P 7150 4750
AR Path="/60A0A66E" Ref="#FLG?"  Part="1" 
AR Path="/6091E188/60A0A66E" Ref="#FLG015"  Part="1" 
F 0 "#FLG015" H 7150 4825 50  0001 C CNN
F 1 "PWR_FLAG" V 7150 4877 50  0000 L CNN
F 2 "" H 7150 4750 50  0001 C CNN
F 3 "~" H 7150 4750 50  0001 C CNN
	1    7150 4750
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7150 4750 7350 4750
Connection ~ 7350 4750
$Comp
L power:PWR_FLAG #FLG?
U 1 1 60A0A676
P 7100 3950
AR Path="/60A0A676" Ref="#FLG?"  Part="1" 
AR Path="/6091E188/60A0A676" Ref="#FLG014"  Part="1" 
F 0 "#FLG014" H 7100 4025 50  0001 C CNN
F 1 "PWR_FLAG" V 7100 4077 50  0000 L CNN
F 2 "" H 7100 3950 50  0001 C CNN
F 3 "~" H 7100 3950 50  0001 C CNN
	1    7100 3950
	0    -1   -1   0   
$EndComp
$Comp
L power:PWR_FLAG #FLG?
U 1 1 60A0A67C
P 7100 3050
AR Path="/60A0A67C" Ref="#FLG?"  Part="1" 
AR Path="/6091E188/60A0A67C" Ref="#FLG013"  Part="1" 
F 0 "#FLG013" H 7100 3125 50  0001 C CNN
F 1 "PWR_FLAG" V 7100 3177 50  0000 L CNN
F 2 "" H 7100 3050 50  0001 C CNN
F 3 "~" H 7100 3050 50  0001 C CNN
	1    7100 3050
	0    -1   -1   0   
$EndComp
$Comp
L power:PWR_FLAG #FLG?
U 1 1 60A0A682
P 7050 2250
AR Path="/60A0A682" Ref="#FLG?"  Part="1" 
AR Path="/6091E188/60A0A682" Ref="#FLG012"  Part="1" 
F 0 "#FLG012" H 7050 2325 50  0001 C CNN
F 1 "PWR_FLAG" V 7050 2377 50  0000 L CNN
F 2 "" H 7050 2250 50  0001 C CNN
F 3 "~" H 7050 2250 50  0001 C CNN
	1    7050 2250
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7050 2250 7350 2250
Connection ~ 7350 2250
Wire Wire Line
	7100 3050 7350 3050
Connection ~ 7350 3050
Wire Wire Line
	7100 3950 7350 3950
Connection ~ 7350 3950
$Comp
L power:PWR_FLAG #FLG?
U 1 1 60A0A68E
P 8900 2250
AR Path="/60A0A68E" Ref="#FLG?"  Part="1" 
AR Path="/6091E188/60A0A68E" Ref="#FLG020"  Part="1" 
F 0 "#FLG020" H 8900 2325 50  0001 C CNN
F 1 "PWR_FLAG" V 8900 2377 50  0000 L CNN
F 2 "" H 8900 2250 50  0001 C CNN
F 3 "~" H 8900 2250 50  0001 C CNN
	1    8900 2250
	0    -1   -1   0   
$EndComp
$Comp
L power:PWR_FLAG #FLG?
U 1 1 60A0A694
P 8900 3050
AR Path="/60A0A694" Ref="#FLG?"  Part="1" 
AR Path="/6091E188/60A0A694" Ref="#FLG021"  Part="1" 
F 0 "#FLG021" H 8900 3125 50  0001 C CNN
F 1 "PWR_FLAG" V 8900 3177 50  0000 L CNN
F 2 "" H 8900 3050 50  0001 C CNN
F 3 "~" H 8900 3050 50  0001 C CNN
	1    8900 3050
	0    -1   -1   0   
$EndComp
$Comp
L power:PWR_FLAG #FLG?
U 1 1 60A0A69A
P 8900 3950
AR Path="/60A0A69A" Ref="#FLG?"  Part="1" 
AR Path="/6091E188/60A0A69A" Ref="#FLG022"  Part="1" 
F 0 "#FLG022" H 8900 4025 50  0001 C CNN
F 1 "PWR_FLAG" V 8900 4077 50  0000 L CNN
F 2 "" H 8900 3950 50  0001 C CNN
F 3 "~" H 8900 3950 50  0001 C CNN
	1    8900 3950
	0    -1   -1   0   
$EndComp
$Comp
L power:PWR_FLAG #FLG?
U 1 1 60A0A6A0
P 8900 4750
AR Path="/60A0A6A0" Ref="#FLG?"  Part="1" 
AR Path="/6091E188/60A0A6A0" Ref="#FLG023"  Part="1" 
F 0 "#FLG023" H 8900 4825 50  0001 C CNN
F 1 "PWR_FLAG" V 8900 4877 50  0000 L CNN
F 2 "" H 8900 4750 50  0001 C CNN
F 3 "~" H 8900 4750 50  0001 C CNN
	1    8900 4750
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8900 4750 9000 4750
Connection ~ 9000 4750
Wire Wire Line
	8900 3950 9000 3950
Connection ~ 9000 3950
Wire Wire Line
	8900 3050 9000 3050
Connection ~ 9000 3050
Wire Wire Line
	8900 2250 9000 2250
Connection ~ 9000 2250
Wire Wire Line
	7350 4450 7350 4400
$Comp
L dk_Transistors-Bipolar-BJT-Single:ZTX450 Q1
U 1 1 60A80B85
P 2800 3200
F 0 "Q1" H 2988 3253 60  0000 L CNN
F 1 "ZTX457" H 2988 3147 60  0000 L CNN
F 2 "digikey-footprints:TO-92-3" H 3000 3400 60  0001 L CNN
F 3 "https://www.diodes.com/assets/Datasheets/ZTX450.pdf" H 3000 3500 60  0001 L CNN
F 4 "ZTX450-ND" H 3000 3600 60  0001 L CNN "Digi-Key_PN"
F 5 "ZTX450" H 3000 3700 60  0001 L CNN "MPN"
F 6 "Discrete Semiconductor Products" H 3000 3800 60  0001 L CNN "Category"
F 7 "Transistors - Bipolar (BJT) - Single" H 3000 3900 60  0001 L CNN "Family"
F 8 "https://www.diodes.com/assets/Datasheets/ZTX450.pdf" H 3000 4000 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/diodes-incorporated/ZTX450/ZTX450-ND/92530" H 3000 4100 60  0001 L CNN "DK_Detail_Page"
F 10 "TRANS NPN 45V 1A E-LINE" H 3000 4200 60  0001 L CNN "Description"
F 11 "Diodes Incorporated" H 3000 4300 60  0001 L CNN "Manufacturer"
F 12 "Active" H 3000 4400 60  0001 L CNN "Status"
	1    2800 3200
	1    0    0    -1  
$EndComp
$Comp
L dk_Transistors-Bipolar-BJT-Single:ZTX450 Q10
U 1 1 60ABC982
P 8900 2500
F 0 "Q10" H 9088 2553 60  0000 L CNN
F 1 "ZTX457" H 9088 2447 60  0000 L CNN
F 2 "digikey-footprints:TO-92-3" H 9100 2700 60  0001 L CNN
F 3 "https://www.diodes.com/assets/Datasheets/ZTX450.pdf" H 9100 2800 60  0001 L CNN
F 4 "ZTX450-ND" H 9100 2900 60  0001 L CNN "Digi-Key_PN"
F 5 "ZTX450" H 9100 3000 60  0001 L CNN "MPN"
F 6 "Discrete Semiconductor Products" H 9100 3100 60  0001 L CNN "Category"
F 7 "Transistors - Bipolar (BJT) - Single" H 9100 3200 60  0001 L CNN "Family"
F 8 "https://www.diodes.com/assets/Datasheets/ZTX450.pdf" H 9100 3300 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/diodes-incorporated/ZTX450/ZTX450-ND/92530" H 9100 3400 60  0001 L CNN "DK_Detail_Page"
F 10 "TRANS NPN 45V 1A E-LINE" H 9100 3500 60  0001 L CNN "Description"
F 11 "Diodes Incorporated" H 9100 3600 60  0001 L CNN "Manufacturer"
F 12 "Active" H 9100 3700 60  0001 L CNN "Status"
	1    8900 2500
	1    0    0    -1  
$EndComp
$Comp
L dk_Transistors-Bipolar-BJT-Single:ZTX450 Q11
U 1 1 60ABDC93
P 8900 3300
F 0 "Q11" H 9088 3353 60  0000 L CNN
F 1 "ZTX457" H 9088 3247 60  0000 L CNN
F 2 "digikey-footprints:TO-92-3" H 9100 3500 60  0001 L CNN
F 3 "https://www.diodes.com/assets/Datasheets/ZTX450.pdf" H 9100 3600 60  0001 L CNN
F 4 "ZTX450-ND" H 9100 3700 60  0001 L CNN "Digi-Key_PN"
F 5 "ZTX450" H 9100 3800 60  0001 L CNN "MPN"
F 6 "Discrete Semiconductor Products" H 9100 3900 60  0001 L CNN "Category"
F 7 "Transistors - Bipolar (BJT) - Single" H 9100 4000 60  0001 L CNN "Family"
F 8 "https://www.diodes.com/assets/Datasheets/ZTX450.pdf" H 9100 4100 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/diodes-incorporated/ZTX450/ZTX450-ND/92530" H 9100 4200 60  0001 L CNN "DK_Detail_Page"
F 10 "TRANS NPN 45V 1A E-LINE" H 9100 4300 60  0001 L CNN "Description"
F 11 "Diodes Incorporated" H 9100 4400 60  0001 L CNN "Manufacturer"
F 12 "Active" H 9100 4500 60  0001 L CNN "Status"
	1    8900 3300
	1    0    0    -1  
$EndComp
$Comp
L dk_Transistors-Bipolar-BJT-Single:ZTX450 Q12
U 1 1 60ABE38C
P 8900 4200
F 0 "Q12" H 9088 4253 60  0000 L CNN
F 1 "ZTX457" H 9088 4147 60  0000 L CNN
F 2 "digikey-footprints:TO-92-3" H 9100 4400 60  0001 L CNN
F 3 "https://www.diodes.com/assets/Datasheets/ZTX450.pdf" H 9100 4500 60  0001 L CNN
F 4 "ZTX450-ND" H 9100 4600 60  0001 L CNN "Digi-Key_PN"
F 5 "ZTX450" H 9100 4700 60  0001 L CNN "MPN"
F 6 "Discrete Semiconductor Products" H 9100 4800 60  0001 L CNN "Category"
F 7 "Transistors - Bipolar (BJT) - Single" H 9100 4900 60  0001 L CNN "Family"
F 8 "https://www.diodes.com/assets/Datasheets/ZTX450.pdf" H 9100 5000 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/diodes-incorporated/ZTX450/ZTX450-ND/92530" H 9100 5100 60  0001 L CNN "DK_Detail_Page"
F 10 "TRANS NPN 45V 1A E-LINE" H 9100 5200 60  0001 L CNN "Description"
F 11 "Diodes Incorporated" H 9100 5300 60  0001 L CNN "Manufacturer"
F 12 "Active" H 9100 5400 60  0001 L CNN "Status"
	1    8900 4200
	1    0    0    -1  
$EndComp
$Comp
L dk_Transistors-Bipolar-BJT-Single:ZTX450 Q13
U 1 1 60ABEC9D
P 8900 5000
F 0 "Q13" H 9088 5053 60  0000 L CNN
F 1 "ZTX457" H 9088 4947 60  0000 L CNN
F 2 "digikey-footprints:TO-92-3" H 9100 5200 60  0001 L CNN
F 3 "https://www.diodes.com/assets/Datasheets/ZTX450.pdf" H 9100 5300 60  0001 L CNN
F 4 "ZTX450-ND" H 9100 5400 60  0001 L CNN "Digi-Key_PN"
F 5 "ZTX450" H 9100 5500 60  0001 L CNN "MPN"
F 6 "Discrete Semiconductor Products" H 9100 5600 60  0001 L CNN "Category"
F 7 "Transistors - Bipolar (BJT) - Single" H 9100 5700 60  0001 L CNN "Family"
F 8 "https://www.diodes.com/assets/Datasheets/ZTX450.pdf" H 9100 5800 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/diodes-incorporated/ZTX450/ZTX450-ND/92530" H 9100 5900 60  0001 L CNN "DK_Detail_Page"
F 10 "TRANS NPN 45V 1A E-LINE" H 9100 6000 60  0001 L CNN "Description"
F 11 "Diodes Incorporated" H 9100 6100 60  0001 L CNN "Manufacturer"
F 12 "Active" H 9100 6200 60  0001 L CNN "Status"
	1    8900 5000
	1    0    0    -1  
$EndComp
$Comp
L dk_Transistors-Bipolar-BJT-Single:ZTX450 Q9
U 1 1 60ABF3F2
P 7250 5000
F 0 "Q9" H 7438 5053 60  0000 L CNN
F 1 "ZTX457" H 7438 4947 60  0000 L CNN
F 2 "digikey-footprints:TO-92-3" H 7450 5200 60  0001 L CNN
F 3 "https://www.diodes.com/assets/Datasheets/ZTX450.pdf" H 7450 5300 60  0001 L CNN
F 4 "ZTX450-ND" H 7450 5400 60  0001 L CNN "Digi-Key_PN"
F 5 "ZTX450" H 7450 5500 60  0001 L CNN "MPN"
F 6 "Discrete Semiconductor Products" H 7450 5600 60  0001 L CNN "Category"
F 7 "Transistors - Bipolar (BJT) - Single" H 7450 5700 60  0001 L CNN "Family"
F 8 "https://www.diodes.com/assets/Datasheets/ZTX450.pdf" H 7450 5800 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/diodes-incorporated/ZTX450/ZTX450-ND/92530" H 7450 5900 60  0001 L CNN "DK_Detail_Page"
F 10 "TRANS NPN 45V 1A E-LINE" H 7450 6000 60  0001 L CNN "Description"
F 11 "Diodes Incorporated" H 7450 6100 60  0001 L CNN "Manufacturer"
F 12 "Active" H 7450 6200 60  0001 L CNN "Status"
	1    7250 5000
	1    0    0    -1  
$EndComp
$Comp
L dk_Transistors-Bipolar-BJT-Single:ZTX450 Q8
U 1 1 60ABFAC3
P 7250 4200
F 0 "Q8" H 7438 4253 60  0000 L CNN
F 1 "ZTX457" H 7438 4147 60  0000 L CNN
F 2 "digikey-footprints:TO-92-3" H 7450 4400 60  0001 L CNN
F 3 "https://www.diodes.com/assets/Datasheets/ZTX450.pdf" H 7450 4500 60  0001 L CNN
F 4 "ZTX450-ND" H 7450 4600 60  0001 L CNN "Digi-Key_PN"
F 5 "ZTX450" H 7450 4700 60  0001 L CNN "MPN"
F 6 "Discrete Semiconductor Products" H 7450 4800 60  0001 L CNN "Category"
F 7 "Transistors - Bipolar (BJT) - Single" H 7450 4900 60  0001 L CNN "Family"
F 8 "https://www.diodes.com/assets/Datasheets/ZTX450.pdf" H 7450 5000 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/diodes-incorporated/ZTX450/ZTX450-ND/92530" H 7450 5100 60  0001 L CNN "DK_Detail_Page"
F 10 "TRANS NPN 45V 1A E-LINE" H 7450 5200 60  0001 L CNN "Description"
F 11 "Diodes Incorporated" H 7450 5300 60  0001 L CNN "Manufacturer"
F 12 "Active" H 7450 5400 60  0001 L CNN "Status"
	1    7250 4200
	1    0    0    -1  
$EndComp
$Comp
L dk_Transistors-Bipolar-BJT-Single:ZTX450 Q7
U 1 1 60AC02E2
P 7250 3300
F 0 "Q7" H 7438 3353 60  0000 L CNN
F 1 "ZTX457" H 7438 3247 60  0000 L CNN
F 2 "digikey-footprints:TO-92-3" H 7450 3500 60  0001 L CNN
F 3 "https://www.diodes.com/assets/Datasheets/ZTX450.pdf" H 7450 3600 60  0001 L CNN
F 4 "ZTX450-ND" H 7450 3700 60  0001 L CNN "Digi-Key_PN"
F 5 "ZTX450" H 7450 3800 60  0001 L CNN "MPN"
F 6 "Discrete Semiconductor Products" H 7450 3900 60  0001 L CNN "Category"
F 7 "Transistors - Bipolar (BJT) - Single" H 7450 4000 60  0001 L CNN "Family"
F 8 "https://www.diodes.com/assets/Datasheets/ZTX450.pdf" H 7450 4100 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/diodes-incorporated/ZTX450/ZTX450-ND/92530" H 7450 4200 60  0001 L CNN "DK_Detail_Page"
F 10 "TRANS NPN 45V 1A E-LINE" H 7450 4300 60  0001 L CNN "Description"
F 11 "Diodes Incorporated" H 7450 4400 60  0001 L CNN "Manufacturer"
F 12 "Active" H 7450 4500 60  0001 L CNN "Status"
	1    7250 3300
	1    0    0    -1  
$EndComp
$Comp
L dk_Transistors-Bipolar-BJT-Single:ZTX450 Q6
U 1 1 60AC0B1B
P 7250 2500
F 0 "Q6" H 7438 2553 60  0000 L CNN
F 1 "ZTX457" H 7438 2447 60  0000 L CNN
F 2 "digikey-footprints:TO-92-3" H 7450 2700 60  0001 L CNN
F 3 "https://www.diodes.com/assets/Datasheets/ZTX450.pdf" H 7450 2800 60  0001 L CNN
F 4 "ZTX450-ND" H 7450 2900 60  0001 L CNN "Digi-Key_PN"
F 5 "ZTX450" H 7450 3000 60  0001 L CNN "MPN"
F 6 "Discrete Semiconductor Products" H 7450 3100 60  0001 L CNN "Category"
F 7 "Transistors - Bipolar (BJT) - Single" H 7450 3200 60  0001 L CNN "Family"
F 8 "https://www.diodes.com/assets/Datasheets/ZTX450.pdf" H 7450 3300 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/diodes-incorporated/ZTX450/ZTX450-ND/92530" H 7450 3400 60  0001 L CNN "DK_Detail_Page"
F 10 "TRANS NPN 45V 1A E-LINE" H 7450 3500 60  0001 L CNN "Description"
F 11 "Diodes Incorporated" H 7450 3600 60  0001 L CNN "Manufacturer"
F 12 "Active" H 7450 3700 60  0001 L CNN "Status"
	1    7250 2500
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D3
U 1 1 60AC353B
P 3900 4300
F 0 "D3" H 3900 4200 50  0000 C CNN
F 1 "White" H 3850 4400 50  0000 C CNN
F 2 "" H 3900 4300 50  0001 C CNN
F 3 "~" H 3900 4300 50  0001 C CNN
	1    3900 4300
	-1   0    0    1   
$EndComp
$Comp
L Device:LED D4
U 1 1 60AC49CB
P 3900 5200
F 0 "D4" H 3900 5100 50  0000 C CNN
F 1 "White" H 3850 5300 50  0000 C CNN
F 2 "" H 3900 5200 50  0001 C CNN
F 3 "~" H 3900 5200 50  0001 C CNN
	1    3900 5200
	-1   0    0    1   
$EndComp
$Comp
L Device:LED D5
U 1 1 60AC4DE8
P 3900 6050
F 0 "D5" H 3900 5950 50  0000 C CNN
F 1 "White" H 3850 6150 50  0000 C CNN
F 2 "" H 3900 6050 50  0001 C CNN
F 3 "~" H 3900 6050 50  0001 C CNN
	1    3900 6050
	-1   0    0    1   
$EndComp
$Comp
L Device:LED D6
U 1 1 60AC51CE
P 3900 6900
F 0 "D6" H 3900 6800 50  0000 C CNN
F 1 "White" H 3850 7000 50  0000 C CNN
F 2 "" H 3900 6900 50  0001 C CNN
F 3 "~" H 3900 6900 50  0001 C CNN
	1    3900 6900
	-1   0    0    1   
$EndComp
$Comp
L Device:R_Small R?
U 1 1 60AC5478
P 2450 4550
AR Path="/60AC5478" Ref="R?"  Part="1" 
AR Path="/6091E188/60AC5478" Ref="R10"  Part="1" 
F 0 "R10" V 2350 4500 50  0000 L CNN
F 1 "270" V 2550 4450 50  0000 L CNN
F 2 "" H 2450 4550 50  0001 C CNN
F 3 "~" H 2450 4550 50  0001 C CNN
	1    2450 4550
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R?
U 1 1 60AC57B7
P 2450 5450
AR Path="/60AC57B7" Ref="R?"  Part="1" 
AR Path="/6091E188/60AC57B7" Ref="R11"  Part="1" 
F 0 "R11" V 2350 5400 50  0000 L CNN
F 1 "270" V 2550 5350 50  0000 L CNN
F 2 "" H 2450 5450 50  0001 C CNN
F 3 "~" H 2450 5450 50  0001 C CNN
	1    2450 5450
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R?
U 1 1 60AC5A14
P 2450 6300
AR Path="/60AC5A14" Ref="R?"  Part="1" 
AR Path="/6091E188/60AC5A14" Ref="R12"  Part="1" 
F 0 "R12" V 2350 6250 50  0000 L CNN
F 1 "270" V 2550 6200 50  0000 L CNN
F 2 "" H 2450 6300 50  0001 C CNN
F 3 "~" H 2450 6300 50  0001 C CNN
	1    2450 6300
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R?
U 1 1 60AC5D09
P 2450 7150
AR Path="/60AC5D09" Ref="R?"  Part="1" 
AR Path="/6091E188/60AC5D09" Ref="R13"  Part="1" 
F 0 "R13" V 2350 7100 50  0000 L CNN
F 1 "270" V 2550 7050 50  0000 L CNN
F 2 "" H 2450 7150 50  0001 C CNN
F 3 "~" H 2450 7150 50  0001 C CNN
	1    2450 7150
	0    1    1    0   
$EndComp
Wire Wire Line
	4050 4300 4500 4300
Wire Wire Line
	4500 5600 4750 5600
Wire Wire Line
	4050 6900 4500 6900
Wire Wire Line
	4050 6050 4500 6050
Wire Wire Line
	4050 5200 4500 5200
$Comp
L Transistor_BJT:2N3904 Q2
U 1 1 60AE213C
P 2850 4550
F 0 "Q2" H 3040 4596 50  0000 L CNN
F 1 "2N3904" H 3040 4505 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 3050 4475 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/2N3903-D.PDF" H 2850 4550 50  0001 L CNN
	1    2850 4550
	1    0    0    -1  
$EndComp
$Comp
L Transistor_BJT:2N3904 Q3
U 1 1 60B035D8
P 2850 5450
F 0 "Q3" H 3040 5496 50  0000 L CNN
F 1 "2N3904" H 3040 5405 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 3050 5375 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/2N3903-D.PDF" H 2850 5450 50  0001 L CNN
	1    2850 5450
	1    0    0    -1  
$EndComp
$Comp
L Transistor_BJT:2N3904 Q4
U 1 1 60B03AA2
P 2850 6300
F 0 "Q4" H 3040 6346 50  0000 L CNN
F 1 "2N3904" H 3040 6255 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 3050 6225 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/2N3903-D.PDF" H 2850 6300 50  0001 L CNN
	1    2850 6300
	1    0    0    -1  
$EndComp
$Comp
L Transistor_BJT:2N3904 Q5
U 1 1 60B0974D
P 2850 7150
F 0 "Q5" H 3040 7196 50  0000 L CNN
F 1 "2N3904" H 3040 7105 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 3050 7075 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/2N3903-D.PDF" H 2850 7150 50  0001 L CNN
	1    2850 7150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 6100 2950 6050
Wire Wire Line
	2950 6950 2950 6900
Wire Wire Line
	2950 5250 2950 5200
Wire Wire Line
	2950 4350 2950 4300
$Comp
L power:GND #PWR?
U 1 1 60C909F4
P 2950 4850
AR Path="/60C909F4" Ref="#PWR?"  Part="1" 
AR Path="/6091E188/60C909F4" Ref="#PWR051"  Part="1" 
F 0 "#PWR051" H 2950 4600 50  0001 C CNN
F 1 "GND" H 2955 4677 50  0000 C CNN
F 2 "" H 2950 4850 50  0001 C CNN
F 3 "" H 2950 4850 50  0001 C CNN
	1    2950 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 4850 2950 4750
$Comp
L power:GND #PWR?
U 1 1 60C9E7DF
P 2950 5750
AR Path="/60C9E7DF" Ref="#PWR?"  Part="1" 
AR Path="/6091E188/60C9E7DF" Ref="#PWR052"  Part="1" 
F 0 "#PWR052" H 2950 5500 50  0001 C CNN
F 1 "GND" H 2955 5577 50  0000 C CNN
F 2 "" H 2950 5750 50  0001 C CNN
F 3 "" H 2950 5750 50  0001 C CNN
	1    2950 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 5750 2950 5650
$Comp
L power:GND #PWR?
U 1 1 60CABABB
P 2950 6600
AR Path="/60CABABB" Ref="#PWR?"  Part="1" 
AR Path="/6091E188/60CABABB" Ref="#PWR053"  Part="1" 
F 0 "#PWR053" H 2950 6350 50  0001 C CNN
F 1 "GND" H 2955 6427 50  0000 C CNN
F 2 "" H 2950 6600 50  0001 C CNN
F 3 "" H 2950 6600 50  0001 C CNN
	1    2950 6600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 6600 2950 6500
$Comp
L power:GND #PWR?
U 1 1 60CBF7AE
P 2950 7450
AR Path="/60CBF7AE" Ref="#PWR?"  Part="1" 
AR Path="/6091E188/60CBF7AE" Ref="#PWR054"  Part="1" 
F 0 "#PWR054" H 2950 7200 50  0001 C CNN
F 1 "GND" H 2955 7277 50  0000 C CNN
F 2 "" H 2950 7450 50  0001 C CNN
F 3 "" H 2950 7450 50  0001 C CNN
	1    2950 7450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 7450 2950 7350
Connection ~ 4500 5600
Wire Wire Line
	4500 5600 4500 6050
Wire Wire Line
	4500 4300 4500 5200
Wire Wire Line
	4500 5200 4500 5600
Connection ~ 4500 5200
Connection ~ 4500 6050
Wire Wire Line
	4500 6050 4500 6900
Wire Wire Line
	2950 4300 3750 4300
Wire Wire Line
	2950 5200 3750 5200
Wire Wire Line
	2950 6050 3750 6050
Wire Wire Line
	2950 6900 3750 6900
Wire Wire Line
	2550 4550 2650 4550
Wire Wire Line
	2550 5450 2650 5450
Wire Wire Line
	2550 6300 2650 6300
Wire Wire Line
	2550 7150 2650 7150
Wire Notes Line
	1500 500  5500 500 
Wire Notes Line
	5500 3800 1500 3800
Wire Notes Line
	1500 7750 5500 7750
Wire Notes Line
	5500 500  5500 7750
Wire Notes Line
	1500 500  1500 7750
Wire Notes Line
	5550 1500 10500 1500
Wire Notes Line
	10500 1500 10500 6000
Wire Notes Line
	10500 6000 5500 6000
Text GLabel 2200 4550 0    50   Input ~ 0
LED_Launch
Text GLabel 2200 5450 0    50   Input ~ 0
LED_Timer
Text GLabel 2200 6300 0    50   Input ~ 0
LED_Clock
Text GLabel 2200 7150 0    50   Input ~ 0
LED_Stopwatch
Wire Wire Line
	2200 4550 2350 4550
Wire Wire Line
	2200 5450 2350 5450
Wire Wire Line
	2200 6300 2350 6300
Wire Wire Line
	2200 7150 2350 7150
Text Notes 5550 1650 0    79   ~ 0
INS-1 Binary Clock
Text Notes 1600 650  0    79   ~ 0
INS-1 Colon Indicators
Text Notes 1550 3950 0    79   ~ 0
Panel Mounting LEDs (Model: 5586403027F)\n
$Comp
L power:+3.3V #PWR059
U 1 1 60874A1A
P 4750 5600
F 0 "#PWR059" H 4750 5450 50  0001 C CNN
F 1 "+3.3V" V 4765 5728 50  0000 L CNN
F 2 "" H 4750 5600 50  0001 C CNN
F 3 "" H 4750 5600 50  0001 C CNN
	1    4750 5600
	0    1    1    0   
$EndComp
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 60A67C8C
P 4800 1950
F 0 "#FLG0102" H 4800 2025 50  0001 C CNN
F 1 "PWR_FLAG" H 4800 2123 50  0000 C CNN
F 2 "" H 4800 1950 50  0001 C CNN
F 3 "~" H 4800 1950 50  0001 C CNN
	1    4800 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 1950 4800 2100
Wire Wire Line
	4800 2100 4450 2100
Connection ~ 4450 2100
Wire Wire Line
	4450 2100 4450 2150
$EndSCHEMATC