EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr USLedger 17000 11000
encoding utf-8
Sheet 1 4
Title "Stewart Platform Main Wiring Schematic"
Date ""
Rev ""
Comp "Marion Garrett Sisk"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Motor:Motor_Servo M1
U 1 1 6072907F
P 11050 3250
F 0 "M1" V 11000 3550 50  0000 R CNN
F 1 "Motor_Servo" V 11100 3900 50  0000 R CNN
F 2 "" H 11050 3060 50  0001 C CNN
F 3 "http://forums.parallax.com/uploads/attachments/46831/74481.png" H 11050 3060 50  0001 C CNN
	1    11050 3250
	1    0    0    -1  
$EndComp
$Comp
L Motor:Motor_Servo M2
U 1 1 6072A198
P 11050 4100
F 0 "M2" V 11000 4400 50  0000 R CNN
F 1 "Motor_Servo" V 11100 4750 50  0000 R CNN
F 2 "" H 11050 3910 50  0001 C CNN
F 3 "http://forums.parallax.com/uploads/attachments/46831/74481.png" H 11050 3910 50  0001 C CNN
	1    11050 4100
	1    0    0    -1  
$EndComp
$Comp
L Motor:Motor_Servo M6
U 1 1 6072AFF7
P 11050 7550
F 0 "M6" V 11000 7850 50  0000 R CNN
F 1 "Motor_Servo" V 11100 8200 50  0000 R CNN
F 2 "" H 11050 7360 50  0001 C CNN
F 3 "http://forums.parallax.com/uploads/attachments/46831/74481.png" H 11050 7360 50  0001 C CNN
	1    11050 7550
	1    0    0    -1  
$EndComp
$Comp
L Motor:Motor_Servo M3
U 1 1 6072C54A
P 11050 4950
F 0 "M3" V 10998 5156 50  0000 L CNN
F 1 "Motor_Servo" V 11089 5156 50  0000 L CNN
F 2 "" H 11050 4760 50  0001 C CNN
F 3 "http://forums.parallax.com/uploads/attachments/46831/74481.png" H 11050 4760 50  0001 C CNN
	1    11050 4950
	1    0    0    -1  
$EndComp
$Comp
L Motor:Motor_Servo M4
U 1 1 6072D5C7
P 11050 5800
F 0 "M4" V 10998 6006 50  0000 L CNN
F 1 "Motor_Servo" V 11089 6006 50  0000 L CNN
F 2 "" H 11050 5610 50  0001 C CNN
F 3 "http://forums.parallax.com/uploads/attachments/46831/74481.png" H 11050 5610 50  0001 C CNN
	1    11050 5800
	1    0    0    -1  
$EndComp
$Comp
L Motor:Motor_Servo M5
U 1 1 6072F500
P 11050 6700
F 0 "M5" V 10998 6906 50  0000 L CNN
F 1 "Motor_Servo" V 11089 6906 50  0000 L CNN
F 2 "" H 11050 6510 50  0001 C CNN
F 3 "http://forums.parallax.com/uploads/attachments/46831/74481.png" H 11050 6510 50  0001 C CNN
	1    11050 6700
	1    0    0    -1  
$EndComp
$Sheet
S 3850 4900 1500 1100
U 607402A6
F0 "Power Supply" 50
F1 "psuModule.sch" 50
F2 "+5VDC" O R 5350 5250 50 
F3 "GND" O R 5350 5650 50 
F4 "120VAC-L" I L 3850 5100 50 
F5 "120VAC-N" I L 3850 5250 50 
F6 "120VAC-GND" I L 3850 5500 50 
$EndSheet
$Sheet
S 6750 2600 1300 1750
U 607A62AF
F0 "arduinoMEGA2560" 50
F1 "Arduino MEGA 2560.sch" 50
F2 "GND" I L 6750 3150 50 
F3 "+5VDC" I L 6750 2950 50 
F4 "D20-SDA" B R 8050 2800 50 
F5 "D21-SCL" B R 8050 2900 50 
F6 "D14-M1-PWM" O R 8050 3150 50 
F7 "D15-M2-PWM" O R 8050 3300 50 
F8 "D2-M3-PWM" O R 8050 3450 50 
F9 "D3-M4-PWM" O R 8050 3600 50 
F10 "D4-M5-PWM" O R 8050 3750 50 
F11 "D5-M6-PWM" O R 8050 3900 50 
$EndSheet
Wire Wire Line
	8050 3300 9850 3300
Wire Wire Line
	9850 3300 9850 4000
Wire Wire Line
	9700 4850 9700 3450
Wire Wire Line
	9700 3450 8050 3450
Wire Wire Line
	9550 5700 9550 3600
Wire Wire Line
	9550 3600 8050 3600
Wire Wire Line
	9400 6600 9400 3750
Wire Wire Line
	9400 3750 8050 3750
Wire Wire Line
	9250 7450 9250 3900
Wire Wire Line
	9250 3900 8050 3900
Wire Wire Line
	8050 3150 10750 3150
Wire Wire Line
	9850 4000 10750 4000
Wire Wire Line
	9700 4850 10750 4850
Wire Wire Line
	9550 5700 10750 5700
Wire Wire Line
	9400 6600 10750 6600
Wire Wire Line
	9250 7450 10750 7450
Wire Wire Line
	6750 2950 5950 2950
Wire Wire Line
	5950 2950 5950 5250
Wire Wire Line
	5950 5250 5350 5250
Wire Wire Line
	10750 3250 10550 3250
Wire Wire Line
	10750 4100 10550 4100
Wire Wire Line
	10550 4100 10550 3250
Wire Wire Line
	10750 4950 10550 4950
Wire Wire Line
	10550 4950 10550 4100
Connection ~ 10550 4100
Wire Wire Line
	10750 5800 10550 5800
Wire Wire Line
	10550 5800 10550 4950
Connection ~ 10550 4950
Wire Wire Line
	10750 6700 10550 6700
Wire Wire Line
	10550 6700 10550 5800
Connection ~ 10550 5800
Wire Wire Line
	10750 7550 10550 7550
Wire Wire Line
	10550 7550 10550 6700
Connection ~ 10550 6700
Wire Wire Line
	10550 7550 5950 7550
Wire Wire Line
	5950 7550 5950 5250
Connection ~ 10550 7550
Connection ~ 5950 5250
Wire Wire Line
	10750 3350 10400 3350
Wire Wire Line
	10400 3350 10400 4200
Wire Wire Line
	10400 4200 10750 4200
Wire Wire Line
	10400 4200 10400 5050
Wire Wire Line
	10400 5050 10750 5050
Connection ~ 10400 4200
Wire Wire Line
	10400 5050 10400 5900
Wire Wire Line
	10400 5900 10750 5900
Connection ~ 10400 5050
Wire Wire Line
	10400 5900 10400 6800
Wire Wire Line
	10400 6800 10750 6800
Connection ~ 10400 5900
Wire Wire Line
	10400 6800 10400 7650
Wire Wire Line
	10400 7650 10750 7650
Connection ~ 10400 6800
Wire Wire Line
	10400 7650 6200 7650
Wire Wire Line
	6200 7650 6200 5650
Wire Wire Line
	6200 5650 5350 5650
Connection ~ 10400 7650
Wire Wire Line
	6200 5650 6200 3150
Wire Wire Line
	6200 3150 6750 3150
Connection ~ 6200 5650
$Sheet
S 12950 1850 1250 800 
U 607BC00D
F0 "MPU6050 IMU" 50
F1 "MPU6050.sch" 50
F2 "+5VDC" I L 12950 1950 50 
F3 "GND" I L 12950 2100 50 
F4 "SDA" I L 12950 2300 50 
F5 "SCL" I L 12950 2450 50 
$EndSheet
Wire Wire Line
	12950 1950 5950 1950
Wire Wire Line
	5950 1950 5950 2950
Connection ~ 5950 2950
Wire Wire Line
	12950 2100 6200 2100
Wire Wire Line
	6200 2100 6200 3150
Connection ~ 6200 3150
Wire Wire Line
	12950 2300 8500 2300
Wire Wire Line
	8500 2300 8500 2800
Wire Wire Line
	8500 2800 8050 2800
Wire Wire Line
	12950 2450 8600 2450
Wire Wire Line
	8600 2450 8600 2900
Wire Wire Line
	8600 2900 8050 2900
$Comp
L power:Earth #PWR?
U 1 1 607C1A72
P 3400 5900
F 0 "#PWR?" H 3400 5650 50  0001 C CNN
F 1 "Earth" H 3400 5750 50  0001 C CNN
F 2 "" H 3400 5900 50  0001 C CNN
F 3 "~" H 3400 5900 50  0001 C CNN
	1    3400 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	3850 5500 3400 5500
Wire Wire Line
	3400 5500 3400 5900
$Comp
L power:VAC #PWR?
U 1 1 607C4326
P 2850 4950
F 0 "#PWR?" H 2850 4850 50  0001 C CNN
F 1 "VAC" H 2850 5225 50  0000 C CNN
F 2 "" H 2850 4950 50  0001 C CNN
F 3 "" H 2850 4950 50  0001 C CNN
	1    2850 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3850 5100 2850 5100
Wire Wire Line
	2850 5100 2850 4950
$Comp
L power:NEUT #PWR?
U 1 1 607C78B1
P 2550 4950
F 0 "#PWR?" H 2550 4800 50  0001 C CNN
F 1 "NEUT" H 2567 5123 50  0000 C CNN
F 2 "" H 2550 4950 50  0001 C CNN
F 3 "" H 2550 4950 50  0001 C CNN
	1    2550 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3850 5250 2550 5250
Wire Wire Line
	2550 5250 2550 4950
$EndSCHEMATC
