#ifndef TPCA9685_H
#define TPCA9685_H

/*
 * REGISTERS
 */

// mode register 1
#define MODE1 0x0
// mode register 2
#define MODE2 0x1

// i2c bus subaddress 1
#define SUBADR1 0x2
// i2c bus subaddress 2
#define SUBADR2 0x3
// i2c bus subaddress 3
#define SUBADR3 0x4

// LED All Call i2c bus address
#define ALLCALLADR 0x5

// LEDN_ON_L is the delay on time
// LEDN_ON_H is the time that the signal is high
// LEDN_OFF_L

// LED0 output and brightness control bytes 0-4
#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9

// LED1 output and brightness control bytes 0-4
#define LED1_ON_L 0xA
#define LED1_ON_H 0xB
#define LED1_OFF_L 0xC
#define LED1_OFF_H 0xD

// LED2 output and brightness control bytes 0-4
#define LED2_ON_L 0xE
#define LED2_ON_H 0xF
#define LED2_OFF_L 0x10
#define LED2_OFF_H 0x11

// LED3 output and brightness control bytes 0-4
#define LED3_ON_L 0x12
#define LED3_ON_H 0x13
#define LED3_OFF_L 0x14
#define LED3_OFF_H 0x15

// LED4 output and brightness control bytes 0-4
#define LED4_ON_L 0x16
#define LED4_ON_H 0x17
#define LED4_OFF_L 0x18
#define LED4_OFF_H 0x19

// LED5 output and brightness control bytes 0-4
#define LED5_ON_L 0x1A
#define LED5_ON_H 0x1B
#define LED5_OFF_L 0x1C
#define LED5_OFF_H 0x1D

// LED6 output and brightness control bytes 0-4
#define LED6_ON_L 0x1E
#define LED6_ON_H 0x1F
#define LED6_OFF_L 0x20
#define LED6_OFF_H 0x21

// LED7 output and brightness control bytes 0-4
#define LED7_ON_L 0x22
#define LED7_ON_H 0x23
#define LED7_OFF_L 0x24
#define LED7_OFF_H 0x25

// LED8 output and brightness control bytes 0-4
#define LED8_ON_L 0x26
#define LED8_ON_H 0x27
#define LED8_OFF_L 0x28
#define LED8_OFF_H 0x29

// LED9 output and brightness control bytes 0-4
#define LED9_ON_L 0x2A
#define LED9_ON_H 0x2B
#define LED9_OFF_L 0x2C
#define LED9_OFF_H 0x2D

// LED10 output and brightness control bytes 0-4
#define LED10_ON_L 0x2E
#define LED10_ON_H 0x2F
#define LED10_OFF_L 0x30
#define LED10_OFF_H 0x31

// LED9 output and brightness control bytes 0-4
#define LED11_ON_L 0x32
#define LED11_ON_H 0x33
#define LED11_OFF_L 0x34
#define LED11_OFF_H 0x35

// LED9 output and brightness control bytes 0-4
#define LED12_ON_L 0x36
#define LED12_ON_H 0x37
#define LED12_OFF_L 0x38
#define LED12_OFF_H 0x39

// LED9 output and brightness control bytes 0-4
#define LED13_ON_L 0x3A
#define LED13_ON_H 0x3B
#define LED13_OFF_L 0x3C
#define LED13_OFF_H 0x3D

// LED9 output and brightness control bytes 0-4
#define LED14_ON_L 0x3E
#define LED14_ON_H 0x3F
#define LED14_OFF_L 0x40
#define LED14_OFF_H 0x41

// LED9 output and brightness control bytes 0-4
#define LED15_ON_L 0x42
#define LED15_ON_H 0x43
#define LED15_OFF_L 0x44
#define LED15_OFF_H 0x45

// load all the LEDn_ON/OFF_H/L registers
#define ALL_LED_ON_L 0xFA
#define ALL_LED_ON_H 0xFB
#define ALL_LED_OFF_L 0xFC
#define ALL_LED_OFF_H 0xFD

// prescaler for PWM output frequency
#define PRE_SCALE 0xFE
// defines the test mode to be entered
#define TestMode 0xFF

/*
 * MODE1 register
 */

#define RESTART 0x7
#define EXTCLK 0x6
#define AI 0x5
#define SLEEP 0x4
#define SUB1 0x3
#define SUB2 0x2
#define SUB3 0x1
#define ALLCALL 0x0

/*
 * MODE2 register
 */
#define INVRT 0x4
#define OCH 0x3
#define OUTDRV 0x2
#define OUTNE 0x1
#endif
