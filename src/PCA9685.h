#ifndef TPCA9685_H
#define TPCA9685_H


class PCA9685Driver
{
  public:
    PCA9685Driver();
    void init(int address);
    void setPWMFreq(int freq);
    void setPWM(int channel, int on, int off);
    void setALLPWM(int on, int off);
    
  private:
    void write8(int reg, int value);
    int read8(int reg);
    int addr,freq;
    double prescaler;

    /*
     * REGISTERS
     */

    // mode register 1
    static const int MODE1 = 0x0;
    // mode register 2
    static const int MODE2 = 0x1;

    // i2c bus subaddress 1
    static const int SUBADR1 = 0x2;
    // i2c bus subaddress 2
    static const int SUBADR2 = 0x3;
    // i2c bus subaddress 3
    static const int SUBADR3 = 0x4;

    // LED All Call i2c bus address
    static const int ALLCALLADR = 0x5;

    // LEDN_ON_L is the delay on time 8 LSD
    // LEDN_ON_H is the delay on time 4 MSD
    // LEDN_OFF_L is the delay off time 8 LSD
    // LEDN_OFF_H is the delay off time 4 MSD

    // LED0 output and brightness control bytes 0-4
    static const int LED0_ON_L = 0x6;
    static const int LED0_ON_H = 0x7;
    static const int LED0_OFF_L = 0x8;
    static const int LED0_OFF_H = 0x9;


    // load all the LEDn_ON/OFF_H/L registers
    static const int ALL_LED_ON_L = 0xFA;
    static const int ALL_LED_ON_H = 0xFB;
    static const int ALL_LED_OFF_L = 0xFC;
    static const int ALL_LED_OFF_H = 0xFD;

    // prescaler for PWM output frequency
    static const int PRE_SCALE = 0xFE;
    // defines the test mode to be entered
    static const int TestMode = 0xFF;

    /*
     * MODE1 register
     */

    static const int RESTART = 0x80;
    static const int EXTCLK = 0x40;
    static const int AI = 0x20;
    static const int SLEEP = 0x10;
    static const int SUB1 = 0x08;
    static const int SUB2 = 0x04;
    static const int SUB3 = 0x02;
    static const int ALLCALL = 0x01;

    /*
     * MODE2 register
     */
    static const int INVRT = 0x4;
    static const int OCH = 0x3;
    static const int OUTDRV = 0x2;
    static const int OUTNE = 0x1;

};

#endif
