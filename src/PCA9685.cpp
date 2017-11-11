class PCA9685Driver
{
  typedef (unsigned char) hex;
  int addr;
  int freq[10];
  int duty[10];
  public:
    initialize();
    Mode1(hex);
    Mode2(hex);
    adjustDutyCycle(int channel, double duty);
}




void PCA9685Driver::Mode1(hex settings)
{
  wiringPiI2CWriteReg8 (addr, MODE1, settings);
}

void PCA9685Driver::Mode2(hex settings)
{
  wiringPiI2CWriteReg8 (addr, MODE2, settings);
}

void PCA9685Driver::adjustDutyCycle(int channel, double duty)
{
  // max is 4095, min is 0
  // assume no delay, because why...?

}
