#include <math.h> // needed for floor
#include "PCA9685.h"
#include <iostream>
#include <stdio.h>

#include <wiringPi.h>
#include <wiringPiI2C.h>

PCA9685Driver::PCA9685Driver()
{
  addr = 0x60;
  freq = 1600;
}

void PCA9685Driver::init(int address = 0x60)
{
  setALLPWM(0,0); // set all of the PWM channels to zero
  write8(MODE2, OUTDRV);
  write8(MODE1, ALLCALL);
  delay(5);
  int mode1 = read8(MODE1);
  mode1 = mode1 | SLEEP; // force sleep high
  write8(MODE1, mode1); //set the device to sleep
  delay(5);
  mode1 = mode1 ^ SLEEP; // make sure sleep is low
  write8(MODE1, mode1);
  delay(5);
}

void PCA9685Driver::setPWMFreq(int freq)
{
  prescaler = 25000000; // 25MHz
  prescaler /= 4096; // 12 bit
  prescaler /= freq;
  freq = (int) (prescaler);
  freq -= 1;
  int mode = read8(MODE1);
  mode = mode | SLEEP; // force sleep to high
  write8(MODE1, mode);
  // in section 7.3.5 on page 25
  write8(PRE_SCALE, freq);
  mode = mode ^ SLEEP; // force sleep back to low
  write8(MODE1, mode);
  delay(5);
  write8(MODE1, mode | RESTART);

}

void PCA9685Driver::setPWM(int channel, int on, int off)
{
  write8(LED0_ON_L + 4 * channel, on & 0xFF);
  write8(LED0_ON_H + 4 * channel, on >> 8);
  write8(LED0_OFF_L + 4 * channel, off & 0xFF);
  write8(LED0_OFF_H + 4 * channel, off >> 8);
}

void PCA9685Driver::setALLPWM(int on, int off)
{
  write8(ALL_LED_ON_L, on & 0xFF);
  write8(ALL_LED_ON_H, on >> 8);
  write8(ALL_LED_OFF_L, off & 0xFF);
  write8(ALL_LED_OFF_H, off >> 8);
}

void PCA9685Driver::write8(int reg, int value)
{
  std::cout << "Writing: reg is " << reg << " and the value is " << value << "\n";
  wiringPiI2CWriteReg8(addr, reg, value);

}

int PCA9685Driver::read8(int reg)
{
  unsigned int read_red = wiringPiI2CReadReg8(addr,reg);
  std::cout << "Reading: reg is " << read_red << "\n";
  return read_red;

}

int main(){
  std::cout<< std::hex;
  wiringPiSetup();
  int pwm = wiringPiI2CSetup(0x60);
  std::cout << "PWM IS "<< pwm << "\n";
  PCA9685Driver p = PCA9685Driver();
  std::cout << "init\n";
  p.init();
  std::cout << "pwm freq\n";
  p.setPWMFreq(1000);
  std::cout << "pwm\n";
  p.setPWM(15,0,3000);

  return 0;
}
