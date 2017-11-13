#include <math.h> // needed for floor
#include <unistd.h> // needed for usleep (UNIX ONLY)
#include "PCA9685.h"
#include <iostream>


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
  // delay 5ms
  int mode1 = read8(MODE1);
  mode1 = mode1 | SLEEP; // force sleep high
  write8(MODE1, mode1); //set the device to sleep
  // delay 5ms
  mode1 = mode1 ^ SLEEP; // make sure sleep is low
  write8(MODE1, mode1);
  // delay 5ms
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
  // delay 5ms
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
}

int PCA9685Driver::read8(int reg)
{
  std::cout << "Reading: reg is " << reg << "\n";
  return 0xFF;
}
int main(){
  PCA9685Driver p = PCA9685Driver();
  p.init();
  p.setPWM(1,0,810);
  std::cout << "This is some text\n";

  return 0;
}
