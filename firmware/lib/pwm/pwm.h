#ifndef PWM_H
#define PWM_H

void initPwm();
void setPin(int pin, int value); // analogWrite()
void setPulse(int pin, int value); // writeMicroseconds()
void setLevel(int pin, int value); // digitalWrite()
void setPWMFreq(float freq); // analogWriteFrequency()

#endif
