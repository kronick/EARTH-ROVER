#include "Actuator.h"

Actuator::Actuator() {
  this->simulated = true;
}

Actuator::Actuator(Leg* parent, int pwmPin, int potPin,
                   float minLength, float maxLength, bool flipped,
                   bool simulate) {
  this->parent = parent;
  this->pwmPin = pwmPin;
  this->potPin = potPin;
  this->minLength = minLength;
  this->maxLength = maxLength;
  this->simulated = simulate;
  this->flipped = flipped;

  this->midLength = minLength + (maxLength-minLength)/2;
  this->kP =50.0f;
  this->kI = 0.0f;
  this->kD = 0.0f;
  
  if(!simulated) {
    this->pwmOut.attach(pwmPin, PWM_USEC_MIN,PWM_USEC_MAX,
                        PWM_MIN, PWM_MAX);
    pinMode(potPin, INPUT_ANALOG);
    this->length = readLength(50, false);
    for(int i=0; i<SMOOTH_SAMPLES; i++)
      this->lengthHistory[i] = length;

  }
  else this->length = midLength;

  setTarget(this->length, true);
}

void Actuator::update() {
  if(!simulated) {
    /*
    for(int i=0; i<10; i++) {
      togglePin(16);
      delay(200);
    }
    delay(200);
    */

    length = readLength();
    //length = 0;
    float error = length - targetLength;
    writePWM(error * kP);
  }
  else {
    length = targetLength;
  }
}

inline bool Actuator::isPossible(float L) {
  return (L <= maxLength && L >= minLength);
}

bool Actuator::setTarget(float L, bool force) {
  if(force || isPossible(L)) {
    targetLength = L;
    return true;
  }
  return false;
}

float Actuator::getTarget() {
  return targetLength;
}

float Actuator::getLength() {
  return length;
}

float Actuator::getMidLength() {
  return midLength;
}

float Actuator::readLength(int n, bool smooth) {
  /** Reads the analog pin n times and returns an average reading. */
  if(!smooth) {
    float _length = 0;
    for(int i=0; i<n; i++)
      _length += analogRead(potPin);

    return INSTALL_LENGTH + (_length / n) / VOLTS_TO_MM;
  }
  else {
    float _length = analogRead(potPin) / VOLTS_TO_MM + INSTALL_LENGTH;
    float avg = 0;
    for(int i=SMOOTH_SAMPLES-2; i>=0; i--) {
      avg += lengthHistory[i];
      lengthHistory[i+1] = lengthHistory[i];
    }
    avg += length;
    lengthHistory[0] = _length;
    return avg / SMOOTH_SAMPLES;
  }
}

void Actuator::writePWM(int value) {
  /*
  static int i=0;
  int out = (value > PWM_MAX ? PWM_MAX : (value < PWM_MIN ? PWM_MIN : value));
  if(i++ % 1 == 0) {
    SerialUSB.print("Writing: "); SerialUSB.println(out);
    SerialUSB.print("On pin: " ); SerialUSB.println(pwmPin);
  }
  */
  if(flipped) value = - value;

  pwmOut.write(value > PWM_MAX ? PWM_MAX :
                (value < PWM_MIN ? PWM_MIN : value));
}
