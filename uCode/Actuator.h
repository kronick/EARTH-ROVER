#ifndef _ACTUATOR_H_
#define _ACTUATOR_H_

#include "libmaple_types.h"
#include "wirish_types.h"

#ifdef MAPLE_IDE
  #include "wirish.h"
#endif

#include "Vec3f.h"
#include <Servo.h>

class Leg;  // Avoid a circular dependency

class Actuator {
  public:
    static const int INSTALL_LENGTH = 353;
    static const float VOLTS_TO_MM = 28.64;
    static const int PWM_USEC_MIN = 1000;
    static const int PWM_USEC_MAX = 2000;
    static const int PWM_MIN = -1000;
    static const int PWM_MAX = 1000;

    Actuator();
    Actuator(Leg* parent, int pwmPin, int potPin,
             float minLength, float maxLength, bool flipped=false,
             bool simulate=false);
    //~Actuator();

    void update();
    bool isPossible(float L);
    bool setTarget(float L, bool force=false);
 
    // Accessors
    float getLength();
    float getTarget();
    float getMidLength();

  private:
    static const int SMOOTH_SAMPLES = 20;
    Leg* parent;
    Servo pwmOut;

    bool simulated;
    bool flipped;

    float length;
    float targetLength;
    float minLength;
    float maxLength;
    float midLength;
    float maxSpeed;

    float lengthHistory[SMOOTH_SAMPLES];

    float lastUpdate;
    float kP, kI, kD;

    int pwmPin;
    int potPin;
    
    // IO functions
    float readLength(int n=10, bool smooth = true);
    void writePWM(int value);
};


#endif
