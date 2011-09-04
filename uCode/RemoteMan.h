#ifndef _REMOTE_MAN_H_
#define _REMOTE_MAN_H_

#include "libmaple_types.h"
#include "wirish_types.h"
#include "Vec3f.h"
#include "Leg.h"
#include "Controller.h"

#ifdef MAPLE_IDE
  #include "wirish.h"
#endif

// Enums
enum RemoteMode { LEG_SELECT, SET_MODE, SET_STATE, SET_GAIT,
                  SET_COORD_X, SET_COORD_Y, SET_COORD_Z, SET_YAW,
                  SET_PITCH, SET_ROLL, NO_MODE};

class RemoteMan {
  public:
    enum ControlMode { WALK, DANCE, LEG, NO_CONTROL_MODE };
    
    RemoteMan();
    RemoteMan(int);
    void sendStatus(int portNum, int legNum, Leg& leg);
    void sendTarget(int portNum, int legNum, const Vec3f& t);
    void sendMode(int portNum, int legNum, Mode m);
    void sendState(int portNum, int legNum, State s);

    void processIncoming();
    void processUSB();
    
    void setController(Controller* c);

  protected:
    // constants
    static const float COORDINATE_MULTIPLIER = 10;
    static const float ANGLE_MULTIPLIER = 1000;

  private:
    // State variables
    static const long USB_TIMEOUT = 1000;
    long timeoutContact;
    long lastContact;
    int incomingLeg[2];
    int incomingX[2];
    int incomingY[2];
    int incomingZ[2];
    bool incomingNegative[2];
    RemoteMode incomingMode[2]; 

    int usbLeg;
    int usbX, usbY, usbZ, usbPitch, usbYaw, usbRoll;
    bool usbNegative;
    RemoteMode usbMode;
    ControlMode usbControlMode;
    bool statusSent;

    Controller* controller;
};

static RemoteMan RemoteManager; // Singleton
#endif
