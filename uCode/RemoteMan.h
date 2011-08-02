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
enum RemoteMode { LEG_SELECT, SET_MODE, SET_STATE,
                  SET_COORD_X, SET_COORD_Y, SET_COORD_Z, NO_MODE};

class RemoteMan {
  public:
    RemoteMan();
    RemoteMan(int);
    void sendStatus(int portNum, int legNum, Leg& leg);
    void sendTarget(int portNum, int legNum, const Vec3f& t);
    void sendMode(int portNum, int legNum, Mode m);
    void sendState(int portNum, int legNum, State s);

    void processIncoming();
    
    void setController(Controller* c);

  protected:
    // constants
    static const int COORDINATE_MULTIPLIER = 10;

  private:
    // State variables
    int incomingLeg[2];
    int incomingX[2];
    int incomingY[2];
    int incomingZ[2];
    int incomingNegative[2];
    RemoteMode incomingMode[2]; 

    Controller* controller;

    // Unimplemented copy constructors
    //RemoteMan(RemoteMan const&);
    //void operator=(RemoteMan const&);
};

static RemoteMan RemoteManager; // Singleton
#endif
