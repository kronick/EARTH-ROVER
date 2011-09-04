#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include "libmaple_types.h"
#include "wirish_types.h"
#include "Vec3f.h"
#include "Controller.h"
#include "Leg.h"

#ifdef MAPLE_IDE
  #include "wirish.h"
#endif



class Controller {
  public:
    static const int FRAME_SIDE = 462;
    static const int LEG_COUNT = 6;
    static const int LEGS_PER_SLAVE = 2;
    enum BodyType {HEX_BODY, RECT_BODY, HEX_SLAVE_1, HEX_SLAVE_2};
    enum ControllerState {LIFT_CMD_STATE, LIFT_WAIT_STATE,
                          PUSH_CMD_STATE, PUSH_WAIT_STATE};                          
    enum WalkingGait {DANCE_GAIT, TRIPOD_GAIT, WAVE_GAIT, RIPPLE_GAIT,
                      MANUAL_GAIT};

    Controller();
    //~Controller();
    void init(int pwmPins[][3], int potPins[][3], BodyType body=HEX_BODY);
    
    void update();
    
    bool moveLegTo(int legNum, Vec3f target);
    float getUpHeight();
    float getDownHeight();
    void changeMoveVector(MoveVector v);
    void changeGait(WalkingGait gait);
    bool isSlave() { return slave; };
    MoveVector getMoveVector() { return currentMove; };

    Leg legs[LEG_COUNT];

    Leg* findLeg(int port, int num);
    int findLegNumber(int port, int num);
  private:
    bool slave;
    float upHeight, downHeight;
    MoveVector currentMove;
    MoveVector nextMove;

    WalkingGait gait;
    ControllerState state;

    long tick;

};


#endif
