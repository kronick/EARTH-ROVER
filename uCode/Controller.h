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
    
    Controller();
    //~Controller();
    void init(int pwmPins[][3], int potPins[][3], BodyType body=HEX_BODY);
    
    void update();
    
    bool moveLegTo(int legNum, Vec3f target);
    float getUpHeight();
    float getDownHeight();
    bool isSlave() { return slave; };

    Leg legs[LEG_COUNT];

    Leg* findLeg(int port, int num);
    int findLegNumber(int port, int num);
  private:
    bool slave;
    float upHeight, downHeight;

};


#endif
