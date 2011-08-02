#include "Controller.h"

Controller::Controller() {

}
void Controller::init(int pwmPins[][3], int potPins[][3], BodyType body) {
  
  this->downHeight = 890;
  this->upHeight = 830;
  this->slave = false;  // Default; might change down below

  if(body == HEX_BODY || body == HEX_SLAVE_1 || body == HEX_SLAVE_2) {
    float rot;
    Vec3f c;

    int startIndex = 0;
    int count = LEG_COUNT;
    if(body == HEX_SLAVE_1) {
      startIndex = 2; count = LEGS_PER_SLAVE;
      this->slave = true;
    }
    if(body == HEX_SLAVE_2) {
      startIndex = 4; count = LEGS_PER_SLAVE;
      this->slave = true; 
    }
    
    for(int i=startIndex; i<startIndex+count; i++) {
      rot = -(PI-i/(float)(LEG_COUNT/2)*PI);
      c = Vec3f(cos(rot)*FRAME_SIDE, sin(rot)*FRAME_SIDE, 0);

      legs[i-startIndex] = Leg(this, c, rot, i%2==0 ? false : true,
                               pwmPins[i-startIndex],potPins[i-startIndex],
                               //i == 0 ? true : false);
                               false);
                               //(i-startIndex)>0 ? true : false);
      //legs[i-startIndex].changeMode(i % 2 == 0 ? PUSH_MODE : RETURN_MODE);
      legs[i-startIndex].changeMode(MANUAL_MODE);
    }
    /*
    for(int i=0; i<LEG_COUNT; i++) {
      rot = -(PI-i/(float)(LEG_COUNT/2)*PI);
      c = Vec3f(cos(rot)*FRAME_SIDE, sin(rot)*FRAME_SIDE, 0);

      legs[i] = Leg(this, c, rot, i%2==0 ? false : true,
                               pwmPins[i],potPins[i],
                               i>0 ? true : false);
      //legs[i].changeMode(i % 2 == 0 ? PUSH_MODE : RETURN_MODE);
      //legs[i].changeMode(MANUAL_MODE);
    }
    */
  }
  
}

void Controller::update() {
  for(int i=0; i<(slave ? LEGS_PER_SLAVE : LEG_COUNT); i++) {
    legs[i].update();
  }
}

Leg* Controller::findLeg(int port, int num) {
  if(slave && num < LEGS_PER_SLAVE)
    return &legs[num];

  for(int i=0; i<LEG_COUNT; i++)
    if(legs[i].isRemote(port,num)) return &legs[i];
  
  return NULL;
}

int Controller::findLegNumber(int port, int num) {
  if(slave && num < LEGS_PER_SLAVE)
    return -num;
  
  for(int i=0; i<LEG_COUNT; i++)
    if(legs[i].isRemote(port,num)) return i;

  return -100;
}

bool Controller::moveLegTo(int legNum, Vec3f target) {
  return legs[legNum].setTarget(target);
}

float Controller::getUpHeight() {
  return this->upHeight;
}

float Controller::getDownHeight() {
  return this->downHeight;
}

