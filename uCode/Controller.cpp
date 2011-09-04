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

    this->gait = TRIPOD_GAIT;
    this->state = LIFT_CMD_STATE;
    this->currentMove = MoveVector();

    for(int i=startIndex; i<startIndex+count; i++) {
      rot = -(PI-i/(float)(LEG_COUNT/2)*PI);
      c = Vec3f(cos(rot)*FRAME_SIDE, sin(rot)*FRAME_SIDE, 0);

      legs[i-startIndex] = Leg(this, c, rot, i%2==0 ? false : true,
                               pwmPins[i-startIndex],potPins[i-startIndex],
                               false);
      legs[i-startIndex].changeMode(i % 2 == 0 ? PUSH_MODE : RETURN_MODE);
      legs[i-startIndex].changeState(MOVE_STATE);
      //legs[i-startIndex].changeMode(MANUAL_MODE);
    }
  }
  
}

void Controller::update() {
  tick++;
  if(slave) {
      for(int i=0; i<LEGS_PER_SLAVE; i++)
        legs[i].update();

      return;
  }
  

  //if(tick%4 < 3) return;

  switch(gait) {
    case MANUAL_GAIT:
      break;
    case DANCE_GAIT: {
      //changeMoveVector(MoveVector(Vec3f(0,0,0),
      //                 0.2*sin(radians(tick)),
      //                 0.07*cos(radians(tick/2.2)), 0));


      Vec3f target;
      
      for(int i=0; i<LEG_COUNT; i++) {
        legs[i].changeMode(MANUAL_MODE);
        target = legs[i].toBodySpace(legs[i].getCenterTarget());
        target.set(target.x, target.y, (upHeight + downHeight) / 2);
        target.rotateZ(currentMove.yaw);
        target.rotateX(currentMove.pitch);
        target.rotateY(currentMove.roll);

        target += currentMove.translation;

        target = legs[i].toLegSpace(target);
        legs[i].setTarget(target, true);
      }
      
      break;
    }
    case TRIPOD_GAIT:
      for(int i=0; i<LEG_COUNT; i++) 
        legs[i].setMoveVector(currentMove);

      switch(state) {
        case LIFT_CMD_STATE:
          for(int i=0; i<LEG_COUNT; i++)
            legs[i].changeMode(legs[i].getMode() == PUSH_MODE ? LIFT_UP_MODE :
                                                              LIFT_DOWN_MODE);
          state = LIFT_WAIT_STATE;  // Go to the next state
          
          break;
        case LIFT_WAIT_STATE: {
          /* Wait in this mode and don't do anything until all legs
             are up or down */

          bool allLegsUp = true;
          bool allLegsDown = true;

          // Change the above flags to false if one leg is not up/down yet
          for(int i=0; i<LEG_COUNT; i++) {
            if(legs[i].getMode() == LIFT_UP_MODE &&
               legs[i].getState() != UP_STATE)
              allLegsUp = false;
            if(legs[i].getMode() == LIFT_DOWN_MODE &&
               legs[i].getState() != DOWN_STATE)
              allLegsDown = false;
          }

          if(allLegsUp && allLegsDown)
            state = PUSH_CMD_STATE; // Go to the next state

          break;
        }
        case PUSH_CMD_STATE:
          // Switch legs from pushing to returning and set center target
          for(int i=0; i<LEG_COUNT; i++) {
            if(legs[i].getMode() == LIFT_UP_MODE) {
              legs[i].setCenterTarget();
              legs[i].changeMode(RETURN_MODE);
            }
            else if(legs[i].getMode() == LIFT_DOWN_MODE)
              legs[i].changeMode(PUSH_MODE);
          }
          state = PUSH_WAIT_STATE;  // Go to the next state
          break;

        case PUSH_WAIT_STATE:
          bool endStop = false; // Goes true if ANY pushing leg is at its limit
          bool returned = true; // Goes false if ALL returning legs have ret'd

          for(int i=0; i<LEG_COUNT; i++) {
            if(legs[i].getMode() == PUSH_MODE &&
               legs[i].getState() == END_STATE)
              endStop = true;
            else if(legs[i].getMode() == RETURN_MODE &&
                    legs[i].getState() != END_STATE)
              returned = false;
          }

          if(endStop) {
            for(int i=0; i<LEG_COUNT; i++) {
              // Set all pushing legs to stop
              if(legs[i].getMode() == PUSH_MODE)
                legs[i].changeState(END_STATE);
            }
            if(returned)  // If one push leg stopped and all returned
              state = LIFT_CMD_STATE; // Go to the next state
          }

          break;
      }
      break;
    case RIPPLE_GAIT:
      
      break;
    case WAVE_GAIT:

      break;
  }

  
  for(int i=0; i<LEG_COUNT; i++) { //_COUNT; i++) {
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

void Controller::changeGait(WalkingGait g) {
  if(g == gait) return;
  if(g != TRIPOD_GAIT && g != DANCE_GAIT && g != RIPPLE_GAIT &&
     g != WAVE_GAIT && g != MANUAL_GAIT) return;
  
  if(g == TRIPOD_GAIT) {
    float tallestHeight = 0;
    int tallestIndex = -1;
    // Decide which legs should push and which should return
    for(int i=0; i<LEG_COUNT; i++) {
      if(legs[i].getFoot().z > tallestHeight) {
        tallestHeight = legs[i].getFoot().z;
        tallestIndex = i;
      }
    }
    int parity = 0;
    if(tallestIndex % 2 == 1) parity = 1;
    for(int i=0; i<LEG_COUNT; i++) {
      legs[i].changeMode(i % 2 == parity ? PUSH_MODE : RETURN_MODE);
      legs[i].changeState(MOVE_STATE);
    }

    state = LIFT_CMD_STATE;
  }

  gait = g;
}

void Controller::changeMoveVector(MoveVector v) {
  currentMove = v;
  for(int i=0; i<LEG_COUNT; i++) {
    legs[i].setMoveVector(v);
  }
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

