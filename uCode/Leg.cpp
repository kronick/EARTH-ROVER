#include "Leg.h"
#include "Controller.h"
#include "RemoteMan.h"

Leg::Leg() {
  this->parent = 0;
  this->center = Vec3f(0,0,0);
  this->centerOfRotation = center;
  this->rotation = 0;
  this->flipped = false;
}

Leg::Leg(Controller* parent, Vec3f center, float rotation, boolean flipped,
          int pwmPins[], int potPins[], bool simulate) {
  this->parent = parent;
  this->center = center;
  this->centerOfRotation = center;
  this->rotation = rotation;
  this->flipped = flipped;

  if(pwmPins[0] >= 0 && potPins[0] >= 0) // Local, or just simulated
    this->remote = false;
  else {
    this->remote = true;
    this->remotePort = abs(potPins[0]) - 1;
    this->remoteIndex = abs(pwmPins[0]) - 1;
  }
  
  bool flip;
  for(int i=0; i<3; i++) {
    if((flipped && i != 1) || (!flipped && i == 1)) flip = true;
    else flip = false;
    //actuators[i] = Actuator(this, 0,0, 353, 499, simulate);
    actuators[i] = Actuator(this, pwmPins[i], potPins[i],
                            353, 499, flip, simulate || remote);
  }

  solveFK();
  setTarget(foot, true);
}
void Leg::update() {
  if(parent->isSlave()) mode = MANUAL_MODE;
  switch (mode) {
    case MANUAL_MODE:
      break;
    default:
      if(state != END_STATE && (mode == PUSH_MODE || mode == RETURN_MODE)) {
        Vec3f nextTarget = target;
        Vec3f deltaTranslation = currentMove.translation *
                        (mode == RETURN_MODE ? - 1 : 1) * MOVE_FACTOR;
        //nextTarget += currentMove.translation *
        //            (mode == RETURN_MODE ? -1 : 1) * MOVE_FACTOR;
        Vec3f translated = center - centerOfRotation;
        Vec3f rotatedTarget = toBodySpace(target) - translated;
        float rot = (mode == RETURN_MODE ? -1 : 1) * currentMove.yaw *
                        MOVE_FACTOR;
        rotatedTarget.rotateZ(rot);
        rotatedTarget += translated;
        Vec3f deltaRotation = target - toLegSpace(rotatedTarget);

        nextTarget += deltaRotation + deltaTranslation;

        centerOfRotation -= toBodySpace(deltaTranslation) - center;
        currentMove.translation.rotateZ(rot);
      
        // Don't move target if the foot is too far away still
        //if(target.dist(foot) > TARGET_THRESHOLD)
        //  nextTarget = target;
      
        if(!canStepTo(nextTarget) || !setTarget(nextTarget)) {
          // End stop condition
          setTarget(target, true);
          changeState(END_STATE);
        }
        else {
          changeState(MOVE_STATE);
          setTarget(nextTarget);
        }
      }
  
      if(mode == LIFT_UP_MODE && state != UP_STATE) {
        moveTargetUp();
        bool force = target.z > parent->getUpHeight()-10;
        //force = false;
        //if(target.z > parent->getUpHeight()-30)
        if(foot.z < parent->getUpHeight()) {
          // End stop condition
          setTarget(Vec3f(target.x, target.y, parent->getUpHeight()));
          changeState(UP_STATE);
        }
        else if (force)
          setTarget(target + currentMove.translation * MOVE_FACTOR, force);
      }
      if(mode == LIFT_DOWN_MODE && state != DOWN_STATE) {
        moveTargetDown();
        bool force = target.z < parent->getDownHeight()+10;
        //force = false;
        //if(target.z < parent->getDownHeight()+30)
        //  setTarget(target + currentMove.translation * MOVE_FACTOR, force);
        if(foot.z > parent->getDownHeight()) {
          setTarget(Vec3f(target.x, target.y, parent->getDownHeight()));
          changeState(DOWN_STATE);
        }
        else if (force)
          setTarget(target + currentMove.translation * MOVE_FACTOR, force);
      }
      break;
  }
  
  /*
  for(int i=0; i<10; i++) {
    togglePin(16);
    delay(50);
  }
  digitalWrite(16,LOW);
  delay(1000);
  */

  if(!remote) {
    for(int i=0; i<3; i++)
      actuators[i].update();
    
    //solveFK();
  }
  solveFK();

}

void Leg::synchronizeRemote(const Vec3f& remoteFoot){
  if(!remote) {
    // This is an incoming command. Call methods to change state.
    setTarget(remoteFoot, true);
  }
  else {
    this->foot = remoteFoot;
    // Update actuator lengths based on foot
    
    ActLens newLens = solveIK(foot);
    actuators[0].setTarget(newLens.a);
    actuators[0].update();
    actuators[1].setTarget(newLens.b);
    actuators[1].update();
    actuators[2].setTarget(newLens.c);
    actuators[2].update();
    
  }
}

void Leg::synchronizeRemote(Mode remoteMode) {
  //if(!remote)
  //  changeMode(remoteMode);
  //else
  //  this->mode = remoteMode;
}

void Leg::synchronizeRemote(State remoteState) {
  //if(!remote)
  //  changeState(remoteState);
  //else
  //  this->state = remoteState;
}



void Leg::toggleMode() {
  if(mode == PUSH_MODE)
    changeMode(RETURN_MODE);
  else if(mode == RETURN_MODE)
    changeMode(PUSH_MODE);
}

Vec3f Leg::toLegSpace(const Vec3f& _p) {
  Vec3f p = _p;
  p -= center;
  p.rotateZ(-rotation);
  return p;
}

Vec3f Leg::toBodySpace(const Vec3f& _p) { 
  Vec3f p = _p;
  p.rotateZ(rotation);
  return (p + center);
}

bool Leg::isRemote(int port, int num) {
  return (remote && remotePort == port && remoteIndex == num);
}

bool Leg::canMoveTo(ActLens A) {
  return (actuators[0].isPossible(A.a) && actuators[1].isPossible(A.b) &&
          actuators[2].isPossible(A.c));
}

bool Leg::canMoveTo(const Vec3f& p) {
  ActLens a = solveIK(p);
  return canMoveTo(a);
}


bool Leg::canStepTo(const Vec3f& p) {
  ActLens a = solveIK(p);
  ActLens b = solveIK(Vec3f(p.x, p.y, parent->getUpHeight()));
  ActLens c = solveIK(Vec3f(p.x, p.y, parent->getDownHeight()));

  return (canMoveTo(a) && canMoveTo(b) && canMoveTo(c));
}


bool Leg::changeMode(Mode m) {
  if(mode == m) return false;
  else {
    mode = m;
    if(remote)
      RemoteManager.sendMode(remotePort, remoteIndex, m);
  }

  return true;
}
Mode Leg::getMode() {
  return mode;
}

bool Leg::changeState(State s) {
  if(state == s) return false;
  else {
    if(s == END_STATE)
      freeze();
   
    state = s;
    
    if(remote) RemoteManager.sendState(remotePort, remoteIndex, s);
    return true;
  }
}

State Leg::getState() {
  return state;
}


bool Leg::setTarget(Vec3f p, bool force) {
  ActLens acts = solveIK(p);
  
  if(force || canMoveTo(acts)) {
    target = p;

    if(!remote) {
      actuators[0].setTarget(acts.a, force);
      actuators[1].setTarget(acts.b, force);
      actuators[2].setTarget(acts.c, force);
    }
    else
      RemoteManager.sendTarget(remotePort, remoteIndex, target);
   

    return true;
  }
  else return false;
}

Vec3f Leg::getTarget() {
  return target;
}

Vec3f Leg::getFoot() {
  //SerialUSB.print("A: "); SerialUSB.println(actuators[0].getLength());
  //SerialUSB.print("B: "); SerialUSB.println(actuators[1].getLength());
  //SerialUSB.print("C: "); SerialUSB.println(actuators[2].getLength());
   
  return foot;
}

void Leg::moveTargetUp() {
  setMoveVector(Vec3f(0,0,-65), 0);
}

void Leg::moveTargetDown() {
  setMoveVector(Vec3f(0,0,65), 0);
}

Vec3f Leg::getCenterTarget() {
  ActLens acts = {actuators[0].getMidLength(), actuators[1].getMidLength(),
                  actuators[2].getMidLength()};
  Vec3f middle = solveFK(acts);
  return Vec3f(middle.x, 0, parent->getUpHeight());
}

void Leg::setCenterTarget() {
  setTarget(getCenterTarget(), true);
}
    
void Leg::setMoveVector(Vec3f t, float r) {
  t.rotateZ(-rotation);
  currentMove = MoveVector(t, r);
}


void Leg::setMoveVector(MoveVector m) {
  Vec3f translation = m.translation;
  translation.rotateZ(-rotation);

  currentMove = MoveVector(translation, m.rotation);
}

MoveVector Leg::getMoveVector() {
  return currentMove;
}


    
float Leg::getDeltaP() {
  return milestone.translation.dist(foot);
}

float Leg::getDeltaTheta() {
  float out = getAngle() - milestone.rotation;
  if(out > PI)       out -= 2*PI;
  else if(out < -PI) out += 2*PI;
  return out;
}

float Leg::getAngle() {
  return getAngle(foot);
}

float Leg::getAngle(const Vec3f& _P) {
  Vec3f P = _P;
  P.rotateZ(rotation);
  P += center;
  return atan2(P.y, P.x);
}

void Leg::freeze() {
  if(mode == PUSH_MODE) {
    //solveFK();
    //setTarget(foot);
  }
  setMoveVector(MoveVector());
}

void Leg::resetMilestone() {
  milestone = MoveVector(foot, getAngle());
}

Vec3f Leg::solveFK() {
  ActLens a = {actuators[0].getLength(), actuators[1].getLength(),
               actuators[2].getLength()};
  return solveFK(a, true);
}

Vec3f Leg::solveFK(ActLens acts, bool assign) {
  /** Solves leg geometry (knee, ankle, foot, swingAngle, unprojectFoot) based on current actuator lengths.
  */
  // Find interior angles
  float alpha = acosf((hip*hip + femur*femur - acts.a*acts.a) /
                      (2*hip*femur)); 
  float beta  = acosf((femur*femur + tibia*tibia - acts.b*acts.b) /
                      (2*femur*tibia));

  Vec3f unprojectKnee(swingArm+femur*sinf(alpha), 0, hip-femur*cosf(alpha));
  Vec3f unprojectAnkle((tibia)*sinf(beta-alpha), 0, (tibia)*cosf(beta-alpha));
  unprojectAnkle += unprojectKnee;

  Vec3f unprojectFoot((tibia+cankle)*sinf(beta-alpha), 0,
                      (tibia+cankle)*cosf(beta-alpha));
  unprojectFoot += unprojectKnee;

  float swingAngle = solveSwingAngle(acts.c);
  
  
  Vec3f foot(unprojectFoot.x*sinf(swingAngle),
             unprojectFoot.x*cosf(swingAngle),
             unprojectFoot.z);
  /* 
  Vec3f knee(unprojectKnee.x*sinf(swingAngle),
             unprojectKnee.x*cosf(swingAngle),
             unprojectKnee.z);
  Vec3f ankle(unprojectAnkle.x*sinf(swingAngle),
              unprojectAnkle.x*cosf(swingAngle),
              unprojectAnkle.z);
  Vec3f hipEnd(swingArm*sinf(swingAngle), swingArm*cosf(swingAngle), hip);
  */

  if(assign) {
    //this->knee = knee;
    //this->ankle = ankle;
    //this->hipEnd = hipEnd;
    this->swingAngle = swingAngle;
    this->foot = foot;
  }
  
  return foot;

}
    
float Leg::solveSwingAngle(float act) {
  if(flipped) return PI - (acosf((swingMount*swingMount + swingBase*swingBase -
                    act*act) / (2*swingBase*swingMount)) - frameAngle);
                            
  else        return acosf((swingMount*swingMount + swingBase*swingBase -
                     act*act) / (2*swingBase*swingMount)) - frameAngle;

}

ActLens Leg::solveIK(const Vec3f& p) {
  /** Given an input foot location p, finds necessary actuator lengths to obtain this position
  */
  
  ActLens output;
    
  // Calculate angle around hip pivot
  float swingAngle = flipped ? PI/2 + atan2f(p.y, p.x) : PI/2-atan2f(p.y, p.x);
    
  // Unproject foot point based on swingAngle so the rest
  // can be solved as flat triangles
  Vec3f f_prime(sqrtf(p.x*p.x + p.y*p.y)-swingArm, 0, p.z);
 
  // Distance from bottom of leg to foot
  float D = Vec3f(0,0,hip).dist(f_prime);  
  
  // Distance from top of leg to foot
  float E = f_prime.mag(); 
  
  // Calculate some interior angles
  float gamma = acosf((femur*femur+D*D-(tibia+cankle)*(tibia+cankle)) /
                (2*femur*D));
  float sigma = acosf((D*D+hip*hip-E*E) / (2*D*hip));
  float alpha = f_prime.x > 0 ? (sigma - gamma) : (TWO_PI - sigma - gamma);
    
  // Law of cosines trigonometry gives magic answers...
  output.a = sqrtf(femur*femur+hip*hip-2*femur*hip*cosf(alpha));
  output.b = sqrtf(femur*femur+tibia*tibia -
             (tibia*((tibia+cankle)*(tibia+cankle)+femur*femur-D*D) /
             (tibia+cankle))); 
  output.c = sqrtf(swingMount*swingMount +
             swingBase*swingBase -
             2*swingMount*swingBase *
             cosf(swingAngle + frameAngle));
                    
  return output;
}


