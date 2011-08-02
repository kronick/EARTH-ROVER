#include "Leg.h"
#include "Controller.h"
#include "RemoteMan.h"

Leg::Leg() {
  this->parent = 0;
  this->center = Vec3f(0,0,0);
  this->rotation = 0;
  this->flipped = false;
}

Leg::Leg(Controller* parent, Vec3f center, float rotation, boolean flipped,
          int pwmPins[], int potPins[], bool simulate) {
  this->parent = parent;
  this->center = center;
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
  switch (mode) {
    case MANUAL_MODE:
      break;
    default:
      if(state != END_STATE && (mode == PUSH_MODE || mode == RETURN_MODE)) {
        Vec3f nextTarget = target;
       
        nextTarget += currentMove.translation *
                    (mode == RETURN_MODE ? -1 : 1) * MOVE_FACTOR;
      
        // Don't move target if the foot is too far away still
        if(target.dist(foot) > TARGET_THRESHOLD)
          nextTarget = target;
      
        if(!canStepTo(nextTarget) || !setTarget(nextTarget)) {
          // End stop condition
          setTarget(target, true);
          changeState(END_STATE);
        }
        else changeState(MOVE_STATE);
      }
  
      if(mode == LIFT_UP_MODE && state != UP_STATE) {
        moveTargetUp();
        setTarget(target + currentMove.translation * MOVE_FACTOR, true);
        if(foot.z < parent->getUpHeight()) {
          // End stop condition
          setTarget(foot);
          changeState(UP_STATE);
          //println(parent.getLegNumber(this) + ": Can't move up any more!");
        }
      }
      if(mode == LIFT_DOWN_MODE && state != DOWN_STATE) {
        moveTargetDown();
        setTarget(target + currentMove.translation * MOVE_FACTOR, true);
        if(foot.z > parent->getDownHeight()) {
          setTarget(foot);
          changeState(DOWN_STATE);
          //println(parent.getLegNumber(this) + ": Can't move down any more!");
        }
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

  if(!remote)
    for(int i=0; i<3; i++)
      actuators[i].update();
    
  solveFK();

}

void Leg::synchronizeRemote(const Vec3f& remoteFoot){
  if(!remote) {
    // This is an incoming command. Call methods to change state.
    setTarget(remoteFoot);
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
  if(!remote)
    changeMode(remoteMode);
  else
    this->mode = remoteMode;
}

void Leg::synchronizeRemote(State remoteState) {
  if(!remote)
    changeState(remoteState);
  else
    this->state = remoteState;
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
    if(!remote) mode = m;
    else RemoteManager.sendMode(remotePort, remoteIndex, m);
  }

  return true;
}
Mode Leg::getMode() {
  return mode;
}

bool Leg::changeState(State s) {
  if(state == s) return false;
  else {
    if(!remote) {
      if(s == END_STATE)
        freeze();
   
      state = s;
    }
    else RemoteManager.sendState(remotePort, remoteIndex, s);
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
  setMoveVector(Vec3f(0,0,-200), 0);
}

void Leg::moveTargetDown() {
  setMoveVector(Vec3f(0,0,200), 0);
}

void Leg::setCenterTarget() {
  ActLens acts = {actuators[0].getMidLength(), actuators[1].getMidLength(),
                  actuators[2].getMidLength()};
  Vec3f middle = solveFK(acts);
  setTarget(Vec3f(middle.x, 0, parent->getUpHeight()), true);
}

    
void Leg::setMoveVector(Vec3f t, float r) {
  t.rotateZ(-rotation);
  currentMove = MoveVector(t, r);
}


void Leg::setMoveVector(MoveVector m) {
  Vec3f translation(m.translation);
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
  solveFK();
  setTarget(foot);
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
  float alpha = acos((hip*hip + femur*femur - acts.a*acts.a) / (2*hip*femur)); 
  float beta  = acos((femur*femur + tibia*tibia - acts.b*acts.b) /
                      (2*femur*tibia));

  Vec3f unprojectKnee(swingArm+femur*sin(alpha), 0, hip-femur*cos(alpha));
  Vec3f unprojectAnkle((tibia)*sin(beta-alpha), 0, (tibia)*cos(beta-alpha));
  unprojectAnkle += unprojectKnee;

  Vec3f unprojectFoot((tibia+cankle)*sin(beta-alpha), 0,
                      (tibia+cankle)*cos(beta-alpha));
  unprojectFoot += unprojectKnee;

  float swingAngle = solveSwingAngle(acts.c);
  
  Vec3f foot(unprojectFoot.x*sin(swingAngle), unprojectFoot.x*cos(swingAngle),
             unprojectFoot.z);
  Vec3f knee(unprojectKnee.x*sin(swingAngle), unprojectKnee.x*cos(swingAngle),
             unprojectKnee.z);
  Vec3f ankle(unprojectAnkle.x*sin(swingAngle),
              unprojectAnkle.x*cos(swingAngle),
              unprojectAnkle.z);
  Vec3f hipEnd(swingArm*sin(swingAngle), swingArm*cos(swingAngle), hip);
 
  if(assign) {
    this->knee = knee;
    this->ankle = ankle;
    this->hipEnd = hipEnd;
    this->swingAngle = swingAngle;
    this->foot = foot;
  }
  
  return foot;

}
    
float Leg::solveSwingAngle(float act) {
  if(flipped) return PI - (acos((swingMount*swingMount + swingBase*swingBase -
                    act*act) / (2*swingBase*swingMount)) - frameAngle);
                            
  else        return acos((swingMount*swingMount + swingBase*swingBase -
                     act*act) / (2*swingBase*swingMount)) - frameAngle;

}

ActLens Leg::solveIK(const Vec3f& p) {
  /** Given an input foot location p, finds necessary actuator lengths to obtain this position
  */
  
  ActLens output;
    
  // Calculate angle around hip pivot
  float swingAngle = flipped ? PI/2 + atan2(p.y, p.x) : PI/2-atan2(p.y, p.x);
    
  // Unproject foot point based on swingAngle so the rest
  // can be solved as flat triangles
  Vec3f f_prime(sqrt(p.x*p.x + p.y*p.y)-swingArm, 0, p.z);
 
  // Distance from bottom of leg to foot
  float D = Vec3f(0,0,hip).dist(f_prime);  
  
  // Distance from top of leg to foot
  float E = f_prime.mag(); 
  
  // Calculate some interior angles
  float gamma = acos((femur*femur+D*D-(tibia+cankle)*(tibia+cankle)) /
                (2*femur*D));
  float sigma = acos((D*D+hip*hip-E*E) / (2*D*hip));
  float alpha = f_prime.x > 0 ? (sigma - gamma) : (TWO_PI - sigma - gamma);
    
  // Law of cosines trigonometry gives magic answers...
  output.a = sqrt(femur*femur+hip*hip-2*femur*hip*cos(alpha));
  output.b = sqrt(femur*femur+tibia*tibia -
             (tibia*((tibia+cankle)*(tibia+cankle)+femur*femur-D*D) /
             (tibia+cankle))); 
  output.c = sqrt(swingMount*swingMount +
             swingBase*swingBase -
             2*swingMount*swingBase *
             cos(swingAngle + frameAngle));
                    
  return output;
}


