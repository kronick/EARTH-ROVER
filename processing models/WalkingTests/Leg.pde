class Leg {
  // Constants
  static final int PUSH_MODE     = 10;
  static final int LIFT_DOWN_MODE = 15;
  static final int RETURN_MODE   = 20;
  static final int LIFT_UP_MODE  = 25;
  
  static final int DANCE_MODE    = 30;
  
  static final int MOVE_STATE    = 0;
  static final int END_STATE     = 1;
  static final int UP_STATE      = 2;
  static final int DOWN_STATE    = 3;
  
  // Physical properties
  PVector center;
  float rotation;
  float rangeX, rangeY, rangeZ;
  float maxSpeed = 3;
  float noiseFactor;
  PVector creep;
  static final float easing = 0.1;
  static final float moveVectorScaleFactor = 0.007;
  
  Controller parent;
  
  // State variables
  int mode;
  int state;
  PVector foot;
  PVector target;
  MoveVector milestone;
  MoveVector currentMove;
  
  Leg(Controller parent, PVector center, float rotation, float x, float y, float z) {
    this.parent = parent;
    this.center = center.get();
    this.rotation = rotation;
    
    this.rangeX = x;
    this.rangeY = y;
    this.rangeZ = z;
    
    this.mode = PUSH_MODE;
    this.state = MOVE_STATE;
    this.foot = new PVector(x/2,0,parent.leg_up + (parent.leg_down-parent.leg_up)/2);
    setTarget(this.foot);
    resetMilestone();
    this.currentMove = new MoveVector(new PVector(0,0,0), 0);
    
    // Noise randomly generated in this range each tick
    noiseFactor = random(0,maxSpeed/80.);
    float creepFactor = maxSpeed/100.;
    creep = new PVector(random(-creepFactor, creepFactor),
                                random(-creepFactor, creepFactor),
                                random(-creepFactor, creepFactor));
  }
  
  boolean canMoveTo(PVector p) {
    return (p.x < rangeX && p.x > 0 && abs(p.y) < rangeY && abs(p.z) < rangeZ);
  }
  
  void update() {
    // Handle state transitions
    if(this.mode == PUSH_MODE || this.mode == RETURN_MODE) {
      if(!setTarget(PVector.add(this.target, PVector.mult(this.currentMove.translation, (mode == RETURN_MODE ? -1 : 1) * moveVectorScaleFactor)))) {
        // End stop condition
        setTarget(this.target);
        this.state = END_STATE;
        println("Can't translate any more!");
      }
      else this.state = MOVE_STATE;
    }
    
    if(this.mode == LIFT_UP_MODE && state != UP_STATE) {
      moveTargetUp();
      setTarget(PVector.add(this.target, PVector.mult(this.currentMove.translation, moveVectorScaleFactor)), true);
      if(this.foot.z < parent.leg_up) {
        // End stop condition
        setTarget(this.foot);
        this.state = UP_STATE;
        println("Can't move up any more!");
      }
    }
    if(this.mode == LIFT_DOWN_MODE && state != DOWN_STATE) {
      moveTargetDown();
      setTarget(PVector.add(this.target, PVector.mult(this.currentMove.translation, moveVectorScaleFactor)), true);
      if(this.foot.z > parent.leg_down) {
        setTarget(this.foot);
        this.state = DOWN_STATE;
        println("Can't move up any more!");
      }
    }
    
    PVector dP = PVector.sub(this.target, this.foot);
    dP.mult(easing);
    
    // Limit speed
    PVector _dP = dP.get();
    if(dP.mag() > maxSpeed) {
      dP.normalize();
      dP.mult(maxSpeed);
      if((dP.x > 0 && _dP.x < 0) || (dP.x < 0 && _dP.x > 0) ||
          (dP.y > 0 && _dP.y < 0) || (dP.y < 0 && _dP.y > 0) ||
          (dP.z > 0 && _dP.z < 0) || (dP.z < 0 && _dP.z > 0))
        dP.mult(-1);  // Correct direction if flipped
    }
    
    PVector noise = new PVector(random(-noiseFactor, noiseFactor),
                                random(-noiseFactor, noiseFactor),
                                random(-noiseFactor, noiseFactor));
    
    PVector newFoot = PVector.add(this.foot, dP);
    newFoot.add(creep);
    newFoot.add(noise);
    this.foot = newFoot;

  }
  
  void draw() {
    pushMatrix();
      translate(this.center.x, this.center.y);
      rotate(this.rotation);
      
      // Draw line from base to foot
      stroke(255,0,128);
      strokeWeight(4);
      line(0,0, foot.x, foot.y);
      
      // Draw base
      fill(0,128,255);
      stroke(0);
      strokeWeight(3);
      ellipse(0,0, 20,20);
      
      // Draw foot
      fill(255,255,0);
      ellipse(foot.x, foot.y, foot.z, foot.z);  
      
      // Draw target
      noFill();
      stroke(200);
      strokeWeight(1);
      ellipse(target.x, target.y, 10,10);
    popMatrix();
    
    fill(200);
    textAlign(CENTER,CENTER);
    text("θ: " + nfs(degrees(getDeltaTheta()), 3,1), this.center.x, this.center.y+20);
    text("Mode: " + (mode == PUSH_MODE ? "PUSHING" : mode == RETURN_MODE ? "RETURNING" :
                     mode == LIFT_UP_MODE ? "RAISING" : mode == LIFT_DOWN_MODE ? "LOWERING" : "???"),
                     this.center.x, this.center.y + 30);
                     
    text("State: " + (state == MOVE_STATE ? "MOVE" : state == END_STATE ? "END" :
                     state == UP_STATE ? "UP" : state == DOWN_STATE ? "DOWN" : "???"),
                     this.center.x, this.center.y + 40);  
  
    text("P=(" + nfs(this.foot.x, 2, 1) + "," + nfs(this.foot.y, 2, 1) + "," + nfs(this.foot.z, 2, 1) + ")",
                     this.center.x, this.center.y + 50);          
    text("dP=(" + nfs(this.currentMove.translation.x, 2, 1) + "," + nfs(this.currentMove.translation.y, 2, 1) + "," + nfs(this.currentMove.translation.z, 2, 1) + ")",
                     this.center.x, this.center.y + 60);                       
  }
  
  void toggleMode() {
    if(mode == PUSH_MODE) mode = RETURN_MODE;
    else if(mode == RETURN_MODE) mode = PUSH_MODE;
  }
  
  boolean setTarget(PVector p) {
    return setTarget(p, false);
  }
  boolean setTarget(PVector p, boolean force) {
    if(canMoveTo(p) || force) {
      this.target = p.get();
      return true;
    }
    else return false;
  }

  public void moveTargetUp() {
    setMoveVector(new PVector(0,0,-100), 0);
  }  

  public void moveTargetDown() {
    setMoveVector(new PVector(0,0,100), 0);
  }
  
  public void setCenterTarget() {
    setTarget(new PVector(rangeX/2, 0, this.foot.z));  
  }
  
  public void freeze() {
    setTarget(this.foot);
    setMoveVector(new MoveVector());
  }
  
  void setMoveVector(PVector t, float r) {
    setMoveVector(new MoveVector(t, r));
  }
  void setMoveVector(MoveVector m) {
    //this.state = MOVE_STATE;
    this.currentMove = new MoveVector(new PVector(cos(-rotation)*m.translation.x - sin(-rotation)*m.translation.y,
                                                  sin(-rotation)*m.translation.x + cos(-rotation)*m.translation.y, m.translation.z),
                                      m.rotation);
  }
  
  void resetMilestone() {
    this.milestone = new MoveVector(this.foot, getAngle());
  }
  
  float getDeltaP() {
    // TODO: Fix this so it takes into account rotation
    return this.milestone.translation.dist(this.foot);
  }
  
  float getDeltaTheta() {
    float out = getAngle() - this.milestone.rotation;
    if(out > PI) out -= 2*PI;
    else if(out < -PI) out += 2*PI;
    return out;
  }

  float getAngle() {
    return getAngle(this.foot);
  }
  
  float getAngle(PVector _P) {  
    /** Gets the angle from the body's center to the given position **/
    // Rotate foot
    PVector P = new PVector(cos(rotation)*_P.x - sin(rotation)*_P.y, sin(rotation)*_P.x + cos(rotation)*_P.y);
    // Translate by center distance
    P.add(this.center);
    // Calculate arctan2 to get angle
    return atan2(P.y, P.x);
  }
}
