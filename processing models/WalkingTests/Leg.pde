class Leg {
  // Constants
  static final int PUSH_MODE      = 10;
  static final int LIFT_DOWN_MODE = 15;
  static final int RETURN_MODE    = 20;
  static final int LIFT_UP_MODE   = 25;
  
  static final int DANCE_MODE     = 30;
  
  static final int MOVE_STATE     = 0;
  static final int END_STATE      = 1;
  static final int UP_STATE       = 2;
  static final int DOWN_STATE     = 3;
  
  static final int TOP_VIEW       = 0;
  static final int SIDE_VIEW      = 1;
  static final int FRONT_VIEW     = 2;
  
  // Positional properties
  PVector center;          // Relative to body center
  PVector centerOfRotation;
  float rotation;          // Relative to body x-axis
  
  // Static leg geometry
  float hip      = 368.5;  // Hight of pivot with frame
  float femur    = 212.7;  // Length of first static element (hip to tibia)
  float tibia    = 368.5;  // Length of second static element (femur to ankle)
  float cankle   = 100;    // Extension beyond ankle to foot
  
  float swingArm   = 150;  // Distance from hip pivot to triangulated leg
  float swingMount = 150;  // Distance along swing arm where swing actuator attaches. Should be less than swingArm.
  float swingBase  = 360;  // Distance along frame where swing actuator attaches. Should be less than frameSide.
  float frameAngle = 30;   // Degrees between leg's y-axis and frame member swing actuator is attached to
  float frameSide  = 462;  // Length of frame side. Used only for drawing.  
  boolean flipped;         // true when horizontal actuator mounted on opposite side
  
  static final float moveVectorScaleFactor = 0.01;  // Starting point for scaling target movement speed
  static final float targetFootThreshold = 20;

  // Drawing properties
  float scale = 0.25;
  boolean constructionLines = false;
  boolean drawConfigSpace =   true;
  boolean drawDebug =         false;
  
  // State variables
  int mode;
  int state;
  PVector knee;           // Used for drawing and possibly collision detection?
  PVector ankle;          // Used for drawing
  PVector unprojectFoot;  // Used for drawing
  PVector unprojectKnee;  // Used for drawing
  PVector unprojectAnkle;  // Used for drawing
  PVector hipEnd;
  
  float swingAngle;
  PVector foot;
  PVector target;         // For foot
  
  MoveVector milestone;    // Zero-point for calculating distance/rotation moved
  MoveVector currentMove;  // For calculating distance/rotation moved (since last milestone)
  
  // Member variables
  Controller parent;
  Actuator[] actuators;
  
  Leg(Controller parent, PVector center, float rotation, boolean flipped) {
    this.parent = parent;
    this.center = center.get();
    this.centerOfRotation = center.get();
    this.rotation = rotation;
    this.flipped = flipped;
    
    this.setMode(PUSH_MODE);
    this.state = MOVE_STATE;
    
    actuators = new Actuator[3];
    for(int i=0; i<3; i++) {
      actuators[i] = new Actuator(this, 353, 499, 50);  // parent, min, max, maxSpeed
    }
    
    solveFK();  // Sets foot and other leg geometry based on current actuator lengths

    setTarget(this.foot);
    resetMilestone();
    this.currentMove = new MoveVector(new PVector(0,0,0), 0);
  }
   
  void update() {
    // Handle state transitions
  if(this.state != END_STATE && (this.mode == PUSH_MODE || this.mode == RETURN_MODE)) {
    
    PVector nextTarget = target.get();
    PVector deltaTranslation = PVector.mult(currentMove.translation, (mode == RETURN_MODE ? -1 : 1) * moveVectorScaleFactor);
    PVector translated = PVector.sub(center, centerOfRotation);
    PVector toRotationCenter = PVector.sub(toBodySpace(target), translated);
    //PVector toRotationCenter = toBodySpace(target);
    float rot = (mode == RETURN_MODE ? -1 : 1) * currentMove.rotation * moveVectorScaleFactor;
    PVector rotatedTarget = new PVector(toRotationCenter.x * cos(rot) + toRotationCenter.y * sin(rot),
                                        toRotationCenter.y * cos(rot) - toRotationCenter.x * sin(rot), toRotationCenter.z);
    rotatedTarget.add(translated);
    PVector deltaRotation = PVector.sub(target, toLegSpace(rotatedTarget));
    
    println(deltaRotation);
    
    nextTarget.add(deltaRotation);
    nextTarget.add(deltaTranslation);
    
    centerOfRotation.sub(toBodySpace(deltaTranslation));
    centerOfRotation.add(center);
    
    
    //PVector nextTarget = PVector.add(this.target, PVector.mult(this.currentMove.translation, (mode == RETURN_MODE ? -1 : 1) * moveVectorScaleFactor));
    
    // Don't move target if the foot is too far away still
    if(this.target.dist(this.foot) > targetFootThreshold)
      nextTarget = this.target.get();
      
    if(!canStepTo(nextTarget) || !setTarget(nextTarget)) {
      // End stop condition
      setTarget(this.target, true);
      this.state = END_STATE;
      println(parent.getLegNumber(this) + ": Can't translate any more!" + (this.mode == PUSH_MODE ? " (PUSHING)" : " (RETURNING)"));
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
      println(parent.getLegNumber(this) + ": Can't move up any more!");
    }
  }
  if(this.mode == LIFT_DOWN_MODE && state != DOWN_STATE) {
    moveTargetDown();
    setTarget(PVector.add(this.target, PVector.mult(this.currentMove.translation, moveVectorScaleFactor)), true);
    if(this.foot.z > parent.leg_down) {
      setTarget(this.foot);
      this.state = DOWN_STATE;
      println(parent.getLegNumber(this) + ": Can't move down any more!");
    }
  }
  
  for(int i=0; i<3; i++)
    actuators[i].update();
    
  solveFK();

  }
  
  void draw() {
    draw(TOP_VIEW, 1);
  }
  void draw(int view, float drawScale) {
    switch(view) {
      case TOP_VIEW:
        pushMatrix();
          scale(drawScale);
          rotate(this.rotation);
          
          if(drawConfigSpace) {
            // Calculate current configuration space
            float minX = -1;
            float maxX = -1;
            float sweepX = 0;
            while(maxX == -1) {
              if(minX == -1) {
                if(canStepTo(new PVector(sweepX, 0, parent.leg_down))) {
                  minX = sweepX;
                }  
              }
              else {
                if(!canStepTo(new PVector(sweepX, 0, parent.leg_down))) {
                  maxX = sweepX;
                }
              }
              sweepX += 10;
            }  
            
            float minSwing = solveSwingAngle(actuators[2].minLength) - PI/2;
            float maxSwing = solveSwingAngle(actuators[2].maxLength) - PI/2;
            //println(degrees(minSwing) + ", MAX: " + degrees(maxSwing));
            
            strokeWeight(3);
            stroke(100);
            for(float sweep=(flipped ? maxSwing : minSwing); sweep<(flipped ? minSwing : maxSwing); sweep+= .1) {
              rotate(-sweep);
              line(minX,0,maxX,0);
              rotate(sweep);
            }
            
            strokeWeight(20);
            pushMatrix();
              stroke(255,0,128,100);
              rotate(-minSwing);
              line(minX,0, maxX,0);
            popMatrix();
            pushMatrix();
              stroke(0,128,255,100);
              rotate(-maxSwing);
              line(minX,0, maxX,0);
            popMatrix(); 
          }
          
          // Draw swingarm
          strokeWeight(10/drawScale);
          stroke(255,255,0);
          line(0,0, hipEnd.x,hipEnd.y);
          
          // Draw horizontal actuator
          stroke(0,128,255);
          line(-swingBase*sin(radians(frameAngle)), (flipped ? 1 : -1) * swingBase*cos(radians(frameAngle)),
                -swingMount * cos(swingAngle + PI/2), swingMount * sin(swingAngle + PI/2));
          
          // Draw line from swingarm end to knee and foot
          strokeWeight(10/drawScale);
          stroke(0,128,255,100);
          line(hipEnd.x,hipEnd.y, knee.x,knee.y);
          stroke(255,0,128);
          line(hipEnd.x,hipEnd.y, foot.x,foot.y);
          
          // Draw hip pivot
          fill(0,128,255);
          stroke(0);
          strokeWeight(3);
          ellipse(0,0, 20,20);
          
          // Draw foot
          fill(255,255,0);
          ellipse(foot.x, foot.y, 30,30);  
          
          // Draw target
          noFill();
          stroke(200);
          strokeWeight(1);
          ellipse(target.x, target.y, 10,10);
        popMatrix();
        break;
    }
    
    if(drawDebug) {
      fill(200);
      textAlign(CENTER,CENTER);
      text("Leg #" + parent.getLegNumber(this), this.center.x*drawScale, this.center.y*drawScale);
      text("θ: " + nfs(degrees(getDeltaTheta()), 3,1), this.center.x*drawScale, this.center.y*drawScale+20);
      text("Mode: " + (mode == PUSH_MODE ? "PUSHING" : mode == RETURN_MODE ? "RETURNING" :
                       mode == LIFT_UP_MODE ? "RAISING" : mode == LIFT_DOWN_MODE ? "LOWERING" : "???"),
                       this.center.x*drawScale, this.center.y*drawScale + 30);
                       
      text("State: " + (state == MOVE_STATE ? "MOVE" : state == END_STATE ? "END" :
                       state == UP_STATE ? "UP" : state == DOWN_STATE ? "DOWN" : "???"),
                       this.center.x*drawScale, this.center.y*drawScale + 40);  
    
      text("P=(" + nfs(this.foot.x, 2, 1) + "," + nfs(this.foot.y, 2, 1) + "," + nfs(this.foot.z, 2, 1) + ")",
                       this.center.x*drawScale, this.center.y*drawScale + 50);          
      text("dP=(" + nfs(this.currentMove.translation.x, 2, 1) + "," + nfs(this.currentMove.translation.y, 2, 1) + "," + nfs(this.currentMove.translation.z, 2, 1) + ")",
                       this.center.x*drawScale, this.center.y*drawScale + 60);                       
    }
  }
  
  void toggleMode() {
    if(mode == PUSH_MODE) setMode(RETURN_MODE);
    else if(mode == RETURN_MODE) setMode(PUSH_MODE);
  }
  
  PVector toLegSpace(PVector _p) {
    PVector p = _p.get();
    p.sub(center);
    p = new PVector(cos(-rotation)*p.x - sin(-rotation)*p.y,
                    sin(-rotation)*p.x + cos(-rotation)*p.y, p.z);
    return p;
  }
  
  PVector toBodySpace(PVector _p) {
    PVector p = _p.get();
    p = new PVector(cos(rotation)*p.x - sin(rotation)*p.y,
                    sin(rotation)*p.x + cos(rotation)*p.y, p.z);
    p.add(center);                    
    return p;    
  }
  
  PVector untransformFoot() { 
    return toBodySpace(foot);
  }
  
  boolean canMoveTo(float[] acts) {
    return (acts.length == 3 && actuators[0].isPossible(acts[0]) && actuators[1].isPossible(acts[1]) && actuators[2].isPossible(acts[2]));
  }
  boolean canMoveTo(PVector p) {
    float[] acts = solveIK(p);
    return canMoveTo(acts);
  }
  boolean canStepTo(PVector p) {
    float[] acts1 = solveIK(p);
    float[] acts2 = solveIK(new PVector(p.x, p.y, parent.leg_up));
    float[] acts3 = solveIK(new PVector(p.x, p.y, parent.leg_down));
    
    return (canMoveTo(acts1) && canMoveTo(acts2) && canMoveTo(acts3));
  }
  
  boolean setMode(int m) {
    if(this.mode == m) return false;
    
    switch(m) {
      case PUSH_MODE:
      case RETURN_MODE:
        // Reset rotation center to default
        centerOfRotation = center.get();
        break;
    }
    this.mode = m;
    return true;
  }
  
  boolean setState(int s) {
    if(this.state == s) return false;

    switch(s) {
      case END_STATE:
        freeze();
        break;
      case MOVE_STATE:
        // Reset rotation center to default
        //centerOfRotation = center.get();
        break;
    }
    this.state = s;
    return true;  
  }
  
  
  boolean setTarget(PVector p) {
    return setTarget(p, false);
  }
  boolean setTarget(PVector p, boolean force) {
    float[] acts = solveIK(p);
    if(force || canMoveTo(acts)) {
      this.target = p.get();
      
      for(int i=0; i<3; i++)
        actuators[i].setTarget(acts[i], force);
        
      
        
      String x = nf((int)(p.x*100), 5,0);
      String y = nf((int)(p.y*100), 5,0);
      String z = nf((int)(p.z*100), 5,0);
      String out = "!L" + parent.getLegNumber(this) + "x" + x + "y" + y + "z" + z + "*";
      //println(out);
      parent.serial.write(out);        
      
      return true;
    }
    else return false;
  }

  public void moveTargetUp() {
    setMoveVector(new PVector(0,0,-200), 0);
  }  

  public void moveTargetDown() {
    setMoveVector(new PVector(0,0,200), 0);
  }
  
  public void setCenterTarget() {
    // TODO: Make this smarter. It should use the current move vector as a heuristic
    float[] acts = {actuators[0].midLength, actuators[1].midLength, actuators[2].midLength};
    PVector middle = solveFK(acts);
    setTarget(new PVector(middle.x, 0, parent.leg_up), true);  
  }
  
  public void freeze() {
    solveFK();
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
    /** Gets the angle from the body's center to the given position */
    // Rotate foot
    PVector P = new PVector(cos(rotation)*_P.x - sin(rotation)*_P.y, sin(rotation)*_P.x + cos(rotation)*_P.y);
    // Translate by center distance
    P.add(this.center);
    // Calculate arctan2 to get angle
    return atan2(P.y, P.x);
  }
  
  PVector solveFK() {
    float[] acts = {actuators[0].length, actuators[1].length, actuators[2].length};
    return solveFK(acts, true);  
  }
  PVector solveFK(float[] acts) {
    return solveFK(acts, false);
  }
  PVector solveFK(float[] acts, boolean assign) {
    /** Solves leg geometry (knee, ankle, foot, swingAngle, unprojectFoot) based on current actuator lengths.
    */
    float alpha = acos((hip*hip + femur*femur - acts[0]*acts[0]) / (2*hip*femur));        // Interior angles
    float beta  = acos((femur*femur + tibia*tibia - acts[1]*acts[1]) / (2*femur*tibia));
  
    PVector unprojectKnee = new PVector(swingArm+femur*sin(alpha), 0, hip-femur*cos(alpha));
    PVector unprojectAnkle = new PVector((tibia)*sin(beta-alpha), 0, (tibia)*cos(beta-alpha));
    unprojectAnkle.add(unprojectKnee);
  
    PVector unprojectFoot = new PVector((tibia+cankle)*sin(beta-alpha), 0, (tibia+cankle)*cos(beta-alpha));
    unprojectFoot.add(unprojectKnee);
  
    float swingAngle = solveSwingAngle(acts[2]);
    
    PVector foot   = new PVector(unprojectFoot.x*sin(swingAngle), unprojectFoot.x*cos(swingAngle), unprojectFoot.z);
    PVector knee   = new PVector(unprojectKnee.x*sin(swingAngle), unprojectKnee.x*cos(swingAngle), unprojectKnee.z);
    PVector ankle  = new PVector(unprojectAnkle.x*sin(swingAngle), unprojectAnkle.x*cos(swingAngle), unprojectAnkle.z);
    PVector hipEnd = new PVector(swingArm*sin(swingAngle), swingArm*cos(swingAngle), hip);
    
    if(assign) {
      this.knee = knee;
      this.ankle = ankle;
      this.unprojectKnee = unprojectKnee;
      this.unprojectAnkle = unprojectAnkle;
      this.unprojectFoot = unprojectFoot;
      this.hipEnd = hipEnd;
      this.swingAngle = swingAngle;
      this.foot = foot;
    }
    
    return foot;
  }
  
  float solveSwingAngle(float act) {
    if(flipped) return PI - (acos((swingMount*swingMount + swingBase*swingBase - act*act) /
                            (2*swingBase*swingMount)) - radians(frameAngle));
                            
    else        return acos((swingMount*swingMount + swingBase*swingBase - act*act) /
                            (2*swingBase*swingMount)) - radians(frameAngle);
  }
  
  float[] solveIK(PVector p) {
    /** Given an input foot location p, finds necessary actuator lengths to obtain this position
    */
    
    float[] output = new float[3];
    
    // Calculate angle around hip pivot
    float swingAngle = flipped ? PI/2 + atan2(p.y, p.x) : PI/2-atan2(p.y, p.x);
    
    // Unproject foot point based on swingAngle so the rest can be solved as flat triangles
    PVector f_prime = new PVector(sqrt(p.x*p.x + p.y*p.y)-swingArm, 0, p.z);
    float D = new PVector(0,0,hip).dist(f_prime);  // Distance from bottom of leg to foot 
    float E = f_prime.mag();                       // Distance from top of leg to foot
    float gamma = acos((femur*femur+D*D-(tibia+cankle)*(tibia+cankle)) / (2*femur*D));
    float sigma = acos((D*D+hip*hip-E*E) / (2*D*hip));
    float alpha = f_prime.x > 0 ? (sigma - gamma) : (TWO_PI - sigma - gamma);  // Conditional checks whether foot is inside swingArm region
    
    // Law of cosines trigonometry gives magic answers...
    output[0] = sqrt(femur*femur+hip*hip-2*femur*hip*cos(alpha));
    output[1] = sqrt(femur*femur+tibia*tibia - (tibia*((tibia+cankle)*(tibia+cankle)+femur*femur-D*D) / (tibia+cankle))); 
    output[2] = sqrt(swingMount*swingMount + swingBase*swingBase - 2*swingMount*swingBase *
                    cos(swingAngle + radians(frameAngle)));
                    
    return output;
  }
  
  void drawSide() {
    pushMatrix();
    scale(0.5);
    //scale(cos(swingAngle),1);
  
    // Swingarm box
    stroke(255,255,0);
    strokeWeight(8);
    noFill();
    line(0,0, swingArm,0);
    line(0,hip, swingArm,hip);
    line(0,0, 0,hip);
    stroke(255,0,128);
    line(swingArm,0, swingArm,hip);
  
    if(constructionLines) {
      // Circles for frame elements
      stroke(255,0,128);
      strokeWeight(3);
      ellipse(swingArm,hip, femur*2, femur*2);
      stroke(0,128,255);
      ellipse(swingArm,0, actuators[0].length*2, actuators[0].length*2);
    }
  
    // Lines to knee
    stroke(255,0,128);
    strokeWeight(8);
  
    line(swingArm,hip, knee.x,knee.z);
    stroke(0,128,255);
    strokeWeight(20);
    line(swingArm,0,   knee.x,knee.z);
  
    if(constructionLines) {
      // Circles for tibia and bottom actuator
      stroke(255,0,128);
      strokeWeight(3);
      ellipse(knee.x, knee.z, (tibia)*2,(tibia)*2);
      stroke(0,128,255);
      ellipse(swingArm,hip, actuators[1].length*2, actuators[1].length*2);
    }
  
    // Line to ankle
    strokeWeight(20);
    stroke(0,128,255);
    line(swingArm,hip, ankle.x,ankle.z);
    strokeWeight(8);
    stroke(255,0,128);
    line(knee.x,knee.z, unprojectFoot.x, unprojectFoot.z);
  
    // Draw knuckles at joints
    fill(255,255,0);
    stroke(20);
    strokeWeight(5);
    ellipse(swingArm,0, 30,30);
    ellipse(swingArm,hip, 30,30);
    ellipse(knee.x, knee.z, 30,30);
    ellipse(ankle.x, ankle.z, 30,30);
  
    // Foot
    fill(255,0,128);
    stroke(20);
    strokeWeight(5);
    ellipse(unprojectFoot.x, unprojectFoot.z, 40,40);
    fill(255);
    text("[" + nfs(foot.x, 3,1) + ", " + nfs(foot.y, 3,1) + ", " + nfs(foot.z, 3,1) + "]", unprojectFoot.x+40, unprojectFoot.z);
    
    popMatrix();
  }  
}
