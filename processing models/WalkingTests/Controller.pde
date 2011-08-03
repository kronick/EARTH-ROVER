class Controller {
  // Constants
  static final int HEX_BODY = 1;
  static final int RECT_BODY = 2;
  static final int TRI_BODY = 3;
  
  static final int LIFT_CMD_STATE  = 10;
  static final int LIFT_WAIT_STATE = 15;
  static final int PUSH_CMD_STATE  = 20;
  static final int PUSH_WAIT_STATE = 25;
  
  static final int LEG_COUNT = 6;
  
  // Members    
  Leg[] legs = new Leg[LEG_COUNT];
  
  // State variables
  int state;
  MoveVector currentMove;
  MoveVector nextMove;

  float[] relaxedDistance = {0,0,0};
  float[] stretchDistance = {0,0,0};
  
  // Step height stuff
  //float leg_down = 920;  // 890
  //float leg_up   = 790;  // 830
  float leg_down = 890;
  float leg_up = 830;
  
  // Drawing controls
  float drawScale = .35;

  PApplet parent;
  Serial serial;
  
  Controller(PApplet parent, int bodyType) {
    this.parent = parent;
    serial = new Serial(parent, Serial.list()[0], 9600);

    this.state = LIFT_CMD_STATE;
    
    // Initialize legs
    float rot = 0;
    PVector c = new PVector(0,0,0);
    for(int i=0; i<LEG_COUNT; i++) {
      
      if(bodyType == RECT_BODY) {
        rot = i<LEG_COUNT/2 ? PI : 0;
        c = new PVector(i<LEG_COUNT/2 ? -100 : 100, (i%(LEG_COUNT/2)-1) * 100, 0);
      }
      else if(bodyType == HEX_BODY) {
        rot = -(PI - i/(float)(LEG_COUNT/2) * PI);
        c = new PVector(cos(rot) * 462, sin(rot) * 462, 0);
      }
      
      legs[i] = new Leg(this, c, rot, i%2 == 0 ? true : false);
      legs[i].setMode(i % 2 == 0 ? Leg.PUSH_MODE : Leg.RETURN_MODE);
    }
    
    changeMoveVector(new MoveVector(new PVector(0,0,0), 0));
  }
  
  void update() {
    if(true) {
      setCurrentMoveVector(this.nextMove);
    }
    
    switch(state) {
      case LIFT_CMD_STATE:
        // Set new leg targets
        for(int i=0; i<LEG_COUNT; i++) {
          //legs[i].freeze();
          if(legs[i].mode == Leg.PUSH_MODE) {
            legs[i].setMode(Leg.LIFT_UP_MODE);
            legs[i].moveTargetUp();
          }
          if(legs[i].mode == Leg.RETURN_MODE) {
            legs[i].setMode(Leg.LIFT_DOWN_MODE);
            legs[i].moveTargetDown();
          }          
        }        
        state = LIFT_WAIT_STATE;
        break;
      case LIFT_WAIT_STATE:
        boolean allLegsUp = true;
        boolean allLegsDown = true;
        for(int i=0; i<LEG_COUNT; i++) {
          if(legs[i].mode == Leg.LIFT_UP_MODE && legs[i].state != Leg.UP_STATE)
            allLegsUp = false;
          if(legs[i].mode == Leg.LIFT_DOWN_MODE && legs[i].state != Leg.DOWN_STATE)
            allLegsDown = false;            
        }      
        if(allLegsUp && allLegsDown) {  // Check if all feet are up/down
          for(int i=0; i<LEG_COUNT; i++) {
            //legs[i].freeze();
          }        
          state = PUSH_CMD_STATE;
        }
        break;
      case PUSH_CMD_STATE:
        // Set new leg motion vectors and reset leg stretch data
        int relaxedIndex = 0;
        int firstRelaxed = -1;
        int lastRelaxed = -1;
        for(int i=0; i<LEG_COUNT; i++) {
          //legs[i].freeze();
          if(legs[i].mode == Leg.LIFT_UP_MODE) {
            legs[i].setCenterTarget();
            legs[i].setMode(Leg.RETURN_MODE);
          }
          if(legs[i].mode == Leg.LIFT_DOWN_MODE) {
            if(lastRelaxed == -1) {
              lastRelaxed = i;
              firstRelaxed = i;
            }
            else {
              relaxedDistance[relaxedIndex++] = legs[lastRelaxed].untransformFoot().dist(legs[i].untransformFoot());
              lastRelaxed = i;
              if(relaxedIndex == relaxedDistance.length-1)
                relaxedDistance[relaxedIndex] = legs[firstRelaxed].untransformFoot().dist(legs[i].untransformFoot());
            } 
            legs[i].setMode(Leg.PUSH_MODE);
          }          
        }           
        
        state = PUSH_WAIT_STATE;
        break;
      case PUSH_WAIT_STATE: {
        boolean endStop = false;
        boolean returned = true;

        int stretchIndex = 0;
        int firstStretch = -1;
        int lastStretch = -1;
        
        for(int i=0; i<LEG_COUNT; i++) {
          if(legs[i].mode == Leg.PUSH_MODE) {
            if(lastStretch == -1) {
              firstStretch = i;
              lastStretch = i;
            }
            else {
              stretchDistance[stretchIndex++] = legs[lastStretch].untransformFoot().dist(legs[i].untransformFoot());
              lastStretch = i;
              
              if(stretchIndex == stretchDistance.length - 1)
                stretchDistance[stretchIndex] = legs[firstStretch].untransformFoot().dist(legs[i].untransformFoot());
            } 
            
            if(legs[i].state == Leg.END_STATE)
              endStop = true;
          }
          else if(legs[i].mode == Leg.RETURN_MODE && legs[i].state != Leg.END_STATE) {
            returned = false;
          }
        }
        if(endStop) { // Is any pushing leg forward/backward?
          for(int i=0; i<LEG_COUNT; i++) {
            if(legs[i].mode == Leg.PUSH_MODE) {
              legs[i].setState(Leg.END_STATE);
            }
          }
          if(returned) {
            state = LIFT_CMD_STATE;
            println("Transitioning to lift state");
          }
        }
        break;
      }
    }
    
    for(int i=0; i<LEG_COUNT; i++) {
      legs[i].update();
    }
  }
  
  void draw() {
    drawStretchMarks();
    // Draw body
    stroke(255,255,0);
    fill(50);
    strokeWeight(10);
    beginShape();
    for(int i=0; i<LEG_COUNT; i++) {
      vertex(legs[i].center.x*drawScale, legs[i].center.y*drawScale);  
    }
    endShape(CLOSE);
    
    for(int i=0; i<LEG_COUNT; i++) {
      pushMatrix();
      translate(legs[i].center.x*drawScale, legs[i].center.y*drawScale);
      legs[i].draw(Leg.TOP_VIEW, drawScale);
      if(i == 1) {
        translate(0,-200);
        //legs[i].drawSide();
      }
      popMatrix();
    }
    
    text(state == LIFT_CMD_STATE ? "LIFT COMMAND" : state == LIFT_WAIT_STATE ? "LIFT WAIT" :
          state == PUSH_CMD_STATE ? "PUSH COMMAND" : state == PUSH_WAIT_STATE ? "PUSH WAIT" : "???", 0,20);
  }
  
  void drawStretchMarks() {
      for(int i=0; i<relaxedDistance.length; i++) {
        text(nfs(stretchDistance[i] - relaxedDistance[i], 2,3), 0, (i+4)*10);
      }
  }
  
  private void setCurrentMoveVector(MoveVector v) {
    this.currentMove = v.get();
    for(int i=0; i<LEG_COUNT; i++) {
      legs[i].setMoveVector(this.currentMove);
    }
  }
  
  void changeMoveVector(MoveVector v) {
    this.nextMove = v.get();
  }
  
  int getLegNumber(Leg l) {
    for(int i=0; i<LEG_COUNT; i++) {
      if(legs[i] == l) return i;
    }
    return -1;
  }
 
}
  
