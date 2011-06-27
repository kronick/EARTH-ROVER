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
  
  float leg_down = 40;
  float leg_up   = 20;

  
  Controller(int bodyType) {
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
        c = new PVector(cos(rot) * 200, sin(rot) * 200, 0);
      }
      
      legs[i] = new Leg(this, c, rot, 100,100,100);
      legs[i].mode = i % 2 == 0 ? Leg.PUSH_MODE : Leg.RETURN_MODE;
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
          legs[i].freeze();
          if(legs[i].mode == Leg.PUSH_MODE) {
            legs[i].mode = Leg.LIFT_UP_MODE;
            legs[i].moveTargetUp();
          }
          if(legs[i].mode == Leg.RETURN_MODE) {
            legs[i].mode = Leg.LIFT_DOWN_MODE;
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
            legs[i].freeze();
          }        
          state = PUSH_CMD_STATE;
        }
        break;
      case PUSH_CMD_STATE:
        // Set new leg motion vectors
        for(int i=0; i<LEG_COUNT; i++) {
          legs[i].freeze();
          if(legs[i].mode == Leg.LIFT_UP_MODE) {
            legs[i].setCenterTarget();
            legs[i].mode = Leg.RETURN_MODE;
          }
          if(legs[i].mode == Leg.LIFT_DOWN_MODE) {
            legs[i].mode = Leg.PUSH_MODE;
          }          
        }                
        state = PUSH_WAIT_STATE;
        break;
      case PUSH_WAIT_STATE: {
        boolean endStop = false;
        for(int i=0; i<LEG_COUNT; i++) {
          if(legs[i].mode == Leg.PUSH_MODE && legs[i].state == Leg.END_STATE)
            endStop = true;
        }
        if(endStop) { // Is any leg forward/backward?
          for(int i=0; i<LEG_COUNT; i++) {
            legs[i].freeze();
          }
          state = LIFT_CMD_STATE;
          println("Transitioning to lift state");
        }
        break;
      }
    }
    
    for(int i=0; i<LEG_COUNT; i++) {
      legs[i].update();
    }
  }
  
  void draw() {
    for(int i=0; i<LEG_COUNT; i++) {
      legs[i].draw();
    }
    
    text(state == LIFT_CMD_STATE ? "LIFT COMMAND" : state == LIFT_WAIT_STATE ? "LIFT WAIT" :
          state == PUSH_CMD_STATE ? "PUSH COMMAND" : state == PUSH_WAIT_STATE ? "PUSH WAIT" : "???", 0,20);
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
  
  void legEndEvent(Leg l) {
      
  }
}
  
