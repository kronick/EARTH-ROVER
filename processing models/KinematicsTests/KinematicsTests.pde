import processing.serial.*;

Controller controller;

PFont monofont;
static final int X_AXIS = 0;
static final int Y_AXIS = 1;
static final int Z_AXIS = 2;
static final int PITCH_AXIS = 10;
static final int YAW_AXIS = 11;
static final int ROLL_AXIS = 12;
int axis;

void setup() {
  size(800,600);
  colorMode(RGB);
  smooth();
  
  frameRate(45);
  
  monofont = createFont("Courier", 10);
  textFont(monofont);
  
  controller = new Controller(this, Controller.HEX_BODY);
  controller.state = Controller.MANUAL_STATE;
  axis = X_AXIS;
}

void draw() {
  background(10);
  float r = 40*sin(frameCount/100.);
  if(axis == YAW_AXIS) {
    controller.changeMoveVector(new MoveVector(new PVector(0,0,0),
                                0.2*sin(radians(frameCount*3)), 0.07*cos(radians(frameCount*7)), 0));
  }
  //PVector t = new PVector(r*cos(radians(frameCount)*20), r*sin(radians(frameCount)*20), 0);
  //println("R: " + r + " t: " + t);
  //controller.changeMoveVector(new MoveVector(t, 0));
  controller.update();
  
  fill(255);
  textAlign(LEFT);
  text(nf(frameRate, 2,2), 10,10);
  
  pushMatrix();
    translate(width/2, height/2);
    scale(1);
    
    fill(200);
    noStroke();
    ellipse(0,0, 10,10);
    
    controller.draw();
  popMatrix();

}

void keyPressed() {
  switch(key) {
    case '1':
      axis = X_AXIS;
      break;
    case '2':
      axis = Y_AXIS;
      break;
    case '3':
      axis = Z_AXIS;
      break;
    case '4':
      axis = PITCH_AXIS;
      break;
    case '5':
      axis = YAW_AXIS;
      break;
  }
}

void mousePressed() {
  mouseDragged();
}

void mouseDragged() {
  PVector t = new PVector(0,0,0);
  float pitch = 0;
  float yaw = 0;
  float roll = 0;
  switch(axis) {
    case X_AXIS:
      t = new PVector(-(mouseX-width/2), height-mouseY-height/2,0);
      break;
    case Y_AXIS:
      t = new PVector(0, height-mouseY-height/2,0);
      break;
    case Z_AXIS:
      t = new PVector(0,0,height-mouseY-height/2);
      break;
    case PITCH_AXIS:
      pitch = (height-mouseY-height/2) / 20.;
      yaw = -(mouseX-width/2) / 20.;
      break;
  }
  controller.changeMoveVector(new MoveVector(t, radians(yaw), radians(pitch), radians(roll)));
}

void mouseReleased() {
  //controller.changeMoveVector(new MoveVector(new PVector(0,0,0), 0));
}
