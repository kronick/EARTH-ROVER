import processing.serial.*;

Controller controller;

PFont monofont;

void setup() {
  size(800,600);
  colorMode(RGB);
  smooth();
  
  frameRate(45);
  
  monofont = createFont("Courier", 10);
  textFont(monofont);
  
  controller = new Controller(this, Controller.HEX_BODY);
}

void draw() {
  background(10);

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

void mousePressed() {
  mouseDragged();
}

void mouseDragged() {
  if(mouseButton == LEFT) {
    PVector t = new PVector(-(mouseX-width/2), height-mouseY-height/2,0);
    t.mult(2);
    controller.changeMoveVector(new MoveVector(t, 0));
  }
  else {
    controller.changeMoveVector(new MoveVector(controller.currentMove.translation, radians(-(mouseX-width/2))));
  }
  //PVector t = new PVector(0, (height-mouseY-height/2)*2,0);
  //controller.changeMoveVector(new MoveVector(t, radians(-(mouseX-width/2))));
}

void mouseReleased() {
  //controller.changeMoveVector(new MoveVector(new PVector(0,0,0), 0));
}
