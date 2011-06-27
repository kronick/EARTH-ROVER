Controller controller;

PFont monofont;

void setup() {
  size(800,600);
  colorMode(RGB);
  smooth();
  
  monofont = createFont("Courier", 10);
  textFont(monofont);
  
  controller = new Controller(Controller.HEX_BODY);
}

void draw() {
  background(10);

  controller.update();
  
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
  PVector t = new PVector(-(mouseX-width/2), height-mouseY-height/2,0);
  controller.changeMoveVector(new MoveVector(t, 0));
}

void mouseReleased() {
  controller.changeMoveVector(new MoveVector(new PVector(0,0,0), 0));
}
