import processing.opengl.*;

ActuatorSet actuators;
float hip      = 368.5;
float femur    = 212.7;
float tibia    = 368.5;
float cankle   = 100;

float swingArm   = 150;  //150
float swingMount = 150;  // 75
float swingBase  = 360;  //380
float frameAngle = 30;  // degrees between y-axis and frame member swing actuator is attached to
float frameSide  = 462;
int view = 1;

boolean constructionLines = false;
boolean drawGraph = false;

static final int TRAIL_FREQ = 5;
PVector[] trails;
float[] graph;

PVector foot;
PVector profileCenter;
float profileScale;

PFont monofont;

void setup() {
  size(800,600); 
  smooth(); 
  frameRate(50);

  actuators = new ActuatorSet(425,425,425);

  trails = new PVector[1000];
  graph = new float[width];
  for(int i=0; i<graph.length; i++) {
    graph[i] = -1;
  }
  
  foot = new PVector(1,0,780);
  profileCenter = new PVector(width/2, height/8);
  profileScale = 0.5;
  
  
  
  monofont = createFont("Courier", 20);
  textFont(monofont);
}

void draw() {
  background(20);

  //actuators.a = 425 + cos(.82*frameCount/75.)*75;
  //actuators.b = 425 + cos(frameCount/75.)*75;

  //actuators.a = 350 + ((frameCount%300 < 150) ? frameCount%150 : 300-frameCount%300);
  //actuators.b = 350 + (((.9*frameCount-100)%300 < 150) ? (.9*frameCount-100)%150 : 300-(.9*frameCount-100)%300);
  //actuators.c = 350 + ((frameCount%300 < 150) ? frameCount%150 : 300-frameCount%300);

  //PVector foot = FK(actuators);
  actuators = IK(foot);
  PVector realFoot = FK(actuators);
  
  float alpha = acos((hip*hip + femur*femur - actuators.a*actuators.a) / (2*hip*femur));
  float beta  = acos((femur*femur + tibia*tibia - actuators.b*actuators.b) / (2*femur*tibia));

  PVector knee = new PVector(swingArm+femur*sin(alpha), 0, hip-femur*cos(alpha));
  PVector ankle = new PVector((tibia)*sin(beta-alpha), 0, (tibia)*cos(beta-alpha));
  ankle.add(knee);
  PVector flatfoot = new PVector((tibia+cankle)*sin(beta-alpha), 0, (tibia+cankle)*cos(beta-alpha));
  flatfoot.add(knee);
  
  float swingAngle = acos((swingMount*swingMount + swingBase*swingBase - actuators.c*actuators.c) /
  (2*swingBase*swingMount)) - radians(frameAngle) - PI/2;

  pushMatrix();
  switch(view) {
  case 1: // profile
      drawProfile();
    break;
  case 2: // top
    float minAngle = acos((swingMount*swingMount + swingBase*swingBase - 353*353) /
                          (2*swingBase*swingMount)) - radians(frameAngle) - PI/2;
    float maxAngle = acos((swingMount*swingMount + swingBase*swingBase - 499*499) /
                          (2*swingBase*swingMount)) - radians(frameAngle) - PI/2;                          
                          
    translate(width/2, height/2);
    pushMatrix();
      scale(.75);
      fill(255);
      textAlign(CENTER,CENTER);
      text("Mount: " + nf(swingMount,3,1),0,0);
      text("Base:  " + nf(swingBase,3,1),0,20);
      text("Range: " + nf(degrees(maxAngle - minAngle),3,1), 0,40);
      
      translate(-130,0);
      noStroke();
      fill(255,0,128);
      rect(-20,-degrees(maxAngle*2), 40, degrees(maxAngle*2));
      text(nfs(degrees(maxAngle),2,1), 60, -degrees(maxAngle*2));
      fill(0,128,255);
      rect(-20,0, 40, -degrees(minAngle*2));  
      text(nfs(degrees(minAngle),2,1), 60, -degrees(minAngle*2));
      stroke(255);
      strokeWeight(3);
      line(-20,0, 20,0);
    popMatrix();
    scale(.4);
      
    for(int i=0; i<6; i++) {
      pushMatrix();
        rotate(PI/3*i);
        translate(frameSide, 0);
        if(i%2 == 0) scale(1,-1);
        fill(255,255,0);
        stroke(20);
        strokeWeight(5);
        ellipse(0,0, 30,30);
        
        // Draw circle representing horizontal actuator size
        noFill();
        stroke(0,128,255);
        strokeWeight(3);
        if(constructionLines) {
          ellipse(-sin(radians(frameAngle)) * swingBase, -cos(radians(frameAngle)) * swingBase, actuators.c*2, actuators.c*2);
          ellipse(-sin(radians(frameAngle)) * swingBase, -cos(radians(frameAngle)) * swingBase, 40,40);
        
          stroke(255,255,0);
          ellipse(0,0, swingMount*2, swingMount*2);
        }
        
        // Hex frame
        stroke(255,255,0);
        strokeWeight(8);
        line(-sin(radians(frameAngle))*swingBase, -cos(radians(frameAngle))*swingBase, 0,0);
        line(-sin(radians(frameAngle))*swingBase, cos(radians(frameAngle))*swingBase, 0,0);
        
        for(int j=0; j<300; j++) {
          actuators.c = 350 + (((0+j)%300 < 150) ? (0+j)%150 : 300-(0+j)%300);
          swingAngle = acos((swingMount*swingMount + swingBase*swingBase - actuators.c*actuators.c) /
                    (2*swingBase*swingMount)) - radians(frameAngle) - PI/2;    
          // Horizontal actuator
          if(constructionLines) {
            stroke(0,128,255);
            line(-sin(radians(frameAngle))*swingBase, -cos(radians(frameAngle))*swingBase,
                  cos(swingAngle)*swingMount, sin(swingAngle)*swingMount);
          }
          pushMatrix();
            rotate(swingAngle);
            stroke(255,255,0);
            strokeWeight(8);
            line(0,0, swingArm,0);
            stroke(0,128,255);
            strokeWeight(1);
            line(0,0, swingArm+290,0);
          popMatrix();
          
          graphSample(degrees(swingAngle*2)+90);
        }
        
        pushMatrix();
          rotate(minAngle);
          stroke(255);
          strokeWeight(4);
          line(0,0, 290+swingArm,0);
        popMatrix();
        pushMatrix();
          rotate(maxAngle);
          stroke(255);
          strokeWeight(4);
          line(0,0, 290+swingArm,0);
        popMatrix();
        
      popMatrix();
    }

    break;
  }

  popMatrix();
  
  if(drawGraph) {
    stroke(255);
    strokeWeight(1);
    for(int i=0; i<graph.length; i++) {
      point(i, height-graph[i]);
    }
  }
}


ActuatorSet IK(PVector foot) {
  float swingAngle = PI/2-atan2(foot.y, foot.x);
  println("IK Swing Angle: " + swingAngle);
  
  PVector f_prime = new PVector(sqrt(foot.x*foot.x + foot.y*foot.y)-swingArm, 0, foot.z);
  float D = new PVector(0,0,hip).dist(f_prime);
  float E = f_prime.mag();
  float gamma = acos((femur*femur+D*D-(tibia+cankle)*(tibia+cankle)) / (2*femur*D));
  float sigma = acos((D*D+hip*hip-E*E) / (2*D*hip));
  float alpha = f_prime.x > 0 ? (sigma - gamma) : (TWO_PI - sigma - gamma);
  
  ActuatorSet result = new ActuatorSet(425,425,425);
  result.a = sqrt(femur*femur+hip*hip-2*femur*hip*cos(alpha));
  result.b = sqrt(femur*femur+tibia*tibia - (tibia*((tibia+cankle)*(tibia+cankle)+femur*femur-D*D) / (tibia+cankle))); 
  
  result.c = sqrt(swingMount*swingMount + swingBase*swingBase - 2*swingMount*swingBase *
                  cos(swingAngle + radians(frameAngle)));
  //float swingAngle = acos((swingMount*swingMount + swingBase*swingBase - S.c*S.c) /
  //  (2*swingBase*swingMount)) - radians(frameAngle) - PI/2;
                  
  if(result.a < 350) result.a = 350;
  if(result.a > 500) result.a = 500;
  if(result.b < 350) result.b = 350;
  if(result.b > 500) result.b = 500;
  return result;
}

PVector FK(ActuatorSet S) {
  float alpha = acos((hip*hip + femur*femur - S.a*S.a) / (2*hip*femur));
  float beta  = acos((femur*femur + tibia*tibia - S.b*S.b) / (2*femur*tibia));

  PVector knee = new PVector(swingArm+femur*sin(alpha), 0, hip-femur*cos(alpha));

  PVector newfoot = new PVector((tibia+cankle)*sin(beta-alpha), 0, (tibia+cankle)*cos(beta-alpha));
  newfoot.add(knee);

  float swingAngle = acos((swingMount*swingMount + swingBase*swingBase - S.c*S.c) /
    (2*swingBase*swingMount)) - radians(frameAngle);
  
  println("FK Swing angle: " + swingAngle);
  //println(swingAngle);
  // REMOVE THIS
  //swingAngle = radians(90);
  
  newfoot = new PVector(newfoot.x*sin(swingAngle), newfoot.x*cos(swingAngle), newfoot.z);

  return newfoot;
}
 
void keyPressed() {
  switch(key) {
  case 'a':
    swingBase += .2;
    break;
  case 'z':
    swingBase -= .2;
    break;
  case 's':
    swingMount += .2;
    break;
  case 'x':
    swingMount -= .2;
    break;
  case 'c':
    constructionLines = !constructionLines;
    break;
  case 'g':
    drawGraph = !drawGraph;
    break;
    
  case '1':
    view = 1;
    break;
  case '2':
    view = 2;
    break;
  }
  
}

void mousePressed() {
  mouseDragged();
}
void mouseDragged() {
  if(mouseButton == LEFT)
    foot = new PVector((mouseX-profileCenter.x)/profileScale, 0, (mouseY-profileCenter.y)/profileScale); 
  else
    foot = new PVector(foot.x, (mouseX-profileCenter.x), foot.z);
  println(foot);
}

void graphSample(float s) {
  for(int i=graph.length-1; i>=0; i--) {
    if(i<graph.length-1) {
      graph[i+1] = graph[i];
    }
  }
  graph[0] = s;  
}


void drawProfile() {
  PVector realFoot = FK(actuators);
  
  float alpha = acos((hip*hip + femur*femur - actuators.a*actuators.a) / (2*hip*femur));
  float beta  = acos((femur*femur + tibia*tibia - actuators.b*actuators.b) / (2*femur*tibia));

  PVector knee = new PVector(swingArm+femur*sin(alpha), 0, hip-femur*cos(alpha));
  PVector ankle = new PVector((tibia)*sin(beta-alpha), 0, (tibia)*cos(beta-alpha));
  ankle.add(knee);
  PVector flatfoot = new PVector((tibia+cankle)*sin(beta-alpha), 0, (tibia+cankle)*cos(beta-alpha));
  flatfoot.add(knee);
  
  float swingAngle = acos((swingMount*swingMount + swingBase*swingBase - actuators.c*actuators.c) /
  (2*swingBase*swingMount)) - radians(frameAngle) - PI/2;
  
  translate(profileCenter.x, profileCenter.y);
  scale(profileScale);
  
  scale(cos(swingAngle),1);
  
  // Draw trails
  //translate(-foot.x, -foot.z);
  fill(255,255,0);
  noStroke();
  colorMode(HSB);
  for(int i=trails.length-1; i>=0; i--) {
    if(i<trails.length-1 && frameCount%TRAIL_FREQ==0)
      trails[i+1] = trails[i];
    if(trails[i] != null) {
      //fill(i/4.,180,255);
      //fill((trails[i].z-800)*2%256, 180,255);
      //println(trails[i].z);
      ellipse(trails[i].x, trails[i].z, 10,10);
    }
  }
  colorMode(RGB);
  if(frameCount%TRAIL_FREQ==0)
    trails[0] = realFoot.get();


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
    ellipse(swingArm,0, actuators.a*2, actuators.a*2);
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
    ellipse(swingArm,hip, actuators.b*2, actuators.b*2);
  }

  // Line to ankle
  strokeWeight(20);
  stroke(0,128,255);
  line(swingArm,hip, ankle.x,ankle.z);
  strokeWeight(8);
  stroke(255,0,128);
  line(knee.x,knee.z, flatfoot.x, flatfoot.z);

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
  ellipse(flatfoot.x, flatfoot.z, 40,40);
  fill(255);
  text("[" + nfs(realFoot.x, 3,1) + ", " + nfs(realFoot.y, 3,1) + ", " + nfs(realFoot.z, 3,1) + "]", flatfoot.x+40, flatfoot.z);
}
