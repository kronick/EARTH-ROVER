#include <Servo.h>

#include <Actuator.h>
#include <Controller.h>
#include <Leg.h>
#include <MoveVector.h>
#include <RemoteMan.h>
#include <Vec3f.h>

const static int RED_LED_PIN = 3;
const static int GREEN_LED_PIN = 2;
const static int TEMP_PIN = 4;
const static int SLAVE_NUMBER = 1; // 0 or 1

// Incoming serial command states
enum Dimension {DIM_X, DIM_Y, DIM_Z, DIM_NONE};
enum CommState {COMM_LEG_SELECT, COMM_COORDINATE, COMM_WAITSTATE};
CommState incomingState;
int incomingLeg             = -1;
Dimension incomingDimension = DIM_NONE;
boolean incomingNegated     = false;
int incomingLegX            = 0;
int incomingLegY            = 0;
int incomingLegZ            = 0;

Controller controller;

void setup() {
  int pwmPins[6][3] = {{5,6,9},{11,12,14},{-1,-1,-1},{-2,-2,-2},{-1,-1,-1},{-2,-2,-2}};
  int potPins[6][3] = {{20,19,18},{17,16,15},{-1,-1,-1},{-1,-1,-1},{-2,-2,-2},{-2,-2,-2}};
  controller.init(pwmPins, potPins,
                  SLAVE_NUMBER == 1 ?  Controller::HEX_SLAVE_2 : Controller::HEX_SLAVE_1);
  RemoteManager.setController(&controller);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  
  for(int i=0; i<4; i++) {
    togglePin(16);
    delay(100);
  }
}

void loop() {
  static int blinkCount = 0;
  if(blinkCount++ % 100 == 0)
    togglePin(GREEN_LED_PIN);
    
  //delay(10);
  controller.update();
  
  //processUSB();
  processSerial();

}

void processSerial() {
  static int m = 0;
  RemoteManager.processIncoming();
  if(controller.isSlave() && m++ % 10 == 0) {
    for(int j=0; j<2; j++) {
      RemoteManager.sendStatus(SLAVE_NUMBER, j, controller.legs[j]);  
    }
  }
}

void processUSB() {
  while(SerialUSB.available() > 0) {
    int inByte = SerialUSB.read();
    if(inByte == '!') {  // Reset everything
      incomingLeg = -1;
      incomingDimension = DIM_NONE;
      incomingLegX = incomingLegY = incomingLegZ = 0;
      incomingState = COMM_WAITSTATE;
      incomingNegated = false;
    }
    else if(inByte == 'L') incomingState = COMM_LEG_SELECT;
    else if(inByte == 'x') { incomingDimension = DIM_X; incomingLegX = 0; incomingNegated = false; incomingState = COMM_COORDINATE; }
    else if(inByte == 'y') { incomingDimension = DIM_Y; incomingLegY = 0; incomingNegated = false; incomingState = COMM_COORDINATE; }
    else if(inByte == 'z') { incomingDimension = DIM_Z; incomingLegZ = 0; incomingNegated = false; incomingState = COMM_COORDINATE; }
    
    else if((inByte <= 57 && inByte >= 48) || inByte == '-') {  // This is a digit or a minus sign
      if(incomingState == COMM_LEG_SELECT && inByte != '-') { // Interpret this as a leg number
        incomingLeg = inByte - '0';
        SerialUSB.print("\nLeg selected: ");
        SerialUSB.println(incomingLeg);
      }
      else if(incomingState == COMM_COORDINATE) {  // Interpret digit as coordinate
        int* active;
        switch(incomingDimension) {
          case DIM_X:
            active = &incomingLegX; break;
          case DIM_Y:
            active = &incomingLegY; break;
          case DIM_Z:
            active = &incomingLegZ; break;
          default: active = 0;
        }
        if(active) {
          if(inByte == '-') {
            if(*active == 0)  // Make sure this is the first character in the string
              incomingNegated = true;
          }
          else {
            int digit = inByte - '0';
            *active *= 10;
            *active += (incomingNegated ? -1 : 1) * digit;
          }
        }
      }
      SerialUSB.print(inByte == '-' ? '-' : (inByte-'0'));
    }
    else if(inByte == '*') {  // Stop code
      boolean success = controller.moveLegTo(incomingLeg, Vec3f(incomingLegX/100., incomingLegY/100., incomingLegZ/100.));
      SerialUSB.print("Moving leg #");
      SerialUSB.print(incomingLeg);
      SerialUSB.print(" to [");
      SerialUSB.print(incomingLegX); SerialUSB.print(", ");
      SerialUSB.print(incomingLegY); SerialUSB.print(", ");
      SerialUSB.print(incomingLegZ); SerialUSB.print("]\n");
      SerialUSB.println(success ? "Success!" : "Failed.");
    }
    else if(inByte == '?') {  // Respond with where the leg is at
      for(int i=0; i<1; i++) {
        Vec3f foot = controller.legs[i].getFoot();
        SerialUSB.print(i);
        SerialUSB.print(" Current position: [");
        SerialUSB.print(foot.x); SerialUSB.print(", ");
        SerialUSB.print(foot.y); SerialUSB.print(", ");
        SerialUSB.print(foot.z); SerialUSB.print("]\n");
      }
    }
  }  
}

