#include <fix16.h>
#include <fixmath.h>
#include <fract32.h>
#include <uint32.h>

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
const static int MODULE_NUMBER = 0; // 0 = MASTER, 1 = SLAVE_1, 2 = SLAVE_2

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

long tick = 0;

void setup() {
  int pwmPins[6][3] = {{5,6,9},{11,12,14},{-1,-1,-1},{-2,-2,-2},{-1,-1,-1},{-2,-2,-2}};
  int potPins[6][3] = {{20,19,18},{17,16,15},{-1,-1,-1},{-1,-1,-1},{-2,-2,-2},{-2,-2,-2}};
  Controller::BodyType type = MODULE_NUMBER == 0 ? Controller::HEX_BODY :
                              MODULE_NUMBER == 1 ? Controller::HEX_SLAVE_1 :
                              Controller::HEX_SLAVE_2;
  controller.init(pwmPins, potPins, type);
  RemoteManager.setController(&controller);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  
  for(int i=0; i<4; i++) {
    togglePin(GREEN_LED_PIN);
    delay(100);
  }
}

void loop() {
  if(tick++ % 100 == 0) {
    togglePin(RED_LED_PIN);
    togglePin(GREEN_LED_PIN);
    if(MODULE_NUMBER == 0)
      SerialUSB.write("!*");
  }
    
  //delay(10);
  controller.update();
  //if(tick % 10 == 0) {
    processUSB();
  //}
  processSerial();
  

}

void processSerial() {
  static int m = 0;
  RemoteManager.processIncoming();
  if(controller.isSlave() && m++ % 50 == 0) {
  //if(controller.isSlave()) {
    for(int j=0; j<2; j++) {
      RemoteManager.sendStatus(MODULE_NUMBER-1, j, controller.legs[j]);  
    }
  }
}

void processUSB() {

  if(!controller.isSlave() &&
     SerialUSB.isConnected() &&(SerialUSB.getDTR() || SerialUSB.getRTS()))
    RemoteManager.processUSB();
}
