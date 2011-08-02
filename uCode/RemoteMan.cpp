#include "RemoteMan.h"


void RemoteMan::setController(Controller* c) {
  controller = c;
}

RemoteMan::RemoteMan() {
  Serial1.begin(115200);
  Serial2.begin(115200);
}

void RemoteMan::sendStatus(int portNum, int legNum, Leg& leg) {
  HardwareSerial* port;
  if(abs(portNum) == 0) port = &Serial1;
  else                  port = &Serial2;

  Vec3f foot = leg.getFoot();
  bool negX = foot.x < 0;
  bool negY = foot.y < 0;
  bool negZ = foot.z < 0;
  int x = (int)(fabs(foot.x) * COORDINATE_MULTIPLIER);
  int y = (int)(fabs(foot.y) * COORDINATE_MULTIPLIER);
  int z = (int)(fabs(foot.z) * COORDINATE_MULTIPLIER);

  char output[26];
  output[0]  = 'L';
  output[1]  = abs(legNum) + '0';
  output[2]  = 'M';
  output[3]  = (int)leg.getMode() + '0';
  output[4]  = 'S';
  output[5]  = (int)leg.getState() + '0';
  output[6]  = 'X';
  output[7]  = negX ? '-' : '+';
  output[8]  = '0' + ((x%10000) / 1000);
  output[9]  = '0' + ((x%1000)/100);
  output[10]  = '0' + ((x%100)/10);
  output[11] = '0' + (x%10);
  output[12] = 'Y';
  output[13]  = negY ? '-' : '+';
  output[14] = '0' + (y / 1000);
  output[15] = '0' + ((y%1000)/100);
  output[16] = '0' + ((y%100)/10);
  output[17] = '0' + (y%10);
  output[18] = 'Z';
  output[19]  = negZ ? '-' : '+';
  output[20] = '0' + (z / 1000);
  output[21] = '0' + ((z%1000)/100);
  output[22] = '0' + ((z%100)/10);
  output[23] = '0' + (z%10);
  output[24] = '*';
  output[25] = '\0';
  port->print(output);
}
void RemoteMan::sendTarget(int portNum, int legNum, const Vec3f& t) {
  HardwareSerial* port;
  if(abs(portNum) == 0) port = &Serial1;
  else                  port = &Serial2;

  bool negX = t.x < 0;
  bool negY = t.y < 0;
  bool negZ = t.z < 0;
  int x = (int)(fabs(t.x) * COORDINATE_MULTIPLIER);
  int y = (int)(fabs(t.y) * COORDINATE_MULTIPLIER);
  int z = (int)(fabs(t.z) * COORDINATE_MULTIPLIER);

  char output[22];
  output[0]  = 'L';
  output[1]  = abs(legNum) + '0';
  output[2]  = 'X';
  output[3]  = negX ? '-' : '+'; 
  output[4]  = '0' + ((x%10000) / 1000);
  output[5]  = '0' + ((x%1000)/100);
  output[6]  = '0' + ((x%100)/10);
  output[7] = '0' + (x%10);
  output[8] = 'Y';
  output[9]  = negY ? '-' : '+';
  output[10] = '0' + (y / 1000);
  output[11] = '0' + ((y%1000)/100);
  output[12] = '0' + ((y%100)/10);
  output[13] = '0' + (y%10);
  output[14] = 'Z';
  output[15]  = negZ ? '-' : '+';
  output[16] = '0' + (z / 1000);
  output[17] = '0' + ((z%1000)/100);
  output[18] = '0' + ((z%100)/10);
  output[19] = '0' + (z%10);
  output[20] = '*';
  output[21] = '\0';
  port->print(output);
}

void RemoteMan::sendMode(int portNum, int legNum, Mode m) {
  HardwareSerial* port;
  if(abs(portNum) == 0) port = &Serial1;
  else                  port = &Serial2;

  port->print('L');    // Addressing a leg
  port->print(abs(legNum));
  port->print('M');    // Setting mode
  port->print((int)m);
}

void RemoteMan::sendState(int portNum, int legNum, State s) {
  HardwareSerial* port;
  if(abs(portNum) == 0) port = &Serial1;
  else                  port = &Serial2;

  port->print('L');    // Addressing a leg
  port->print(abs(legNum));
  port->print('S');    // Setting state
  port->print((int)s);
}

void RemoteMan::processIncoming() {
  HardwareSerial* port;
  for(int portNum = 0; portNum<2; portNum++) {
    if(portNum == 0) port = &Serial1;
    else             port = &Serial2;

    while(port->available() > 0) {
      int inByte = port->read();
      if(inByte == 'L') incomingMode[portNum] = LEG_SELECT;
      else if(inByte == 'M') incomingMode[portNum] = SET_MODE;
      else if(inByte == 'S') incomingMode[portNum] = SET_STATE;
      else if(inByte == 'X') {
        incomingMode[portNum] = SET_COORD_X;
        incomingX[portNum] = 0;
        incomingNegative[portNum] = false;
      }
      else if(inByte == 'Y') {
        incomingMode[portNum] = SET_COORD_Y;
        incomingY[portNum] = 0;
        incomingNegative[portNum] = false;
      }
      else if(inByte == 'Z') {
        incomingMode[portNum] = SET_COORD_Z;
        incomingZ[portNum] = 0;
        incomingNegative[portNum] = false;
      }
      else if(inByte == '*') {  // Stop character
        Leg* leg = controller->findLeg(portNum, incomingLeg[portNum]);
        Vec3f incomingFoot(incomingX[portNum]/(float)COORDINATE_MULTIPLIER,
                           incomingY[portNum]/(float)COORDINATE_MULTIPLIER,
                           incomingZ[portNum]/(float)COORDINATE_MULTIPLIER);
        if(leg != NULL)
          leg->synchronizeRemote(incomingFoot);
        
        //togglePin(13);
        
        /*
        SerialUSB.print("Incoming data: ");
        SerialUSB.print("P: ");
        SerialUSB.println(portNum);
        SerialUSB.print('L');
        SerialUSB.print(incomingLeg[portNum]);
        SerialUSB.print('X');
        SerialUSB.print(incomingX[portNum]);
        SerialUSB.print('Y');
        SerialUSB.print(incomingY[portNum]);
        SerialUSB.print('Z');
        SerialUSB.println(incomingZ[portNum]);
        */
        
        /*
        SerialUSB.print("Update for leg:  ");
        SerialUSB.println(controller->findLegNumber(portNum,
                                      incomingLeg[portNum]));

        SerialUSB.print("On port #");
        SerialUSB.println(portNum);
        */

        // Clean up
        incomingX[portNum] = 0;
        incomingY[portNum] = 0;
        incomingZ[portNum] = 0;
        incomingNegative[portNum] = 0;
        incomingMode[portNum] = NO_MODE;
      }
      else if(inByte == '-') incomingNegative[portNum] = true;
      else if(inByte <= 57 && inByte >= 48) {
        // This is a digit
        int digit = inByte - '0';
        int* accumulator = NULL;
        Leg* leg;
        switch(incomingMode[portNum]) {
          case LEG_SELECT:
            incomingLeg[portNum] = digit;
            break;
          case SET_MODE:
            leg = controller->findLeg(portNum, incomingLeg[portNum]);
            if(leg != NULL)
                leg->synchronizeRemote((Mode)digit);
            break;
          case SET_STATE:
            leg = controller->findLeg(portNum, incomingLeg[portNum]);
            if(leg != NULL)
                leg->synchronizeRemote((State)digit);
            break;
          case SET_COORD_X:
            accumulator = &incomingX[portNum];
            break;
          case SET_COORD_Y:
            accumulator = &incomingY[portNum];
            break;
          case SET_COORD_Z:
            accumulator = &incomingZ[portNum];
            break;
          default:
            break;
        }
        if(accumulator != NULL) {
          *accumulator *= 10;
          *accumulator += (incomingNegative[portNum] ? -1 : 1) * digit;
        }
      }

    }
  }
}

