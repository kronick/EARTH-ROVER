#include "RemoteMan.h"


void RemoteMan::setController(Controller* c) {
  controller = c;
}

RemoteMan::RemoteMan() {
  Serial1.begin(115200);
  Serial2.begin(115200);
  Serial1.flush();
  Serial2.flush();
  lastContact = 0;
  timeoutContact = 0;
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
  if(portNum == 0) port = &Serial1;
  else             port = &Serial2;

  bool negX = t.x < 0;
  bool negY = t.y < 0;
  bool negZ = t.z < 0;
  int x = (int)((negX ? -1 : 1) * t.x * COORDINATE_MULTIPLIER);
  int y = (int)((negY ? -1 : 1) * t.y * COORDINATE_MULTIPLIER);
  int z = (int)((negZ ? -1 : 1) * t.z * COORDINATE_MULTIPLIER);
  
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
        //port->flush();
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

void RemoteMan::processUSB() {
  /* Possible commands:
     !WX0000Y0000Z0000A0000* Walk with MoveVector(X,Y,Z, A) where A is rotation
     !L0X0000Y0000Z0000*     Move specified leg #'s target to (X,Y,Z)
     !DX0000Y0000Z0000A0000B0000C0000* Kinematics dance to (X,Y,Z, A,B,C)
                                        where A=yaw, B=pitch, C=roll
     !G0*                    Change gait. TRIPOD = 1 WAVE = 2 RIPPLE = 3

  */

  statusSent = false;

  timeoutContact++;
  if(SerialUSB.available() > 0)
    lastContact = timeoutContact;
  else if(timeoutContact - lastContact > USB_TIMEOUT) {
    // If nothing has been sent for a while, freeze.
    controller->changeGait(Controller::DANCE_GAIT);
    controller->changeMoveVector(MoveVector(Vec3f(0,0,0), 0,0,0));
  }
  while(SerialUSB.available() > 0) {
    int inByte = SerialUSB.read();
    if(inByte == '?' && !statusSent) { // Print out leg positions
      statusSent = true;
      int statusLength = 25;
      char output[statusLength * controller->LEG_COUNT*2 + 1];
      for(int i=0; i<controller->LEG_COUNT*2; i++) {
        int n = i % controller->LEG_COUNT;

        Vec3f foot = i < controller->LEG_COUNT ?
                            controller->legs[n].getFoot() :
                            controller->legs[n].getTarget();

        bool negX = foot.x < 0;
        bool negY = foot.y < 0;
        bool negZ = foot.z < 0;
        int x = (int)(fabs(foot.x) * COORDINATE_MULTIPLIER);
        int y = (int)(fabs(foot.y) * COORDINATE_MULTIPLIER);
        int z = (int)(fabs(foot.z) * COORDINATE_MULTIPLIER);


        output[i*statusLength + 0]  = (i < controller->LEG_COUNT ? 'L' : 'T');
        output[i*statusLength + 1]  = n + '0';
        output[i*statusLength + 2]  = 'X';
        output[i*statusLength + 3]  = negX ? '-' : '+'; 
        output[i*statusLength + 4]  = '0' + ((x%10000) / 1000);
        output[i*statusLength + 5]  = '0' + ((x%1000)/100);
        output[i*statusLength + 6]  = '0' + ((x%100)/10);
        output[i*statusLength + 7] = '0' + (x%10);
        output[i*statusLength + 8] = 'Y';
        output[i*statusLength + 9]  = negY ? '-' : '+';
        output[i*statusLength + 10] = '0' + (y / 1000);
        output[i*statusLength + 11] = '0' + ((y%1000)/100);
        output[i*statusLength + 12] = '0' + ((y%100)/10);
        output[i*statusLength + 13] = '0' + (y%10);
        output[i*statusLength + 14] = 'Z';
        output[i*statusLength + 15]  = negZ ? '-' : '+';
        output[i*statusLength + 16] = '0' + (z / 1000);
        output[i*statusLength + 17] = '0' + ((z%1000)/100);
        output[i*statusLength + 18] = '0' + ((z%100)/10);
        output[i*statusLength + 19] = '0' + (z%10);
        output[i*statusLength + 20] = 'S';
        output[i*statusLength + 21] = '0'+(int)controller->legs[n].getState();
        output[i*statusLength + 22] = 'M';
        output[i*statusLength + 23] = '0'+(int)controller->legs[n].getMode();
        output[i*statusLength + 24] = '*';
      }
      output[statusLength*controller->LEG_COUNT*2] = '\0';
      SerialUSB.print(output);
    }
    if(inByte == '!') { // Reset everything
      usbLeg = 0;
      usbX = 0;
      usbY = 0;
      usbZ = 0;
      usbPitch = 0;
      usbYaw = 0;
      usbRoll = 0;
      usbNegative = false;
      usbMode = NO_MODE;
      usbControlMode = NO_CONTROL_MODE;
    }
    else if(inByte == 'W') usbControlMode = WALK;
    else if(inByte == 'D') usbControlMode = DANCE;
    else if(inByte == 'L') {
      usbControlMode = LEG;
      usbMode = LEG_SELECT;
    }
    else if(inByte == 'G') usbMode = SET_GAIT;
    
    else if(inByte == 'X') {
        usbMode = SET_COORD_X;
        usbX = 0;
        usbNegative = false;
    }
    else if(inByte == 'Y') {
        usbMode = SET_COORD_Y;
        usbY = 0;
        usbNegative = false;
    }
    else if(inByte == 'Z') {
        usbMode = SET_COORD_Z;
        usbZ = 0;
        usbNegative = false;
    }
    else if(inByte == 'A') {
      usbMode = SET_YAW;
      usbYaw = 0;
      usbNegative = false;
    }
    else if(inByte == 'B') {
      usbMode = SET_PITCH;
      usbPitch = 0;
      usbNegative = false;
    }
    else if(inByte == 'C') {
      usbMode = SET_ROLL;
      usbRoll = 0;
      usbNegative = false;
    }
    else if(inByte == '*') {  // Stop code. Time to take action!
      switch(usbControlMode) {
        case WALK: {
          controller->changeGait(Controller::TRIPOD_GAIT);
          Vec3f t(usbX/COORDINATE_MULTIPLIER, usbY/COORDINATE_MULTIPLIER,
                  usbZ/COORDINATE_MULTIPLIER);
          MoveVector v(t, usbYaw/ANGLE_MULTIPLIER);
          controller->changeMoveVector(v);
          
          SerialUSB.print("X = ");
          SerialUSB.println(controller->getMoveVector().translation.x);
          SerialUSB.print("Y = ");
          SerialUSB.println(controller->getMoveVector().translation.y);
          SerialUSB.print("Z = ");
          SerialUSB.println(controller->getMoveVector().translation.z);
          SerialUSB.print("R = ");
          SerialUSB.println(controller->getMoveVector().yaw);
          SerialUSB.print("*");
          break;
        }
        case DANCE: {
          controller->changeGait(Controller::DANCE_GAIT);
          Vec3f t(usbX/COORDINATE_MULTIPLIER, usbY/COORDINATE_MULTIPLIER,
                  usbZ/COORDINATE_MULTIPLIER);
          MoveVector v(t, usbYaw/ANGLE_MULTIPLIER,
                          usbPitch/ANGLE_MULTIPLIER,
                          usbRoll/ANGLE_MULTIPLIER);
          controller->changeMoveVector(v);
         
          SerialUSB.print("X = ");
          SerialUSB.println(controller->getMoveVector().translation.x);
          SerialUSB.print("Y = ");
          SerialUSB.println(controller->getMoveVector().translation.y);
          SerialUSB.print("Z = ");
          SerialUSB.println(controller->getMoveVector().translation.z);
          SerialUSB.print("YAW   = ");
          SerialUSB.println(controller->getMoveVector().yaw);
          SerialUSB.print("PITCH = ");
          SerialUSB.println(controller->getMoveVector().pitch);
          SerialUSB.print("ROLL  = ");
          SerialUSB.println(controller->getMoveVector().roll);

          SerialUSB.print("*");
         
          break;
        }
        case LEG: {
          controller->changeGait(Controller::MANUAL_GAIT);
          controller->moveLegTo(usbLeg, Vec3f(usbX/COORDINATE_MULTIPLIER,
                                             usbY/COORDINATE_MULTIPLIER,
                                             usbZ/COORDINATE_MULTIPLIER));
          break;
        }
        default:
          break;
       }

       while(SerialUSB.available() > 0)
         SerialUSB.read();
    }
    else if(inByte == '-') usbNegative = true;
    else if(inByte == '+') usbNegative = false;
    else if(inByte <= 57 && inByte >= 48) { // It's a digit 0-9
      int digit = inByte - '0';
      int* accumulator = NULL;
      switch(usbMode) {
        case LEG_SELECT:
          usbLeg = digit;
          break;
        case SET_GAIT:
          controller->changeGait((Controller::WalkingGait)digit);
          break;
        case SET_COORD_X:
          accumulator = &usbX;
          break;
        case SET_COORD_Y:
          accumulator = &usbY;
          break;
        case SET_COORD_Z:
          accumulator = &usbZ;
          break;
        case SET_YAW:
          accumulator = &usbYaw;
          break;
        case SET_PITCH:
          accumulator = &usbPitch;
          break;
        case SET_ROLL:
          accumulator = &usbRoll;
          break;
        default:
          break;
      }
      if(accumulator != NULL) {
        *accumulator *= 10;
        *accumulator += (usbNegative? -1 : 1) * digit;
      }
    }
    

  }
}
