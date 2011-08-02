#ifndef _LEG_H_
#define _LEG_H_

#include "libmaple_types.h"
#include "wirish_types.h"
#include "Vec3f.h"
#include "Actuator.h"
#include "MoveVector.h"

#ifdef MAPLE_IDE
  #include "wirish.h"
#endif

class Controller; 

// Constants
enum Mode {PUSH_MODE, LIFT_DOWN_MODE, RETURN_MODE, LIFT_UP_MODE,
           DANCE_MODE, MANUAL_MODE};
enum State { MOVE_STATE, END_STATE, UP_STATE, DOWN_STATE};

struct ActLens {
  float a,b,c;
};

class Leg {
  public:
    // Static leg geometry
    static const float hip    = 368.f;  // Height of pivot with frame
    static const float femur  = 212.7;  // Length of hip->tibia
    static const float tibia  = 368.5;  // Length of femur->ankle
    static const float cankle = 100;    // Extension beyond ankle to foot

    static const float swingArm   = 150;
    static const float swingMount = 150;
    static const float swingBase  = 360;
    static const float frameAngle = 30 * PI / 180.;
    static const float frameSide  = 462;

    // Movement increment constants
    static const float MOVE_FACTOR      = 0.007;
    static const float TARGET_THRESHOLD = 20;

    
    // Public member functions
    // ---------------------------------------------------------------------
    Leg();
    Leg(Controller* parent, Vec3f center, float rotation, boolean flipped,
        int pwnPins[], int potPins[], bool simulate=false);
    //~Leg();
    void update();
    void toggleMode();

    Vec3f toLegSpace(const Vec3f& _p);
    Vec3f toBodySpace(const Vec3f& _p);
  
    bool canMoveTo(ActLens acts);
    bool canMoveTo(const Vec3f& p);
    bool canStepTo(const Vec3f& p);

    // Accessors
    bool isRemote(int port, int num);
    void synchronizeRemote(const Vec3f& remoteFoot);
    void synchronizeRemote(Mode remoteMode);
    void synchronizeRemote(State remoteState);
    
    bool changeMode(Mode m);
    Mode getMode();

    bool changeState(State s);
    State getState();

    bool setTarget(Vec3f p, bool force=false);
    Vec3f getTarget();

    Vec3f getFoot();
    Vec3f getKnee();
    Vec3f getAnkle();
    Vec3f getHipEnd();

    void moveTargetUp();
    void moveTargetDown();
    void setCenterTarget();
    
    void setMoveVector(Vec3f t, float r);
    void setMoveVector(MoveVector m);
    MoveVector getMoveVector();

    float getDeltaP();
    float getDeltaTheta();
    float getAngle();
    float getAngle(const Vec3f& _P);

  private:
    // Positional properties
    Vec3f center;
    Vec3f centerOfRotation;
    float rotation;
    bool flipped;

    // State variables
    bool remote;
    int remotePort;
    int remoteIndex;

    Mode mode;
    State state;
    Vec3f knee;
    Vec3f ankle;
    Vec3f hipEnd;
    float swingAngle;
    
    Vec3f foot;
    Vec3f target;
    
    MoveVector milestone;
    MoveVector currentMove;

    // Member objects
    Controller* parent;
    Actuator actuators[3];


    // Private methods
    void freeze();
    void resetMilestone();
    
    Vec3f solveFK();
    Vec3f solveFK(ActLens acts, bool assign=false);
    
    float solveSwingAngle(float act);
    ActLens solveIK(const Vec3f& p);
};
#endif
