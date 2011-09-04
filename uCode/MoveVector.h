#ifndef _MOVE_VECTOR_H_
#define _MOVE_VECTOR_H_

class MoveVector {
  public:
    Vec3f translation;
    float heading;
    float speed;
    float rotation;
    float pitch;
    float yaw;
    float roll;

    MoveVector() {
      this->translation = Vec3f(0,0,0);
      this->heading = 0;
      this->speed = 0;
      this->rotation = 0;
      this->pitch = 0;
      this->yaw = 0;
      this->roll = 0;
    }

    MoveVector(const Vec3f& t, float r) {
      this->translation = t;
      this->rotation = r;
      this->yaw = r;
      this->pitch = 0;
      this->roll = 0;
    }

    MoveVector(const Vec3f& t, float yaw, float pitch, float roll) {
      this->translation = t;
      this->rotation = yaw;
      this->yaw = yaw;
      this->pitch = pitch;
      this->roll = roll;
    }
};

#endif
