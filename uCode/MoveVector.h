#ifndef _MOVE_VECTOR_H_
#define _MOVE_VECTOR_H_

class MoveVector {
  public:
    Vec3f translation;
    float heading;
    float speed;
    float rotation;

    MoveVector() {
      this->translation = Vec3f(0,0,0);
      this->heading = 0;
      this->speed = 0;
      this->rotation = 0;
    }

    MoveVector(const Vec3f& t, float r) {
      this->translation = t;
      this->rotation = r;
    }
};

#endif
