#ifndef _VEC3F_H_
#define _VEC3F_H_

#include <math.h>

class Vec3f {
  public:
    float x, y, z;

    Vec3f(float x=0, float y=0, float z=0);
    Vec3f(const Vec3f &v);
    //~Vec3f();

    void set(float i=0, float j=0, float k=0);
    Vec3f get();

    float mag();
    float dist(const Vec3f& v2);

    Vec3f operator+(const Vec3f& v2);
    void operator+=(const Vec3f& v2);
    Vec3f operator-(const Vec3f& v2);
    void operator-=(const Vec3f& v2);
    Vec3f operator=(const Vec3f& v2);
    Vec3f operator*(float k);
    void operator*=(float k);
    Vec3f operator/(float k);
    void operator/=(float k);

    void add(const Vec3f& v2);
    void sub(const Vec3f& v2);
    void mult(float k);
    void div(float k);
    void normalize();

    float dot(const Vec3f& v2);
    Vec3f cross(const Vec3f& v2);
    
    void rotateX(float theta);
    void rotateY(float theta);
    void rotateZ(float theta);
};

#endif
