#include "Vec3f.h"

Vec3f::Vec3f(float x, float y, float z) {
  this->x = x;
  this->y = y;
  this->z = z;
}

Vec3f::Vec3f(const Vec3f &v) {
  this->x = v.x;
  this->y = v.y;
  this->z = v.z;
}

Vec3f Vec3f::operator=(const Vec3f& v2) {
  x = v2.x;
  y = v2.y;
  z = v2.z;
  return *this;
}
void Vec3f::set(float i, float j, float k) {
  this->x = i;
  this->y = j;
  this->z = k;
}

Vec3f Vec3f::get() {
  return Vec3f(x, y, z);
}

float Vec3f::mag() {
  return sqrt(x*x + y*y + z*z);
}

float Vec3f::dist(const Vec3f& v2) {
  return sqrt((x - v2.x)*(x - v2.x) + (y - v2.y)*(y - v2.y) +
              (z - v2.z)*(z - v2.z));
}

void Vec3f::operator+=(const Vec3f& v2) {
  x += v2.x;
  y += v2.y;
  z += v2.z;
}
Vec3f Vec3f::operator+(const Vec3f& v2) {
  return Vec3f(x+v2.x, y+v2.y, z+v2.z);
}
void Vec3f::add(const Vec3f& v2) {
  x += v2.x;
  y += v2.y;
  z += v2.z;
}

void Vec3f::operator-=(const Vec3f& v2) {
  x -= v2.x;
  y -= v2.y;
  z -= v2.z;
}
Vec3f Vec3f::operator-(const Vec3f& v2) {
  return Vec3f(x-v2.x, y-v2.y, z-v2.z);
}
void Vec3f::sub(const Vec3f& v2) {
  x -= v2.x;
  y -= v2.y;
  z -= v2.z;
}

Vec3f Vec3f::operator*(float k) {
  return Vec3f(k*x, k*y, k*z);
}
void Vec3f::mult(float k) {
  x *= k;
  y *= k;
  z *= k;
}
void Vec3f::operator*=(float k) {
  x *= k;
  y *= k;
  z *= k;
}

Vec3f Vec3f::operator/(float k) {
  return Vec3f(x/k, y/k, z/k);
}
void Vec3f::div(float k) {
  x /= k;
  y /= k;
  z /= k;
}
void Vec3f::operator/=(float k) {
  x /= k;
  y /= k;
  z /= k;
}

void Vec3f::normalize() {
  float mag = this->mag();
  x /= mag;
  y /= mag;
  z /= mag;
}

float Vec3f::dot(const Vec3f& v2) {
  return x*v2.x + y*v2.y + z*v2.z;
}

Vec3f Vec3f::cross(const Vec3f& v) {
  return Vec3f(y*v.z - z*v.y,
               z*v.x - x*v.z,
               x*v.y - y*v.x);
}
   
void Vec3f::rotateX(float theta) {

}

void Vec3f::rotateY(float theta) {

}

void Vec3f::rotateZ(float theta) {
  set(cos(theta) * x - sin(theta) * y, sin(theta) * x + cos(theta) * y, z);
}

