#ifndef __VECTOR3D_HPP__
#define __VECTOR3D_HPP__ 

#include <iostream>
#include <cmath>

class vector3d {
  // Utility class for three-dimensional vector operations
public:
  vector3d() {x=0.0; y=0.0; z=0.0;}
  vector3d (double a, double b, double c=0.0) {x=a; y=b; z=c;}
  bool operator== (const vector3d &v) const { if ((x==v.x)&&(y==v.y)&&(z==v.z)) return true; else return false; }
  bool operator!= (const vector3d &v) const { if ((x!=v.x)||(y!=v.y)||(z!=v.z)) return true; else return false; }
  vector3d operator+ (const vector3d &v) const { return vector3d(x+v.x, y+v.y, z+v.z); }
  vector3d operator- (const vector3d &v) const { return vector3d(x-v.x, y-v.y, z-v.z); }
  friend vector3d operator- (const vector3d &v) { return vector3d(-v.x, -v.y, -v.z); }
  vector3d& operator+= (const vector3d &v) { x+=v.x; y+=v.y; z+=v.z; return *this; }
  vector3d& operator-= (const vector3d &v) { x-=v.x; y-=v.y; z-=v.z; return *this; }
  vector3d operator^ (const vector3d &v) const { return vector3d(y*v.z-z*v.y, z*v.x-x*v.z, x*v.y-y*v.x); }
  double operator* (const vector3d &v) const { return (x*v.x + y*v.y +z*v.z); }
  friend vector3d operator* (const vector3d &v, const double &a) { return vector3d(v.x*a, v.y*a, v.z*a); }
  friend vector3d operator* (const double &a, const vector3d &v) { return vector3d(v.x*a, v.y*a, v.z*a); }
  vector3d& operator*= (const double &a) { x*=a; y*=a; z*=a; return *this; }
  vector3d operator/ (const double &a) const { return vector3d(x/a, y/a, z/a); }
  vector3d& operator/= (const double &a) { x/=a; y/=a; z/=a; return *this; }
  double abs2() const { return (x*x + y*y + z*z); }
  double abs() const { return sqrt(this->abs2()); }
  vector3d norm() const { double s(this->abs()); if (s==0) return *this; else return vector3d(x/s, y/s, z/s); }
  friend std::ostream& operator << (std::ostream &out, const vector3d &v) { out << v.x << ' ' << v.y << ' ' << v.z; return out; }
  double x, y, z;
private:
};

#endif /* end of include guard: __VECTOR3D_HPP__ */
