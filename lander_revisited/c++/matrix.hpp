#ifndef __MATRIX_HPP__
#define __MATRIX_HPP__

#include <vector>

#include "vector3d.hpp"

class Matrix {
public:
  Matrix ();

  static Matrix eye ();
  static Matrix from_euler (vector3d euler_angles);

  double & at(int i, int j) { return m[i * 4 + j]; }

  Matrix inverse ();
  vector3d to_euler ();

private:
  std::vector<double> m;
};

Matrix::Matrix () : m(16, 0) {}

Matrix Matrix::eye() {
  Matrix m;
  m.m[0] = 1.0; m.m[5] = 1.0;
  m.m[10] = 1.0; m.m[15] = 1.0;
  return m;
}

#endif /* end of include guard: __MATRIX_HPP__ */
