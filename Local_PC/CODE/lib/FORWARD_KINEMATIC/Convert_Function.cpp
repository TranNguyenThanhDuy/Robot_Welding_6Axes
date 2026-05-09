#include "Convert_Function.h"
#include <Arduino.h>
#include <math.h>

// Hàm tính arcsin (asin) theo độ
double asin2d(double x) {
  if (x >= -1.0 && x <= 1.0) {
    return asin(x) * 180.0 / M_PI;
  } else {
    return NAN; // Trả về NaN nếu x không nằm trong khoảng [-1, 1]
  }
}

// Hàm tính arccos (acos) theo độ
double acos2d(double x) {
  if (x >= -1.0 && x <= 1.0) {
    return acos(x) * 180.0 / M_PI;
  } else {
    return NAN; // Trả về NaN nếu x không nằm trong khoảng [-1, 1]
  }
}

// Hàm tính arctan (atan2) theo độ
double atan2d(double y, double x) {
  return atan2(y, x) * 180.0 / M_PI;
}

// Chuyển đổi từ radian sang độ
double cosd(double degrees) {
  return cos(degrees * M_PI / 180);
}

double sind(double degrees) {
  return sin(degrees * M_PI / 180);
}

double tand(double degrees) {
  return tan(degrees * M_PI / 180);
}
