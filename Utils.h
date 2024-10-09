#ifndef UTILS_H
#define UTILS_H

#include "CommonDataStructure.h"
#include <cmath>
#include <cassert>
#include <iostream>
#include <sstream>
#include <iomanip>


namespace utils {

inline float ToDegree(float radian) { return radian * 180.0 / PI; }

// Make the angle in radian [0, 2*PI)
inline float ToNormalizedAngle(float radian) {
  while (radian < 0) {
    radian += (PI * 2);
  }
  while (radian >= 2 * PI) {
    radian -= (PI * 2);
  }
  return radian;
}

// compute Principal Argument of a vector, return value between (-PI, PI]
inline float PrincipalArgument(Point p) {
  assert(p.x != 0 || p.y != 0);
  if (p.y != 0) {
    return 2 * atan(p.y / (p.x + std::sqrt(p.x * p.x + p.y * p.y)));
  } else {
    if (p.x > 0) {
      return 0;
    } else {
      return PI;
    }
  }
}

// In radian, angle1 - angle2, in [0 2*PI)
// Meaning, angle2 turn counter-clock wise to angle1
inline float DiffAngleRadian(float angle1, float angle2) {
  float a1 = utils::ToNormalizedAngle(angle1);
  float a2 = utils::ToNormalizedAngle(angle2);

  if (a1 >= a2) {
    return a1 - a2;
  } else {
    return 2 * PI - (a2 - a1);
  }
}

// angle1 - angle2, in [-PI, PI)
inline float DiffAngleRadianInNegativePositivePi(float angle1, float angle2) {
  float a1 = utils::ToNormalizedAngle(angle1);
  float a2 = utils::ToNormalizedAngle(angle2);
  float diff = a1 - a2;
  if (diff < -PI) {
    return diff + 2 * PI;
  } else if (diff < PI) {
    return diff; 
  } else if (diff == PI) {
    return -diff;
  } else {
    return diff - 2 * PI;
  }
}

inline std::string ToStringWithPrecision(float number, int precision) {
  // Create an output string stream
  std::ostringstream str_obj;
  // Set Fixed -Point Notation
  str_obj << std::fixed;
  // Set precision to 2 digits
  str_obj << std::setprecision(precision);
  // Add double to stream
  str_obj << number;
  // Get string from output string stream
  return str_obj.str();
}

inline float Distance(Point p1, Point p2) {
  return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

inline bool IsEven(int value) { return value % 2 == 0; }

inline float Sign(float x) { return x < 0 ? -1.0 : 1.0; }

inline int ToKmPerHour(float x) { return int(std::round(x * 3.6)); }

inline std::string ToLapTime(float t) {
  if (t < 0) {
    std::cout << "LAP time should be greater than zero!";
    return "LAP time is negative!";
  } else if (t == 0) {
    return "0:0.000";
  } else {
    int sec = int(std::floor(t));
    t = t - sec;
    int minute = 0;
    while (sec >= 60) {
      sec -= 60;
      minute++;
    }
    std::string s, mi;
    if (sec >= 10) {
      s = std::to_string(sec);
    } else {
      s = "0" + std::to_string(sec);
    }
    mi = ToStringWithPrecision(t, 3);
    
    return std::to_string(minute) + ":" + s + ":" + mi.substr(2,3);
  }
}

inline float ToPositiveNegativePi(float angle_in_radian) {
  while (angle_in_radian > PI) {
    angle_in_radian -= PI;
  }
  while (angle_in_radian < -PI) {
    angle_in_radian += PI;
  }
  return angle_in_radian;
}

}


#endif // UTILS_H
