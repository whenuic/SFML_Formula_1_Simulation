#ifndef STEER_H
#define STEER_H

#include "CommonDataStructure.h"

class Steer {
 public:
  Steer(float steer_lock, float max_speed);

  void ReconfigSteerLock(float steer_lock);

  float GetSteerLock();
  float GetMaxSpeed();
  float GetSteer();

  void Update(float dt, float steer_input);
 
 private:
  // max rad the front wheel can turn, for formula 1 car, it's about
  // 0.35 rad (20 degrees), for commercial family car, it's about 0.78
  // rad (45 degrees)
  float steer_lock_ = 0.35;

  // max rad/s the front wheel can turn
  float max_speed_ = 2*PI;

  // current steer value in rad
  // Left: positive
  // Right: negative
  float current_steer_ = 0;

};



#endif // STEER_H