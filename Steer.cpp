#include "Steer.h"
#include "Utils.h"


Steer::Steer(float steer_lock, float max_speed) :
  steer_lock_(steer_lock),
  max_speed_(max_speed) {}

void Steer::ReconfigSteerLock(float steer_lock) { steer_lock_ = steer_lock; }

float Steer::GetSteerLock() { return steer_lock_; }

float Steer::GetMaxSpeed() { return max_speed_; }

float Steer::GetSteer() { return current_steer_; }

void Steer::Update(float dt, float steer_input) {
  float target_steer = steer_input * steer_lock_;
  float steer_delta = target_steer - current_steer_;

  if (std::fabs(steer_delta) / dt > max_speed_) {
    target_steer = utils::Sign(steer_delta) * max_speed_ * dt + current_steer_;
  }

  current_steer_ = target_steer;
}