#include "Car.h"
#include "Utils.h"
#include "math.h"
#include <cfloat>
#include <cassert>
#include <cmath>

#include <iostream>

#define g 9.8


Car::Car(sf::RenderWindow* app, sf::Font* font, CarSpecificParams params, CarDisplayParams display_params)
: app_(app),
  font_(font) {

  assert(params.car_number > 0);
  assert(params.max_drive_force > 0);
  assert(params.max_brake_force > 0);
  assert(params.wheelbase > 0);
  assert(params.width > 0);
  assert(params.length > params.wheelbase);
  assert(params.body_drag_coeff > 0);
  assert(params.wing_down_coeff > 0);
  assert(params.wing_drag_coeff > 0);
  assert(params.team_name != "");
  assert(params.input_policy_name >= 0);

  display_params_ = display_params;

  CircuitBoundarySensingData sensing_data;
  std::vector<int> sensing_angle = {0,30,60,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,120,150,180};
  for (auto& i : sensing_angle) {
    sensing_data.direction = i*PI/180;
    boundary_sensing_.push_back(sensing_data);
  }

  car_width_ = params.width;
  car_length_ = params.length;
  car_body_.setSize(sf::Vector2f(car_length_, car_width_));
  car_body_.setOrigin(car_length_/2, car_width_/2);
  car_body_.setPosition(x_, -y_);
  float forward_boundary_point_radius = 3;

  // Initialize the dynamic_target_point visual dot object.
  dynamic_target_point_circle_.setFillColor(sf::Color::White);
  dynamic_target_point_circle_.setRadius(5);
  dynamic_target_point_circle_.setOrigin(5, 5);
  dynamic_target_point_circle_.setPosition(x_, -y_);

  heading_ = PI/2.5;
  car_body_.setRotation(-utils::ToDegree(heading_));
  car_body_.setFillColor(params.color);
  mass_body_ = 650;
  mass_fuel_ = 150;
  fuel_tank_empty_ = false;
  mass_total_ = 800;
  wheelbase_ = params.wheelbase;
  fuel_mass_per_km_ = 0.3;


  f_drive_max_ = params.max_drive_force;
  f_brake_max_ = params.max_brake_force;
  car_number_ = params.car_number;
  body_drag_coeff_ = params.body_drag_coeff;
  wing_down_coeff_ = params.wing_down_coeff;
  wing_drag_coeff_ = params.wing_drag_coeff;
  color_ = params.color;

  input_policy_name_ = params.input_policy_name;
  team_ = params.team_name;

  text_.setFont(*font_);
  text_.setCharacterSize(15);
  text_.setFillColor(sf::Color::White);

  lap_times_.clear();
}

Car::~Car() {
  track_ = nullptr;
}

void Car::SetDynamicTargetPoint(float x, float y) {
  dynamic_target_point_.x = x;
  dynamic_target_point_.y = y;
}

void Car::Update(float dt) {
  dt += residual_dt_from_last_update_;
  while (dt > max_min_step_dt_) {
    auto input = InputPolicy(max_min_step_dt_);
    last_input_ = input;
    dt -= max_min_step_dt_;
    MinStepUpdate(max_min_step_dt_, input);
  }
  residual_dt_from_last_update_ = dt;
  int triangle_strip_index = -1;
  is_on_track_ = track_->IsOnTrack({x_, y_}, triangle_strip_index);
  if (is_on_track_) {
    x_last_known_on_track_ = x_;
    y_last_known_on_track_ = y_;
  }
  if (display_params_.draw_car_label) {
    // Set speed, total mass, current lap time and distance to start string.
    text_.setFillColor(is_on_track_ ? sf::Color::White : sf::Color::Red);
    text_.setPosition(x_ + 3, -y_ - 30);
    text_.setString(std::to_string(GetSpeed() * 3.6) + "\n" +
                    std::to_string(mass_total_) + "\n" +
                    std::to_string(current_lap_time_) + "\n" +
                    std::to_string(track_->GetDistanceOnTrack({x_, y_})));
  }
}

void Car::SetDistanceOnTrack(float d) {
  distance_on_track_ = d;
}

void Car::SetLapCompleted(int lap) { lap_completed_ = lap; }

float Car::GetDistanceOnTrack() { return distance_on_track_; }

int Car::GetLapCompleted() { return lap_completed_; }

void Car::MinStepUpdate(float dt, CarInput input) {
  // update input
  // update immediate states
  // Simulate one step
  // Update delayed states(anything requires * dt)
  // update tyre condition
  LOG_D("DT: " << dt << " Acc: " << input.accel << " Bra: " << input.brake
        << " Ste: " << input.steering_angle);

  if (fuel_tank_empty_) {
    input.accel = 0;
  }
  f_drive_ = f_drive_max_ * input.accel;
  f_brake_ = f_brake_max_ * input.brake;
  turning_angle_ = -1 * input.steering_angle * max_turn_angle_;
  if (input.drs && input.brake == 0) {
    drs_ = true;
  }
  if (drs_ && input.brake != 0) {
    drs_ = false;
  }

  turning_radius_ = turning_angle_ == 0 ? FLT_MAX : wheelbase_ / sin(turning_angle_);

  // update forces
  float v_sqr = v_ * v_;
  f_body_drag_ = v_sqr * body_drag_coeff_;
  f_wing_drag_ = v_sqr * wing_drag_coeff_ * wing_angle_setting_ * (drs_ ? 0.8 : 1);
  f_down_ = mass_total_ * g + wing_down_coeff_ * f_wing_drag_;
  f_center_ = mass_total_ * v_sqr / turning_radius_;
  f_wheel_drag_ = v_ > 0 ? mass_total_ * g * wheel_drag_coeff_ : 0;
  heading_dot_ = v_ * sin(turning_angle_) / wheelbase_;

  bool sliding = std::sqrt(f_center_ * f_center_ + a_ * a_ * mass_total_ * mass_total_) > f_down_ * max_friction_coeff_;
  if (sliding) {
    LOG_D("!!! SLIDING !!!" << sqrt(f_center_ * f_center_ + a_ * a_ * mass_total_ * mass_total_)/f_down_/max_friction_coeff_
        << " Accel = " << input.accel << " Brake = " << input.brake << " Steer = " << input.steering_angle);
    // car_body_.setFillColor(sf::Color::Green);
  } else {
    car_body_.setFillColor(color_);
  }

  // update a, v, d
  a_ = (f_drive_ - f_body_drag_ - f_wing_drag_ - f_brake_ - f_wheel_drag_) / mass_total_;
  a_ *= tyre_.GetAccelCoeff();

  LOG_D("f_drive_: " << f_drive_ << " f_body_drag_: " << f_body_drag_ <<
        " f_wing_drag_: " << f_wing_drag_ << " f_wheel_drag_: " << f_wheel_drag_ <<
        " f_brake_: " << f_brake_);

  auto dv = a_ * dt;
  auto dd = (v_ + dv / 2) * dt;
  d_ += dd;
  v_ = std::max(0.f, v_ + dv);

  // update tyre condition
  tyre_.UpdateCondition(dt, v_, a_);

  // update fuel mass
  DecreaseFuelMass(fuel_mass_per_km_ * dd / 1000.0f);
  LOG_D("a: " << a_ << " v: " << v_ << " d: " << d_ << " m: " << mass_total_);

  // update heading
  auto dh = heading_dot_ * dt;
  heading_ = utils::ToNormalizedAngle(heading_ + dh);

  // update position
  auto dx = v_ * cos(heading_) * dt;
  auto dy = v_ * sin(heading_) * dt;

  // update lap time
  if (track_->IsCrossFinishLine(x_, y_, dx, dy)) {
    float l_old = GetDistanceOnTrack();
    float l_new = track_->GetDistanceOnTrack({x_+dx, y_+dy});
    SetDistanceOnTrack(l_new);
    float track_l = track_->GetTrackLength();
    float time_ratio_to_complete_last_lap = (track_l - l_old) / (l_new + track_l - l_old);
    current_lap_time_ += (time_ratio_to_complete_last_lap * dt);
    lap_completed_++;
    if (lap_completed_ == 1) {
      best_lap_info_.lap_time = current_lap_time_;
      best_lap_info_.lap_number = lap_completed_;
      best_lap_info_.tyre_type = tyre_.GetTypeString();
    } else {
      if (current_lap_time_ < best_lap_info_.lap_time) {
        best_lap_info_.lap_time = current_lap_time_;
        best_lap_info_.lap_number = lap_completed_;
        best_lap_info_.tyre_type = tyre_.GetTypeString();
      }
    }
    // Rely on lap_completed_ == 0 to filter out any car start behind the cross line
    if (lap_completed_ != 0) {
      lap_times_.push_back(current_lap_time_);
      LOG("Lap time: " << current_lap_time_ << " car: " << car_number_
                       << " Laps completed: " << lap_completed_);
      current_lap_time_ = (1 - time_ratio_to_complete_last_lap) * dt;
      if (current_lap_time_ < 0) {
        LOG("current_lap_time_ < 0! for car: " + std::to_string(car_number_));
        LOG("l_old: " + std::to_string(l_old));
        float l_old_debug = track_->GetDistanceOnTrack({x_, y_});
        LOG("l_old_debug: " + std::to_string(l_old_debug));
        LOG("l_new: " + std::to_string(l_new));
        LOG("track_l: " + std::to_string(track_l));
      }
    }
  } else {
    SetDistanceOnTrack(track_->GetDistanceOnTrack({x_ + dx, y_ + dy}));
    current_lap_time_ += dt;
  }

  x_prev_ = x_;
  y_prev_ = y_;
  x_ += dx;
  y_ += dy;
  xh_ = x_ + wheelbase_ * cos(heading_);
  yh_ = y_ + wheelbase_ * sin(heading_);
}

float Car::GetLastLapTime() {
  return lap_times_.empty() ? 0.f : lap_times_.back();
}

void Car::UpdateMass() {
  mass_total_ = mass_body_ + mass_fuel_ + mass_driver_;
}

void Car::SetBodyMass(float body_mass) {
  mass_body_ = body_mass;
  UpdateMass();
}

void Car::ResetLapCompleted() { lap_completed_ = 0; }

void Car::SetDriverMass(float driver_mass) {
  mass_driver_ = driver_mass;
  UpdateMass();
}

void Car::SetFuelMass(float fuel_mass) {
  mass_fuel_ = fuel_mass;
  UpdateMass();
}

void Car::DecreaseFuelMass(float decrease_mass) {
  if (mass_fuel_ >= decrease_mass) {
    mass_fuel_ -= decrease_mass;
  } else {
    mass_fuel_ = 0;
    fuel_tank_empty_ = true;
  }
  UpdateMass();
}

void Car::Draw() {
  car_body_.setRotation(-utils::ToDegree(heading_));
  car_body_.setPosition(sf::Vector2f(x_, -y_));
  app_->draw(car_body_);

  if (input_policy_name_ == INPUTPOLICYNAME::DYNAMIC_TARGET_POINT) {
    dynamic_target_point_circle_.setPosition(
        sf::Vector2f(dynamic_target_point_.x, -dynamic_target_point_.y));
    app_->draw(dynamic_target_point_circle_);
  }
  if (display_params_.draw_car_label) {
    app_->draw(text_);
    LOG_D("Heading: " << heading_ << " v: " << v_ << " Car position x: " << x_
                      << " y: " << y_);
  }
  if (display_params_.draw_car_sensor_beam) {
    for (auto& sd : boundary_sensing_) {
      app_->draw(sd.c);
      app_->draw(sd.line, 2, sf::Lines);
    }
  }
  // Draw waypoints if not empty
  if (input_policy_name_ == INPUTPOLICYNAME::WAYPOINTS && !waypoints_.empty()) {
    for (auto& w : waypoints_) {
      app_->draw(w.display_point);
    }
  }

  // Update driver panel if having one
  if (driver_panel_ != nullptr) {
    driver_panel_->SetLabelText(LabelName::SPEED,
        "SPD: " + std::to_string(utils::ToKmPerHour(GetSpeed())) + "km/h");
    driver_panel_->SetLabelText(LabelName::TEAM_NAME, team_);
    driver_panel_->SetLabelText(LabelName::DRIVER_NAME,
                                "DUMMY # " + std::to_string(car_number_));
    driver_panel_->SetLabelText(LabelName::FUEL_MASS,
        "Fuel: " + utils::ToStringWithPrecision(mass_fuel_, 2) + "kg");
    driver_panel_->SetLabelText(LabelName::CURRENT_LAP_TIME,
                                utils::ToLapTime(current_lap_time_));
    driver_panel_->SetLabelText(
        LabelName::BEST_LAP_TIME, "L" + std::to_string(best_lap_info_.lap_number) + " " +
        utils::ToLapTime(best_lap_info_.lap_time) + " " + best_lap_info_.tyre_type);
    driver_panel_->SetLabelText(LabelName::TYRE_CONDITION,
        utils::ToStringWithPrecision(tyre_.GetCondition() * 100, 0) + "% " + tyre_.GetTypeString());
  }
}

// set real world coordinates x and y, for drawing, y should be -y.
void Car::SetPosition(float x, float y) {
  x_ = x;
  y_ = y;
}

void Car::SetLastKnownOnTrackPosition(float x, float y) {
  x_last_known_on_track_ = x;
  y_last_known_on_track_ = y;
}

void Car::SetHeading(float h) {
  heading_ = h;
}

float Car::GetSpeed() {
  return v_;
}

float Car::GetHeading() {
  return heading_;
}

float Car::GetX() {
  return x_;
}

float Car::GetY() {
  return y_;
}

float Car::GetAcceleration() {
  return a_;
}

CarInput Car::InputPolicy(float dt) {
  {
    // Compute the max turning angle
    float f_margin = f_down_ * max_friction_coeff_ - std::sqrt(f_center_ * f_center_ + a_ * a_ * mass_total_ * mass_total_);
  }

  float accel = 0, brake = 0, driving_wheel_angle = 0;
  bool drs = false;

  float max_accel_delta = dt * full_accel_time_;
  float max_brake_delta = dt * full_brake_time_;
  float max_steer_delta = dt * full_turn_time_ / 2; // full turn is from -1 to 1(2)

  float old_accel = last_input_.accel;
  float old_brake = last_input_.brake;
  float old_steer = last_input_.steering_angle;

  // Do boundary detection, find out which the direction along which sd.d has maximum.
  longest_sensor_beam_ = 0; // important to reset for each update step
  for (auto& sd : boundary_sensing_) {
    sd.d = track_->DistanceToBoundary({xh_, yh_}, heading_ - PI/2 + sd.direction, boundary_sensor_precision_, boundary_sensor_range_);
    if (sd.d > longest_sensor_beam_) {
      longest_sensor_beam_ = sd.d;
      best_sensing_direction_ = heading_ - PI / 2 + sd.direction;
    }
    Point boundary_position = {xh_ + sd.d * cos(heading_ - PI/2 + sd.direction), yh_ + sd.d * sin(heading_ - PI/2 + sd.direction)};
    if (display_params_.draw_car_sensor_beam) {
      sd.c.setPosition(boundary_position.x, -boundary_position.y);
      sd.c.setFillColor(sd.d < boundary_sensor_range_ ? sf::Color::Red
                                                      : sf::Color::Green);
      sd.line[0].position = {xh_, -yh_};
      sd.line[1].position = {boundary_position.x, -boundary_position.y};
    }
  }

  if (input_policy_name_ == INPUTPOLICYNAME::JOYSTICK) {
    accel = sf::Joystick::getAxisPosition(0, sf::Joystick::R);
    accel = (accel + 100) / 200;
    brake = sf::Joystick::getAxisPosition(0, sf::Joystick::Z);
    brake = (brake + 100) / 200;
    driving_wheel_angle = sf::Joystick::getAxisPosition(0, sf::Joystick::X);
    float steering_threshold = 20;
    drs = sf::Joystick::isButtonPressed(0, 2);
    if (std::abs(driving_wheel_angle) >= steering_threshold) {
      if (driving_wheel_angle > 0) {
        (driving_wheel_angle -= steering_threshold) /= (100 - steering_threshold);
      } else {
        (driving_wheel_angle += steering_threshold) /= (100 - steering_threshold);
      }
    } else {
      driving_wheel_angle = 0;
    }
  } else if (input_policy_name_ == INPUTPOLICYNAME::DEFAULT) {
    // The goal is to set {accel, brake, driving_wheel_angle, drs}
    // Rely on five/three beams, forward, left, right, or more.

    // find sensor reading max direction
    float right_sum = 0;
    float left_sum = 0;
    float right_89 = 0;
    float left_91 = 0;
    float left = 0;
    float right = 0;

    float max_d = 0;
    float max_d_angle = 0;
    float forward_d = 0;
    for (auto& sd : boundary_sensing_) {
      if (sd.d > max_d || (sd.d == max_d && sd.direction == PI/2)) {
        max_d = sd.d;
        max_d_angle = sd.direction;
      }
      if (sd.direction == PI/2) {
        forward_d = sd.d;
      }
      if (sd.direction < PI/2) {
        right_sum += sd.d;
      }
      if (sd.direction > PI/2) {
        left_sum += sd.d;
      }
    }

    right_89 = boundary_sensing_[boundary_sensing_.size()/2-1].d;
    left_91 = boundary_sensing_[boundary_sensing_.size()/2+1].d;
    bool next_turn_is_right = (right_89 > left_91);
    bool next_turn_is_left = (left_91 > right_89);


    right = boundary_sensing_[0].d;
    left = boundary_sensing_.back().d;
    Point current_furthest_point_ = {x_ + max_d * cos(max_d_angle), y_ + max_d * sin(max_d_angle)};

    // determine driving_wheel_angle
    int direction = (PI/2 - max_d_angle) >= 0 ? 1 : -1;
    driving_wheel_angle = direction * pow(abs(PI/2 - max_d_angle) / (PI/2), 0.1);
    if (right > left && max_d < boundary_sensor_range_*0.5) {
      driving_wheel_angle = std::min(1.0f, driving_wheel_angle += 0.7);
    } else if (left > right && max_d < boundary_sensor_range_*0.5) {
      driving_wheel_angle = std::max(-1.0f, driving_wheel_angle -= 0.7);
    } else {
      driving_wheel_angle += 0;
    }
    // LOG("Driving wheel angle: " << driving_wheel_angle);
    if (max_d >= 150) {
      accel = 1;
    } else if (max_d > 50) {
      accel = 0.5;
    }

    if (max_d > 0 && max_d < boundary_sensor_range_ && v_*v_ > 2*30*max_d) {
      brake = 1;
      accel = 0;
    }
  } else if (input_policy_name_ == INPUTPOLICYNAME::CENTERLINE) {
    float left = 0;
    float right = 0;
    right = boundary_sensing_[0].d;
    left = boundary_sensing_.back().d;
    float magnitude = 0;
    if (v_ > 60) {
      magnitude = 0.8;
    } else if (v_ > 40) {
      magnitude = 1;
    } else if (v_ > 20) {
      magnitude = 1;
    } else if (v_ > 0) {
      magnitude = 1;
    }
    if (!is_on_track_ || fabs(heading_ - track_->GetTrackDirection({x_, y_})) > PI/18) {
      magnitude = 1.0;
    } else {
      magnitude = 0.6;
    }
    if(right > left) {
      driving_wheel_angle = 1;
    } else if (left > right) {
      driving_wheel_angle = -1;
    } else {
      driving_wheel_angle = 0;
    }

    float max_d = 0;
    float max_d_angle = 0;
    for (auto& sd : boundary_sensing_) {
      if (sd.d > max_d || (sd.d == max_d && sd.direction == PI/2)) {
        max_d = sd.d;
        max_d_angle = sd.direction;
      }
    }

    accel = 1;

    if (max_d > 0 && max_d < boundary_sensor_range_ && v_*v_ > 2*30*max_d) {
      brake = 1;
      accel = 0;
    }
  } else if (input_policy_name_ == INPUTPOLICYNAME::LOOKFAR) {
    // direction to the best direction
    // if dist is short, reduce speed
    if (is_on_track_) {
      float diff_angle = utils::DiffAngleRadianInNegativePositivePi(
          best_sensing_direction_, heading_);
      if (diff_angle > 0) {
        driving_wheel_angle = -1.f * std::pow(diff_angle / PI, 0.25);  // left is negative
      } else if (diff_angle < 0) {
        driving_wheel_angle =
            1.f * std::pow(diff_angle / PI, 0.25);  // right is positive
      } else {
        driving_wheel_angle = 0;
      }

      if (longest_sensor_beam_ > v_ * v_ / 2 / 60) {
        brake = 0;
        accel = 1;
      } else {
        accel = 0;
        brake = 0.8;
      }
    } else {
      float prev_track_direction = track_->GetTrackDirection({x_last_known_on_track_, y_last_known_on_track_});
      float diff_angle_in_radian = utils::DiffAngleRadianInNegativePositivePi(
          prev_track_direction, heading_);
      if (diff_angle_in_radian > 0) {
        driving_wheel_angle =
            -1.f * std::pow(diff_angle_in_radian / PI, 0.25);  // turn left
      } else if (diff_angle_in_radian < 0){
        driving_wheel_angle = 1.f * std::pow(diff_angle_in_radian / PI, 0.25);
        ;  // turn right
      }
      accel = 0.f;
      brake = 0.f;
      LOG("I saw this again." + std::to_string(heading_) + " " + std::to_string(prev_track_direction));
    }

  } else if (input_policy_name_ == INPUTPOLICYNAME::DEFAULT_2) {
    // The goal is to set {accel, brake, driving_wheel_angle, drs}
    // Rely on five/three beams, forward, left, right, or more.

    // find sensor reading max direction
    float right_sum = 0;
    float left_sum = 0;
    float right_89 = 0;
    float left_91 = 0;
    float left = 0;
    float right = 0;

    float max_d = 0;
    float max_d_angle = 0;
    float forward_d = 0;
    for (auto& sd : boundary_sensing_) {
      if (sd.d > max_d || (sd.d == max_d && sd.direction == PI/2)) {
        max_d = sd.d;
        max_d_angle = sd.direction;
      }
      if (sd.direction == PI/2) {
        forward_d = sd.d;
      }
      if (sd.direction < PI/2) {
        right_sum += sd.d;
      }
      if (sd.direction > PI/2) {
        left_sum += sd.d;
      }
    }

    right_89 = boundary_sensing_[boundary_sensing_.size()/2-1].d;
    left_91 = boundary_sensing_[boundary_sensing_.size()/2+1].d;
    bool next_turn_is_right = (right_89 > left_91);
    bool next_turn_is_left = (left_91 > right_89);


    right = boundary_sensing_[0].d;
    left = boundary_sensing_.back().d;
    Point current_furthest_point_ = {x_ + max_d * cos(max_d_angle), y_ + max_d * sin(max_d_angle)};

    // determine driving_wheel_angle
    int direction = (PI/2 - max_d_angle) >= 0 ? 1 : -1;
    driving_wheel_angle = direction * pow(abs(PI/2 - max_d_angle) / (PI/2), 0.1);
    if (right > left && max_d < boundary_sensor_range_*0.5) {
      driving_wheel_angle = std::min(1.0f, driving_wheel_angle += 0.5);
    } else if (left > right && max_d < boundary_sensor_range_*0.5) {
      driving_wheel_angle = std::max(-1.0f, driving_wheel_angle -= 0.5);
    } else {
      driving_wheel_angle += 0;
    }
    // LOG("Driving wheel angle: " << driving_wheel_angle);
    if (max_d >= 150) {
      accel = 1;
    } else if (max_d > 50) {
      accel = 0.5;
    }

    if (max_d > 0 && max_d < boundary_sensor_range_ && v_*v_ > 2*20*max_d) {
      brake = 0.5;
      accel = 0;
    } else if (max_d > 0 && max_d < boundary_sensor_range_ && v_*v_ > 2*28*max_d) {
      brake = 1;
      accel = 0;
    }
  } else if (input_policy_name_ == INPUTPOLICYNAME::DYNAMIC_TARGET_POINT) {
    if (is_on_track_) {
      // calculate the furthest point
      if (longest_sensor_beam_ == 0) {
        driving_wheel_angle = old_steer;
        accel = old_accel;
        brake = old_brake;
      } else {
        Point current_furthest_point = {
            x_ + longest_sensor_beam_ * cos(best_sensing_direction_),
            y_ + longest_sensor_beam_ * sin(best_sensing_direction_)};

        // determine if the target point should be updated
        if (utils::Distance(dynamic_target_point_, {x_, y_}) <=
            utils::Distance(dynamic_target_point_, current_furthest_point)) {
          dynamic_target_point_.x = current_furthest_point.x;
          dynamic_target_point_.y = current_furthest_point.y;
        }

        // set driving params toward the dynamic_target_point
        float target_direction = utils::PrincipalArgument(
            {dynamic_target_point_.x - x_, dynamic_target_point_.y - y_});
        float diff_angle =
            utils::DiffAngleRadianInNegativePositivePi(target_direction, heading_);
        if (diff_angle > 0) {
          driving_wheel_angle = -1 * std::pow(diff_angle / PI, 0.25);  // left is negative
        } else if (diff_angle < 0) {
          driving_wheel_angle = 1 * std::pow(-diff_angle / PI, 0.25);  // right is positive
        } else {
          driving_wheel_angle = 0;
        }
        if (utils::Distance(current_furthest_point, {x_, y_}) >
            v_ * v_ / 2 / 25) {
          brake = 0;
          accel = 1;
        } else {
          accel = 0;
          brake = 1;
        }
      }
    } else {
      // LOG("Not on track.");
      float target_direction = utils::PrincipalArgument(
          {dynamic_target_point_.x - x_, dynamic_target_point_.y - y_});
      float diff_angle = utils::DiffAngleRadianInNegativePositivePi(
          target_direction, heading_);
      if (diff_angle > 0) {
        driving_wheel_angle = -1 * std::pow(diff_angle / PI, 0.25);  // left is negative
      } else if (diff_angle < 0) {
        driving_wheel_angle = 1 * std::pow(-diff_angle / PI, 0.25);  // right is positive
      } else {
        driving_wheel_angle = 0;
      }
      if (GetSpeed() > 100) {
        brake = 1;
        accel = 0;
      } else {
        accel = 1;
        brake = 0;
      }
    }

  } else if (input_policy_name_ == INPUTPOLICYNAME::WAYPOINTS) {
    // determine the next waypoint
    /*int waypoint_index = 0;
    float actual_distance_on_track = distance_on_track_;
    LOG("Actual distance on track: " + std::to_string(actual_distance_on_track));
    while (actual_distance_on_track < 0) {
      actual_distance_on_track += track_->GetTrackLength();
    }
    LOG("Actual distance on track: " + std::to_string(actual_distance_on_track));
    while (actual_distance_on_track >= waypoints_[waypoint_index].distance_on_track) {
      waypoint_index++;
    }
    if (waypoint_index == waypoints_.size()) {
      waypoint_index = 0;
    }
    */
    if (utils::Distance({x_,y_}, waypoints_[waypoint_index_].p) < 3) {
      waypoint_index_++;
      if (waypoint_index_ == waypoints_.size()) {
        waypoint_index_ = 0;
      }
    }
    LOG("Waypoint index: " + std::to_string(waypoint_index_));

    dynamic_target_point_.x = waypoints_[waypoint_index_].p.x;
    dynamic_target_point_.y = waypoints_[waypoint_index_].p.y;

    // set driving params toward the dynamic_target_point
    float target_direction = utils::PrincipalArgument(
        {dynamic_target_point_.x - x_, dynamic_target_point_.y - y_});
    float diff_angle =
        utils::DiffAngleRadianInNegativePositivePi(target_direction, heading_);
    // LOG("Heading: " + std::to_string(heading_));
    // LOG("Target direction: " + std::to_string(target_direction));
    // LOG("Diff angle: " + std::to_string(diff_angle));
    if (diff_angle > 0) {
      driving_wheel_angle = -1 * std::pow(diff_angle / PI, 0.25);  // left is negative
      // LOG("Turn left");
    } else if (diff_angle < 0) {
      driving_wheel_angle = 1 * std::pow(-diff_angle / PI, 0.25);  // right is positive
      // LOG("Turn right");
    } else {
      driving_wheel_angle = 0;
    }

    if (!is_on_track_ || longest_sensor_beam_ == 0) {
      accel = old_accel;
      brake = old_brake;
    } else {
      Point current_furthest_point = {
          x_ + longest_sensor_beam_ * cos(best_sensing_direction_),
          y_ + longest_sensor_beam_ * sin(best_sensing_direction_)};


      if (utils::Distance(current_furthest_point, {x_, y_}) >
          v_ * v_ / 2 / 25) {
        brake = 0;
        accel = 1;
      } else {
        accel = 0;
        brake = 1;
      }
    }
  }

  if (accel > old_accel + max_accel_delta) {
    // accel = old_accel + max_accel_delta;
  } else if (accel < old_accel - max_accel_delta) {
    // accel = old_accel - max_accel_delta;
  }

  if (brake > old_brake + max_brake_delta) {
    // brake = old_brake + max_brake_delta;
  } else if (brake < old_brake - max_brake_delta) {
    // brake = old_brake - max_brake_delta;
  }

  if (driving_wheel_angle > old_steer + max_steer_delta) {
    // driving_wheel_angle = old_steer + max_steer_delta;
  } else if (driving_wheel_angle < old_steer - max_steer_delta) {
    // driving_wheel_angle = old_steer - max_steer_delta;
  }

  return {accel, brake, driving_wheel_angle, drs};
}

void Car::SetTrack(Track* track) {
  track_ = track;
}

void Car::SetWaypoints(std::vector<WayPoint> waypoints) {
  waypoints_ = waypoints;
  for (auto& w : waypoints_) {
    Point cp = track_->GetTrackCenterPoint(w.distance_on_track);
    float direction = track_->GetTrackDirection(cp);
    LOG("Direction at distance: " + std::to_string(w.distance_on_track) + " is: " + std::to_string(direction));
    float bias_direction = w.bias >= 0 ? -PI/2 : PI/2;
    Point wp = {cp.x + std::cos(direction + bias_direction) * std::abs(w.bias), -cp.y - std::sin(direction + bias_direction) * std::abs(w.bias)};
    w.p.x = wp.x;
    w.p.y = -wp.y;
    w.display_point.setPosition(wp.x, wp.y);
  }
}

void Car::SetDriverPanel(DriverPanel* driver_panel) {
  driver_panel_ = driver_panel;
}

void Car::SetRaceManager(RaceManager* race_manager) {
  race_manager_ = race_manager;
}

int Car::GetCarNumber() { return car_number_; }

bool Car::IsBehind(Car* car) {
  if (car->GetLapCompleted() > lap_completed_) {
    return true;
  } else if (car->GetLapCompleted() == lap_completed_){
    return GetDistanceOnTrack() < car->GetDistanceOnTrack();
  } else {
    return false;
  }
}

void Car::MountTyre(TYRETYPE tyre_type) { tyre_ = Tyre(tyre_type); }

void Car::SetOnTrack(bool is_on_track) { is_on_track_ = is_on_track; }
