#ifndef CAR_H
#define CAR_H

#include <SFML/Graphics.hpp>
#include "Debug.h"
#include "Track.h"
#include "CommonDataStructure.h"
#include "DriverPanel.h"

class Track;
class RaceManager;

// Waypoint is used when input policy is WAYPOINTS. Navigate the car to each waypoint to complete a lap.
struct WayPoint {
  // Distance on the track
  float distance_on_track = 0;
  // Bias distance to the center point. Left is negative, right is positive.
  float bias = 0;
  // x,y coordinates
  Point p = {0, 0};
  // display circleshape
  sf::CircleShape display_point;

  WayPoint() {
    display_point.setFillColor(sf::Color::Magenta);
    int radius = 3;
    display_point.setRadius(radius);
    display_point.setOrigin(radius, radius);
  }
};

enum INPUTPOLICYNAME {
  DEFAULT = 0,
  JOYSTICK = 1,
  CENTERLINE = 2,
  DEFAULT_2 = 3,
  LOOKFAR = 4, // Heading toward the direction in which sensor beam has longest direction to boundary.
  DYNAMIC_TARGET_POINT = 5, // Dynamically update a target point on the track and always driving toward that point
  WAYPOINTS = 6, // Used preconfigured waypoints as target points to complete the lap
};

enum TYRETYPE {
  C1,
  C2,
  C3,
  C4,
  C5,
  INTERMEDIATE,
  WET,
};

// The simple design would be using a multiplier to the dt to simulate different tyre's lap time
struct TyreFactoryParams {
  // The compound
  TYRETYPE type;
  // type string
  std::string type_string;
  // Color
  sf::Color color;
  // Multiplier to the acceleration when brand new
  float accel_coeff_new;
  // Multiplier to the accel when at lowest working condition, that is the cliff
  // point
  float accel_coeff_cliff_point;
  // Multiplier to the dt when at zero condition
  float accel_coeff_worn_out;
  // lowest working condition-cliff point, e.g., 0.2. Below this cond, accel_coeff
  // would drop dramatically.
  float cliff_point_condition;

  // Tyre degrading params
  float dist_degrade_coeff;   // -cond / meter
  float accel_degrade_coeff;  // -cond / ((m/s^2) * dt), both accel and brake will just
                      // use the acceleration from the car
};

class Tyre {
 public:
  Tyre() = default;
  Tyre(TYRETYPE type) {
    if (type == TYRETYPE::C1) {
      factory_params_.type = type;
      factory_params_.type_string = "C1";
      factory_params_.accel_coeff_new = 1.f;
      factory_params_.accel_coeff_cliff_point = 0.98;
      factory_params_.accel_coeff_worn_out = 0.94;
      factory_params_.cliff_point_condition = 0.2;

      factory_params_.dist_degrade_coeff = 0.00001f;
      factory_params_.accel_degrade_coeff = 0.0000025f;
      accel_coeff_ = factory_params_.accel_coeff_new;
    }
  }

  void UpdateCondition(float dt, float v, float accel) {
    condition_ -= (v * dt * factory_params_.dist_degrade_coeff);
    condition_ -= (fabs(accel) * dt * factory_params_.accel_degrade_coeff);
    if (condition_ < 0) {
      condition_ = 0.f;
    }
    if (condition_ >= factory_params_.cliff_point_condition) {
      accel_coeff_ = (condition_ - factory_params_.cliff_point_condition) /
                      (1.f - factory_params_.cliff_point_condition) *
                      (factory_params_.accel_coeff_new -
                       factory_params_.accel_coeff_cliff_point) +
                  factory_params_.accel_coeff_cliff_point;
    } else {
      accel_coeff_ = (condition_) / factory_params_.cliff_point_condition *
                      (factory_params_.accel_coeff_cliff_point -
                       factory_params_.accel_coeff_worn_out) +
                  factory_params_.accel_coeff_worn_out;
    }
  }

  float GetAccelCoeff() {
    return accel_coeff_;
  }
  float GetCondition() { return condition_; }
  std::string GetTypeString() { return factory_params_.type_string; }

 private:
  TyreFactoryParams factory_params_;
  float condition_ = 1.f;
  float accel_coeff_;

};

struct CarInput {
  // [0, 1]
  float accel = 0;
  // [0, 1]
  float brake = 0;
  // [-1, 1] left is negative, right is positive
  float steering_angle = 0;
  // is DRS open
  bool drs = false;
};

struct CarSpecificParams {
  int car_number = -1;
  // max driving force
  float max_drive_force = 0;
  // max braking force
  float max_brake_force = 0;
  // wheelbase, length between front and rear wheel axis
  float wheelbase = 0;
  // car width
  float width = 0;
  // car length
  float length = 0;
  // body drag coeff
  float body_drag_coeff = 0;
  // wing downforce over gravity gain
  float wing_down_coeff = 0;
  // wing drag coeff
  float wing_drag_coeff = 0;
  // team name
  std::string team_name = "";
  // car display color
  sf::Color color = sf::Color::Transparent;
  // Input policy name
  INPUTPOLICYNAME input_policy_name = INPUTPOLICYNAME::DEFAULT;
};

struct CarDisplayParams {
  // Draw car label on the circuit
  bool draw_car_label = false;
  // Draw car sensor_beam on the circuit
  bool draw_car_sensor_beam = false;

};

struct CircuitBoundarySensingData {
  // The direction of the sensing beam in radian. Beam starting from the car
  // position. Direction range is 0(right) to PI(left). PI/2 means heading
  // direction.
  float direction = 0;
  // Distance to the circuit boundary in this direction.
  float d = 0;
  // Circle object
  sf::CircleShape c;
  // Line shape connecting the position of the car to the Circle object.
  sf::Vertex line[2];

  CircuitBoundarySensingData() {
    c.setRadius(2);
    c.setOrigin(2,2);
  }
};

struct BestLapInfo {
  // On which lap is the best lap occurs
  int lap_number = 0;
  // best lap time in float
  float lap_time = 0;
  // Tyre type string this best lap was on
  std::string tyre_type;
};

class Car
{
  public:
    Car(sf::RenderWindow* app, sf::Font* font, CarSpecificParams params, CarDisplayParams display_params);
    virtual ~Car();

    void Update(float dt);
    void MinStepUpdate(float dt, CarInput input);

    void SetBodyMass(float body_mass);
    void SetDriverMass(float driver_mass);
    void SetFuelMass(float fuel_mass);
    void DecreaseFuelMass(float decrease_mass);
    void UpdateMass();

    void KeyboardAccel(bool accel);
    void KeyboardBrake(bool brake);
    void KeyboardLeft(bool left);
    void KeyboardRight(bool right);

    void SetWaypoints(std::vector<WayPoint> waypoints);

    void SetPosition(float x, float y);
    void SetLastKnownOnTrackPosition(float x, float y);
    void SetHeading(float h); // in radian

    float GetSpeed();
    float GetHeading();
    float GetX();
    float GetY();
    float GetAcceleration();

    void SetTrack(Track* track);
    // Called in Track::RegisterCar to initialize the target point to the car position before update.
    void SetDynamicTargetPoint(float x, float y);
    void SetOnTrack(bool is_on_track);


    void Draw();

    void SetDriverPanel(DriverPanel* driver_panel);

    void SetRaceManager(RaceManager* race_manager);

    void ResetLapCompleted();
    void SetLapCompleted(int lap);
    int GetLapCompleted();
    float GetLastLapTime();
    void SetDistanceOnTrack(float d);
    float GetDistanceOnTrack();

    int GetCarNumber();

    bool IsBehind(Car* car);
    void MountTyre(TYRETYPE tyre_type);

  private:
    CarInput InputPolicy(float dt);

  private:
    // Panel
    DriverPanel* driver_panel_ = nullptr;

    // UI
    sf::RenderWindow* app_;
    sf::Font* font_;

    sf::View view_;

    sf::RectangleShape car_body_;
    sf::CircleShape dynamic_target_point_circle_;

    // Waypoints vector
    std::vector<WayPoint> waypoints_;
    int waypoint_index_ = 0;

    // Circuit boundary sensing data.
    std::vector<CircuitBoundarySensingData> boundary_sensing_;
    // Detect the boundary within this range
    float boundary_sensor_range_ = 500;
    // Detect the boundary distance within this precision
    float boundary_sensor_precision_ = 0.2;

    // mass unit in Kg
    float mass_body_ = 0;
    float mass_fuel_ = 0;
    float mass_driver_ = 0;
    float mass_total_ = 0;

    bool drs_ = false;

    // force unit in Newton
    float f_drive_ = 0;
    float f_body_drag_ = 0;
    float f_wing_drag_ = 0;
    float f_down_ = 0;
    float f_drive_max_ = 15000;
    float f_wheel_drag_ = 0;
    float f_brake_ = 0;
    float f_brake_max_ = 40000;
    float wheel_drag_coeff_ = 0.015;
    float f_center_ = 0;

    float heading_ = 0; // in radian
    float best_sensing_direction_ = 0; // the sensor beam has longest distance to boundary along this direction. in radian.
    float longest_sensor_beam_ = 0; // from car to boundary distance along best_sensing_direction_
    float heading_dot_ = 0; // derivative of heading, radian/second
    float turning_angle_ = 0; // angle between front wheel and the rear wheel(body) middle = 0, left: positive, right : negative
    float turning_radius_ = 0; // wheelbase_ / sin(turning_angle_)
    bool is_on_track_ = false;

    float x_ = 0; // position x
    float y_ = 0; // position y
    float x_prev_ = 0; // previous step's x position
    float y_prev_ = 0; // previous step's y position
    float x_last_known_on_track_;
    float y_last_known_on_track_;
    float xh_ = 0; // head x position
    float yh_ = 0; // head y position

    float a_ = 0; // acceleration
    float v_ = 0; // velocity
    float d_ = 0; // displacement

    float wheelbase_ = 0; // distance between front and rear wheel axis
    float car_width_ = 0; // car width
    float car_length_ = 0; // car total length

    float max_friction_coeff_ = 2.83;

    // brand specific params
    float fuel_mass_per_km_ = 0; // fuel mass usage per kilometer
    float body_drag_coeff_ = 0.75;
    float wing_down_coeff_ = 6.4;
    float wing_drag_coeff_ = 0.75;


    // max possible dt for accuracy
    float max_min_step_dt_ = 0.05; // seconds
    float residual_dt_from_last_update_ = 0.f; // this number is less than max_min_step_dt_, at each update, add this residual time to the dt and update every max_min_step_dt_, then leave the residual to the next update.

    // wing setting
    float wing_angle_setting_ = 1.5; // must between 1(italy) and 2.5(monaco)

    // max turn angle
    float max_turn_angle_ = 14 * (PI/180);

    // Fuel tank empty flag
    bool fuel_tank_empty_ = true;

    // keyboard control
    float keyboard_turn_rate_ = 0.2; // seconds to reach maximum from zero;
    float keyboard_accel_rate_ = 0.2; // seconds to reach maximum;

    Track* track_ = nullptr;

    sf::Text text_;

    float current_lap_time_ = 0;
    std::vector<float> lap_times_;
    BestLapInfo best_lap_info_;
    int lap_completed_ = 0;
    float distance_on_track_ = 0; // distance from the track start point, this variable is always positive

    int car_number_ = 0;
    std::string team_;
    sf::Color color_;

    Point furthest_point_ = {FLT_MAX, FLT_MAX};
    Point dynamic_target_point_ = {FLT_MAX, FLT_MAX}; // used when input policy is dynamic_target_point. Always drive toward this point.

    INPUTPOLICYNAME input_policy_name_ = INPUTPOLICYNAME::DEFAULT;

    // moved to driver params
    float full_turn_time_ = 0.5; // time used to steer from -1 to 1
    float full_accel_time_ = 0.2; // time used to accel from 0 to 1
    float full_brake_time_ = 0.2; // time used to brake from 0 to 1

    // input from last update
    CarInput last_input_;

    // Display params
    CarDisplayParams display_params_;

    // Racemanager
    RaceManager* race_manager_ = nullptr;

    Tyre tyre_;

};

#endif // CAR_H
