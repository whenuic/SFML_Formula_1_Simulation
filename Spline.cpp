#include "Spline.h"
#include <iostream>
#include <math.h>
#include <cfloat>
#include <unordered_map>

Spline::Spline(sf::RenderWindow* app, sf::Font* font, float dt, float pixel_to_dist_gain) :
  app_(app),
  font_(font),
  dt_(dt),
  pixel_to_dist_gain_(pixel_to_dist_gain) {
  car_.setFillColor(sf::Color::White);
  car_.setRadius(10);
  car_.setOrigin(10, 10);
  car_speed_text_.setFont(*font_);
  car_speed_text_.setFillColor(sf::Color::White);
  car_speed_text_.setPosition(10, 10);
  lap_time_ = 0.0f;
  lap_time_text_.setFont(*font_);
  lap_time_text_.setFillColor(sf::Color::White);
  lap_time_text_.setPosition(10, 50);
  dt_text_.setFont(*font_);
  dt_text_.setFillColor(sf::Color::White);
  dt_text_.setPosition(10, 90);

  track_width_ = 15.0f / pixel_to_dist_gain_;
  pit_width_ = 15.0f / pixel_to_dist_gain_;
  car_speed_ = 50;
  laps_time_.clear();
}

Spline::~Spline() {

}

float GetDistance(Point p1, Point p2) {
  return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

int FindClosestControlPoint(const std::vector<ControlPoint>& control_points, Point p) {
  int result_index = -1;
  float min_dist = FLT_MAX;
  for (int i=0; i<control_points.size(); i++) {
    auto dist = GetDistance(control_points[i].p, p);
    if (dist < min_dist) {
      min_dist = dist;
      result_index = i;
    }
  }
  return result_index;
}

void Spline::HighlightPoint(int index) {
  control_points_[index].circle.setFillColor(sf::Color::Yellow);
}

void Spline::UnHighlightPoint(int index) {
  control_points_[index].circle.setFillColor(sf::Color::Red);
}

void Spline::SelectPoint(Point p) {
  int selected_index = FindClosestControlPoint(control_points_, p);
  if (GetDistance(control_points_[selected_index].p, p) <= select_point_distance_threshold_) {
    UnHighlightPoint(selected_control_point_index_);
    selected_control_point_index_ = selected_index;
    HighlightPoint(selected_control_point_index_);
  }
}

void Spline::SelectNextPoint() {
  UnHighlightPoint(selected_control_point_index_);
  selected_control_point_index_++;
  if (selected_control_point_index_ > control_points_.size() - 1) {
    selected_control_point_index_ -= control_points_.size();
  }
  HighlightPoint(selected_control_point_index_);
}

void Spline::SelectPreviousPoint() {
  UnHighlightPoint(selected_control_point_index_);
  selected_control_point_index_--;
  if (selected_control_point_index_ < 0) {
    selected_control_point_index_ += control_points_.size();
  }
  HighlightPoint(selected_control_point_index_);
}

void Spline::RemovePoint(Point p) {
  int remove_index = FindClosestControlPoint(control_points_, p);
  if (remove_index < 0 ||
      GetDistance(control_points_[remove_index].p, p) > select_point_distance_threshold_) {
    return;
  }
  // Update control point index
  for (int i=remove_index; i<control_points_.size(); i++) {
    control_points_[i].index--;
    control_points_[i].text.setString(std::to_string(control_points_[i].index));
  }
  // Remove control point and update Splines interpolated points
  if (remove_index >= 0) {
    auto iter = control_points_.begin() + remove_index;
    control_points_.erase(iter);
  }
  UpdateSplines(control_points_, interpolated_points_, track_strip_, track_bound2index_, track_width_, true/*is_loop*/);
}

void Spline::RemoveSelectedPoint() {
  // Update control point index
  for (int i=selected_control_point_index_+1; i<control_points_.size(); i++) {
    control_points_[i].index--;
    control_points_[i].text.setString(std::to_string(control_points_[i].index));
  }
  // Remove control point and update Splines interpolated points
  if (selected_control_point_index_ >= 0) {
    auto iter = control_points_.begin() + selected_control_point_index_;
    control_points_.erase(iter);
  }
  UpdateSplines(control_points_, interpolated_points_, track_strip_, track_bound2index_, track_width_, true/*is_loop*/);
}

void Spline::AddPoint(Point p) {
  AddPointAt(p, control_points_.size());
}

void Spline::AddPoints(const std::vector<Point>& pts) {
  for (const auto& p : pts) {
    AddPoint(p);
  }
}

void Spline::SetPitPoints(const std::vector<Point> pts,
                          int in_index,
                          int out_index) {
  LOG(in_index << " " << out_index << " " << pts.size());
  ControlPoint in, out;
  pit_control_points_.clear();
  in.p = control_points_[in_index].p;
  in.index = 0;

  sf::CircleShape circle;
  circle.setFillColor(sf::Color::Green);
  circle.setRadius(5);
  circle.setOrigin(5,5);
  circle.setPosition(in.p.x, -in.p.y);
  LOG(in.p.x << " " << -in.p.y);
  in.circle = circle;

  sf::Text text;
  text.setCharacterSize(15);
  text.setFillColor(sf::Color::Green);
  text.setFont(*font_);
  text.setPosition(circle.getPosition());
  text.setString(std::to_string(in.index));
  in.text = text;

  pit_control_points_.push_back(in);

  out.p = control_points_[out_index].p;
  out.index = pts.size() + 1;

  sf::CircleShape circle2;
  circle2.setFillColor(sf::Color::Green);
  circle2.setRadius(5);
  circle2.setOrigin(5,5);
  circle2.setPosition(out.p.x, -out.p.y);
  out.circle = circle2;

  sf::Text text2;
  text2.setCharacterSize(15);
  text2.setFillColor(sf::Color::Green);
  text2.setFont(*font_);
  text2.setPosition(circle2.getPosition());
  text2.setString(std::to_string(out.index));
  out.text = text2;

  for (int i=0; i<pts.size(); i++) {
    ControlPoint ctrl_pt;
    ctrl_pt.p = pts[i];
    ctrl_pt.index = i+1;

    sf::CircleShape circle3;
    circle3.setFillColor(sf::Color::Green);
    circle3.setRadius(5);
    circle3.setOrigin(5,5);
    circle3.setPosition(ctrl_pt.p.x, -ctrl_pt.p.y);
    ctrl_pt.circle = circle3;

    sf::Text text3;
    text3.setCharacterSize(15);
    text3.setFillColor(sf::Color::Green);
    text3.setFont(*font_);
    text3.setPosition(circle3.getPosition());
    text3.setString(std::to_string(ctrl_pt.index));
    ctrl_pt.text = text3;

    pit_control_points_.push_back(ctrl_pt);
  }
  pit_control_points_.push_back(out);

  UpdateSplines(pit_control_points_, pit_interpolated_points_, pit_strip_, pit_bound2index_, pit_width_, false/*is not loop*/);

}

void Spline::AddPointAt(Point pt, int index) {
  if (index > control_points_.size()) {
    std::cerr << "Target index beyond control_points_ length." << std::endl;
  }
  ControlPoint cp;
  cp.p = pt;
  sf::CircleShape circle;
  circle.setFillColor(sf::Color::Red);
  circle.setRadius(5);
  circle.setOrigin(5,5);
  circle.setPosition(pt.x, -pt.y);
  cp.circle = circle;

  sf::Text text;
  text.setCharacterSize(15);
  text.setFillColor(sf::Color::White);
  text.setFont(*font_);
  text.setPosition(circle.getPosition());
  cp.text = text;

  if (index != control_points_.size()) {
    auto iter = control_points_.begin() + index;
    control_points_.insert(iter, cp);
    for (int i=index; i<control_points_.size(); i++) {
      control_points_[i].index = i;
      control_points_[i].text.setString(std::to_string(i));
    }
  } else {
    cp.index = index;
    cp.text.setString(std::to_string(index));
    control_points_.push_back(cp);
  }

  UpdateSplines(control_points_, interpolated_points_, track_strip_, track_bound2index_, track_width_, true/*is loop*/);
}

void Spline::UpdateSplines(std::vector<ControlPoint>& control_points,
                           std::vector<InterpolatedPoint>& interpolated_points,
                           std::vector<sf::Vertex>& triangle_strip,
                           std::map<std::pair<std::pair<float, float>,
                           std::pair<float, float>>, int>& bound2index,
                           float width,
                           bool is_loop) {
  interpolated_points.clear();
  if (control_points.size() < 4) {
    return;
  }
  float interval = 1.0f/dt_;
  int int_interval = ceil(interval); // this is somewhat inconsistent
  // std::cout << "Interval: " << int_interval << std::endl;
  for (int i=0; i<control_points.size()/dt_; i++) {
    InterpolatedPoint ip;
    Interpolate(i * dt_, is_loop, ip, control_points, interpolated_points, width);
    interpolated_points.push_back(ip);
    if (i % int_interval == 0) {
      control_points[i*dt_].outside_circle = ip.outside_circle;
      control_points[i*dt_].inside_circle = ip.inside_circle;
      // std::cout << "Control points: " << i*dt_ << " out x=" << control_points[i*dt_].outside_circle.getPosition().x << " y=" << control_points_[i*dt_].outside_circle.getPosition().y << std::endl;
    }
  }

  // Construct triangle strip
  triangle_strip.clear();
  for (int i=0; i<interpolated_points.size(); i++) {
    sf::Vertex v1, v2;
    v1.color = sf::Color::Blue;
    v2.color = sf::Color::Blue;
    v1.position = interpolated_points[i].outside_circle.getPosition();
    v2.position = interpolated_points[i].inside_circle.getPosition();
    triangle_strip.push_back(v1);
    triangle_strip.push_back(v2);
    bound2index[{{v1.position.x, v1.position.y}, {v2.position.x, v2.position.y}}] = i;
  }
  triangle_strip.push_back(triangle_strip[0]);
  triangle_strip.push_back(triangle_strip[1]);
}

// Catmull-Rom spline
void Spline::Interpolate(float t, bool is_loop, InterpolatedPoint& ip,
                         const std::vector<ControlPoint>& cps,
                         const std::vector<InterpolatedPoint>& ips,
                         float width) {
  if (cps.size() < 4) {
    return;
  }

  if (!is_loop) {
    ip.p1_index = (int)t + 1;
    ip.p2_index = ip.p1_index + 1;
    ip.p3_index = (ip.p2_index == cps.size() - 1 ? ip.p2_index : ip.p2_index + 1);
    ip.p0_index = (ip.p1_index == 0 ? 0 : ip.p1_index - 1);
  } else {
    ip.p1_index = (int)t;
    ip.p2_index = (ip.p1_index + 1) % cps.size();
    ip.p3_index = (ip.p2_index + 1) % cps.size();
    ip.p0_index = ip.p1_index >= 1 ? ip.p1_index - 1 : cps.size() - 1;
  }
  ip.t = t;
  t = t - (int)t;

  float tt = t * t;
  float ttt = tt * t;

  float mu = 0.5f;

  float q0 = -mu*ttt + 2.0f*mu*tt - mu*t;
  float q1 = (2.0f-mu)*ttt + (mu-3.0f)*tt + 1.0f;
  float q2 = (mu-2.0f)*ttt + (3.0f-2.0f*mu)*tt + mu*t;
  float q3 = mu*ttt - mu*tt;

  float tx = (cps[ip.p0_index].p.x * q0 + cps[ip.p1_index].p.x * q1 + cps[ip.p2_index].p.x * q2 + cps[ip.p3_index].p.x * q3);
  float ty = (cps[ip.p0_index].p.y * q0 + cps[ip.p1_index].p.y * q1 + cps[ip.p2_index].p.y * q2 + cps[ip.p3_index].p.y * q3);

  ip.p = {tx, ty};

  float g0 = -3.0f*mu*tt + 4.0f*mu*t - mu;
  float g1 = 3.0f*(2.0f-mu)*tt + 2.0f*(mu-3.0f)*t;
  float g2 = 3.0f*(mu-2.0f)*tt + 2.0f*(3.0f-2.0f*mu)*t + mu;
  float g3 = 3.0f*mu*tt - 2.0f*mu*t;

  float gx = (cps[ip.p0_index].p.x * g0 + cps[ip.p1_index].p.x * g1 + cps[ip.p2_index].p.x * g2 + cps[ip.p3_index].p.x * g3);
  float gy = (cps[ip.p0_index].p.y * g0 + cps[ip.p1_index].p.y * g1 + cps[ip.p2_index].p.y * g2 + cps[ip.p3_index].p.y * g3);

  ip.gradient_angle = atan2(gy, gx);

  float h0 = -6.0f*mu*t + 4.0f*mu;
  float h1 = 6.0f*(2.0f-mu)*t + 2.0f*(mu-3.0f);
  float h2 = 6.0f*(mu-2.0f)*t + 2.0f*(3.0f-2.0f*mu);
  float h3 = 6.0f*mu*t - 2.0f*mu;

  float hx = (cps[ip.p0_index].p.x * h0 + cps[ip.p1_index].p.x * h1 + cps[ip.p2_index].p.x * h2 + cps[ip.p3_index].p.x * h3);
  float hy = (cps[ip.p0_index].p.y * h0 + cps[ip.p1_index].p.y * h1 + cps[ip.p2_index].p.y * h2 + cps[ip.p3_index].p.y * h3);

  ip.curvature = abs((gx * hy - gy * hx)) / pow(pow(gx, 2) + pow(gy, 2), 1.5);
  ip.radius = pow(ip.curvature, -1);

  ip.p_outside = {width * static_cast<float>(cos(ip.gradient_angle + PI/2)) + ip.p.x, width * static_cast<float>(sin(ip.gradient_angle + PI/2)) + ip.p.y};
  ip.p_inside = {-width * static_cast<float>(cos(ip.gradient_angle + PI/2)) + ip.p.x, -width * static_cast<float>(sin(ip.gradient_angle + PI/2)) + ip.p.y};

  sf::CircleShape circle;
  circle.setFillColor(sf::Color::White);
  circle.setRadius(2);
  circle.setOrigin(2,2);
  circle.setPointCount(10);
  circle.setPosition(ip.p.x, -ip.p.y);
  ip.circle = circle;

  sf::CircleShape outside_circle;
  outside_circle.setFillColor(sf::Color::Green);
  outside_circle.setRadius(2);
  outside_circle.setOrigin(2,2);
  outside_circle.setPointCount(10);
  outside_circle.setPosition(ip.p_outside.x, -ip.p_outside.y);
  ip.outside_circle = outside_circle;

  sf::CircleShape inside_circle = outside_circle;
  inside_circle.setFillColor(sf::Color::Red);
  inside_circle.setPosition(ip.p_inside.x, -ip.p_inside.y);
  ip.inside_circle = inside_circle;

  ip.d_prev = ips.empty() ? 0 : GetDistance(ip.p, ips.back().p);
  ip.d_accumulative = ips.empty() ? 0 : ips.back().d_accumulative + ip.d_prev;

  ip.d_curv_prev = ips.empty() ? 0 : Integral(ips.back(), ip);
  ip.d_curv_accumualative = ips.empty() ? 0 : ips.back().d_curv_accumualative + ip.d_curv_prev;
}

float Spline::Integral(const InterpolatedPoint& p, const InterpolatedPoint& q) {
  float d = 0;
  float dt = dt_/100;
  for (float t=p.t; t<q.t; t+=dt) {
    float h = t - (int)t;
    float hh = h * h;
    float hhh = hh * h;
    float g0 = -3.0f * hh + 4.0f * h - 1.0f;
    float g1 = 9.0f * hh -10.0f * h;
    float g2 = -9.0f * hh + 8.0f * h + 1;
    float g3 = 3.0f * hh - 2.0f * h;

    float dx = 0.5 * (control_points_[p.p0_index].p.x * g0 + control_points_[p.p1_index].p.x * g1 + control_points_[p.p2_index].p.x * g2 + control_points_[p.p3_index].p.x * g3);
    float dy = 0.5 * (control_points_[p.p0_index].p.y * g0 + control_points_[p.p1_index].p.y * g1 + control_points_[p.p2_index].p.y * g2 + control_points_[p.p3_index].p.y * g3);
    dx *= dx;
    dy *= dy;
    float ds = sqrt(dx + dy) * dt;
    d += ds;
  }
  return d;
}

float Spline::DistanceToT(float distance) {
  float t;
  for (int i=0; i<interpolated_points_.size()-1; i++) {
    if (interpolated_points_[i].d_curv_accumualative <= distance) {
      if (interpolated_points_[i+1].d_curv_accumualative > distance ) {
        return interpolated_points_[i].t;
      }
    }
  }
  return interpolated_points_.back().t;
}

float Spline::DistanceToT2(float distance) {
  int lb = 0;
  int ub = interpolated_points_.size() - 2;
  while (lb <= ub) {
    int mid = lb + (ub - lb) / 2;
    if (interpolated_points_[mid].d_curv_accumualative <= distance && interpolated_points_[mid+1].d_curv_accumualative > distance) {
      return interpolated_points_[mid].t;
    } else if (interpolated_points_[mid].d_curv_accumualative > distance) {
      ub = mid - 1;
    } else {
      lb = mid + 1;
    }
  }
  return interpolated_points_.back().t;
}

void Spline::Draw() {
  app_->draw(&track_strip_[0], track_strip_.size(), sf::TriangleStrip);

  for (auto& cp : control_points_) {
    app_->draw(cp.circle);
    app_->draw(cp.text);
  }
  for (int i=0; i<interpolated_points_.size(); i+=25) {
    // app_->draw(interpolated_points_[i].circle);
    app_->draw(interpolated_points_[i].outside_circle);
    app_->draw(interpolated_points_[i].inside_circle);
  }
  for (const auto& rl : racing_line_control_points_) {
    app_->draw(rl.circle);
    app_->draw(rl.text);
  }
  for (int i=0; i<racing_line_interpolated_points_.size(); i+=25) {
    app_->draw(racing_line_interpolated_points_[i].circle);
  }

  // LOG(pit_control_points_.size());
  for (auto& cp : pit_control_points_) {
    app_->draw(cp.circle);
    app_->draw(cp.text);
  }
  // LOG(pit_interpolated_points_.size());
  for (int i=0; i<pit_interpolated_points_.size(); i+=25) {
    // app_->draw(interpolated_points_[i].circle);
    app_->draw(pit_interpolated_points_[i].outside_circle);
    app_->draw(pit_interpolated_points_[i].inside_circle);
  }


  app_->draw(car_);
  app_->draw(car_speed_text_);
  app_->draw(lap_time_text_);
  app_->draw(dt_text_);
}

void Spline::UpdateCar(float dt) {

  car_d_ += car_speed_ * dt;
  lap_time_ += dt;
  lap_time_text_.setString(std::to_string(lap_time_));
  if (car_d_ > interpolated_points_.back().d_curv_accumualative) {
    car_d_ -= interpolated_points_.back().d_curv_accumualative;
    laps_time_.push_back(lap_time_);
    std::cout << "Lap " << laps_time_.size() << ": " << laps_time_.back() << std::endl;
    lap_time_ = 0;
  }
  car_t_ = DistanceToT2(car_d_);
  InterpolatedPoint car_ip;
  Interpolate(car_t_, true, car_ip, racing_line_control_points_, racing_line_interpolated_points_, track_width_);
  car_.setPosition(car_ip.p.x, -car_ip.p.y);
  car_speed_text_.setString(std::to_string(car_speed_));
  car_speed_ = car_ip.radius > 100 ? 90 : car_ip.radius;
  dt_text_.setString(std::to_string(dt));
}

void Spline::MoveSelectedPoint(float dx, float dy) {
  float x = control_points_[selected_control_point_index_].p.x + dx;
  float y = control_points_[selected_control_point_index_].p.y + dy;
  control_points_[selected_control_point_index_].p.x = x;
  control_points_[selected_control_point_index_].p.y = y;
  control_points_[selected_control_point_index_].circle.setPosition(x, -y);
  control_points_[selected_control_point_index_].text.setPosition(x, -y);

  UpdateSplines(control_points_, interpolated_points_, track_strip_, track_bound2index_, track_width_, true/*is loop*/);
}

void Spline::DisplayError() {
  if (interpolated_points_.empty()) {
    return;
  }
  float e = 0;
  float min_prev = FLT_MAX;
  int min_index = -1;
  int min_p1_index = -1;
  float max_prev = 0;
  int max_index = -1;
  int max_p1_index = -1;
  for (int i=0; i<interpolated_points_.size(); i++) {
    float diff = abs(interpolated_points_[i].d_prev - interpolated_points_[i].d_curv_prev);
    if (diff > e) {
      e = diff;
    }
    if (i!=0) {
      if (interpolated_points_[i].d_curv_prev < min_prev) {
        min_prev = interpolated_points_[i].d_curv_prev;
        min_index = i;
        min_p1_index = interpolated_points_[i].p1_index;
      }
      if (interpolated_points_[i].d_curv_prev > max_prev) {
        max_prev = interpolated_points_[i].d_curv_prev;
        max_index = i;
        max_p1_index = interpolated_points_[i].p1_index;
      }
    }
  }
  LOG("Max integral error: " << e);
  LOG("Accumulative error: " << abs(interpolated_points_.back().d_accumulative - interpolated_points_.back().d_curv_accumualative));
  LOG("Total length: " << interpolated_points_.back().d_curv_accumualative);
  LOG("Max prev: " << max_prev << " at: " << max_index << " p1: " << max_p1_index << " t: " << interpolated_points_[max_p1_index].t);
  LOG("Min prev: " << min_prev << " at: " << min_index << " p1: " << min_p1_index << " t: " << interpolated_points_[min_p1_index].t);
  LOG("Num of interpolated points: " << interpolated_points_.size());

  float min_radius = FLT_MAX;
  float max_radius = 0;
  for (int i=0; i<racing_line_interpolated_points_.size(); i++) {
    if (racing_line_interpolated_points_[i].radius > max_radius) { max_radius = racing_line_interpolated_points_[i].radius; }
    if (racing_line_interpolated_points_[i].radius < min_radius) { min_radius = racing_line_interpolated_points_[i].radius; }
  }
  LOG("Max racing radius: " << max_radius);
  LOG("Min racing radius: " << min_radius);
}

void Spline::ConstructRacingLine() {
  racing_line_control_points_.clear();
  racing_line_interpolated_points_.clear();
  //
  /*
  // This is for the self generated circuit.
  std::unordered_map<int, float> ctrl_index_2_h = {{0,0.3}, {1,0.4}, {2,0.3}, {3,0.2}, {4,0.2}, {5,0.1}, {6,0.2}, {7,0.8},
                                                   {8,0}, {9,1}, {10,0.7}, {11, 0.7}, {12,1}, {13, 0.7}, {14,0.85}, {15,1}, {16,0}, {17, 0.2}};
  */
  std::unordered_map<int, float> ctrl_index_2_h;
  for (int i=0; i<control_points_.size(); i++) {
    ctrl_index_2_h[i] = 0.5f;
  }
  LOG("Control Points number: " << control_points_.size());
  float h;
  std::cout << "Control point 0: " << control_points_[0].p.x << " " << control_points_[0].p.y << " out: "
            << control_points_[0].outside_circle.getPosition().x << std::endl;
  for (int i=0; i<control_points_.size(); i++) {
    ControlPoint cp;
    cp.index = i;
    cp.circle.setFillColor(sf::Color::Yellow);
    cp.circle.setRadius(5);
    cp.circle.setOrigin(5,5);
    if (ctrl_index_2_h.count(i) > 0) {
      h = ctrl_index_2_h[i];
    } else {
      h = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    }
    cp.circle.setPosition(h*control_points_[i].outside_circle.getPosition().x + (1-h)*control_points_[i].inside_circle.getPosition().x,
                          h*control_points_[i].outside_circle.getPosition().y + (1-h)*control_points_[i].inside_circle.getPosition().y);
    cp.text.setCharacterSize(15);
    cp.text.setString(std::to_string(i));
    cp.text.setPosition(cp.circle.getPosition());
    cp.text.setFillColor(sf::Color::Red);
    cp.text.setFont(*font_);
    std::cout << "Racing line point " << i << " x: " << cp.circle.getPosition().x << " y: " << cp.circle.getPosition().y << " h = " << h << std::endl;
    cp.p = {cp.circle.getPosition().x, -cp.circle.getPosition().y};
    racing_line_control_points_.push_back(cp);
  }

  for (int i=0; i<interpolated_points_.size(); i++) {
    InterpolatedPoint ip;
    Interpolate(i * dt_, true, ip, racing_line_control_points_, racing_line_interpolated_points_, track_width_);
    racing_line_interpolated_points_.push_back(ip);
  }
}

void Spline::RegisterCar(Car* car) {
  car->SetPosition(control_points_[0].p.x + 1, control_points_[0].p.y);
  LOG("RegisterCar at [" << car->GetX() << " " << car->GetY() << "]" );
  car->SetHeading(interpolated_points_[0].gradient_angle);
  //car->SetTrack(this);
}

// better to translate all coordinates into real world coordinates
int Spline::InWhichTriangle(Point p) {
  Point p0,p1,p2;
  for (int i=0; i<track_strip_.size()-2; i++) {
    p0.x = track_strip_[i].position.x;
    p0.y = -track_strip_[i].position.y;
    p1.x = track_strip_[i+1].position.x;
    p1.y = -track_strip_[i+1].position.y;
    p2.x = track_strip_[i+2].position.x;
    p2.y = -track_strip_[i+2].position.y;
    if (IsPointInsideTriangle(p, p0, p1, p2)) {
      return i;
    }
  }
  return -1;
}

bool Spline::IsCrossFinishLine(float x, float y, float dx, float dy) {
  float a = 1/tan(interpolated_points_[0].gradient_angle);
  float b = control_points_[0].p.y - a*control_points_[0].p.x;
  return (y <= control_points_[0].p.y + track_width_) &&
         (y >= control_points_[0].p.y - track_width_) &&
         (a*x - y + b) * (a*(x+dx) - (y+dy) + b) <= 0;
}

/*bool Spline::IsPointInsideTriangle(Point p, Point p0, Point p1, Point p2) {
  float Area = 0.5 *(-p1.y*p2.x + p0.y*(-p1.x + p2.x) + p0.x*(p1.y - p2.y) + p1.x*p2.y);
  float s = 1/(2*Area)*(p0.y*p2.x - p0.x*p2.y + (p2.y - p0.y)*p.x + (p0.x - p2.x)*p.y);
  float t = 1/(2*Area)*(p0.x*p1.y - p0.y*p1.x + (p0.y - p1.y)*p.x + (p1.x - p0.x)*p.y);
  return s>0 && t>0 && 1-s-t>0;
}*/

bool Spline::IsPointInsideTriangle(Point p, Point a, Point b, Point c) {
		Point AB, AC, AP;
		AB = {b.x - a.x, b.y - a.y};
		AC = {c.x - a.x, c.y - a.y};
		AP = {p.x - a.x, p.y - a.y};
		float dot00 = AC.x * AC.x + AC.y * AC.y;
		float dot01 = AC.x * AB.x + AC.y * AB.y;
		float dot02 = AC.x * AP.x + AC.y * AP.y;
		float dot11 = AB.x * AB.x + AB.y * AB.y;
		float dot12 = AB.x * AP.x + AB.y * AP.y;
		float inverDeno = 1 / (dot00 * dot11 - dot01 * dot01);

		float u = (dot11 * dot02 - dot01 * dot12) * inverDeno;
		float v = (dot00 * dot12 - dot01 * dot02) * inverDeno;

		return (u >= 0) && (v >= 0) && (u + v <= 1);
}

