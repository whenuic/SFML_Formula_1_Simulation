#include "WenSpline.h"

namespace wenspline {

  WenSpline::WenSpline() {
    //ctor
  }

  WenSpline::~WenSpline() {
    //dtor
  }

  void WenSpline::AddInstructions(std::vector<WenSplineParam> instructions) {
    instructions_ = instructions;
  }

  void WenSpline::SetInterval(float dt) {
    dt_ = dt;
  }

  void WenSpline::SetInitialCondition(WenSplineInitialCondition initial_condition) {
    initial_condition_ = initial_condition;
  }

  float WenSpline::ComputeSplineLength(const WenSplineParam & spline_param) {
    float length;
    switch (spline_param.type) {
    case WenSplineType::LINE: {
      // Line param, length([0])
      length = spline_param.param[0];
      break;
    }
    case WenSplineType::ARC: {
      // Arc param, radius([0]), sweep_angle([1])
      length = spline_param.param[0] * spline_param.param[1];
      break;
    }
    case WenSplineType::SPIRAL: {
      // Spiral param, a([0]), b([1]), sweep_angle([2]), is_relaxed([3])
      float ro1 = spline_param.param[0];
      float ro2 = spline_param.param[0] * exp(spline_param.param[1] * spline_param.param[3] * spline_param.param[2]);
      float phi = atan(1 / spline_param.param[1]); // rotate tangent(direction of the spiral in theta increasing direction) clockwise phi degree is the direction from origin to the point on the spiral
      length = abs(ro2-ro1)/cos(phi);
      break;
    }
    default:
      assert(false);
      break;
    }
    return length;
  }

  void WenSpline::Interpolate() {
    assert(dt_ >= 0);
    int spline_piece_index = 0;
    int interpolated_point_index = 0;
    WenSplineInterpolatedPoint p;
    WenSplineInfo sp_info;
    for (auto& ins : instructions_) {
      float residual_from_last_spline = GetResidualFromLastSplinePiece();
      float delta = dt_ - residual_from_last_spline;
      float length = ComputeSplineLength(ins);
      // n is how many interpolated points this piece contains, n-1 is how many
      // equal length pieces this spline piece contains.
      int n = (length - delta) / dt_ + 1;

      // Set spline info
      sp_info.length = length;
      sp_info.accumulative_length = GetAccumulativeLengthFromLastSplinePiece();
      sp_info.delta = delta;
      sp_info.residual = length - (n-1) * dt_  - delta;
      sp_info.number_of_interpolated_points = n;
      assert(sp_info.residual >= 0);
      assert(sp_info.delta >= 0);

      switch (ins.type) {
      case WenSplineType::LINE:
        {
          float angle = ins.mode == WenSplineConnectingMode::CONTINUOUS ? (spline_info_.empty() ? initial_condition_.initial_angle : spline_info_.back().end_angle) : ins.init.initial_angle;
          float x = ins.mode == WenSplineConnectingMode::CONTINUOUS ? (spline_info_.empty() ? initial_condition_.p.x : spline_info_.back().p1.x) : ins.init.p.x;
          float y = ins.mode == WenSplineConnectingMode::CONTINUOUS ? (spline_info_.empty() ? initial_condition_.p.y : spline_info_.back().p1.y) : ins.init.p.y;
          for (int i=0; i<n; i++) {
            p.p.x = x + delta * cos(angle) + i * dt_ * cos(angle);
            p.p.y = y + delta * sin(angle) + i * dt_ * sin(angle);
            p.spline_piece_index = spline_piece_index;
            p.gradient_angle = angle;
            p.index = interpolated_point_index;
            interpolated_point_index++;
            interpolated_points_.push_back(p);
          }
          // push spline info
          sp_info.type = WenSplineType::LINE;
          sp_info.init_angle = utils::ToNormalizedAngle(angle);
          sp_info.end_angle = utils::ToNormalizedAngle(angle);
          sp_info.p0 = {x, y};
          sp_info.p1 = {x + length * cos(angle), y + length * sin(angle)};
          spline_info_.push_back(sp_info);
        }
        break;
      case WenSplineType::ARC:
        {
          float angle = ins.mode == WenSplineConnectingMode::CONTINUOUS ? (spline_info_.empty() ? initial_condition_.initial_angle : spline_info_.back().end_angle) : ins.init.initial_angle;
          float x = ins.mode == WenSplineConnectingMode::CONTINUOUS ? (spline_info_.empty() ? initial_condition_.p.x : spline_info_.back().p1.x) : ins.init.p.x;
          float y = ins.mode == WenSplineConnectingMode::CONTINUOUS ? (spline_info_.empty() ? initial_condition_.p.y : spline_info_.back().p1.y) : ins.init.p.y;
          float radius = ins.param[0];
          float sweep_angle = ins.param[1];

          // find center of the arc
          float angle_to_center, angle_from_center_to_end_point;
          if (ins.direction == WenSplineBendDirection::LEFT) {
            angle_to_center = angle + PI/2;
            angle_from_center_to_end_point = angle_to_center + PI + sweep_angle;
          } else if (ins.direction == WenSplineBendDirection::RIGHT) {
            angle_to_center = angle - PI/2;
            angle_from_center_to_end_point = angle_to_center + PI - sweep_angle;
          } else {
            assert(false);
          }
          float center_x = x + radius * cos(angle_to_center);
          float center_y = y + radius * sin(angle_to_center);

          // compute starting angle
          float delta_angle = delta / radius;
          float dt_angle = dt_ / radius;

          // construct interpolated point
          for (int i=0; i<n; i++) {
            float center_to_interpolated_angle;
            if (ins.direction == WenSplineBendDirection::LEFT) {
              center_to_interpolated_angle = angle_to_center + PI + delta_angle + i * dt_angle;
              p.gradient_angle = center_to_interpolated_angle + PI/2;
            } else if (ins.direction == WenSplineBendDirection::RIGHT) {
              center_to_interpolated_angle = angle_to_center + PI - delta_angle - i * dt_angle;
              p.gradient_angle = center_to_interpolated_angle - PI/2;
            } else {
              assert(false);
            }
            p.p.x = center_x + radius * cos(center_to_interpolated_angle);
            p.p.y = center_y + radius * sin(center_to_interpolated_angle);
            p.spline_piece_index = spline_piece_index;
            p.index = interpolated_point_index;
            interpolated_point_index++;
            interpolated_points_.push_back(p);
          }

          // push spline_info
          sp_info.type = WenSplineType::ARC;
          sp_info.init_angle = utils::ToNormalizedAngle(angle);
          if (ins.direction == WenSplineBendDirection::LEFT) {
            sp_info.end_angle = utils::ToNormalizedAngle(angle + sweep_angle);
          } else if (ins.direction == WenSplineBendDirection::RIGHT) {
            sp_info.end_angle = utils::ToNormalizedAngle(angle - sweep_angle);
          } else {
            assert(false);
          }
          sp_info.p0 = {x, y};
          sp_info.p1 = {center_x + radius * cos(angle_from_center_to_end_point), center_y + radius * sin(angle_from_center_to_end_point)};
          spline_info_.push_back(sp_info);
        }
        break;
      case WenSplineType::SPIRAL:
        {
          float angle = ins.mode == WenSplineConnectingMode::CONTINUOUS ? (spline_info_.empty() ? initial_condition_.initial_angle : spline_info_.back().end_angle) : ins.init.initial_angle;
          WenSplineBendDirection bend_direction = ins.direction;
          float x = ins.mode == WenSplineConnectingMode::CONTINUOUS ? (spline_info_.empty() ? initial_condition_.p.x : spline_info_.back().p1.x) : ins.init.p.x;
          float y = ins.mode == WenSplineConnectingMode::CONTINUOUS ? (spline_info_.empty() ? initial_condition_.p.y : spline_info_.back().p1.y) : ins.init.p.y;
          float a = ins.param[0];
          float b = ins.param[1];
          float sweep_angle = ins.param[2];

          float ro1 = a;
          float ro2 = a*exp(b*ins.param[3]*sweep_angle);
          float phi = atan(1/b); // rotate tangent(direction of the spiral in theta increasing direction) clockwise phi degree is the direction from origin to the point on the spiral

          // find origin of the spiral
          float angle_to_origin, angle_from_origin_to_end_point;
          if (bend_direction == WenSplineBendDirection::LEFT) {
            angle_to_origin = angle - (ins.param[3] == 1 ? phi - PI : -phi);
            angle_from_origin_to_end_point = angle_to_origin + PI + sweep_angle;
          } else if (bend_direction == WenSplineBendDirection::RIGHT) {
            angle_to_origin = angle + (ins.param[3] == 1 ? phi - PI : -phi);
            angle_from_origin_to_end_point = angle_to_origin + PI - sweep_angle;
          } else {
            assert(false);
          }
          float origin_x = x + a * cos(angle_to_origin);
          float origin_y = y + a * sin(angle_to_origin);

          // construct interpolated point
          for (int i=0; i<n; i++) {
            float origin_to_interpolated_angle, swept_angle;
            swept_angle = log(ins.param[3] * (delta + i*dt_) * cos(phi) / a + 1) / b;
            if (bend_direction == WenSplineBendDirection::LEFT) {
              // We cannot flip the sign of the swept_angle in the previous calculation because that variable is used to compute r(radius) several lines down.
              origin_to_interpolated_angle = angle_to_origin + PI + ins.param[3] * swept_angle;
              p.gradient_angle = origin_to_interpolated_angle + PI/2;
            } else if (bend_direction == WenSplineBendDirection::RIGHT) {
              origin_to_interpolated_angle = angle_to_origin + PI - ins.param[3] * swept_angle;
              p.gradient_angle = origin_to_interpolated_angle - PI/2;
            } else {
              assert(false);
            }
            float r = a * exp(b * swept_angle);
            p.p.x = origin_x + r * cos(origin_to_interpolated_angle);
            p.p.y = origin_y + r * sin(origin_to_interpolated_angle);
            p.spline_piece_index = spline_piece_index;
            p.index = interpolated_point_index;
            interpolated_point_index++;
            interpolated_points_.push_back(p);
          }

          // push spline_info
          sp_info.type = WenSplineType::SPIRAL;
          sp_info.init_angle = utils::ToNormalizedAngle(angle);
          if (bend_direction == WenSplineBendDirection::LEFT) {
            sp_info.end_angle = utils::ToNormalizedAngle(angle + sweep_angle);
          } else if (bend_direction == WenSplineBendDirection::RIGHT) {
            sp_info.end_angle = utils::ToNormalizedAngle(angle - sweep_angle);
          } else {
            assert(false);
          }
          sp_info.p0 = {x, y};
          sp_info.p1 = {origin_x + a*exp(b*sweep_angle) * cos(angle_from_origin_to_end_point), origin_y + a*exp(b*sweep_angle) * sin(angle_from_origin_to_end_point)};
          spline_info_.push_back(sp_info);
        }
        break;
      } // end switch
      spline_piece_index++;
    } // end for
  } // end Interpolate() function

  float WenSpline::GetResidualFromLastSplinePiece() {
    return spline_info_.empty() ? dt_ : spline_info_.back().residual;
  }

  float WenSpline::GetAccumulativeLengthFromLastSplinePiece() {
    return spline_info_.empty() ? 0.0f :
           spline_info_.back().accumulative_length + spline_info_.back().length;
  }

  void WenSpline::GetInterpolatedPoints(std::vector<WenSplineInterpolatedPoint>& points) {
    points = interpolated_points_;
  }

  void WenSpline::GetSplineInfo(std::vector<WenSplineInfo>& spline_info) {
    spline_info = spline_info_;
  }

  int WenSpline::GetInterpolatedPointsSize() {
    return interpolated_points_.size();
  }

  int WenSpline::GetSplineInfoSize() {
    return spline_info_.size();
  }

} // namespace wenspline
