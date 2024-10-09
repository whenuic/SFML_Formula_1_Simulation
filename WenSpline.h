#ifndef WENSPLINE_H
#define WENSPLINE_H

#include <vector>
#include <cassert>
#include <cmath>

#include "Debug.h"
#include "CommonDataStructure.h"

// This spline is the Wen Jiang's implementation of a spline that contains
// straight lines (y=ax+b) partial circle arches (x=acos(theta), y=asin(theta))
// and logarithmic spirals (r=ae^(b*theta)).

// The whole closed curve is consist of the above three types of spline.
// User is responsible for computing the params to make sure the circuit is closed loop.
// For pit, it doesn't have to be closed loop.

// Line spline is defined as [angle, x, y, length] (4 dof)
// Arc spline is defined as [angle, x, y, direction, radius, sweep_angle] (6 dof)
// Spiral spline is defined as [angle, x, y, direction, a, b] (6 dof)

namespace wenspline {

enum WenSplineType {
  UNDEFINED = -1,
  LINE = 0, // straight line
  ARC = 1, // partial circle
  SPIRAL = 2, // logarithmic spirals
};

enum WenSplineBendDirection {
  LEFT = -1,
  DEFAULT = 0,
  RIGHT = 1,
};

enum WenSplineConnectingMode {
  CONTINUOUS = 0,
  USING_OWN_INIT_CONDITION = 1,
};

// Initial condition includes 1) point and 2) shooting angle
struct WenSplineInitialCondition {
  float initial_angle = 0; // in radian
  Point p;
};

struct WenSplineParam {
  WenSplineType type;
  WenSplineInitialCondition init;
  WenSplineConnectingMode mode = WenSplineConnectingMode::CONTINUOUS;
  WenSplineBendDirection direction = WenSplineBendDirection::DEFAULT;
  // for line, {length}
  // for arc, {radius, sweep_angle}
  // logrithmatic spiral, r=ae^(b*theta).
  // for spiral, {a(magnitude, how far the curve intersects theta=0 axis),
  //              b(the angle between tangent and the line to the origin is 1/cot(b), when b=0, it becomes a circle),
  //              sweep_angle(angle swept, controls when the spiral ends),
  //              is_relaxed(1-relaxed, -1-not relaxed)}.
  std::vector<float> param;
  WenSplineParam(WenSplineType t, WenSplineInitialCondition init_cond, WenSplineConnectingMode connection_mode, WenSplineBendDirection d, std::vector<float> p) {
      type = t;
      init = init_cond;
      mode = connection_mode;
      direction = d;
      param = p;
  }
};

struct WenSplineInfo {
  WenSplineType type = WenSplineType::UNDEFINED;
  // how long this spline piece is
  float length = 0;
  // circuit length at the beginning of this spline piece
  float accumulative_length = 0;
  // distance between the starting point and the first interpolation point of
  // this piece due to the residual from the last spline piece
  float delta = 0;
  // residual is equal to the distance between the last interpolation point of this spline piece and the ending point of the spline
  float residual = 0;
  // initial spline tangent direction
  float init_angle = 0;
  // ending spline tangent direction
  float end_angle = 0;

  // spline piece starting point
  Point p0;
  // spline piece ending point
  Point p1;

  // number of interpolated points this spline has
  int number_of_interpolated_points = 0;
};

// This is the computation of splines by the instructions<WenSplineParam>.
class WenSpline {
  public:
    WenSpline();
    virtual ~WenSpline();

    // Load instructions to create spline pieces
    void AddInstructions(std::vector<WenSplineParam> instructions);
    // Set circuit distance between two consecutive interpolation points along circuit center line
    void SetInterval(float dt);
    // Calculate interpolated points
    void Interpolate();
    // Copy interpolation calculation results to the input vector
    void GetInterpolatedPoints(std::vector<WenSplineInterpolatedPoint>& points);
    // Copy spline_info results to the input vector
    void GetSplineInfo(std::vector<WenSplineInfo>& spline_info);
    // Set initial condition for the 1st piece of splines
    void SetInitialCondition(WenSplineInitialCondition initial_condition);

    // Return interpolation points container size. The caller will prepare the container to hold the interpolated points.
    int GetInterpolatedPointsSize();
    // Return spline info container size. The caller will prepare the container to hold the spline_info.
    int GetSplineInfoSize();

  private:
    // The circuit distance between the last interpolation point and the actual end point of the spline piece
    float GetResidualFromLastSplinePiece();
    // The circuit accumulative distance at the end point of the last spline piece
    float GetAccumulativeLengthFromLastSplinePiece();
    // Compute the circuit length of the spline piece
    float ComputeSplineLength(const WenSplineParam& spline_param);

  private:

    // interpolated points interval, using curve integral length rather than points distance
    float dt_ = 0;
    // This is the direction of the first instruction should start to draw the circuit.
    WenSplineInitialCondition initial_condition_;

    // Instruction container
    std::vector<WenSplineParam> instructions_;

    // Interpolation points container
    std::vector<WenSplineInterpolatedPoint> interpolated_points_;

    // Spine pieces info
    std::vector<WenSplineInfo> spline_info_;

};

} // namespace wenspline

#endif // WENSPLINE_H
