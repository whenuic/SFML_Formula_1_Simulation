#ifndef CUBICSPLINE_H
#define CUBICSPLINE_H

#include <SFML/Graphics.hpp>
#include <vector>
#include "GaussianElimination.h"


// Wen Jiang's implementation of Cubic Spline solver.
// Each piece has the form of: a(x-xi)^3 + b*(x-xi)^2 + c(x-xi) + d, xi is the left point's x value
// Default boundary condition would be second derivatives are 0 at both ends.

// Examples to use this solver
// CubicSpline cs;
// std::vector<CubicSplinePoint> pts = ...
// cs.AddPoints(pts);
// SplineBoundaryCondition c1, c2;
// c1 = ...
// c2 = ...
// cs.SetBoundaryCondition(c1, c2);
// cs.Solve();
// double x = ...;
// double y = cs.Interpolate(x);

namespace cubicspline {

struct CubicSplinePoint {
  double x; // position, x
  double y; // value, y
  double d1; // first order derivative
  double d2; // second order derivative
};

struct SplinePiece {
  CubicSplinePoint p1; // start point
  CubicSplinePoint p2; // end point
  // a(x-x1)^3 + b(x-x1)^2 + c(x-x1) + d, d = p1.y
  double a;
  double b;
  double c;
};

enum SplineBoundary {
  LEFT = 0,
  RIGHT = 1,
};

enum SplineBoundaryConditionType {
  FIRST_DERIVATIVE = 0,
  SECOND_DERIVATIVE = 1,
};

struct SplineBoundaryCondition {
  // which boundary, left or right?
  SplineBoundary boundary;
  // First derivative or second derivative?
  SplineBoundaryConditionType type;
  // value
  double value;
};

class CubicSpline {
  public:
    CubicSpline();
    void AddPoints(std::vector<CubicSplinePoint> points);
    void Interpolate(std::vector<CubicSplinePoint>& points);
    void SetBoundaryCondition(SplineBoundaryCondition cond_1,
                              SplineBoundaryCondition cond2);
    void Solve();
    void Draw();
    double Interpolate(double x);

  private:
    void UpdateDisplayPoints(double dx);

    void SolveForMMatrix(std::vector<std::vector<double>>& m);

  private:

    SplineBoundaryCondition cond_1_;
    SplineBoundaryCondition cond_2_;

    // Store points in connection order
    std::vector<CubicSplinePoint> points_;

    // Stores splines
    std::vector<SplinePiece> splines_;

    // Stores Matrix A for solving for mi
    std::vector<std::vector<double>> A_;

    // Stores di for solving for mi
    std::vector<double> D_;

    // Stores the result of Mi
    std::vector<double> M_;

    // Stores param a_i, b_i, c_i;
    std::vector<double> a_i_;
    std::vector<double> b_i_;
    std::vector<double> c_i_;

    std::vector<sf::CircleShape> display_points_;
};

} // namespace cubicspline

#endif // CUBICSPLINE_H
