#include "CubicSpline.h"
#include <math.h>
#include <cassert>

namespace cubicspline {

CubicSpline::CubicSpline() {
    SplineBoundaryCondition c1, c2;
    c1.boundary = SplineBoundary::LEFT;
    c2.boundary = SplineBoundary::RIGHT;
    c1.type = SplineBoundaryConditionType::SECOND_DERIVATIVE;
    c2.type = SplineBoundaryConditionType::SECOND_DERIVATIVE;
    c1.value = 0.0;
    c2.value = 0.0;
    cond_1_ = c1;
    cond_2_ = c2;
}

void PrintMatrix(const std::vector<std::vector<double>>& m) {
  std::cout << "The matrix is:" << std::endl;
  std::cout << std::endl;
  for (auto& r : m) {
    std::cout << "  ";
    for (auto& c : r) {
      std::cout << c << " ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
}

void CubicSpline::SetBoundaryCondition(SplineBoundaryCondition c1,
                                       SplineBoundaryCondition c2) {
  cond_1_ = c1;
  cond_2_ = c2;
}

void CubicSpline::AddPoints(std::vector<CubicSplinePoint> points) {
  for (auto p : points) {
    points_.push_back(p);
  }
}


void CubicSpline::Solve() {
  int num_of_points = points_.size();

  // A * m = D. Solve for m first, then compute a, b, c for the cubic spline pieces
  // First, set the 2 to n-1 rows.
  A_ = std::vector<std::vector<double>>(points_.size(), std::vector<double>(points_.size(), 0));
  D_ = std::vector<double>(points_.size(), 0);
  M_ = std::vector<double>(points_.size(), 0);
  a_i_ = std::vector<double>(points_.size() - 1, 0);
  b_i_ = std::vector<double>(points_.size() - 1, 0);
  c_i_ = std::vector<double>(points_.size() - 1, 0);
  for (int i=0; i<points_.size()-1; i++) {
    SplinePiece spline;
    spline.p1 = points_[i];
    spline.p2 = points_[i+1];
    splines_.push_back(spline);
  }

  for (int i=1; i<num_of_points - 1; i++) {
    A_[i][i] = 2;
    A_[i][i-1] = (points_[i].x - points_[i-1].x) / (points_[i+1].x - points_[i-1].x);
    A_[i][i+1] = (points_[i+1].x - points_[i].x) / (points_[i+1].x - points_[i-1].x);

    D_[i] = 6 / (points_[i+1].x - points_[i-1].x) * ((points_[i+1].y - points_[i].y)/(points_[i+1].x - points_[i].x) -(points_[i].y - points_[i-1].y)/(points_[i].x - points_[i-1].x) );
  }


  // Based on the boundary condition, set the 1st and nth row.
  if (cond_1_.boundary == SplineBoundary::LEFT) {
    if (cond_1_.type == SplineBoundaryConditionType::FIRST_DERIVATIVE) {
      A_[0][0] = 2;
      A_[0][1] = 1;
      D_[0] = 6 * ((points_[1].y - points_[0].y) / (points_[1].x - points_[0].x) - cond_1_.value) / (points_[1].x - points_[0].x);
    } else {
      A_[0][0] = 1;
      D_[0] = cond_1_.value;
    }
  } else {
    if (cond_1_.type == SplineBoundaryConditionType::FIRST_DERIVATIVE) {
      A_[0][num_of_points-2] = 1;
      A_[0][num_of_points-1] = 2;
      D_[0] = 6 * (cond_1_.value - (points_[num_of_points-1].y - points_[num_of_points-2].y)/(points_[num_of_points-1].x - points_[num_of_points-2].x)) / (points_[num_of_points-1].x - points_[num_of_points-2].x);
    } else {
      A_[0][num_of_points-1] = 1;
      D_[0] = cond_1_.value;
    }
  }

  if (cond_2_.boundary == SplineBoundary::LEFT) {
    if (cond_2_.type == SplineBoundaryConditionType::FIRST_DERIVATIVE) {
      A_[num_of_points][0] = 2;
      A_[num_of_points][1] = 1;
      D_[num_of_points] = 6 * ((points_[1].y - points_[0].y) / (points_[1].x - points_[0].x) - cond_2_.value) / (points_[1].x - points_[0].x);
    } else {
      A_[num_of_points-1][0] = 1;
      D_[num_of_points-1] = cond_2_.value;
    }
  } else {
    if (cond_2_.type == SplineBoundaryConditionType::FIRST_DERIVATIVE) {
      A_[num_of_points-1][num_of_points-2] = 1;
      A_[num_of_points-1][num_of_points-1] = 2;
      D_[num_of_points-1] = 6 * (cond_2_.value - (points_[num_of_points-1].y - points_[num_of_points-2].y)/(points_[num_of_points-1].x - points_[num_of_points-2].x)) / (points_[num_of_points-1].x - points_[num_of_points-2].x);
    } else {
      A_[num_of_points-1][num_of_points-1] = 1;
      D_[num_of_points-1] = cond_2_.value;
    }
  }

  bool first_line_all_zero = true;
  bool last_line_all_zero = true;
  for (int i=0; i<num_of_points; i++) {
    if (A_[0][i] != 0) {
      first_line_all_zero = false;
    }
    if (A_[num_of_points-1][i] != 0) {
      last_line_all_zero = false;
    }
  }
  assert(!first_line_all_zero && !last_line_all_zero);

  std::vector<std::vector<double>> combined_matrix = A_;
  for (int i=0; i<A_.size(); i++) {
    combined_matrix[i].push_back(D_[i]);
  }

  SolveForMMatrix(combined_matrix);
  for (int i=0; i<A_.size(); i++) {
    M_[i] = combined_matrix[i][num_of_points];
  }

  for (auto& m : M_) {
    std::cout << m << std::endl;
  }

  std::cout << "M solved." << std::endl;

  // compute a_i, b_i, and c_i;
  for (int i=0; i<M_.size()-1; i++) {
    double h = points_[i+1].x - points_[i].x;
    a_i_[i] = (M_[i+1] - M_[i]) / 6 / h;
    b_i_[i] = M_[i] / 2;
    c_i_[i] = (points_[i+1].y - points_[i].y) / h - (2 * M_[i] + M_[i+1]) / 6 * h;
    splines_[i].a = a_i_[i];
    splines_[i].b = b_i_[i];
    splines_[i].c = c_i_[i];
  }

  UpdateDisplayPoints(0.05);
}

void CubicSpline::UpdateDisplayPoints(double dx) {
  display_points_.clear();
  double x0 = splines_[0].p1.x;
  for (int i=0; i<splines_.size(); i++) {
    bool direction = splines_[i].p1.x < splines_[i].p2.x;
    while (direction ? (x0 < splines_[i].p2.x) : (x0 > splines_[i].p2.x)) {
      sf::CircleShape circle;
      circle.setFillColor(sf::Color::Blue);
      circle.setRadius(2);
      circle.setPointCount(10);
      circle.setOrigin(2,2);
      circle.setPosition(x0, splines_[i].a * pow((x0-splines_[i].p1.x), 3) + splines_[i].b * pow((x0-splines_[i].p1.x), 2) + splines_[i].c * (x0-splines_[i].p1.x) + splines_[i].p1.y);
      display_points_.push_back(circle);
      x0 += (direction ? dx : -dx);
    }
  }
}

void CubicSpline::SolveForMMatrix(std::vector<std::vector<double>>& m) {
  if (m.empty() || m[0].empty()) {
    std::cerr << "Input matrix is empty!" << std::endl;
  }
  if (m.size() + 1 != m[0].size()) {
    std::cerr << "Input matrix has wrong dimension!" << std::endl;
  }

  // Get dimension of the matrix
  int num_of_row = m.size();
  int num_of_col = m[0].size();
  std::cout << "Dimension of the A matrix is: " << num_of_row << "x" << num_of_col << std::endl;
  PrintMatrix(m);

  // find out the lowest row that has a leading non-zero
  int least_non_zero_leading_row_index = 0;
  for (int i=0; i<num_of_row; i++) {
    if (m[i][0] != 0) {
      break;
    } else {
      least_non_zero_leading_row_index++;
    }
  }
  if (least_non_zero_leading_row_index + 1 > num_of_row) {
    std::cerr << "Input matrix does not have full rank." << std::endl;
  }
  if (least_non_zero_leading_row_index != 0) {
    for (int i=0; i<num_of_col; i++) {
      m[0][i] = m[least_non_zero_leading_row_index][i] + m[0][i];
    }
  } else {
    double k = m[0][0];
    for (int i=0; i<=num_of_col; i++) {
      m[0][i] /= k;
    }
  }

  PrintMatrix(m);

  // Elimination
  std::cout << "Starting elimination:" << std::endl;
  for (int i=0; i<num_of_row - 1; i++) {
    std::cout << "i = " << i << std::endl;
    // make leading element 1
    double gain = m[i][i];
    for (int j=0; j<num_of_col; j++) {
      m[i][j] /= gain;
    }

    // make the rows below zero
    for (int j=i+1; j<num_of_row; j++) {
      gain = -m[j][i];
      for (int k=0; k<num_of_col; k++) {
        m[j][k] = m[j][k] + gain * m[i][k];
      }
    }
    PrintMatrix(m);
  }

  // Back substitution
  std::cout << "Starting back substitution:" << std::endl;
  for (int i=num_of_row-1; i>=1; i--) {
    std::cout << "i = " << i << std::endl;
    // make leading element 1
    double gain = m[i][i];
    for (int j=i; j<num_of_col; j++) {
      m[i][j] /= gain;
    }
    // PrintMatrix(m);

    // make the rows above zero
    for (int j=i-1; j>=0; j--) {
      gain = -m[j][i];
      m[j][i] = 0;
      m[j][num_of_col-1] = m[j][num_of_col-1] + gain * m[i][num_of_col-1];
    }
    // PrintMatrix(m);
  }
}

void CubicSpline::Interpolate(std::vector<CubicSplinePoint>& points) {
  for (auto& p : points) {
    // determine which spline function it should use
    for (auto& sp : splines_) {
      if (p.x >= sp.p1.x && p.x <= sp.p2.x) {
        p.y = sp.a * pow((p.x-sp.p1.x), 3) + sp.b * pow((p.x-sp.p1.x), 2) + sp.c * (p.x-sp.p1.x) + sp.p1.y;
      }
    }
  }
}

double CubicSpline::Interpolate(double x) {
  bool in_interpolation_range = false;
  for (auto& sp : splines_) {
    if (x >= sp.p1.x && x <= sp.p2.x) {
      return sp.a * pow((x-sp.p1.x), 3) + sp.b * pow((x-sp.p1.x), 2) + sp.c * (x-sp.p1.x) + sp.p1.y;
    }
  }
  assert(in_interpolation_range == true);
  std::cout << "Input: " << x << " is out of interpolation range." << std::endl;
}

} // namespace cubicspline

