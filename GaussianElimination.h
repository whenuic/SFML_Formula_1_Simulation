#ifndef GAUSSIANELIMINATION_H
#define GAUSSIANELIMINATION_H

#include <vector>
#include <iostream>

class GaussianElimination {
    public:
        GaussianElimination() {};
        // Solve Ax=b, where A is n-by-n.
        // The outer input should be n, representing n rows, the inner vector is n+1, the last element is is b.
        std::vector<double> Solve(std::vector<std::vector<double>> m) {
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
          std::vector<double> x(num_of_row, 0);
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

          for (int i=0; i<num_of_row; i++) {
            x[i] = m[i][num_of_col-1];
          }
          return x;
        }

    private:
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
};

#endif // GAUSSIANELIMINATION_H
