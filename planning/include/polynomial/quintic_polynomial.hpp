#ifndef _QUINTIC_POLYNOMIAL_H
#define _QUINTIC_POLYNOMIAL_H

#include <Eigen/Eigen>
#include <array>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>


class QuinticPolynomial {
 public:
  // current parameter at t=0
  double xs;
  double vxs;
  double axs;

  // parameters at target t=t_j
  double xe;
  double vxe;
  double axe;

  // function parameters
  double a0, a1, a2, a3, a4, a5;

  QuinticPolynomial(){};

  // polynomial parameters
  QuinticPolynomial(double xs_, double vxs_, double axs_, double xe_, double vxe_,
                    double axe_, double T)
      : xs(xs_),
        vxs(vxs_),
        axs(axs_),
        xe(xe_),
        vxe(vxe_),
        axe(axe_),
        a0(xs_),
        a1(vxs_),
        a2(axs_ / 2.0) {
    Eigen::Matrix3f A;
    A << std::pow(T, 3), std::pow(T, 4), std::pow(T, 5), 3 * std::pow(T, 2),
        4 * std::pow(T, 3), 5 * std::pow(T, 4), 6 * T, 12 * std::pow(T, 2),
        20 * std::pow(T, 3);
    Eigen::Vector3f B;
    B << xe - a0 - a1 * T - a2 * std::pow(T, 2), vxe - a1 - 2 * a2 * T,
        axe - 2 * a2;

    Eigen::Vector3f c_eigen = A.colPivHouseholderQr().solve(B);
    a3 = c_eigen[0];
    a4 = c_eigen[1];
    a5 = c_eigen[2];
  };

  double calc_point(double t) {
    return a0 + a1 * t + a2 * std::pow(t, 2) + a3 * std::pow(t, 3) +
           a4 * std::pow(t, 4) + a5 * std::pow(t, 5);
  };

  double calc_first_derivative(double t) {
    return a1 + 2 * a2 * t + 3 * a3 * std::pow(t, 2) + 4 * a4 * std::pow(t, 3) +
           a5 * std::pow(t, 4);
  };

  double calc_second_derivative(double t) {
    return 2 * a2 + 6 * a3 * t + 12 * a4 * std::pow(t, 2) +
           20 * a5 * std::pow(t, 3);
  };

  double calc_third_derivative(double t) {
    return 6 * a3 + 24 * a4 * t + 60 * a5 * std::pow(t, 2);
  };
};

#endif
