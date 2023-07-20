/**
 * @file quartic_polynomial.hpp
 * @author czj
 * @brief
 * @version 0.1
 * @date 2023-07-18
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef _QUARTIC_POLYNOMIAL_H
#define _QUARTIC_POLYNOMIAL_H

#include <Eigen/Eigen>
#include <array>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

using std::pow;

class QuarticPolynomial
{
public:
  // l = a0 + a1*s + a2*s^2 + a3*s^3 + a4*s^4
  double a0, a1, a2, a3, a4;

  QuarticPolynomial(){};

  QuarticPolynomial(double xs, double vs, double as, double ve,
                    double ae, double Ts, double Te)
  {
    Eigen::MatrixXd A(5, 5);
    A << 1, Ts, pow(Ts, 2), pow(Ts, 3), pow(Ts, 4),
        0, 1, 2 * Ts, 3 * pow(Ts, 2), 4 * pow(Ts, 3),
        0, 0, 2, 6 * Ts, 12 * pow(Ts, 2),
        0, 1, 2 * Te, 3 * pow(Te, 2), 4 * pow(Te, 3),
        0, 0, 2, 6 * Te, 12 * pow(Te, 2);

    Eigen::VectorXd B(5);
    B << xs, vs, as, ve, ae;

    // 求解AX = B
    Eigen::VectorXd coeff(5);
    coeff = A.colPivHouseholderQr().solve(B);
    a0 = coeff[0];
    a1 = coeff[1];
    a2 = coeff[2];
    a3 = coeff[3];
    a4 = coeff[4];
  };

  double calc_point(double t)
  {
    return a0 + a1 * t + a2 * std::pow(t, 2) + a3 * std::pow(t, 3) +
           a4 * std::pow(t, 4);
  };

  double calc_first_derivative(double t)
  {
    return a1 + 2 * a2 * t + 3 * a3 * std::pow(t, 2) + 4 * a4 * std::pow(t, 3);
  };

  double calc_second_derivative(double t)
  {
    return 2 * a2 + 6 * a3 * t + 12 * a4 * std::pow(t, 2);
  };

  double calc_third_derivative(double t) { return 6 * a3 + 24 * a4 * t; };
};

#endif
