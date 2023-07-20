/**
 * @file quintic_polynomial.hpp
 * @author czj
 * @brief
 * @version 0.1
 * @date 2023-07-18
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef _QUINTIC_POLYNOMIAL_H
#define _QUINTIC_POLYNOMIAL_H

#include <Eigen/Eigen>
#include <array>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

using std::pow;

// 五次多项式
class QuinticPolynomial
{
public:
  // l = a0 + a1*s + a2*s^2 + a3*s^3 + a4*s^4 + a5*s^5
  double a0, a1, a2, a3, a4, a5;

  QuinticPolynomial(){};

  // polynomial parameters
  QuinticPolynomial(double xs, double vs, double as, double xe, double ve,
                    double ae, double Ts, double Te)
  {
    Eigen::MatrixXd A(6, 6);
    A << 1, Ts, pow(Ts, 2), pow(Ts, 3), pow(Ts, 4), pow(Ts, 5),
        0, 1, 2 * Ts, 3 * pow(Ts, 2), 4 * pow(Ts, 3), 5 * pow(Ts, 4),
        0, 0, 2, 6 * Ts, 12 * pow(Ts, 2), 20 * pow(Ts, 3),
        1, Te, pow(Te, 2), pow(Te, 3), pow(Te, 4), pow(Te, 5),
        0, 1, 2 * Te, 3 * pow(Te, 2), 4 * pow(Te, 3), 5 * pow(Te, 4),
        0, 0, 2, 6 * Te, 12 * pow(Te, 2), 20 * pow(Te, 3);

    Eigen::VectorXd B(6);
    B << xs, vs, as, xe, ve, ae;

    // 求解AX = B
    Eigen::VectorXd coeff(6);
    coeff = A.colPivHouseholderQr().solve(B);
    a0 = coeff[0];
    a1 = coeff[1];
    a2 = coeff[2];
    a3 = coeff[3];
    a4 = coeff[4];
    a5 = coeff[5];
    // std::cout << "create 五阶 successfully" << std::endl;

  };

  double calc_point(double t)
  {
    return a0 + a1 * t + a2 * std::pow(t, 2) + a3 * std::pow(t, 3) +
           a4 * std::pow(t, 4) + a5 * std::pow(t, 5);
  };

  double calc_first_derivative(double t)
  {
    return a1 + 2 * a2 * t + 3 * a3 * std::pow(t, 2) + 4 * a4 * std::pow(t, 3) +
           4 * a5 * std::pow(t, 4);
  };

  double calc_second_derivative(double t)
  {
    return 2 * a2 + 6 * a3 * t + 12 * a4 * std::pow(t, 2) +
           20 * a5 * std::pow(t, 3);
  };

  double calc_third_derivative(double t)
  {
    return 6 * a3 + 24 * a4 * t + 60 * a5 * std::pow(t, 2);
  };
};

#endif