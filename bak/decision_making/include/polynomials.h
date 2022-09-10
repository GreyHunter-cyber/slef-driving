/**
 * @brief Cubic Spline, Qintic and Quartic polynomials definition
 * @author Renjie Zhu
 * @date 2019-08-10
 */
#ifndef _POLYNOMIALS_H_
#define _POLYNOMIALS_H_

#include <iostream>
#include <math.h>
#include <string>
#include "spline/spline.h"

#define PI 3.14159265

#define dist(x0, y0, x1, y1) sqrt((x1-x0)*(x1-x0)+(y1-y0)*(y1-y0))

/**
 * @brief 2d cubic spline
 */
class Spline2D
{
public:
  Spline2D();
  Spline2D(std::vector<double>& points_x, std::vector<double>& points_y);
  Spline2D(std::vector<double>& points_x, std::vector<double>& points_y, std::vector<double>& points_s);
  virtual ~Spline2D();

public:
  /**
   * @brief from miles s to x, y position
   */
  std::vector<double> calc_pos(double s);
  /**
   * @brief from miles s to heading angle yaw
   */
  double calc_yaw(double s);
  /**
   * @brief from miles s to curvature ks
   */
  double calc_ks(double s);

private:
  tk::spline spline_x;
  tk::spline spline_y;

  /**
   * @brief from positon sequence to miles sequence
   */
  std::vector<double> calc_s(std::vector<double>& points_x, std::vector<double>& points_y);

};

/**
 * @brief 5次多项式, 多项式求解与微分计算
 */
class QuinticPolynomial
{
public:
  QuinticPolynomial();
  virtual ~QuinticPolynomial();

public:
  /**
   * @brief 输入初始状态，与目标状态，以及时间间隔
   * @param ss包含xs(位置), vs(速度), as(加速度)
   *        es包含xe(位置), ve(速度), ae(加速度)
   */
  void init(double* ss, double* es, double T);
  /**
   * @brief 计算位置状态
   */
  double calc_point(double t);
  /**
   * @brief 计算一阶导数
   */
  double calc_first_derivative(double t);
  /**
   * @brief 计算二阶导数
   */
  double calc_second_derivative(double t);
  /**
   * @brief 计算三阶导数
   */
  double calc_third_derivative(double t);

private:
  double a0, a1, a2, a3, a4, a5;
};

/**
 * @brief 4次多项式
 */
class QuarticPolynomial
{
public:
  QuarticPolynomial();
  virtual ~QuarticPolynomial();

public:
  /**
   * @brief 输入初始状态，与目标状态，以及时间间隔
   * @param ss包含xs(位置), vs(速度), as(加速度)
   *        ve(速度), ae(加速度)
   */
  void init(double* ss, double* es, double T);
  /**
   * @brief 计算位置状态
   */
  double calc_point(double t);
  /**
   * @brief 计算一阶导数
   */
  double calc_first_derivative(double t);
  /**
   * @brief 计算二阶导数
   */
  double calc_second_derivative(double t);
  /**
   * @brief 计算三阶导数
   */
  double calc_third_derivative(double t);

private:
  double a0, a1, a2, a3, a4;
};

#endif