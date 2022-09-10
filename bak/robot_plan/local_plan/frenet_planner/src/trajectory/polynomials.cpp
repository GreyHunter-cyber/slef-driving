/**
 * @brief Qintic and Quartic polynomials definition
 * @author Renjie Zhu
 * @date 2019-08-10
 */

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include "polynomials.h"
Spline2D::Spline2D(){

}
Spline2D::Spline2D(std::vector<double>& points_x, std::vector<double>& points_y){
  std::vector<double> points_s = calc_s(points_x, points_y);
  spline_x.set_points(points_s, points_x);
  spline_y.set_points(points_s, points_y);
}

Spline2D::Spline2D(std::vector<double>& points_x, std::vector<double>& points_y, std::vector<double>& points_s){
  if(points_s.size() == 0){
    points_s = calc_s(points_x, points_y);
  }
  spline_x.set_points(points_s, points_x);
  spline_y.set_points(points_s, points_y);
}

Spline2D::~Spline2D(){

}

std::vector<double> Spline2D::calc_s(std::vector<double>& points_x, std::vector<double>& points_y)
{
  int sz = points_x.size();
  std::vector<double> points_s(sz);
  double mileage = 0;
  points_s[0] = mileage;
  for(int i = 1; i< sz; ++i)
  {
    mileage+= dist(points_x[i-1], points_y[i-1], points_x[i], points_y[i]);
    points_s[i] = mileage;
  }
  return points_s;
}

std::vector<double> Spline2D::calc_pos(double s){
  return {spline_x(s), spline_y(s)};
}

double Spline2D::calc_yaw(double s){
  double dx = spline_x.first_deriv_s(s);
  double dy = spline_y.first_deriv_s(s);
  return atan2(dy, dx);
}

double Spline2D::calc_ks(double s){
  double dx = spline_x.first_deriv_s(s);
  double ddx = spline_x.second_deriv_s(s);
  double dy = spline_y.first_deriv_s(s);
  double ddy = spline_y.second_deriv_s(s);
  double ks = (ddy * dx - ddx * dy) / (dx*dx + dy*dy);
  return ks;  
}

QuinticPolynomial::QuinticPolynomial(){
    a0 = 0; a1 = 0; a2 = 0;
    a3 = 0; a4 = 0; a5 = 0;
}
QuinticPolynomial::~QuinticPolynomial(){

}
/**
 * @brief 输入初始状态，与目标状态，以及时间间隔
 * @param ss包含xs(位置), vs(速度), as(加速度)
 *        es包含xe(位置), ve(速度), ae(加速度)
 */
void QuinticPolynomial::init(double* ss, double* es, double T){
    a0 = ss[0];
    a1 = ss[1];
    a2 = ss[2]/2.0;
    Eigen::Matrix3d A;
    A << pow(T, 3), pow(T, 4), pow(T, 5),
         3*pow(T, 2), 4*pow(T, 3), 5*pow(T, 4),
         6*T, 12*pow(T, 2), 20*pow(T, 3);
    Eigen::Vector3d b;
    b << es[0] - a0 - a1*T - a2*pow(T, 2),
         es[1] - a1 - 2*a2*T, 
         es[2] - 2*a2;
    Eigen::Vector3d x = A.inverse()*b;
    a3 = x[0];
    a4 = x[1];
    a5 = x[2];
}
/**
 * @brief 计算位置状态
 */
double QuinticPolynomial::calc_point(double t){
    return a0 + a1*t + a2*pow(t,2) + a3*pow(t, 3) + a4*pow(t, 4) + a5*pow(t, 5);
}
/**
 * @brief 计算一阶导数
 */
double QuinticPolynomial::calc_first_derivative(double t){
    return a1 + 2*a2*t + 3*a3*pow(t, 2) + 4*a4*pow(t, 3) + 5*a5*pow(t, 4);
}
/**
 * @brief 计算二阶导数
 */
double QuinticPolynomial::calc_second_derivative(double t){
    return 2*a2 + 6*a3*t + 12*a4*pow(t, 2) + 20*a5*pow(t, 3);
}
/**
 * @brief 计算三阶导数
 */
double QuinticPolynomial::calc_third_derivative(double t){
    return 6*a3 + 24*a4*t + 60*a5*pow(t, 2);
}

/**
 * 4阶
 */
QuarticPolynomial::QuarticPolynomial(){
    a0 = 0; a1 = 0; a2 = 0;
    a3 = 0; a4 = 0;
}
QuarticPolynomial::~QuarticPolynomial(){

}
/**
 * @brief 输入初始状态，与目标状态，以及时间间隔
 * @param ss包含xs(位置), vs(速度), as(加速度)
 *        ve(速度), ae(加速度)
 */
void QuarticPolynomial::init(double* ss, double* es, double T){
    a0 = ss[0];
    a1 = ss[1];
    a2 = ss[2]/2.0;
    Eigen::Matrix2d A;
    A << 3*pow(T, 2), 4*pow(T, 3), 
         6*T, 12*pow(T, 2);
    Eigen::Vector2d b;
    b << es[0] - a1 - 2*a2*T,
         es[1] - 2*a2;
    Eigen::Vector2d x = A.inverse()*b;
    a3 = x[0];
    a4 = x[1];
}
/**
 * @brief 计算位置状态
 */
double QuarticPolynomial::calc_point(double t){
    return a0 + a1*t + a2*pow(t,2) + a3*pow(t, 3) + a4*pow(t, 4);
}
/**
 * @brief 计算一阶导数
 */
double QuarticPolynomial::calc_first_derivative(double t){
    return a1 + 2*a2*t + 3*a3*pow(t, 2) + 4*a4*pow(t, 3);
}
/**
 * @brief 计算二阶导数
 */
double QuarticPolynomial::calc_second_derivative(double t){
    return 2*a2 + 6*a3*t + 12*a4*pow(t, 2);
}
/**
 * @brief 计算三阶导数
 */
double QuarticPolynomial::calc_third_derivative(double t){
    return 6*a3 + 24*a4*t;
}
