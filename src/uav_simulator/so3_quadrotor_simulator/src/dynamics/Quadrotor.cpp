#include "quadrotor_simulator/Quadrotor.h"
#include "ode/boost/numeric/odeint.hpp"
#include <Eigen/Geometry>
#include <boost/bind.hpp>
#include <iostream>

#include <ros/ros.h>
namespace odeint = boost::numeric::odeint;

/* 四旋翼仿真模型 */
namespace QuadrotorSimulator
{

Quadrotor::Quadrotor(void)
{
  alpha0     = 48; // degree
  g_         = 9.81;
  mass_      = 0.98; // 0.5;
  double Ixx = 2.64e-3, Iyy = 2.64e-3, Izz = 4.96e-3;
  prop_radius_ = 0.062;
  J_           = Eigen::Vector3d(Ixx, Iyy, Izz).asDiagonal();

  kf_ = 8.98132e-9;
  // km_ = 2.5e-9; // from Nate
  // km = (Cq/Ct)*Dia*kf
  // Cq/Ct for 8 inch props from UIUC prop db ~ 0.07
  km_ = 0.07 * (3 * prop_radius_) * kf_;

  arm_length_          = 0.26;
  motor_time_constant_ = 1.0 / 30;
  min_rpm_             = 1200;
  max_rpm_             = 35000;

  state_.x = Eigen::Vector3d::Zero();
  // state_.x << 40.0, -60.0, 10.0;
  state_.v         = Eigen::Vector3d::Zero();
  state_.R         = Eigen::Matrix3d::Identity();
  state_.omega     = Eigen::Vector3d::Zero();
  state_.motor_rpm = Eigen::Array4d::Zero();

  external_force_.setZero();

  updateInternalState();

  input_ = Eigen::Array4d::Zero();
}

/* 更新状态 */
void
Quadrotor::step(double dt)
{
  auto save = internal_state_;

  /* 使用 Boost 的 odeint 库来集成飞行器的动力学方程,更新 internal_state_ 变量。这个过程模拟了飞行器在时间间隔 dt 内的状态变化。 */
  /* 
    odeint::integrate() 是 odeint 库提供的一个积分函数,用于数值积分微分方程。
    这行代码的作用是:
    将 Quadrotor 类对象自身作为参数传递给 odeint::integrate() 函数,这样 odeint 库就可以访问 Quadrotor 类中定义的动力学方程。
    使用 internal_state_ 作为初始状态,在时间间隔 dt 内,以步长 dt 进行数值积分,得到飞行器在该时间间隔内的状态变化。
    积分结果会被写回到 internal_state_ 数组中,更新飞行器的内部状态。
   */
  odeint::integrate(boost::ref(*this), internal_state_, 0.0, dt, dt);

  for (int i = 0; i < 22; ++i)
  {
    if (std::isnan(internal_state_[i]))
    {
      std::cout << "dump " << i << " << pos ";
      for (int j = 0; j < 22; ++j)
      {
        std::cout << save[j] << " ";
      }
      std::cout << std::endl;
      internal_state_ = save;
      break;
    }
  }

  for (int i = 0; i < 3; i++)
  {
    state_.x(i) = internal_state_[0 + i];
    state_.v(i) = internal_state_[3 + i];
    state_.R(i, 0) = internal_state_[6 + i];
    state_.R(i, 1) = internal_state_[9 + i];
    state_.R(i, 2) = internal_state_[12 + i];
    state_.omega(i) = internal_state_[15 + i];
  }
  state_.motor_rpm(0) = internal_state_[18];
  state_.motor_rpm(1) = internal_state_[19];
  state_.motor_rpm(2) = internal_state_[20];
  state_.motor_rpm(3) = internal_state_[21];

  // Re-orthonormalize R (polar decomposition)
  Eigen::LLT<Eigen::Matrix3d> llt(state_.R.transpose() * state_.R);
  Eigen::Matrix3d             P = llt.matrixL();
  Eigen::Matrix3d             R = state_.R * P.inverse();
  state_.R                      = R;

  // Don't go below zero, simulate floor
  if (state_.x(2) < 0.0 && state_.v(2) < 0)
  {
    state_.x(2) = 0;
    state_.v(2) = 0;
  }
  updateInternalState();
}

/* 这段代码实现了四轴飞行器的动力学模型。 */
void
Quadrotor::operator()(const Quadrotor::InternalState& x,
                      Quadrotor::InternalState& dxdt, const double /* t */)
{
  /* 初始化当前状态（cur_state）:
     从输入的状态向量 x 中提取出当前的位置、速度、旋转矩阵和角速度。
     同时提取出当前 4 个电机的转速。 */
  State cur_state;
  for (int i = 0; i < 3; i++)
  {
    cur_state.x(i) = x[0 + i];
    cur_state.v(i) = x[3 + i];
    cur_state.R(i, 0) = x[6 + i];
    cur_state.R(i, 1) = x[9 + i];
    cur_state.R(i, 2) = x[12 + i];
    cur_state.omega(i) = x[15 + i];
  }
  for (int i = 0; i < 4; i++)
  {
    cur_state.motor_rpm(i) = x[18 + i];
  }

  // std::cout << "Omega: " << cur_state.omega << std::endl;
  // std::cout << "motor_rpm: " << cur_state.motor_rpm << std::endl;

  /* 
    正交化旋转矩阵 R: 使用 LLT 分解来正交化旋转矩阵,确保其仍然是一个正交矩阵。 
    旋转矩阵 R 可能会随着时间的推移而偏离正交矩阵的性质(即 R^T * R = I)。
    使用 LLT 分解可以通过找到一个新的正交矩阵 R 来修正这种偏离,从而确保旋转矩阵保持正交性。
  */
  // Re-orthonormalize R (polar decomposition)
  // 首先计算旋转矩阵 R 的转置乘以自身,这会得到一个 3x3 的对称正定矩阵。
  Eigen::LLT<Eigen::Matrix3d> llt(cur_state.R.transpose() * cur_state.R); 
  // 然后使用 LLT 分解得到下三角矩阵 P
  Eigen::Matrix3d             P = llt.matrixL();
  // 通过乘以 P 的逆矩阵,可以得到一个正交的旋转矩阵 R
  Eigen::Matrix3d             R = cur_state.R * P.inverse();

  Eigen::Vector3d x_dot, v_dot, omega_dot;
  Eigen::Matrix3d R_dot;
  Eigen::Array4d  motor_rpm_dot;
  Eigen::Vector3d vnorm;  // 速度方向的单位向量
  Eigen::Array4d  motor_rpm_sq;   // 各个电机的转速的平方值
  Eigen::Matrix3d omega_vee(Eigen::Matrix3d::Zero());

  omega_vee(2, 1) = cur_state.omega(0);
  omega_vee(1, 2) = -cur_state.omega(0);
  omega_vee(0, 2) = cur_state.omega(1);
  omega_vee(2, 0) = -cur_state.omega(1);
  omega_vee(1, 0) = cur_state.omega(2);
  omega_vee(0, 1) = -cur_state.omega(2);

  // .array() 会将这个向量转换成一个 Eigen::Array4d 类型的数组。
  /* 
    Eigen 库中的 .square() 操作是针对 Eigen::Array 类型的数据结构定义的。
    具体来说:
    Eigen::Vector4d 是一个表示 4 维 向量 的数据类型。它提供了向量运算的接口, 如 加法、乘法 等。
    而 Eigen::Array4d 是一个表示 4 个 标量值 的数据结构。它提供了对应元素的逐个运算, 如 加法、乘法、平方 等。
    所以在这种情况下,我们需要先将 Eigen::Vector4d 转换成 Eigen::Array4d，才能使用 .square() 操作计算各个元素的平方。
   */
  motor_rpm_sq = cur_state.motor_rpm.array().square();  // 计算当前状态下各个电机的转速的平方值。

  // //! @todo implement
  // Eigen::Array4d blade_linear_velocity;
  // Eigen::Array4d motor_linear_velocity;
  // Eigen::Array4d AOA;
  // blade_linear_velocity = 0.104719755 // rpm to rad/s
  //                         * cur_state.motor_rpm.array() * prop_radius_;
  // for (int i = 0; i < 4; ++i)
  //   AOA[i]   = alpha0 - atan2(motor_linear_velocity[i], blade_linear_velocity[i]) * 180 / 3.14159265;
  // //! @todo end

  // double totalF = kf_ * motor_rpm_sq.sum();
  double thrust = kf_ * motor_rpm_sq.sum(); // 根据 4 个电机的转速平方值,计算出整个飞行器的总推力

  Eigen::Vector3d moments;
  /* 
    motor_rpm_sq(2) 和 motor_rpm_sq(3) 分别表示第3个和第4个电机的转速平方值。
    通过计算它们的差值 (motor_rpm_sq(2) - motor_rpm_sq(3))，可以得到沿 x 轴的力矩大小。
    这个差值乘以 kf_ 系数和 arm_length_ 常量(可能是电机到质心的距离,即臂长)，就得到了绕 x 轴的力矩 moments(0)。
 */
  moments(0) = kf_ * (motor_rpm_sq(2) - motor_rpm_sq(3)) * arm_length_;
  // 同上，得到沿 y 轴的力矩大小
  moments(1) = kf_ * (motor_rpm_sq(1) - motor_rpm_sq(0)) * arm_length_;
  // 沿 z 轴的力矩大小
  moments(2) = km_ * (motor_rpm_sq(0) + motor_rpm_sq(1) - motor_rpm_sq(2) - motor_rpm_sq(3));

  // 飞行器的空气阻力(resistance)
  /* 
    第一部分 0.1 * 是一个常数系数,表示空气阻力系数。这个系数是通过实验测量或理论计算得到的,与飞行器的外形、材质等有关。
    第二部分 3.14159265 * (arm_length_) * (arm_length_) * 计算了飞行器的截面积。
      由于这里是一个多旋翼飞行器,我们可以假设其截面积近似为一个圆形,半径就是臂长 arm_length_。所以这部分代码计算了圆形截面积 π * r^2。
    第三部分 cur_state.v.norm() * cur_state.v.norm() 计算了飞行器的速度平方值。
      cur_state.v 表示当前的速度向量,norm() 函数计算其模长(速度大小)。
   */
  double resistance = 0.1 *                                        // C
                      3.14159265 * (arm_length_) * (arm_length_) * // S
                      cur_state.v.norm() * cur_state.v.norm();

  //  ROS_INFO("resistance: %lf, Thrust: %lf%% ", resistance,
  //           motor_rpm_sq.sum() / (4 * max_rpm_ * max_rpm_) * 100.0);

  vnorm = cur_state.v;
  if (vnorm.norm() != 0)
  {
    vnorm.normalize();
  }
  /* 
    飞行器动力学公式
   */
  x_dot = cur_state.v;
  v_dot = -Eigen::Vector3d(0, 0, g_) + thrust * R.col(2) / mass_ +
          external_force_ / mass_ /*; //*/ - resistance * vnorm / mass_;

  acc_ = v_dot;
  //  acc_[2] = -acc_[2]; // to NED

  R_dot = R * omega_vee;
  omega_dot =
    J_.inverse() *
    (moments - cur_state.omega.cross(J_ * cur_state.omega) + external_moment_);
  motor_rpm_dot = (input_ - cur_state.motor_rpm) / motor_time_constant_;

  for (int i = 0; i < 3; i++)
  {
    dxdt[0 + i]  = x_dot(i);
    dxdt[3 + i]  = v_dot(i);
    dxdt[6 + i]  = R_dot(i, 0);
    dxdt[9 + i]  = R_dot(i, 1);
    dxdt[12 + i] = R_dot(i, 2);
    dxdt[15 + i] = omega_dot(i);
  }
  for (int i = 0; i < 4; i++)
  {
    dxdt[18 + i] = motor_rpm_dot(i);
  }
  for (int i = 0; i < 22; ++i)
  {
    if (std::isnan(dxdt[i]))
    {
      dxdt[i] = 0;
      //      std::cout << "nan apply to 0 for " << i << std::endl;
    }
  }
}

void
Quadrotor::setInput(double u1, double u2, double u3, double u4)
{
  input_(0) = u1;
  input_(1) = u2;
  input_(2) = u3;
  input_(3) = u4;
  for (int i = 0; i < 4; i++)
  {
    /* 
      在这段代码中,当输入值出现 NAN (Not a Number) 时,将该输入值设置为电机的最大转速和最小转速的平均值是有一定考虑的。
      这样做的主要原因是:
        异常值处理:
      当输入值出现 NAN 时,表明可能存在数值计算错误或其他异常情况。
      直接使用 NAN 值作为电机输入可能会导致严重的控制问题,甚至系统崩溃。
        安全考虑:
      将 NAN 输入值设置为电机的中间值,可以确保电机不会因过高或过低的输入而产生不正常的行为。
      这样可以在一定程度上保证无人机的飞行安全。
        平稳过渡:
      使用中间值作为输入可以使得电机平稳过渡到正常工作状态,避免突然的转速变化。
      这有助于维护无人机的稳定性和控制效果。
      总的来说,这种处理 NAN 输入值的方式可以在保证安全的同时,尽量减少异常输入对无人机控制的影响。当然,更好的做法是排查并修复导致 NAN 值出现的根源问题,从而避免这种情况的发生。
     */
    if (std::isnan(input_(i)))
    {
      input_(i) = (max_rpm_ + min_rpm_) / 2;
      std::cout << "NAN input ";
    }
    if (input_(i) > max_rpm_)
      input_(i) = max_rpm_;
    else if (input_(i) < min_rpm_)
      input_(i) = min_rpm_;
  }
}

const Quadrotor::State&
Quadrotor::getState(void) const
{
  return state_;
}
void
Quadrotor::setState(const Quadrotor::State& state)
{
  state_.x         = state.x;
  state_.v         = state.v;
  state_.R         = state.R;
  state_.omega     = state.omega;
  state_.motor_rpm = state.motor_rpm;

  updateInternalState();
}

void
Quadrotor::setStatePos(const Eigen::Vector3d& Pos)
{
  state_.x = Pos;

  updateInternalState();
}

double
Quadrotor::getMass(void) const
{
  return mass_;
}
void
Quadrotor::setMass(double mass)
{
  mass_ = mass;
}

double
Quadrotor::getGravity(void) const
{
  return g_;
}
void
Quadrotor::setGravity(double g)
{
  g_ = g;
}

const Eigen::Matrix3d&
Quadrotor::getInertia(void) const
{
  return J_;
}
void
Quadrotor::setInertia(const Eigen::Matrix3d& inertia)
{
  if (inertia != inertia.transpose())
  {
    std::cerr << "Inertia matrix not symmetric, not setting" << std::endl;
    return;
  }
  J_ = inertia;
}

double
Quadrotor::getArmLength(void) const
{
  return arm_length_;
}
void
Quadrotor::setArmLength(double d)
{
  if (d <= 0)
  {
    std::cerr << "Arm length <= 0, not setting" << std::endl;
    return;
  }

  arm_length_ = d;
}

double
Quadrotor::getPropRadius(void) const
{
  return prop_radius_;
}
void
Quadrotor::setPropRadius(double r)
{
  if (r <= 0)
  {
    std::cerr << "Prop radius <= 0, not setting" << std::endl;
    return;
  }
  prop_radius_ = r;
}

double
Quadrotor::getPropellerThrustCoefficient(void) const
{
  return kf_;
}
void
Quadrotor::setPropellerThrustCoefficient(double kf)
{
  if (kf <= 0)
  {
    std::cerr << "Thrust coefficient <= 0, not setting" << std::endl;
    return;
  }

  kf_ = kf;
}

double
Quadrotor::getPropellerMomentCoefficient(void) const
{
  return km_;
}
void
Quadrotor::setPropellerMomentCoefficient(double km)
{
  if (km <= 0)
  {
    std::cerr << "Moment coefficient <= 0, not setting" << std::endl;
    return;
  }

  km_ = km;
}

double
Quadrotor::getMotorTimeConstant(void) const
{
  return motor_time_constant_;
}
void
Quadrotor::setMotorTimeConstant(double k)
{
  if (k <= 0)
  {
    std::cerr << "Motor time constant <= 0, not setting" << std::endl;
    return;
  }

  motor_time_constant_ = k;
}

const Eigen::Vector3d&
Quadrotor::getExternalForce(void) const
{
  return external_force_;
}
void
Quadrotor::setExternalForce(const Eigen::Vector3d& force)
{
  external_force_ = force;
}

const Eigen::Vector3d&
Quadrotor::getExternalMoment(void) const
{
  return external_moment_;
}
void
Quadrotor::setExternalMoment(const Eigen::Vector3d& moment)
{
  external_moment_ = moment;
}

double
Quadrotor::getMaxRPM(void) const
{
  return max_rpm_;
}
void
Quadrotor::setMaxRPM(double max_rpm)
{
  if (max_rpm <= 0)
  {
    std::cerr << "Max rpm <= 0, not setting" << std::endl;
    return;
  }
  max_rpm_ = max_rpm;
}

double
Quadrotor::getMinRPM(void) const
{
  return min_rpm_;
}
void
Quadrotor::setMinRPM(double min_rpm)
{
  if (min_rpm < 0)
  {
    std::cerr << "Min rpm < 0, not setting" << std::endl;
    return;
  }
  min_rpm_ = min_rpm;
}

/* 通常是为了便于存储或传输当前的无人机状态。 */
void
Quadrotor::updateInternalState(void)
{
  for (int i = 0; i < 3; i++)
  {
    internal_state_[0 + i]  = state_.x(i);
    internal_state_[3 + i]  = state_.v(i);
    internal_state_[6 + i]  = state_.R(i, 0);
    internal_state_[9 + i]  = state_.R(i, 1);
    internal_state_[12 + i] = state_.R(i, 2);
    internal_state_[15 + i] = state_.omega(i);
  }
  internal_state_[18] = state_.motor_rpm(0);
  internal_state_[19] = state_.motor_rpm(1);
  internal_state_[20] = state_.motor_rpm(2);
  internal_state_[21] = state_.motor_rpm(3);
}

Eigen::Vector3d
Quadrotor::getAcc() const
{
  return acc_;
}
}
