/* 这是一个用于模拟四旋翼无人机动力学的C++类。该类封装了四旋翼无人机的状态和参数,并提供了相关的访问和设置函数。 */
#ifndef __QUADROTOR_SIMULATOR_QUADROTOR_H__
#define __QUADROTOR_SIMULATOR_QUADROTOR_H__

#include <Eigen/Core>
#include <boost/array.hpp>

namespace QuadrotorSimulator
{

class Quadrotor
{
public:
  /* 
  状态结构体: Quadrotor 类包含一个名为 State 的内部结构体,它定义了四旋翼无人机的状态,包括位置、速度、姿态和每个电机的转速。
  这个结构体使用了 Eigen 库中的向量和矩阵类型 
  */
  struct State
  {
    Eigen::Vector3d x;  // 位置
    Eigen::Vector3d v;  // 速度
    Eigen::Matrix3d R;  // 姿态矩阵，也就是旋转矩阵
    Eigen::Vector3d omega;  // 无人机在 3 个坐标轴上的旋转速度,单位通常为弧度每秒(rad/s)。
    Eigen::Array4d  motor_rpm;  // 电机转速
    // 对于任何包含 Eigen 类型成员变量的自定义类,都应该在类定义中添加 EIGEN_MAKE_ALIGNED_OPERATOR_NEW 宏。
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;  // Eigen 库提供的一个宏定义,它的作用是为包含 Eigen 类型的自定义类型提供适当的内存分配和对齐机制。
  };

  Quadrotor();

  // 获取当前的无人机状态，返回一个引用
  const Quadrotor::State& getState(void) const;

  void setState(const Quadrotor::State& state);

  void setStatePos(const Eigen::Vector3d& Pos);

  double getMass(void) const;
  void setMass(double mass);

  double getGravity(void) const;
  void setGravity(double g);

  const Eigen::Matrix3d& getInertia(void) const;
  void setInertia(const Eigen::Matrix3d& inertia);

  // 臂长
  double getArmLength(void) const;
  void setArmLength(double d);

  // 桨叶半径
  double getPropRadius(void) const;
  void setPropRadius(double r);

  // 无人机桨叶推力系数
  double getPropellerThrustCoefficient(void) const;
  void setPropellerThrustCoefficient(double kf);

  // 无人机桨叶力矩系数
  double getPropellerMomentCoefficient(void) const;
  void setPropellerMomentCoefficient(double km);

  // 电机时间常数
  double getMotorTimeConstant(void) const;
  void setMotorTimeConstant(double k);

  // 外部作用力
  const Eigen::Vector3d& getExternalForce(void) const;
  void setExternalForce(const Eigen::Vector3d& force);

  // 外部作用力矩
  const Eigen::Vector3d& getExternalMoment(void) const;
  void setExternalMoment(const Eigen::Vector3d& moment);

  // 电机最大转速
  double getMaxRPM(void) const;
  void setMaxRPM(double max_rpm);

  // 电机最小转速
  /* 
    无人机电机的最小转速通常不会设置为0。这里有几个原因:
    电机机械特性限制:
    大多数电机无法完全停止转动,即使给定0转速指令,也会存在一个最小的转速值。这是由于电机内部机械摩擦、电磁阻力等因素造成的。
    飞行控制需求:
    四旋翼无人机需要保持最小的推力才能悬停。如果电机转速设置为0,无人机很难保持稳定的悬停状态。
    安全考虑:
    将最小转速设置为一个较小但非零的值,可以确保电机始终保持最小的推力输出。
    这可以防止电机完全停止转动,从而避免在起飞或降落时出现意外情况。
    因此,在实际应用中,无人机电机的最小转速通常会设置为一个较小但非零的值,例如500-1000 rpm左右。这样既满足飞行控制的需求,又能确保电机的安全性和可靠性。
  */
  double getMinRPM(void) const;
  void setMinRPM(double min_rpm);

  // Inputs are desired RPM for the motors    RPM: 每分钟转数（Revolutions Per Minute）
  // Rotor numbering is:
  //   *1*    Front
  // 3     4
  //    2
  // with 1 and 2 clockwise and 3 and 4 counter-clockwise (looking from top)
  void setInput(double u1, double u2, double u3, double u4);

  // Runs the actual dynamics simulation with a time step of dt
  void step(double dt);

  // For internal use, but needs to be public for odeint
  typedef boost::array<double, 22> InternalState;
  void operator()(const Quadrotor::InternalState& x,
                  Quadrotor::InternalState&       dxdt, const double /* t */);

  Eigen::Vector3d getAcc() const;

private:
  void updateInternalState(void);

  double          alpha0; // AOA  机体迎角 (Angle of Attack)
  double          g_;     // gravity
  double          mass_;
  Eigen::Matrix3d J_; // Inertia
  double          kf_;  // 桨叶推力系数
  double          km_;  // 桨叶力矩系数
  double          prop_radius_; // 桨叶半径
  double          arm_length_;  // 臂长
  double          motor_time_constant_; // unit: sec  // 电机时间常数
  double          max_rpm_;
  double          min_rpm_;

  Quadrotor::State state_;

  Eigen::Vector3d acc_;

  Eigen::Array4d  input_;
  Eigen::Vector3d external_force_;  // 外部干扰力
  Eigen::Vector3d external_moment_; // 外部干扰力矩

  InternalState internal_state_;
};
}
#endif
