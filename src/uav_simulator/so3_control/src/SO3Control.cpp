#include <iostream>
#include <so3_control/SO3Control.h>

#include <ros/ros.h>

SO3Control::SO3Control()
  : mass_(0.5)
  , g_(9.81)
{
  acc_.setZero();
}

void
SO3Control::setMass(const double mass)
{
  mass_ = mass;
}

void
SO3Control::setGravity(const double g)
{
  g_ = g;
}

void
SO3Control::setPosition(const Eigen::Vector3d& position)
{
  pos_ = position;
}

void
SO3Control::setVelocity(const Eigen::Vector3d& velocity)
{
  vel_ = velocity;
}

/* 
  这段代码实现了一个SE3控制器,用于计算期望的力force_和期望的姿态orientation_,以使无人机能够跟踪给定的位置、速度、加速度和航向角。
  上层控制器
    输入：四旋翼总推力F
    输出：无人机位置和偏航角  以跟踪 期望轨迹所提供的期望位置des_pos和期望偏航角des_yaw
*/
void
SO3Control::calculateControl(const Eigen::Vector3d& des_pos,
                             const Eigen::Vector3d& des_vel,
                             const Eigen::Vector3d& des_acc,
                             const double des_yaw, const double des_yaw_dot,
                             const Eigen::Vector3d& kx,
                             const Eigen::Vector3d& kv)
{
  //  ROS_INFO("Error %lf %lf %lf", (des_pos - pos_).norm(),
  //           (des_vel - vel_).norm(), (des_acc - acc_).norm());

  // 检查期望位置des_pos、期望速度des_vel和期望加速度des_acc是否有效(不是NaN)。
  bool flag_use_pos = !(std::isnan(des_pos(0)) || std::isnan(des_pos(1)) || std::isnan(des_pos(2)));
  bool flag_use_vel = !(std::isnan(des_vel(0)) || std::isnan(des_vel(1)) || std::isnan(des_vel(2)));
  bool flag_use_acc = !(std::isnan(des_acc(0)) || std::isnan(des_acc(1)) || std::isnan(des_acc(2)));

  // 计算总位置、速度、加速度误差totalError。 .noalias()是将 des_pos - pos_ 的结果直接累加到 totalError 上,而不创建临时量，从而提高运算效率。
  Eigen::Vector3d totalError(Eigen::Vector3d::Zero());
  if ( flag_use_pos ) totalError.noalias() += des_pos - pos_;   // .noalias()可以避免创建和销毁临时变量,从而提高运算效率
  if ( flag_use_vel ) totalError.noalias() += des_vel - vel_;
  if ( flag_use_acc ) totalError.noalias() += des_acc - acc_;

  // 当误差绝对值大于3时,该方向的加速度补偿力被设为0,否则设为与误差成比例的力。
  /* 
    这里设置当误差绝对值大于3时,该方向的加速度补偿力反而被设为0,主要有以下几个考虑:
    防止过大补偿力导致的不稳定:
        当误差很大时,如果继续大幅补偿加速度,可能会导致系统振荡甚至失控。设置一个阈值(这里是3),当误差超过这个阈值时,停止补偿,可以避免系统失稳。
    保证系统可控性:
        当误差太大时,系统可能已经进入了难以控制的状态。停止补偿可以让系统恢复到可控范围内,避免进一步偏离目标状态。
    防止饱和问题:
        很多执行机构(电机等)都有功率或转矩的饱和限制。如果补偿力过大,容易导致执行机构饱和,从而无法提供足够的控制力。
        设置阈值可以防止补偿力过大导致的饱和问题。
    保持预测模型的准确性:
        当误差很大时,系统可能已经偏离了预测模型的适用范围。停止补偿可以避免使用不准确的预测模型进行控制,保证控制效果。
  */
  Eigen::Vector3d ka(fabs(totalError[0]) > 3 ? 0 : (fabs(totalError[0]) * 0.2),
                     fabs(totalError[1]) > 3 ? 0 : (fabs(totalError[1]) * 0.2),
                     fabs(totalError[2]) > 3 ? 0 : (fabs(totalError[2]) * 0.2));

  // std::cout << des_pos.transpose() << std::endl;
  // std::cout << des_vel.transpose() << std::endl;
  // std::cout << des_acc.transpose() << std::endl;
  // std::cout << des_yaw << std::endl;
  // std::cout << pos_.transpose() << std::endl;
  // std::cout << vel_.transpose() << std::endl;
  // std::cout << acc_.transpose() << std::endl;
  

  force_ = mass_ * g_ * Eigen::Vector3d(0, 0, 1);   // 先将force_初始化为重力方向的补偿力大小
  if ( flag_use_pos ) force_.noalias() += kx.asDiagonal() * (des_pos - pos_); // 如果使用位置,则加上kx * (des_pos - pos_)
  if ( flag_use_vel ) force_.noalias() += kv.asDiagonal() * (des_vel - vel_); // 如果使用速度,则加上kv * (des_vel - vel_)
  /*
    1.mass_ * des_acc 用于跟踪期望加速度
    2.mass_ * ka * (des_acc - acc_) 用于抑制加速度误差
    将这两部分相加,就能够综合考虑期望加速度和实际加速度偏差,从而更好地控制无人机的运动。 
  */
  if ( flag_use_acc ) force_.noalias() += mass_ * ka.asDiagonal() * (des_acc - acc_) + mass_ * (des_acc); 

  // Limit control angle to 45 degree   限制force_的方向,使其与z轴的夹角不超过45度
  /*
    限制F方向，保证 F 与 z 轴夹角不超过 theta, 超过theta时，即调整回theta 
  */
  double          theta = M_PI / 2;   // 这里是不是应该改成 M_PI/4 ?
  // 虽然直接赋值c=0.0看起来更加简单,但是通过计算cos(theta)的方式能够提高代码的可读性和可维护性,其与向量的点乘相联系
  double          c     = cos(theta);   
  Eigen::Vector3d f;
  f.noalias() = force_ - mass_ * g_ * Eigen::Vector3d(0, 0, 1);   // 去除重力后的力,在z轴方向可能也会有分量
  if (Eigen::Vector3d(0, 0, 1).dot(force_ / force_.norm()) < c) // cos(a,b) < c,判断z轴与force_的夹角是否大于theta
  {
    double nf        = f.norm();
    double A         = c * c * nf * nf - f(2) * f(2);
    double B         = 2 * (c * c - 1) * f(2) * mass_ * g_;
    double C         = (c * c - 1) * mass_ * mass_ * g_ * g_;
    double s         = (-B + sqrt(B * B - 4 * A * C)) / (2 * A);
    force_.noalias() = s * f + mass_ * g_ * Eigen::Vector3d(0, 0, 1); // 调整后的 force 为 f的s倍+补偿重力的力
  }
  // Limit control angle to 45 degree

  /* 
    根据期望控制力force_和期望的偏航角des_yaw,计算出期望的四元数orientation_,即无人机的期望姿态
    也就是说，为了得到期望的force_和des_yaw,无人机的姿态orientation_也需要唯一确定,使得force_和des_yaw能够同时被满足
  */
  Eigen::Vector3d b1c, b2c, b3c;  // 无人机机体坐标系在期望位置的三个基向量
  Eigen::Vector3d b1d(cos(des_yaw), sin(des_yaw), 0);   // 期望的偏航角在水平面内的投影

  if (force_.norm() > 1e-6)   // 如果force_的范数大于一个很小的值(避免除0错误)
    b3c.noalias() = force_.normalized();  // 将b3c赋值为force_的单位向量。b3c代表机体z轴的方向,即推力方向。
  else
    b3c.noalias() = Eigen::Vector3d(0, 0, 1);   // 否则赋值为(0,0,1)

  b2c.noalias() = b3c.cross(b1d).normalized();  // b2c垂直于b3c和b1d,代表期望位置机体坐标系的y轴方向。
  b1c.noalias() = b2c.cross(b3c).normalized();

  Eigen::Matrix3d R;  // 将b1c,b2c,b3c作为机体坐标系三个基向量的列向量,构造 旋转矩阵R。
  R << b1c, b2c, b3c;

  orientation_ = Eigen::Quaterniond(R);   // 使用旋转R计算四元数,最终得到了 无人机在当前时刻的 期望姿态orientation_
}

const Eigen::Vector3d&
SO3Control::getComputedForce(void)
{
  return force_;
}

const Eigen::Quaterniond&
SO3Control::getComputedOrientation(void)
{
  return orientation_;
}

void
SO3Control::setAcc(const Eigen::Vector3d& acc)
{
  acc_ = acc;
}
