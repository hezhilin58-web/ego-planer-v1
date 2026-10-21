#ifndef __SO3_CONTROL_H__
#define __SO3_CONTROL_H__

#include <Eigen/Geometry>

/* 位姿控制器大类 */
class SO3Control
{
public:
  SO3Control();

  void setMass(const double mass);
  void setGravity(const double g);
  void setPosition(const Eigen::Vector3d& position);
  void setVelocity(const Eigen::Vector3d& velocity);
  void setAcc(const Eigen::Vector3d& acc);

  /* 计算四旋翼无人机的控制力force_和期望姿态orientation_。 */
  void calculateControl(const Eigen::Vector3d& des_pos,
                        const Eigen::Vector3d& des_vel,
                        const Eigen::Vector3d& des_acc, const double des_yaw,
                        const double des_yaw_dot, const Eigen::Vector3d& kx,
                        const Eigen::Vector3d& kv);

  const Eigen::Vector3d&    getComputedForce(void);
  const Eigen::Quaterniond& getComputedOrientation(void);

  /* 
  默认情况下,C++的new操作符无法保证内存对齐。
  为了解决这个问题,Eigen提供了EIGEN_MAKE_ALIGNED_OPERATOR_NEW宏,它会重写类中的operator new和operator delete操作符,
  使得内存分配和释放过程中能够保证内存对齐。
  具体来说,当我们在类中使用EIGEN_MAKE_ALIGNED_OPERATOR_NEW时,
  该类的对象在使用new和delete进行内存分配和释放时,就会使用Eigen提供的内存分配器,确保内存地址满足Eigen数据类型对齐的要求。
  这样做可以提高Eigen库中涉及矩阵和向量运算的性能,因为CPU的SIMD指令集可以更好地利用对齐的内存数据进行并行计算。
  对于需要频繁创建和销毁大量Eigen对象的场景,使用EIGEN_MAKE_ALIGNED_OPERATOR_NEW尤其重要。
  */
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  // Inputs for the controller
  double          mass_;
  double          g_;
  Eigen::Vector3d pos_;
  Eigen::Vector3d vel_;
  Eigen::Vector3d acc_;

  // Outputs of the controller
  Eigen::Vector3d    force_;  
  Eigen::Quaterniond orientation_;  // 姿态
};

#endif
