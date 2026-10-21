#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/SO3Command.h>
#include <quadrotor_simulator/Quadrotor.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <uav_utils/geometry_utils.h>

typedef struct _Control
{
  double rpm[4];
} Control;

typedef struct _Command
{
  float force[3];
  float qx, qy, qz, qw;
  float kR[3];
  float kOm[3];
  float corrections[3];
  float current_yaw;
  bool  use_external_yaw;
} Command;

typedef struct _Disturbance
{
  Eigen::Vector3d f;
  Eigen::Vector3d m;
} Disturbance;

static Command     command;
static Disturbance disturbance;

void stateToOdomMsg(const QuadrotorSimulator::Quadrotor::State& state,
                    nav_msgs::Odometry&                         odom);
void quadToImuMsg(const QuadrotorSimulator::Quadrotor& quad,
                  sensor_msgs::Imu&                    imu);

/* 
  这段代码实现了一个四旋翼飞行器的底层控制器(Geometric Controller)。 下位机(onboard controller)的控制SE3控制
    它的作用是根据飞行器当前的状态(位置、姿态、角速度)和 订阅的控制命令(期望力、期望姿态、控制增益等),计算出四个电机的期望转速,作为控制输出。 
    来源：Geometric Tracking Control of a Quadrotor UAV on SE(3)
          Minimum Snap Trajectory Generation and Control for Quadrotors （引用了上面的文章）
*/
static Control
getControl(const QuadrotorSimulator::Quadrotor& quad, const Command& cmd)
{
  const double _kf = quad.getPropellerThrustCoefficient();  // 螺旋桨推力系数
  const double _km = quad.getPropellerMomentCoefficient();  // 螺旋桨力矩系数
  const double kf  = _kf - cmd.corrections[0];              // 对 _kf 做 校正或修正
  const double km  = _km / _kf * kf;                        // 根据 修正的kf 来调整 km 的值

  const double          d       = quad.getArmLength();      // 臂长
  const Eigen::Matrix3f J       = quad.getInertia().cast<float>();  // 转动惯量 .cast<float>()的作用是将返回的 Eigen::Matrix3d 类型的矩阵转换为 Eigen::Matrix3f 类型。
  /*
    作者选择使用二维数组 I 而不是直接使用 Eigen::Matrix3f 类型的 J。这主要是因为 Eigen 库中的矩阵类型有一些不同的访问方式,与普通的二维数组稍有不同。
    具体来说:
      对于二维数组 I，可以直接使用 I[row][col] 的方式访问矩阵元素。这种访问方式与普通的 C 风格二维数组完全一致。
      而对于 Eigen::Matrix3f 类型的矩阵 J，无法直接使用 J[2][0] 的方式访问元素。 
      Eigen 库中的矩阵采用的是重载的()运算符访问元素,应该使用 J(2, 0) 的方式。
      所以,如果直接使用 J[2][0] 是无法编译通过的。必须使用 J(2, 0) 的方式才能正确访问 Eigen 矩阵中的元素。
    总的来说,使用二维数组 I 的主要原因是为了保持与常规数组访问的一致性,而不是使用 Eigen 库特有的矩阵访问方式。这样做可以让代码更加简洁易读。 
    此外，在运算速度方面,使用二维数组 I[2][0] 通常会比使用 Eigen::Matrix3f 类型的 J(2, 0) 更快一些。
    对于追求极致性能的场景,使用二维数组可能会更加合适。
   */
  const float           I[3][3] = { { J(0, 0), J(0, 1), J(0, 2) },
                          { J(1, 0), J(1, 1), J(1, 2) },
                          { J(2, 0), J(2, 1), J(2, 2) } };
  const QuadrotorSimulator::Quadrotor::State state = quad.getState();

  // Rotation, may use external yaw
  Eigen::Vector3d _ypr = uav_utils::R_to_ypr(state.R);  // 从旋转矩阵 state.R 中提取出欧拉角(yaw, pitch, roll)值
  Eigen::Vector3d ypr  = _ypr;  // 这一步是为了方便后续的操作,因为可能会修改 ypr 变量中的某些分量值。
  if (cmd.use_external_yaw)
    ypr[0] = cmd.current_yaw;
  Eigen::Matrix3d R;
  /* 
    根据修改后的 ypr 变量重新构建旋转矩阵 R （这里是 当下的位姿，旋转矩阵）
    使用Eigen库提供的 Eigen::AngleAxisd 类型,分别围绕 Z、Y、X轴构建三个旋转矩阵,然后将它们乘积得到最终的旋转矩阵 R。
    这个过程相当于通过指定的欧拉角(yaw, pitch, roll)值来重新构建旋转矩阵。  
  */
  R = Eigen::AngleAxisd(ypr[0], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(ypr[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(ypr[2], Eigen::Vector3d::UnitX());
  // if (cmd.use_external_yaw) {
  //   ypr[0] = cmd.current_yaw;
  //   Eigen::Matrix3d R;
  //   R = Eigen::AngleAxisd(ypr[0], Eigen::Vector3d::UnitZ()) *
  //       Eigen::AngleAxisd(ypr[1], Eigen::Vector3d::UnitY()) *
  //       Eigen::AngleAxisd(ypr[2], Eigen::Vector3d::UnitX());
  // } else {
  //   Eigen::Matrix3d R;
  //   R = state.R
  // }
  // 注意，此处是 实际姿态，当下的四旋翼状态所对应的实际旋转矩阵
  float R11 = R(0, 0);
  float R12 = R(0, 1);
  float R13 = R(0, 2);
  float R21 = R(1, 0);
  float R22 = R(1, 1);
  float R23 = R(1, 2);
  float R31 = R(2, 0);
  float R32 = R(2, 1);
  float R33 = R(2, 2);
  /*
    float R11 = state.R(0,0);
    float R12 = state.R(0,1);
    float R13 = state.R(0,2);
    float R21 = state.R(1,0);
    float R22 = state.R(1,1);
    float R23 = state.R(1,2);
    float R31 = state.R(2,0);
    float R32 = state.R(2,1);
    float R33 = state.R(2,2);
  */
  float Om1 = state.omega(0);
  float Om2 = state.omega(1);
  float Om3 = state.omega(2);

  /* 
    根据四元数 (cmd.qw, cmd.qx, cmd.qy, cmd.qz) 计算出一个（期望）旋转矩阵 R。 
    这些计算公式可以直接从四元数到旋转矩阵的转换公式中得到。这种方式比使用Eigen库中的Quaternion到Matrix3d的转换要更加直接和高效。
  */
  // 注意，这里 Rd 对应的 应该是 期望的旋转矩阵
  float Rd11 =
    cmd.qw * cmd.qw + cmd.qx * cmd.qx - cmd.qy * cmd.qy - cmd.qz * cmd.qz;
  float Rd12 = 2 * (cmd.qx * cmd.qy - cmd.qw * cmd.qz);
  float Rd13 = 2 * (cmd.qx * cmd.qz + cmd.qw * cmd.qy);
  float Rd21 = 2 * (cmd.qx * cmd.qy + cmd.qw * cmd.qz);
  float Rd22 =
    cmd.qw * cmd.qw - cmd.qx * cmd.qx + cmd.qy * cmd.qy - cmd.qz * cmd.qz;
  float Rd23 = 2 * (cmd.qy * cmd.qz - cmd.qw * cmd.qx);
  float Rd31 = 2 * (cmd.qx * cmd.qz - cmd.qw * cmd.qy);
  float Rd32 = 2 * (cmd.qy * cmd.qz + cmd.qw * cmd.qx);
  float Rd33 =
    cmd.qw * cmd.qw - cmd.qx * cmd.qx - cmd.qy * cmd.qy + cmd.qz * cmd.qz;

  /* 
    计算 当前旋转矩阵R 与 期望旋转矩阵Rd 之间的差异程度 Psi。  在C/C++ 中,当一个小数字面量没有指定类型后缀时,编译器会默认将其解释为 double 类型。
    由于旋转矩阵是正交矩阵,其行列式为1,因此 Rd 和 R 的 对应元素乘积之和 最大为3。
        当 Rd 和 R 完全相同时,Psi = 0; 当 Rd 和 R 完全不同时, Psi = 1.5。
   */
  float Psi = 0.5f * (3.0f - (Rd11 * R11 + Rd21 * R21 + Rd31 * R31 +
                              Rd12 * R12 + Rd22 * R22 + Rd32 * R32 +
                              Rd13 * R13 + Rd23 * R23 + Rd33 * R33));

  float force = 0;
  // 如果 Psi < 1.0f，说明姿态误差较小,可以进行位置控制。
  if (Psi < 1.0f) // Position control stability guaranteed only when Psi < 1 只有当Psi小于1.0时,位置控制的稳定性才能得到保证。
    // cmd.force是一个包含三个元素的向量,分别表示无人机在机体x、y、z三个轴方向上期望施加的控制力。
    // R13、R23、R33分别是当前旋转矩阵 R 的最后一列,表示无人机机体z轴在惯性(世界)坐标系下的方向。
    // 这句代码的作用是将期望力的表示从惯性(世界)坐标系转换到当前机体坐标系,以便在当前机体坐标系下进行位置控制。
    force = cmd.force[0] * R13 + cmd.force[1] * R23 + cmd.force[2] * R33; // 将期望力 cmd.force 通过旋转矩阵 R 变换到机体坐标系下

  /* 
    计算角度误差: 这些是当前旋转矩阵 R 和 期望旋转矩阵 Rd 之间的角度误差。它们用于反馈到角速度控制器中。
    calculate the error between the current rotation matrix R and the desired rotation matrix Rd using the Rodrigues' rotation formula.
   */
  float eR1 = 0.5f * (R12 * Rd13 - R13 * Rd12 + R22 * Rd23 - R23 * Rd22 +
                      R32 * Rd33 - R33 * Rd32);
  float eR2 = 0.5f * (R13 * Rd11 - R11 * Rd13 - R21 * Rd23 + R23 * Rd21 -
                      R31 * Rd33 + R33 * Rd31);
  float eR3 = 0.5f * (R11 * Rd12 - R12 * Rd11 + R21 * Rd22 - R22 * Rd21 +
                      R31 * Rd32 - R32 * Rd31);

  float eOm1 = Om1;
  float eOm2 = Om2;
  float eOm3 = Om3;

  float in1 = Om2 * (I[2][0] * Om1 + I[2][1] * Om2 + I[2][2] * Om3) -
              Om3 * (I[1][0] * Om1 + I[1][1] * Om2 + I[1][2] * Om3);
  float in2 = Om3 * (I[0][0] * Om1 + I[0][1] * Om2 + I[0][2] * Om3) -
              Om1 * (I[2][0] * Om1 + I[2][1] * Om2 + I[2][2] * Om3);
  float in3 = Om1 * (I[1][0] * Om1 + I[1][1] * Om2 + I[1][2] * Om3) -
              Om2 * (I[0][0] * Om1 + I[0][1] * Om2 + I[0][2] * Om3);
  /*
    // Robust Control --------------------------------------------
    float c2       = 0.6;
    float epsilonR = 0.04;
    float deltaR   = 0.1;
    float eA1 = eOm1 + c2 * 1.0/I[0][0] * eR1;
    float eA2 = eOm2 + c2 * 1.0/I[1][1] * eR2;
    float eA3 = eOm3 + c2 * 1.0/I[2][2] * eR3;
    float neA = sqrt(eA1*eA1 + eA2*eA2 + eA3*eA3);
    float muR1 = -deltaR*deltaR * eA1 / (deltaR * neA + epsilonR);
    float muR2 = -deltaR*deltaR * eA2 / (deltaR * neA + epsilonR);
    float muR3 = -deltaR*deltaR * eA3 / (deltaR * neA + epsilonR);
    // Robust Control --------------------------------------------
  */
  float M1 = -cmd.kR[0] * eR1 - cmd.kOm[0] * eOm1 + in1; // - I[0][0]*muR1;
  float M2 = -cmd.kR[1] * eR2 - cmd.kOm[1] * eOm2 + in2; // - I[1][1]*muR2;
  float M3 = -cmd.kR[2] * eR3 - cmd.kOm[2] * eOm3 + in3; // - I[2][2]*muR3;

  float w_sq[4];
  w_sq[0] = force / (4 * kf) - M2 / (2 * d * kf) + M3 / (4 * km);
  w_sq[1] = force / (4 * kf) + M2 / (2 * d * kf) + M3 / (4 * km);
  w_sq[2] = force / (4 * kf) + M1 / (2 * d * kf) - M3 / (4 * km);
  w_sq[3] = force / (4 * kf) - M1 / (2 * d * kf) - M3 / (4 * km);

  Control control;
  for (int i = 0; i < 4; i++)
  {
    if (w_sq[i] < 0)
      w_sq[i] = 0;

    control.rpm[i] = sqrtf(w_sq[i]);
  }
  return control;
}

/* 
  这个回调函数是用于处理由quadrotor_msgs::SO3Command消息发送的命令的。
  总的来说，这个回调函数是用来更新命令对象的各种参数,这些参数将被用于控制四轴飞行器的运动。
  通过接收来自外部的SO3Command消息,可以动态地调整四轴飞行器的控制参数。
 */
static void
cmd_callback(const quadrotor_msgs::SO3Command::ConstPtr& cmd)
{
  // 将消息中的力(force.x, force.y, force.z)赋值给command.force数组。
  command.force[0]         = cmd->force.x;
  command.force[1]         = cmd->force.y;
  command.force[2]         = cmd->force.z;
  // 将消息中的姿态四元数(orientation.x, orientation.y, orientation.z, orientation.w)赋值给command对象。
  command.qx               = cmd->orientation.x;
  command.qy               = cmd->orientation.y;
  command.qz               = cmd->orientation.z;
  command.qw               = cmd->orientation.w;
  // 将消息中的姿态控制增益(kR[0], kR[1], kR[2])赋值给command.kR数组。
  command.kR[0]            = cmd->kR[0];
  command.kR[1]            = cmd->kR[1];
  command.kR[2]            = cmd->kR[2];
  // 将消息中的角速度控制增益(kOm[0], kOm[1], kOm[2])赋值给command.kOm数组。
  command.kOm[0]           = cmd->kOm[0];
  command.kOm[1]           = cmd->kOm[1];
  command.kOm[2]           = cmd->kOm[2];
  // 将消息中的辅助参数(kf_correction, angle_corrections[0], angle_corrections[1], current_yaw, use_external_yaw)赋值给command对象的相应字段。
  command.corrections[0]   = cmd->aux.kf_correction;
  command.corrections[1]   = cmd->aux.angle_corrections[0];
  command.corrections[2]   = cmd->aux.angle_corrections[1];
  command.current_yaw      = cmd->aux.current_yaw;
  command.use_external_yaw = cmd->aux.use_external_yaw;
}

static void
force_disturbance_callback(const geometry_msgs::Vector3::ConstPtr& f)
{
  disturbance.f(0) = f->x;
  disturbance.f(1) = f->y;
  disturbance.f(2) = f->z;
}

static void
moment_disturbance_callback(const geometry_msgs::Vector3::ConstPtr& m)
{
  disturbance.m(0) = m->x;
  disturbance.m(1) = m->y;
  disturbance.m(2) = m->z;
}

int
main(int argc, char** argv)
{
  /* 这个节点的作用是接收来自其他控制器节点发布的控制命令,并对四旋翼飞行器进行动力学模拟,发布模拟得到的状态信息。
     它自身不包含控制算法的实现,只是作为一个模拟器,接收控制输入并模拟响应。 */
  ros::init(argc, argv, "quadrotor_simulator_so3");

  ros::NodeHandle n("~");

  ros::Publisher  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 100);  // 发布机器人的odometry(位姿)信息
  ros::Publisher  imu_pub  = n.advertise<sensor_msgs::Imu>("imu", 10);      // 发布机器人的IMU(惯性测量单元)数据
  /* 
    cos::TransportHints().tcpNoDelay()用于设置传输提示,禁用Nagle算法以减少延迟
    在ROS中,通过ros::TransportHints().tcpNoDelay()可以禁用Nagle算法,避免小数据包的延迟传输,从而减小通信延迟。
    这在一些实时性要求较高的机器人控制系统中比较常见。 
  */
  // 接收控制命令，所接受话题在 simulator.xml文件中 被重映射为 “so3_cmd”，此话题来自于文件 so3_control_nodelet.cpp
  ros::Subscriber cmd_sub =
    n.subscribe("cmd", 100, &cmd_callback, ros::TransportHints().tcpNoDelay()); 

  // 下面的 "force_disturbance" 和 "moment_disturbance" 来自于 simulator.xml 文件中 被注释的133-138行
  ros::Subscriber f_sub =
    n.subscribe("force_disturbance", 100, &force_disturbance_callback,
                ros::TransportHints().tcpNoDelay());  // 接收外部力的干扰信息
  ros::Subscriber m_sub =
    n.subscribe("moment_disturbance", 100, &moment_disturbance_callback,
                ros::TransportHints().tcpNoDelay());  // 接收外部力矩的干扰信息

  QuadrotorSimulator::Quadrotor quad;   // 四旋翼
  double                        _init_x, _init_y, _init_z;
  n.param("simulator/init_state_x", _init_x, 0.0);
  n.param("simulator/init_state_y", _init_y, 0.0);
  n.param("simulator/init_state_z", _init_z, 1.0);

  Eigen::Vector3d position = Eigen::Vector3d(_init_x, _init_y, _init_z);
  quad.setStatePos(position);

  double simulation_rate;
  n.param("rate/simulation", simulation_rate, 1000.0);  // 控制频率默认为 1000Hz
  ROS_ASSERT(simulation_rate > 0);

  double odom_rate;
  n.param("rate/odom", odom_rate, 100.0);   // 里程计话题频率, 200Hz，来自文件simulator.xml
  const ros::Duration odom_pub_duration(1 / odom_rate);

  std::string quad_name;
  n.param("quadrotor_name", quad_name, std::string("quadrotor"));

  QuadrotorSimulator::Quadrotor::State state = quad.getState();

  ros::Rate    r(simulation_rate);
  const double dt = 1 / simulation_rate;

  Control control;

  nav_msgs::Odometry odom_msg;
  odom_msg.header.frame_id = "/simulator";
  odom_msg.child_frame_id  = "/" + quad_name;

  sensor_msgs::Imu imu;
  imu.header.frame_id = "/simulator";

  /*
  command.force[0] = 0;
  command.force[1] = 0;
  command.force[2] = quad.getMass()*quad.getGravity() + 0.1;
  command.qx = 0;
  command.qy = 0;
  command.qz = 0;
  command.qw = 1;
  command.kR[0] = 2;
  command.kR[1] = 2;
  command.kR[2] = 2;
  command.kOm[0] = 0.15;
  command.kOm[1] = 0.15;
  command.kOm[2] = 0.15;
  */

  /* 这里应该是控制模块仿真的主要流程 */
  ros::Time next_odom_pub_time = ros::Time::now();  // 初始化 next_odom_pub_time 为当前时间。
  while (n.ok())  // 进入 while 循环,持续到节点退出
  {
    ros::spinOnce();  // 调用 ros::spinOnce() 处理 ROS 消息回调。

    auto last = control;
    control   = getControl(quad, command);  // 获取底层 发动机转速控制量
    for (int i = 0; i < 4; ++i) // 检查 control.rpm 中是否有 NaN 值,即4个旋翼的转速，如果有则用上一次的值替换。
    {
      //! @bug might have nan when the input is legal
      if (std::isnan(control.rpm[i]))
        control.rpm[i] = last.rpm[i];
    }
    quad.setInput(control.rpm[0], control.rpm[1], control.rpm[2],
                  control.rpm[3]);
    quad.setExternalForce(disturbance.f);
    quad.setExternalMoment(disturbance.m);
    quad.step(dt);

    ros::Time tnow = ros::Time::now();

    if (tnow >= next_odom_pub_time)
    {
      next_odom_pub_time += odom_pub_duration;
      odom_msg.header.stamp = tnow;
      state                 = quad.getState();
      stateToOdomMsg(state, odom_msg);
      quadToImuMsg(quad, imu);
      odom_pub.publish(odom_msg);
      imu_pub.publish(imu);
    }

    r.sleep();
  }

  return 0;
}

/* 状态 转 姿态 */
void
stateToOdomMsg(const QuadrotorSimulator::Quadrotor::State& state,
               nav_msgs::Odometry&                         odom)
{
  /*
    Odometry信息通常来自轮式编码器、里程计等外部传感器,用于测量 位移 和 角度。
    Odometry信息通常是通过机器人的轮式里程计、里程传感器等装置测量并计算得出的。
    Odometry信息的主要来源包括以下几种传感器设备:

    轮式里程计(Wheel Odometry)
        通过测量车轮或机器人底盘上的编码器数据,计算出行驶距离和转角。
    里程测距仪(Odometer)
        利用轮胎或其他部件的旋转次数来估算行驶路程。
    惯性测量单元(IMU)
        IMU包含加速度计和陀螺仪,可以测量机器人的线加速度和角速度。
    激光雷达/LIDAR
        激光雷达可以测量环境中物体的位置变化,从而估算机器人的运动轨迹。
        利用SLAM(同步定位与制图)技术可以获得精确的里程计信息。
    视觉里程计(Visual Odometry)
        通过处理摄像头或深度相机拍摄的图像序列,估算机器人的位姿变化。
        这种方法可以获得更加精确的里程信息,但计算量较大。
    全球导航卫星系统(GNSS)
  */
  odom.pose.pose.position.x = state.x(0); // 位置
  odom.pose.pose.position.y = state.x(1);
  odom.pose.pose.position.z = state.x(2);

  Eigen::Quaterniond q(state.R);  // 在 Eigen 库中,可以使用旋转矩阵来直接定义四元数
  odom.pose.pose.orientation.x = q.x();   // 姿态
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();

  odom.twist.twist.linear.x = state.v(0); // 速度
  odom.twist.twist.linear.y = state.v(1);
  odom.twist.twist.linear.z = state.v(2);

  odom.twist.twist.angular.x = state.omega(0);  // 角速度
  odom.twist.twist.angular.y = state.omega(1);
  odom.twist.twist.angular.z = state.omega(2);
}

/* 四元数 转 Imu */
void
quadToImuMsg(const QuadrotorSimulator::Quadrotor& quad, sensor_msgs::Imu& imu)

{
  /*
    IMU信息来自加速度计和陀螺仪,用于测量 线加速度 和 角速度。  
    IMU通常包含三轴加速度计和三轴陀螺仪,这些传感器可以测量机器人或无人机的线性加速度和角速度。通过对这些测量数据进行积分和融合,就可以得到姿态的估计。
  */
  QuadrotorSimulator::Quadrotor::State state = quad.getState();
  Eigen::Quaterniond                   q(state.R);
  imu.orientation.x = q.x();  // 姿态
  imu.orientation.y = q.y();
  imu.orientation.z = q.z();
  imu.orientation.w = q.w();

  imu.angular_velocity.x = state.omega(0);  // 角速度
  imu.angular_velocity.y = state.omega(1);
  imu.angular_velocity.z = state.omega(2);

  imu.linear_acceleration.x = quad.getAcc()[0];   // 线加速度
  imu.linear_acceleration.y = quad.getAcc()[1];
  imu.linear_acceleration.z = quad.getAcc()[2];
}
