#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <quadrotor_msgs/Corrections.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/SO3Command.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <so3_control/SO3Control.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>

/* 
  这个代码定义了一个名为SO3ControlNodelet的类,它继承自nodelet::Nodelet。
  这个类是一个ROS节let(Nodelet),用于四轴飞行器的姿态控制。 
  总的来说,这个类实现了四轴飞行器的姿态控制,通过订阅各种传感器数据并发布SO3控制指令来实现。
  它的主要功能是根据期望状态和反馈状态计算出合适的控制输出,并发布到ROS话题上供四轴飞行器使用。
*/
class SO3ControlNodelet : public nodelet::Nodelet
{
public:
  // 构造函数,初始化成员变量
  SO3ControlNodelet()
    : position_cmd_updated_(false)
    , position_cmd_init_(false)
    , des_yaw_(0)
    , des_yaw_dot_(0)
    , current_yaw_(0)
    , enable_motors_(true)
    , // FIXME
    use_external_yaw_(false)
  {
  }

  void onInit(void);

  /* 
    默认情况下,C++的new操作符无法保证内存对齐。
    为了解决这个问题,Eigen提供了EIGEN_MAKE_ALIGNED_OPERATOR_NEW宏,它会重写类中的operator new和operator delete操作符,
    使得内存分配和释放过程中能够保证内存对齐。
    具体来说,当我们在类中使用EIGEN_MAKE_ALIGNED_OPERATOR_NEW时,
    该类的对象在使用new和delete进行内存分配和释放时,就会使用Eigen提供的内存分配器,确保内存地址满足Eigen数据类型对齐的要求。
    这样做  可以提高Eigen库中涉及矩阵和向量运算的性能,  因为CPU的SIMD指令集可以更好地利用对齐的内存数据进行并行计算。
    对于需要频繁创建和销毁大量Eigen对象的场景,使用EIGEN_MAKE_ALIGNED_OPERATOR_NEW尤其重要。
  */
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  void publishSO3Command(void);
  void position_cmd_callback(
    const quadrotor_msgs::PositionCommand::ConstPtr& cmd);
  void odom_callback(const nav_msgs::Odometry::ConstPtr& odom);
  void enable_motors_callback(const std_msgs::Bool::ConstPtr& msg);
  void corrections_callback(const quadrotor_msgs::Corrections::ConstPtr& msg);
  void imu_callback(const sensor_msgs::Imu& imu);

  SO3Control      controller_;  // SO3Control类型对象,代表了姿态控制器。负责根据期望的姿态和当前的姿态信息,计算出需要输出给电机的控制指令。
  ros::Publisher  so3_command_pub_; // ros::Publisher类型的对象,用于发布计算出的姿态控制指令。
  ros::Subscriber odom_sub_;  // 订阅里程计(Odometry)话题。里程计话题提供了四轴飞行器的当前位置和姿态信息,是控制器计算控制指令的重要输入。
  ros::Subscriber position_cmd_sub_;  // 订阅期望位置
  ros::Subscriber enable_motors_sub_; // 接收电机使能的命令,可以动态地控制电机的使能状态,为系统的安全性提供保障。
  ros::Subscriber corrections_sub_;   // 用于接收外部校正信息,比如来自视觉系统的位置修正等。控制器可以利用这些校正信息来改善控制精度。
  ros::Subscriber imu_sub_; // 用于接收四轴飞行器的IMU(惯性测量单元)数据,包括加速度、角速度等信息。

  bool        position_cmd_updated_, position_cmd_init_;  // 标记是否有新的位置命令更新,以及位置命令是否已经初始化。
  std::string frame_id_;

  Eigen::Vector3d des_pos_, des_vel_, des_acc_, kx_, kv_;
  double          des_yaw_, des_yaw_dot_;     // 期望偏航角,期望偏航角速度
  double          current_yaw_;   // 当前偏航角
  /* 
    enable_motors_这个变量表示是否使能电机的标志位。
    在四轴飞行器的控制系统中,通常会有一个电机使能开关,用于控制电机是否能够正常工作。
    当enable_motors_为true时,表示允许电机正常工作,可以根据控制算法输出的指令来驱动电机。
    而当enable_motors_为false时,则表示禁止电机工作,即使控制算法输出了控制指令,电机也不会执行。
    这个变量的作用主要有两个:
    安全性:当需要暂停四轴飞行器的运行时,可以通过设置enable_motors_为false来立即停止电机,保证安全。
    调试方便:在调试控制算法时,可以暂时禁用电机,防止因算法问题而导致的不安全行为,同时也方便观察控制指令的变化情况。
    总之,enable_motors_这个变量是四轴飞行器控制系统中一个非常重要的安全保护机制,可以灵活地控制电机的使能状态,提高系统的安全性和可靠性。
   */
  bool            enable_motors_;
  bool            use_external_yaw_;  // 是否使用外部提供的偏航角
  double          kR_[3], kOm_[3], corrections_[3];   // kR: 位置环的 P 增益矩阵。 kOm: 速度环的 P 增益矩阵。 corrections: 修正项
  double          init_x_, init_y_, init_z_;
};

/*
  发布上层控制命令
*/
void
SO3ControlNodelet::publishSO3Command(void)
{
  // 调用控制器算法，计算控制命令
  controller_.calculateControl(des_pos_, des_vel_, des_acc_, des_yaw_,
                               des_yaw_dot_, kx_, kv_);

  const Eigen::Vector3d&    force       = controller_.getComputedForce();
  const Eigen::Quaterniond& orientation = controller_.getComputedOrientation();

  /* 
    quadrotor_msgs::SO3Command::Ptr 是一个指向 quadrotor_msgs::SO3Command 消息类型的智能指针。
    quadrotor_msgs::SO3Command 是 ROS 消息(Message)类型之一,定义在 quadrotor_msgs 包中。
        这个消息类型用于表示四轴飞行器(quadrotor)的期望3D旋转姿态控制命令。
    该消息包含以下字段:
        header: ROS标准消息头,包含时间戳和坐标系等元信息。
        orientation: 期望的四元数姿态。
        angular_rates: 期望的角速度。
        kR: 位置环的 P 增益矩阵。
        kOm: 速度环的 P 增益矩阵。
  */
  quadrotor_msgs::SO3Command::Ptr so3_command(
    new quadrotor_msgs::SO3Command); //! @note memory leak?
  so3_command->header.stamp    = ros::Time::now();  // stamp 时间戳
  so3_command->header.frame_id = frame_id_;  // 一般是指定坐标矿建，但这里实际上是一个标识某个特定的 quadrotor 实例的字符串标识符。
  so3_command->force.x         = force(0);
  so3_command->force.y         = force(1);
  so3_command->force.z         = force(2);
  so3_command->orientation.x   = orientation.x();
  so3_command->orientation.y   = orientation.y();
  so3_command->orientation.z   = orientation.z();
  so3_command->orientation.w   = orientation.w();
  for (int i = 0; i < 3; i++)
  {
    so3_command->kR[i]  = kR_[i];
    so3_command->kOm[i] = kOm_[i];
  }
  so3_command->aux.current_yaw          = current_yaw_;
  so3_command->aux.kf_correction        = corrections_[0];
  so3_command->aux.angle_corrections[0] = corrections_[1];
  so3_command->aux.angle_corrections[1] = corrections_[2];
  so3_command->aux.enable_motors        = enable_motors_;
  so3_command->aux.use_external_yaw     = use_external_yaw_;
  so3_command_pub_.publish(so3_command);
}

/* 处理接收到的期望位置命令消息，cmd是 规划出的轨迹 在当前时刻的值，发布于文件traj_server.cpp中 */
void
SO3ControlNodelet::position_cmd_callback(
  const quadrotor_msgs::PositionCommand::ConstPtr& cmd)
{
  des_pos_ = Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);
  des_vel_ = Eigen::Vector3d(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
  des_acc_ = Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y, cmd->acceleration.z);

  /* 
    检查位置控制器的比例增益(kx_)和速度控制器的比例增益(kv_)是否大于 1e-5:
    如果大于 1e-5,则将相应的增益值从消息中提取并赋值到 kx_ 和 kv_ 变量中。 
  */
  /*
    注意到，kx_ 的值由两个地方决定，其可以从参数服务器上获取默认值。
    同时，也可能会根据接收到的话题消息中值来更新。
    也就是说，kx_ 的值会先尝试使用命令消息中的值,如果消息中的值无效,则使用参数服务器上设置的默认值。
    这种设计可以让用户在运行时动态调整控制器的增益,如果没有设置新的增益,则使用之前设置的默认值。 
  */
  if ( cmd->kx[0] > 1e-5 || cmd->kx[1] > 1e-5 || cmd->kx[2] > 1e-5 )
  {
    kx_ = Eigen::Vector3d(cmd->kx[0], cmd->kx[1], cmd->kx[2]);
  }
  if ( cmd->kv[0] > 1e-5 || cmd->kv[1] > 1e-5 || cmd->kv[2] > 1e-5 )
  {
    kv_ = Eigen::Vector3d(cmd->kv[0], cmd->kv[1], cmd->kv[2]);
  }

  des_yaw_              = cmd->yaw;
  des_yaw_dot_          = cmd->yaw_dot;
  position_cmd_updated_ = true;
  position_cmd_init_    = true;

  publishSO3Command();
}

/* 以下的回调函数负责处理从里程计(odometry)中接收到的数据。 */
void
SO3ControlNodelet::odom_callback(const nav_msgs::Odometry::ConstPtr& odom)
{
  // 从 nav_msgs::Odometry 消息中 提取位置和速度信息,分别赋值给 position 和 velocity 变量。
  const Eigen::Vector3d position(odom->pose.pose.position.x,
                                 odom->pose.pose.position.y,
                                 odom->pose.pose.position.z);
  const Eigen::Vector3d velocity(odom->twist.twist.linear.x,
                                 odom->twist.twist.linear.y,
                                 odom->twist.twist.linear.z);

  // 从 nav_msgs::Odometry 消息中 提取姿态的偏航角(yaw),并赋值给 current_yaw_ 变量。
  current_yaw_ = tf::getYaw(odom->pose.pose.orientation);

  // 将 position 和 velocity 设置到控制器(controller_)中。
  controller_.setPosition(position);
  controller_.setVelocity(velocity);

  if (position_cmd_init_)   // 已经接收到过位置命令,已被初始化
  {
    // We set position_cmd_updated_ = false and expect that the
    // position_cmd_callback would set it to true since typically a position_cmd
    // message would follow an odom message. If not, the position_cmd_callback
    // hasn't been called and we publish the so3 command ourselves
    // TODO: Fallback to hover if position_cmd hasn't been received for some
    // time
    if (!position_cmd_updated_)   // 若位置命令未更新,则发布 SO3 指令
      publishSO3Command();    // 这种情况通常是在 position_cmd_callback 尚未被调用的情况下。
    position_cmd_updated_ = false;  // 将 position_cmd_updated_ 标志位恢复为 false,以等待下一次位置命令的更新。
  }
  else if ( init_z_ > -9999.0 )   // 还未接收到过位置命令,检查 init_z_ 是否大于 -9999.0(即是否已经设置了初始位置,其初始值为-10000)
  {
    /* 
      已设置初始位置,则将期望的位置(des_pos_)设置为初始位置,期望的速度(des_vel_)和加速度(des_acc_)设置为 0,
      然后发布 SO3 指令(publishSO3Command())。
    */
    des_pos_ = Eigen::Vector3d(init_x_, init_y_, init_z_);
    des_vel_ = Eigen::Vector3d(0,0,0);
    des_acc_ = Eigen::Vector3d(0,0,0);
    publishSO3Command();
  }
  
}

void
SO3ControlNodelet::enable_motors_callback(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data)
    ROS_INFO("Enabling motors");
  else
    ROS_INFO("Disabling motors");

  enable_motors_ = msg->data;
}

void
SO3ControlNodelet::corrections_callback(
  const quadrotor_msgs::Corrections::ConstPtr& msg)
{
  corrections_[0] = msg->kf_correction;
  corrections_[1] = msg->angle_corrections[0];
  corrections_[2] = msg->angle_corrections[1];
}

void
SO3ControlNodelet::imu_callback(const sensor_msgs::Imu& imu)
{
  const Eigen::Vector3d acc(imu.linear_acceleration.x,
                            imu.linear_acceleration.y,
                            imu.linear_acceleration.z);
  controller_.setAcc(acc);
}

/* 
  ROS节点的初始化函数，设置了节点所需的各种参数,并建立了与其他模块的订阅/发布关系,为四轴飞行器的姿态控制做好了准备。

  这个函数是在ROS的nodelet框架中使用的,它在SO3ControlNodelet类初始化时被调用。

  在ROS中,nodelet是一种特殊的节点,它能够  在同一个进程中运行多个模块,从而提高性能和降低资源占用。
  SO3ControlNodelet就是一个这样的nodelet类。

  onInit()函数是nodelet类中的一个虚函数,当nodelet被加载和初始化时,ROS框架就会自动调用这个函数,类似于main()函数入口。
  通常情况下,开发者会在这个函数中完成以下工作:

  初始化nodelet所需的成员变量。
  订阅和发布所需的ROS话题。
  设置参数服务器上的私有参数。
  创建并初始化nodelet所需的其他对象,如控制器、状态机等。
  启动nodelet所需的任务或线程。

  也就是说,SO3ControlNodelet::onInit(void){}这个函数是在SO3ControlNodelet被加载和初始化时被自动调用的,
  开发者可以在这个函数中完成SO3ControlNodelet的初始化工作。这确保了nodelet在运行前已经完成了所有必要的设置和准备工作。
 */
void
SO3ControlNodelet::onInit(void)
{
  /* 
    这句代码是在ROS(Robot Operating System)中创建一个私有的NodeHandle对象。
    在ROS中,每个节点都有两种类型的NodeHandle:
    公共的NodeHandle:    这种NodeHandle是在节点启动时自动创建的,它用于访问节点的全局参数和话题。
                        使用ros::NodeHandle n;即可获得公共的NodeHandle。
    私有的NodeHandle:    这种NodeHandle需要手动创建。
                        它用于访问节点的私有参数,这些参数是在   节点的私有命名空间下    定义的。
                        使用ros::NodeHandle n(getPrivateNodeHandle());即可获得私有的NodeHandle。
                        这句代码中的getPrivateNodeHandle()是一个ROS提供的函数,它返回当前节点的私有NodeHandle对象。
                        然后,我们将这个私有NodeHandle传递给ros::NodeHandle n的构造函数,创建了一个新的私有NodeHandle对象。
    使用私有NodeHandle的好处是:
                        可以更好地管理节点的参数,避免与其他节点的参数发生冲突。
                        可以更方便地读取和修改节点的私有参数。
                        可以更好地组织节点的话题订阅和发布,使代码更加清晰和模块化。
                        总之,这句代码体现了ROS节点开发中常见的模式,即使用私有NodeHandle来管理节点的私有资源和参数。
   */
  ros::NodeHandle n(getPrivateNodeHandle());

  std::string quadrotor_name;
  n.param("quadrotor_name", quadrotor_name, std::string("quadrotor"));
  frame_id_ = "/" + quadrotor_name;

  /* 
    相关参数在 /src/planner/plan_manage/launch/simulator.xml 文件中设定
    另外，这一文件在执行 simple_run.launch 文件时会被调用
   */
  double mass;
  n.param("mass", mass, 0.5);
  controller_.setMass(mass);

  n.param("use_external_yaw", use_external_yaw_, true);

  /* 相关参数见文件: $(find so3_control)/config/gains_hummingbird.yaml */
  n.param("gains/rot/x", kR_[0], 1.5);
  n.param("gains/rot/y", kR_[1], 1.5);
  n.param("gains/rot/z", kR_[2], 1.0);
  n.param("gains/ang/x", kOm_[0], 0.13);
  n.param("gains/ang/y", kOm_[1], 0.13);
  n.param("gains/ang/z", kOm_[2], 0.1);
  /*
  注意到，kx_ 的值由两个地方决定，其可以从参数服务器上获取默认值。
  同时，也可能会根据接收到的话题消息中值来更新。
  也就是说，kx_ 的值会先尝试使用命令消息中的值,如果消息中的值无效,则使用参数服务器上设置的默认值。
  这种设计可以让用户在运行时动态调整控制器的增益,如果没有设置新的增益,则使用之前设置的默认值。 
  */
  n.param("gains/kx/x", kx_[0], 5.7);
  n.param("gains/kx/y", kx_[1], 5.7);
  n.param("gains/kx/z", kx_[2], 6.2);
  n.param("gains/kv/x", kv_[0], 3.4);
  n.param("gains/kv/y", kv_[1], 3.4);
  n.param("gains/kv/z", kv_[2], 4.0);

  // corrections_的初始化参数设置见文件"$(find so3_control)/config/corrections_hummingbird.yaml"
  n.param("corrections/z", corrections_[0], 0.0);
  n.param("corrections/r", corrections_[1], 0.0);
  n.param("corrections/p", corrections_[2], 0.0);

  n.param("so3_control/init_state_x", init_x_, 0.0);
  n.param("so3_control/init_state_y", init_y_, 0.0);
  n.param("so3_control/init_state_z", init_z_, -10000.0);

  so3_command_pub_ = n.advertise<quadrotor_msgs::SO3Command>("so3_cmd", 10);

  odom_sub_ = n.subscribe("odom", 10, &SO3ControlNodelet::odom_callback, this,
                          ros::TransportHints().tcpNoDelay());
  position_cmd_sub_ =
    n.subscribe("position_cmd", 10, &SO3ControlNodelet::position_cmd_callback,
                this, ros::TransportHints().tcpNoDelay());

  enable_motors_sub_ =
    n.subscribe("motors", 2, &SO3ControlNodelet::enable_motors_callback, this,
                ros::TransportHints().tcpNoDelay());
  corrections_sub_ =
    n.subscribe("corrections", 10, &SO3ControlNodelet::corrections_callback,
                this, ros::TransportHints().tcpNoDelay());

  imu_sub_ = n.subscribe("imu", 10, &SO3ControlNodelet::imu_callback, this,
                         ros::TransportHints().tcpNoDelay());
}

/*
  以下代码是用于将一个自定义的 Nodelet 类注册到 ROS 的插件系统中,以便于其他模块可以动态发现和使用该Nodelet.
*/
#include <pluginlib/class_list_macros.h>
//PLUGINLIB_DECLARE_CLASS(so3_control, SO3ControlNodelet, SO3ControlNodelet,
//                        nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(SO3ControlNodelet, nodelet::Nodelet);
