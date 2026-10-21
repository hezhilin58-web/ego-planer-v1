/* 
  这段代码是一个 ROS 节点,用于处理无人机轨迹规划和控制的逻辑。
  这段代码实现了一个基于 B 样条轨迹的无人机位置控制器,包括轨迹解析、偏航角计算和控制命令发布等功能。
 */
#include "bspline_opt/uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "ego_planner/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float64.h"
#include <algorithm>
#include <cmath>
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>

ros::Publisher pos_cmd_pub;           // 发布位置命令

quadrotor_msgs::PositionCommand cmd;  // 存储位置命令的具体内容,包括位置、速度、加速度、偏航角和偏航角导数等
double pos_gain[3] = {0, 0, 0};       // 存储位置的控制增益
double vel_gain[3] = {0, 0, 0};       // 存储速度的控制增益

using ego_planner::UniformBspline;

bool receive_traj_ = false;           // 表示是否已经接收到轨迹数据
vector<UniformBspline> traj_;         // 存储轨迹的向量,包含位置、速度和加速度3条B样条曲线。
double traj_duration_;                // 轨迹的持续时间
ros::Time start_time_;                // 轨迹开始的时间
int traj_id_;                         // 当前轨迹的ID

// yaw control
double last_yaw_, last_yaw_dot_;      // 上一时刻的偏航角 & 上一时刻的偏航角导数。
double time_forward_;                 // 计算偏航角的前瞻时间

// Inspection heading requests affect only the final hover, never a flying trajectory.
bool inspection_yaw_active_ = false;
double inspection_yaw_ = 0.0;
double inspection_yaw_rate_ = 0.5;
void inspectionYawCallback(const std_msgs::Float64::ConstPtr &msg)
{
  inspection_yaw_active_ = std::isfinite(msg->data);
  if (inspection_yaw_active_)
    inspection_yaw_ = msg->data;
}

/* 
  解析从 ego_planner/Bspline 话题接收到的轨迹数据,并将其存储在内部的数据结构中。
 */
void bsplineCallback(ego_planner::BsplineConstPtr msg)
{
  // parse pos traj
  /* 从接收到的 ego_planner::BsplineConstPtr 消息中提取位置轨迹的点集 pos_pts 和时间节点 knots。 */
  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());  // 位置 B样条控制点

  Eigen::VectorXd knots(msg->knots.size()); // 节点向量
  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
  }

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  /*
    对应文件 uniform_bspline.cpp 中的 
    void UniformBspline::setUniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval) 
  */
  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);  // 正是由于 这一行代码将knots单独设置，所以上一句代码的 interval 无关紧要

  // parse yaw traj

  // Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  // for (int i = 0; i < msg->yaw_pts.size(); ++i) {
  //   yaw_pts(i, 0) = msg->yaw_pts[i];
  // }

  //UniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.push_back(pos_traj);                        // 添加traj_的第0列 为 pos_traj （UniformBspline类型）
  traj_.push_back(traj_[0].getDerivative());        // 添加traj_的第1列 为 pos_traj的一阶导数
  traj_.push_back(traj_[1].getDerivative());        // 添加traj_的第0列 为 pos_traj的二阶导数

  traj_duration_ = traj_[0].getTimeSum();           // 轨迹持续时间

  receive_traj_ = true;
}

/* 计算航向角（yaw）及其导数(yawdot) */
std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, ros::Time &time_now, ros::Time &time_last)
{
  /* 
    constexpr 是 C++11 中引入的一个关键字,它用于声明一个"编译时常量表达式"。
    简单地说,constexpr 修饰的变量或函数,其值或返回值必须在编译时就能确定下来。这意味着这些变量或函数的计算在编译阶段就可以完成,从而提高了程序的运行效率。
    一些 constexpr 的特点包括:
    constexpr 变量必须在声明时就初始化,且初始化表达式必须是常量表达式。
    constexpr 函数的返回值和函数体都必须是常量表达式。
    编译器会尽可能将 constexpr 修饰的变量和函数的求值过程移到编译时完成,从而提高运行时的效率。
    constexpr 可以用于数组大小的声明、常量初始化等场景,以增强程序的性能和安全性。
    总之,constexpr 是 C++11 引入的一个非常有用的特性,它可以帮助程序员编写更高效和更安全的代码。合理使用 constexpr 可以大大提升程序的运行效率。 
  */
  constexpr double PI = 3.1415926;
  constexpr double YAW_DOT_MAX_PER_SEC = PI;  // 航向角的最大变化速度,单位是弧度每秒。
  // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
  std::pair<double, double> yaw_yawdot(0, 0);
  double yaw = 0;
  double yawdot = 0;

  // 计算当前时间点及预测时间点的位置差
  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_ ? 
                        traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos : traj_[0].evaluateDeBoorT(traj_duration_) - pos;
  /* 
    如果 dir 的模长大于 0.1,说明当前位置与预测位置还有一定距离,则使用 atan2 函数计算 dir 向量在 xy 平面上的角度。
    atan2 函数可以得到一个范围在 [-π, π] 之间的角度值,表示当前位置相对于预测位置或轨迹结束位置的航向角。
    如果 dir 的模长小于等于 0.1,说明当前位置与预测位置非常接近,此时使用上一次计算的航向角 last_yaw_。
    使用上一次的航向角可以保证航向角的平滑过渡,避免出现突变。 
    Eigen 库默认使用的右手坐标系。
  */
  double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
  double max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now - time_last).toSec(); // 当前时间间隔下最大的航向角变化量
  /* 
    当前期望的航向角与上次的航向角之间的差值超过了180度。
    这种情况通常发生在飞行器绕z轴旋转超过180度的时候,比如从正北方向逆时针旋转到正南方向。
    代码通过判断yaw - last_yaw_与PI的大小关系,来确定航向角的变化是顺时针还是逆时针,并对yaw和yawdot(航向角变化速度)进行相应的计算和限幅。
    这样做的目的是为了保证航向角的连续性,避免出现由于数值计算等原因导致的航向角突变。
   */
  // yaw \in (-pi,pi)
  if (yaw_temp - last_yaw_ > PI)  // 飞行器 顺时针 穿越x负半轴，其中一种情况如下 last_yaw \in (-pi,-pi/2), yaw_temp \in (pi/2,pi)
  { // 此时 飞行器 真实的旋转角度 需要 -2pi，并且顺时针旋转，此时结果一定为 负，负代表旋转方向
    if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)    // 实际旋转角度（标量）>当前时间间隔下最大的航向角变化量
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)    // yaw \in (-pi,pi)，因此需要 + 2pi
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else    // 实际旋转角度（标量）< 当前时间间隔下最大的航向角变化量
    {
      yaw = yaw_temp;
      /* 
        当飞行器需要从第三象限顺时针旋转到第二象限时,yaw角需要跨越 -π和π 这个临界点。
        作者将yawdot直接设置为-YAW_DOT_MAX_PER_SEC,主要是希望飞行器能够快速、平滑地跨越这个π的临界点,实现大角度旋转。具体来说:
        如果直接根据yaw_temp和last_yaw_的差值计算yawdot,在跨越π时,可能会得到一个瞬时很大的yawdot值,导致航向角的变化过于剧烈。
        将yawdot设置为-YAW_DOT_MAX_PER_SEC,可以确保yaw以最大负变化速率"快速逆向变化",从而快速跨越π这个临界点。
        通过这种方式,飞行器可以快速平滑地完成绕z轴180度以上的大角度旋转,避免了由于数值计算等原因导致的航向角突变。

        当飞行器需要绕z轴旋转超过180度时,如从第三象限顺时针旋转到第二象限,yaw_temp和last_yaw_就会分别位于不同的象限,它们的差值会接近π或-π。
        如果此时由于数值计算精度问题或时间间隔较小,导致计算出的yawdot值瞬时变得非常大,那么就可能会引起飞行器的剧烈运动,影响控制的稳定性和连续性。
        为了避免这种情况,代码采取了一种保守的做法,即直接将yawdot设置为一个固定的最大值(-YAW_DOT_MAX_PER_SEC),而不是直接根据差值计算。
        这种做法虽然可能会牺牲一些精度,但能够确保航向角变化速率不会出现瞬时的极大值,从而提高控制系统的稳定性和鲁棒性。
      */
      if (yaw - last_yaw_ > PI) 
        yawdot = -YAW_DOT_MAX_PER_SEC;    // 总之应该就是想 快速 跨越 pi和-pi 这个容易出现 奇异值 的 不安全位置
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }
  else if (yaw_temp - last_yaw_ < -PI) // 飞行器 逆时针 穿越x负半轴，需要 跨越 pi和-pi 这个容易出现 奇异值 的 不安全位置
  { // 此时 飞行器 真实的旋转角度 需要 +2pi
    if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)    // 实际旋转角度（标量）>当前时间间隔下最大的航向角变化量
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }
  else
  {   // 一般情况，不需要跨越 pi和-pi 这个容易出现 奇异值 的 不安全位置
    if (yaw_temp - last_yaw_ < -max_yaw_change)    // 实际旋转角度（标量）> 当前时间间隔下最大的航向角变化量
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else if (yaw_temp - last_yaw_ > max_yaw_change)    // 实际旋转角度（标量）> 当前时间间隔下最大的航向角变化量
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else    // 实际旋转角度（标量）< 当前时间间隔下最大的航向角变化量
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)   // 快速 跨越 pi和-pi 这个容易出现 奇异值 的 不安全位置
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else if (yaw - last_yaw_ < -PI)   // 快速 跨越 pi和-pi 这个容易出现 奇异值 的 不安全位置
        yawdot = YAW_DOT_MAX_PER_SEC;
      else  // 不涉及 -pi和pi 这个位置
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }

  /* 
    对计算出的航向角yaw和航向角变化速率yawdot进行 低通滤波,使它们的变化更加平滑。
    代码中的低通滤波处理是为了使yaw和yawdot的变化更加平滑,避免出现突变。
    但是,当yaw与last_yaw的差值超过max_yaw_change时,代码已经通过其他方式(如限制最大变化速率等)来确保yaw和yawdot的平滑变化。
    因此,只有当yaw与last_yaw的差值在一个较小的范围内时,才需要使用低通滤波来进一步平滑它们的变化。当差值较大时,低通滤波的作用就不那么必要了。
   */
  if (fabs(yaw - last_yaw_) <= max_yaw_change)
    yaw = 0.5 * last_yaw_ + 0.5 * yaw; // nieve LPF "naive low-pass filter",即简单的低通滤波器。这样可以抑制yaw的高频变化,使其变化更加平滑。
  yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
  
  last_yaw_ = yaw;
  last_yaw_dot_ = yawdot;

  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  return yaw_yawdot;
}

/* 
  根据已经接收到的轨迹数据,计算并发布当前时间点的目标（期望）位置、目标（期望）速度、目标（期望）加速度和目标（期望）偏航角命令。
*/
void cmdCallback(const ros::TimerEvent &e)
{
  /* no publishing before receive traj_ */
  if (!receive_traj_)
    return;

  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - start_time_).toSec();

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0); // yaw: 偏航角

  /* 计算飞行器在给定轨迹上的位置、速度、加速度以及航向角和航向角速度。 */
  /* 
    以下这行代码会在程序启动时执行,用于获取并存储程序启动时的时间戳作为time_last的初始值。
    具体来说:
    time_last被声明为静态(static)变量,这意味着它在程序的整个生命周期中只会被初始化一次。
    time_last的初始值是通过ros::Time::now()获取的,即程序启动时的当前时间戳。
    这行代码位于cmdCallback函数内部,但由于time_last是静态变量,它的初始化发生在函数第一次被调用之前,也就是程序启动时。
    之后，time_last都保持为程序启动时赋予的初始值,除非代码中另有赋值操作。
  */
  static ros::Time time_last = ros::Time::now();  // 静态变量的生命周期跨越整个程序运行期间,即使在函数调用结束后,它的值也会被保留。
  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);

    /*** calculate yaw ***/
    yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last);
    /*** calculate yaw ***/

    double tf = min(traj_duration_, t_cur + 2.0);   // tf = 当前时间 t_cur 往后 2 秒内的最小值,不会超过整个轨迹的持续时间 traj_duration_。
    pos_f = traj_[0].evaluateDeBoorT(tf); // pos_f 是用来表示飞行器在未来某个时刻的预测位置
  }
  else if (t_cur >= traj_duration_)   // 如果当前时间 t_cur 已经超过了轨迹持续时间 traj_duration_,说明轨迹已完成。
  { // 此时设置位置 pos 为轨迹结束时的位置, 速度 vel 和加速度 acc 为 0。
    /* hover when finish traj_ */
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();

    yaw_yawdot.first = last_yaw_;
    yaw_yawdot.second = 0;

    if (inspection_yaw_active_)
    {
      const double dt = std::min(0.1, std::max(0.0, (time_now - time_last).toSec()));
      const double error = std::atan2(std::sin(inspection_yaw_ - last_yaw_),
                                      std::cos(inspection_yaw_ - last_yaw_));
      const double step = std::clamp(error, -inspection_yaw_rate_ * dt,
                                    inspection_yaw_rate_ * dt);
      yaw_yawdot.first = std::atan2(std::sin(last_yaw_ + step), std::cos(last_yaw_ + step));
      yaw_yawdot.second = dt > 1e-6 ? step / dt : 0.0;
      last_yaw_dot_ = yaw_yawdot.second;
    }

    pos_f = pos;
  }
  else
  {
    cout << "[Traj server]: invalid time." << endl;
  }
  time_last = time_now;

  cmd.header.stamp = time_now;  // 将当前时间戳赋值给 cmd 消息的时间戳 (header.stamp)。
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);

  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);

  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);

  cmd.yaw = yaw_yawdot.first;
  cmd.yaw_dot = yaw_yawdot.second;

  last_yaw_ = cmd.yaw;

  pos_cmd_pub.publish(cmd);
}

/* 
  这段代码是一个 ROS 节点,用于处理无人机轨迹规划和控制的逻辑。
  这段代码实现了一个基于 B 样条轨迹的无人机位置控制器,包括轨迹解析、偏航角计算和控制命令发布等功能。
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  /* 
    对于以下几行代码，只是初步创建和初始化这些回调函数，
    其具体的发布操作都还没有真正执行。
    只有当 ros::spin() 被调用,程序进入事件循环后,ROS 系统才会根据事件的触发来自动执行这些回调函数和发布操作。 
  */
  // 对应文件 planner_manager.cpp 文件的 bspline_pub_ = nh.advertise<ego_planner::Bspline>("/planning/bspline", 10);
  ros::Subscriber bspline_sub = node.subscribe("planning/bspline", 10, bsplineCallback);
  ros::Subscriber inspection_yaw_sub = node.subscribe("inspection/yaw", 1, inspectionYawCallback);
  nh.param("inspection_yaw_rate", inspection_yaw_rate_, 0.5);
  if (!std::isfinite(inspection_yaw_rate_) || inspection_yaw_rate_ <= 0.0)
    inspection_yaw_rate_ = 0.5;

  pos_cmd_pub = node.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);

  ros::Timer cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);

  /* 
    以下代码只会执行一次。这段代码出现在一个 ROS 节点的初始化过程中。
  */
  /* control parameter */ 
  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  // 文件中所设定的前瞻值time_forward_为 1.0 <param name="traj_server/time_forward" value="1.0" type="double"/>
  nh.param("traj_server/time_forward", time_forward_, -1.0);
  last_yaw_ = 0.0;
  last_yaw_dot_ = 0.0;

  // 这 1 秒的休眠时间是为了确保节点的参数和初始化操作都能够顺利完成,为后续的正常运行做好准备。
  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");

  /* 
    ros::spin() 是一个阻塞式的函数调用,它会一直等待并处理 ROS 话题消息,直到节点被停止或者程序退出。
    因此,除非节点被显式地停止,否则这段代码中的 ros::spin() 调用是一个无限循环,不会停止执行。

    在进入 ros::spin() 之前,这段代码已经:
    1.订阅了 "planning/bspline" 话题,准备好 bsplineCallback 函数。
    2.创建了向 "/position_cmd" 话题发布消息的发布者。
    3.创建了一个 10ms 周期的定时器,准备好了 cmdCallback 函数。
    但是这些回调函数和发布操作都还没有真正执行。只有当 ros::spin() 被调用,程序进入事件循环后,ROS 系统才会根据事件的触发来自动执行这些回调函数和发布操作。
   */
  ros::spin();

  return 0;
}
