#include <Eigen/Eigen>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>

/* 这段代码是一个ROS节点的示例,用于向四旋翼无人机发送位置、速度和加速度控制命令。
   它通过发布quadrotor_msgs/PositionCommand类型的消息来控制无人机的运动。 */
int main(int argc, char **argv)
{
  // 初始化ROS系统,并告诉ROS要运行的节点的名字是"quad_sim_example"。
  ros::init(argc, argv, "quad_sim_example");
  // 创建了ROS中的NodeHandle对象,是ROS通信的主要入口点。NodeHandle对象允许你在节点中发布或订阅话题(topics)、服务(services)等。
  // nh("~")表示NodeHandle将会在 私有命名空间 下工作,即命名空间为"~节点名"。这样可以避免命名冲突。
  ros::NodeHandle nh("~");

  // 创建一个发布者,用于发布/position_cmd话题
  ros::Publisher cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 10);

  // 等待2秒,确保发布者已经准备就绪
  ros::Duration(2.0).sleep();

  while (ros::ok())
  {

    /*** example 1: position control ***/
    /* \033[42m 是一个ANSI转义序列,表示设置输出文本的背景颜色为绿色。
       "Position Control to (2,0,1) meters" 是要输出的字符串。
       \033[0m 是另一个ANSI转义序列,表示重置终端的文本格式(颜色、加粗等)为默认值。 */
    std::cout << "\033[42m"
              << "Position Control to (2,0,1) meters"
              << "\033[0m" << std::endl;
    // 发送500个位置命令,目标位置为(2.0, 0.0, 1.0)米
    for (int i = 0; i < 500; i++)
    {
      quadrotor_msgs::PositionCommand cmd;
      cmd.position.x = 2.0;
      cmd.position.y = 0.0;
      cmd.position.z = 1.0;
      cmd_pub.publish(cmd);

      ros::Duration(0.01).sleep();
      ros::spinOnce();   // 处理回调函数
    }

    /*** example 2: velocity control ***/
    std::cout << "\033[42m"
              << "Velocity Control to (-1,0,0) meters/second"
              << "\033[0m" << std::endl;
    for (int i = 0; i < 500; i++)
    {
      quadrotor_msgs::PositionCommand cmd;
      cmd.position.x = std::numeric_limits<float>::quiet_NaN(); // lower-order commands must be disabled by nan
      cmd.position.y = std::numeric_limits<float>::quiet_NaN(); // lower-order commands must be disabled by nan
      cmd.position.z = std::numeric_limits<float>::quiet_NaN(); // lower-order commands must be disabled by nan
      cmd.velocity.x = -1.0;
      cmd.velocity.y = 0.0;
      cmd.velocity.z = 0.0;
      cmd_pub.publish(cmd);

      ros::Duration(0.01).sleep();
      ros::spinOnce();
    }

    /*** example 3: accelleration control ***/
    std::cout << "\033[42m"
              << "Accelleration Control to (1,0,0) meters/second^2"
              << "\033[0m" << std::endl;
    for (int i = 0; i < 500; i++)
    {
      /* 通过设置对应的命令为NaN值,可以禁用低阶控制(如加速度控制时禁用位置和速度控制)。 */
      quadrotor_msgs::PositionCommand cmd;
      cmd.position.x = std::numeric_limits<float>::quiet_NaN(); // lower-order commands must be disabled by nan
      cmd.position.y = std::numeric_limits<float>::quiet_NaN(); // lower-order commands must be disabled by nan
      cmd.position.z = std::numeric_limits<float>::quiet_NaN(); // lower-order commands must be disabled by nan
      cmd.velocity.x = std::numeric_limits<float>::quiet_NaN();
      cmd.velocity.y = std::numeric_limits<float>::quiet_NaN();
      cmd.velocity.z = std::numeric_limits<float>::quiet_NaN();
      cmd.acceleration.x = 1.0;
      cmd.acceleration.y = 0.0;
      cmd.acceleration.z = 0.0;
      cmd_pub.publish(cmd);

      ros::Duration(0.01).sleep();
      ros::spinOnce();
    }

  }

  return 0;
}
