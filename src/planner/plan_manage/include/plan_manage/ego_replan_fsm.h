#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <bspline_opt/bspline_optimizer.h>
#include <plan_env/grid_map.h>
#include <ego_planner/Bspline.h>
#include <ego_planner/DataDisp.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>

using std::vector;

namespace ego_planner
{
  /* 这个类实现了一个基于有限状态机的轨迹重规划算法,用于无人机的自主导航。 */
  class EGOReplanFSM
  {

  private:
    /* ---------- flag 状态机 ---------- */
    /* 状态机枚举:
      FSM_EXEC_STATE: 定义了无人机执行状态的枚举,包括初始化、等待目标、生成新轨迹、重规划、执行轨迹和紧急停止等状态。
      TARGET_TYPE: 定义了目标类型的枚举,包括手动目标、预设目标和参考路径。 
    */
    enum FSM_EXEC_STATE     // 在C++中,枚举类型的输出默认是整型类型。当在代码中使用这些枚举值时,它们会被自动转换为对应从0开始计的整数值。
    {
      INIT,                 // 初始化
      WAIT_TARGET,          // 等待目标
      GEN_NEW_TRAJ,         // 生成新轨迹
      REPLAN_TRAJ,          // 重规划
      EXEC_TRAJ,            // 执行轨迹
      EMERGENCY_STOP        // 紧急停止
    };
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,    // 手动目标
      PRESET_TARGET = 2,    // 预设目标
      REFENCE_PATH = 3      // 参考路径
    };

    /* planning utils */
    /* planner_manager_, visualization_, data_disp_: 规划管理器、可视化工具和数据显示器。 */
    EGOPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    ego_planner::DataDisp data_disp_;

    /* parameters */
    int target_type_; // 1 mannual select, 2 hard code 标识当前目标的类型,它对应了前面定义的 enum TARGET_TYPE 中的值
    double no_replan_thresh_, replan_thresh_;
    double waypoints_[50][3];
    int waypoint_num_;
    double planning_horizen_, planning_horizen_time_; // 规划视界通常被设置为感知视界的 1.5 倍,这能进一步确保机器人提前发现并规划避障等动作。
    double emergency_time_;

    /* planning data */
    // trigger_ 标志 实时订阅的目标点 的订阅状态
    // have_target_ 标志 是否具有 目标点   无论是实时订阅的还是预先设定的
    bool trigger_, have_target_, have_odom_, have_new_target_;
    FSM_EXEC_STATE exec_state_;
    int continously_called_times_{0};   // 程序连续执行同一种状态的次数

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
    Eigen::Quaterniond odom_orient_;

    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state 无人机初始（当前）状态
    Eigen::Vector3d end_pt_, end_vel_;                                       // goal state  目标状态
    Eigen::Vector3d local_target_pt_, local_target_vel_;                     // local target state  局部目标状态
    int current_wp_;    // 记录当前所在的航点索引

    bool flag_escape_emergency_;    // 成功"逃脱"了某种紧急情况

    /* ROS utils ROS 相关的订阅者和发布者 */
    ros::NodeHandle node_;  // ROS节点句柄,用于创建和管理该节点的各种资源,如订阅者、发布者等。
    ros::Timer exec_timer_, safety_timer_;  // ROS定时器, exec_timer_应该是用于执行状态机    safety_timer_ 用于 定期检查安全状况
    ros::Subscriber waypoint_sub_, odom_sub_; // ROS订阅者,waypoint_sub_接收来自其他节点发布的新的航点信息;odom_sub_用于订阅里程计话题,获取无人机当前的位置、速度等信息。
    /*
      ROS发布者,用于发布轨迹重规划的结果,供其他节点订阅使用。
        new_pub_可能用于发布新的航点信息,让其他节点知道有新的目标产生。
        bspline_pub_可能用于发布平滑的B样条曲线轨迹,供可视化或其他模块使用。
        data_disp_pub_可能用于发布一些调试信息,供可视化界面或日志记录使用。 
     */
    ros::Publisher replan_pub_, new_pub_, bspline_pub_, data_disp_pub_;

    /* helper functions */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj); // front-end and back-end method
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method
    bool planFromCurrentTraj();

    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();

    void planGlobalTrajbyGivenWps();
    void getLocalTarget();

    /* ROS functions */
    void execFSMCallback(const ros::TimerEvent &e);
    void checkCollisionCallback(const ros::TimerEvent &e);
    void waypointCallback(const nav_msgs::PathConstPtr &msg);
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);

    bool checkCollision();

  public:
    EGOReplanFSM(/* args */)
    {
    }
    ~EGOReplanFSM()
    {
    }

    // 函数在头文件中声明，其具体实现可以在相关cpp文件中找到。
    void init(ros::NodeHandle &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace ego_planner

#endif