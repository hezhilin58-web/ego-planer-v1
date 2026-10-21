// fsm: Finite State Machine, 即有限状态机
#include <plan_manage/ego_replan_fsm.h>

namespace ego_planner
{

  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;   // 初始化状态标志
    have_target_ = false;   
    have_odom_ = false;

    /*  fsm param  */
    nh.param("fsm/flight_type", target_type_, -1);  // 飞行类型，即目标是 预先指定(2) 或者 手动给定(1)
    // value="1.5"  也就是说无人机每间隔1.5m规划一次
    nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
    // value="2.0"  与目标点距离一旦小于2.0m，便不再继续规划
    nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);
    // value="7.5"  局部规划的视野范围  规划视界通常被设置为感知视界的 1.5 倍,这能进一步确保机器人提前发现并规划避障等动作。
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/planning_horizen_time", planning_horizen_time_, -1.0);
    nh.param("fsm/emergency_time_", emergency_time_, 1.0);

    /* 在从 ROS 参数服务器上加载一组预先设定的航点坐标。 */
    nh.param("fsm/waypoint_num", waypoint_num_, -1);  // 此代码中预设点数量均为 5，
    // waypoints_[50][3]: 二维数组，表示最多可以存储 50 个三维坐标点。
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);   // to_string(i)将各种数据类型转换为字符串类型
      nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }

    /* initialize main modules */
    // reset()函数可以重新分配指针对象所引用的内存，并将其初始化为新的对象。
    // 这在动态内存管理和对象生命周期控制方面非常有用。
    // 在调用reset()之前，应当确保指针对象不为空，否则可能会导致未定义的行为。
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    // 如果一个指针在调用 reset() 之前状态未定义,那么直接调用 reset() 是非常危险的。
    // 未定义状态的指针可能指向任何地方,甚至可能是空指针。如果这种情况下直接调用 reset()，很可能会导致程序崩溃或出现其他严重的错误。
    // 正确的做法应该是在调用 reset() 之前,先检查指针是否为空,或者确保指针在调用 reset() 之前已经被正确地初始化。
    // 为确保指针对象不为空，可以先检查 planner_manager_ 是否为空指针
    // if (planner_manager_ != nullptr) {
    //     // 如果之前有指向的对象,使用 reset() 释放掉该对象
    //     planner_manager_.reset(new EGOPlannerManager);
    // } else {
    //     // 如果之前是空指针,直接赋值新对象
    //     planner_manager_ = std::make_unique<EGOPlannerManager>();
    // }
    planner_manager_->initPlanModules(nh, visualization_);  // 完成 规划模块的初始化工作

    /* 有限状态机模块 */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this);
    /* 可行性检测以及紧急情况处理 */
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::checkCollisionCallback, this);
    /* 接收 里程计话题消息,提取并保存消息中的位置、速度、姿态等信息 */
    odom_sub_ = nh.subscribe("/odom_world", 1, &EGOReplanFSM::odometryCallback, this);

    bspline_pub_ = nh.advertise<ego_planner::Bspline>("/planning/bspline", 10);
    data_disp_pub_ = nh.advertise<ego_planner::DataDisp>("/planning/data_display", 100);

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)   // 根据实时订阅到的目标点生成全局轨迹
      waypoint_sub_ = nh.subscribe("/waypoint_generator/waypoints", 1, &EGOReplanFSM::waypointCallback, this);
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)  // 根据预先给定的目标点生成全局轨迹
    {
      ros::Duration(1.0).sleep();   // 给系统预留1秒钟的时间, 以确保在规划全局轨迹之前能够成功接收到里程计数据。
      while (ros::ok() && !have_odom_)
        ros::spinOnce();
      planGlobalTrajbyGivenWps();
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  /* 根据给定的航点生成全局轨迹，并将其可视化 */
  void EGOReplanFSM::planGlobalTrajbyGivenWps()
  {
    std::vector<Eigen::Vector3d> wps(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      /* waypoints_数组存储了航点信息，但这里需要将航点信息转换为一个向量形式，以便于在全局轨迹规划中使用 */
      wps[i](0) = waypoints_[i][0];
      wps[i](1) = waypoints_[i][1];
      wps[i](2) = waypoints_[i][2];

      end_pt_ = wps.back();   // 将最后一个航点存储在end_pt_变量中。
    }
    /* 生成全局轨迹 */
    bool success = planner_manager_->planGlobalTrajWaypoints(odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    /* 可视化目标点 */
    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    if (success)
    {
      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      std::vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      // if (exec_state_ == WAIT_TARGET) 注意到此时 状态机 会切换至 GEN_NEW_TRAJ，进而跳出 init 状态
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      // else if (exec_state_ == EXEC_TRAJ)
      //   changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      ros::Duration(0.001).sleep();
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
      ros::Duration(0.001).sleep();
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  /* 根据实时获取的目标点生成全局轨迹 */
  void EGOReplanFSM::waypointCallback(const nav_msgs::PathConstPtr &msg)
  {
    if (msg->poses[0].pose.position.z < -0.1) // 目标点有误
      return;

    cout << "Triggered!" << endl;
    trigger_ = true;
    init_pt_ = odom_pos_;   // 初始位置，未使用

    bool success = false;
    end_pt_ << msg->poses[0].pose.position.x,   // 将订阅到的位置作为 目标位置
               msg->poses[0].pose.position.y, 
               1.0;
    // 尝试根据 当前所订阅到的目标位置 以及 机器人当前（初始）状态 作全局轨迹规划
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  /* 接收 里程计话题消息,提取并保存消息中的位置、速度、姿态等信息 */
  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    // 位置信息
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    // 速度信息
    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    // 姿态信息，四元数
    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
  }

  /* 切换状态机的执行状态,同时记录程序连续执行同一种状态的次数,并输出状态切换的调试信息 */
  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)  // pos_call: 调用此函数的位置(用于输出调试信息)
  {
    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  /* 打印状态机执行状态 */
  void EGOReplanFSM::printFSMExecState() {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  /* 状态机（State Machine）模块，根据所返回的flag值执行不同的任务状态。 */
  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e) {
    /* 
      这个函数是一个回调函数,会在某个时间间隔内被周期性地调用。
      它维护了一个计数器 fsm_num，每执行100次,会打印出当前的执行状态和一些辅助信息。 
    */
    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!trigger_)
        cout << "wait for goal." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
      /* 初始化状态 */
    case INIT:
    {
      if (!have_odom_)  // 没有里程计数据
      {
        return;
      }
      if (!trigger_)    // 未订阅到目标点信息
      {
        return;
      }
      // 订阅到了 里程计数据 和 目标点信息
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        return;
      else  // 一旦 have_target_ == true，则说明 全局轨迹 规划成功
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    /* 一旦exec_state_进入GEN_NEW_TRAJ状态,就说明全局轨迹此时已经是规划成功了的。 */
    case GEN_NEW_TRAJ:
    {
      start_pt_ = odom_pos_;  // 将当前位置设为规划轨迹的起点
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      bool flag_random_poly_init;
      // 执行状态由其他状态更新为 GEN_NEW_TRAJ (产生新轨迹) 时，直接尝试采用单段五次多项式生成轨迹，若成功，则可以极大节省规划时间
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;      // 执行状态由其他状态更新为GEN_NEW_TRAJ时，flag_random_poly_init = false
      else
        flag_random_poly_init = true;       // 执行状态一直为GEN_NEW_TRAJ时，flag_random_poly_init = true

      // 第一个参数 传入 true, 表示不使用之前的轨迹,直接进行新的规划。
      bool success = callReboundReplan(true, flag_random_poly_init);
      // 成功则切换到 EXEC_TRAJ 状态
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else  // 失败则继续尝试生成新轨迹，此时 continously_called_times_ 会自动+1，进而使用随机多项式进行规划，增大规划成功率
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromCurrentTraj())
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_; // info指针指向当前规划好的局部轨迹
      ros::Time time_now = ros::Time::now();    // 当前的绝对ROS时刻
      double t_cur = (time_now - info->start_time_).toSec();  // 以当前局部轨迹起点为原点的时间偏移
      t_cur = min(info->duration_, t_cur);    // 保证 t_cur 处于所规划的局部轨迹时间跨度内，以确保后续能够进行插值运算 

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);  // 当前时刻所对应的局部轨迹上的位置点

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2)
      {           
        /* 
          注意这里实际上说明了当前时刻机器人已抵达 **全局轨迹的末端**  
          因为只要全局轨迹没有执行完，局部轨迹会在抵达局部轨迹末端前不断更新。
          另外，到达全局轨迹末端后便不需要再继续执行或重规划轨迹，因此需要将have_target_设置为false并切换执行状态; 
        */
        have_target_ = false;   // 为后续exec_state_切换为 WAIT_TARGET 做准备

        changeFSMExecState(WAIT_TARGET, "FSM");   // 直到 have_target_ 再次变为 true时，exec_state_再好变为生成新轨迹
        return;
      }
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)  // no_replan_thresh_ = 2.0
      {
        // 这里的 end_pt_ 指代整个轨迹的最终目的地。将当前位置pos与end_pt_的距离进行比较,如果足够近则不重新规划。
        // cout << "near end" << endl;
        return;
      }
      else if ((info->start_pos_ - pos).norm() < replan_thresh_)
      {
        /* 
          如果无人机当前位置pos距离局部轨迹的起点info->start_pos_的距离小于一个阈值replan_thresh_,
          距离局部轨迹起点较近不需要重新规划轨迹,直接返回。
        */
        // cout << "near start" << endl;
        return;
      }
      /* 
        这个else分支主要是为了在无人机执行轨迹的中间阶段,如果当前位置与终点和起点都不够近,
        就触发重新规划局部轨迹的操作,以应对可能的环境变化或轨迹偏移等情况,保证轨迹的有效性和安全性。 
      */
      /*
        根据代码逻辑:
        当无人机还未抵达局部轨迹的末端时(t_cur < info->duration_-1e-2),会进入 else 分支。
          在这个 else 分支中,会触发重新规划局部轨迹的操作(changeFSMExecState(REPLAN_TRAJ, "FSM")).
          也就是说,只要无人机还没有完全执行完当前的局部轨迹,就会一直重新规划局部轨迹,无法真正抵达局部轨迹的末端。
          除非局部轨迹的末端就是全局轨迹的末端,否则无人机是无法完全执行完当前的局部轨迹的。
          因为在中途就会触发重新规划,导致无法抵达局部轨迹的末端。 
      */
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case EMERGENCY_STOP:
    {
      /* 
        flag_escape_emergency_为真,说明还未执行过case EMERGENCY_STOP。 
      */
      if (flag_escape_emergency_) // Avoiding repeated calls  
      {
        callEmergencyStop(odom_pos_);
      }
      /* 
        如果flag_escape_emergency_为假,说明当前正处于紧急停止状态,则检查当前的odom_vel_速度范数是否小于0.1。
        如果小于0.1,说明无人机已经几乎静止,则切换有限状态机的执行状态为GEN_NEW_TRAJ,表示需要重新生成轨迹。
      */
      else
      {
        if (odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);
  }

  /* 
    根据当前轨迹信息, 以机器人当前位置作为起点 尝试重新规划 新轨迹。
    它会逐步尝试不同的重规划策略,直到找到一个可行的方案。
    这种方法可以应对在执行轨迹过程中出现的障碍或其他突发情况。 
  */
  bool EGOReplanFSM::planFromCurrentTraj()
  {
    LocalTrajData *info = &planner_manager_->local_data_; // 从 planner_manager_ 获取当前轨迹的相关数据,保存在 info 变量中。
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - info->start_time_).toSec();  // 计算当前时间相对于轨迹起始时间的时间差 t_cur。

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);  // 无人机当前位置
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur); // 无人机当前速度
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur); // 无人机当前加速度

    /*
      根据以下代码可以推断
      对于函数bool ego_planner::EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
        第一个参数 传入 false 时,表示尝试基于之前规划的轨迹来做调整；当传入 true 时,表示不使用之前的轨迹,而是直接进行新的规划。
        第二个参数 传入 false 时,表示使用五次多项式来进行新的轨迹规划；当传入 true 时,表示使用随机多项式来进行新的轨迹规划。
    */
    bool success = callReboundReplan(false, false); // 首先尝试基于之前规划的轨迹来做调整

    if (!success)
    {
      success = callReboundReplan(true, false); // 不行，则尝试直接使用五次多项式重新规划
      //changeFSMExecState(EXEC_TRAJ, "FSM");
      if (!success)
      {
        success = callReboundReplan(true, true);  // 再不行，则尝试使用随机多项式来作规划
        if (!success)
        {
          return false;
        }
      }
    }

    return true;
  }

  /* 定期检查当前规划的局部轨迹是否会发生碰撞,并根据检查结果做出相应的响应,包括尝试重新规划或者触发紧急停车。 */
  void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {
    // info 为 结构体指针
    LocalTrajData *info = &planner_manager_->local_data_;   // planner_manager_ 为 EGOPlannerManager类指针，LocalTrajData 为 结构体类型
    auto map = planner_manager_->grid_map_;

    /* 
      如果当前执行状态是 WAIT_TARGET 或者轨迹的起始时间小于 1e-5 秒,则直接返回,不进行任何检查。 
      不过，需要注意的是，在INIT状态下，planner_manager_->local_data_相关数据均违背定义，因此有可能导致checkCollisionCallback()函数 报错
        因此，最好做以下处理（修改）:
          1. 在进入INIT状态时,对local_data_中的所有相关成员变量进行适当的初始化,确保它们有合理的已知值。
          或者2. 在checkCollisionCallback()函数的开头,添加额外的检查,如果发现local_data_未初始化,直接返回而不执行后续代码。 
    */
    // if (exec_state_ == INIT || exec_state_ == WAIT_TARGET || info->start_time_.toSec() < 1e-5)
    if (exec_state_ == WAIT_TARGET || info->start_time_.toSec() < 1e-5)
      return;

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = (ros::Time::now() - info->start_time_).toSec();  // 相对于 目前所规划的局部轨迹 的起始时刻的时间偏差
    double t_2_3 = info->duration_ * 2 / 3;   // 考虑到所规划的局部轨迹长度是感知视界的1.5倍，因此只检查所规划轨迹的前2/3即可
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      if (map->getInflateOccupancy(info->position_traj_.evaluateDeBoorT(t)))  // 存在碰撞
      {
        if (planFromCurrentTraj()) // Make a chance 尝试以当前状态作重规划，若成功，则切换状态至执行
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        else    // 从当前状态 无法 规划出 安全可行的轨迹，则要考虑减速
        {
          // 如果 当前时刻t_cur 距离 将发生碰撞的时间点t 小于 emergency_time_(0.8 秒), 则触发紧急停车,将执行状态切换到 EMERGENCY_STOP。
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            ROS_WARN("Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else  // 当前时刻t_cur 距离 将发生碰撞的时间点t 大于 emergency_time_,执行重规划
          {
            //ROS_WARN("current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  // 其主要作用是调用轨迹规划器进行重规划,并发布和可视化生成的B样条曲线轨迹。
  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj) {
    getLocalTarget();   // 获取局部目标点。

    // 调用planner_manager_->reboundReplan()函数进行轨迹重规划。
    // 这个函数的参数包括起点位置、速度、加速度,以及局部目标点位置、速度,是否使用多项式初始化轨迹,是否使用随机多项式轨迹等。
    // 函数返回一个布尔值,表示规划是否成功。
    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, 
                                        local_target_pt_, local_target_vel_, 
                                        (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false;   // 重置 标志 为false

    cout << "final_plan_success=" << plan_success << endl;

    if (plan_success)
    {
      // 从planner_manager_中获取本地数据local_data_的引用。
      auto info = &planner_manager_->local_data_;

      /* publish traj */
      // 将本地数据中的B样条曲线信息(阶数、起始时间、轨迹ID、控制点、节点向量)复制到bspline对象中。
      ego_planner::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      // reserve(预留)是std::vector的一个成员函数,它的作用是预先分配足够的内存空间,以容纳指定数量的元素,而不实际构造这些元素。
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }
      // 发布B样条信息,包括 样条阶数、开始时间、轨迹编号、控制点和节点向量
      bspline_pub_.publish(bspline);

      visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);
    }

    return plan_success;
  }

  /* 在紧急情况下停止无人机的航线重规划,并发布当前的飞行轨迹。 */
  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)  // 函数的输入参数是 stop_pos，表示要停止航线重规划的目标位置。
  {
    planner_manager_->EmergencyStop(stop_pos);  // 设置紧急停止状态并更新相关的航线规划数据

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    ego_planner::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_.publish(bspline);

    return true;
  }

  /* 根据全局轨迹计算 局部目标点local_target_pt_ 和 局部目标速度local_target_vel_ ,作为后续局部轨迹规划的目标。 */
  void EGOReplanFSM::getLocalTarget() {
    double t;

    // 设置时间步长，确保机器人每隔t_step时间移动的距离不会超过 planning_horizen_ / 20
    // planning_horizen_是局部规划的视野范围,即期望的局部轨迹长度。
    double t_step = planning_horizen_ / 20 / planner_manager_->pp_.max_vel_;
    // dist_min will store the minimum distance from the start point to the global trajectory
    // dist_min_t will store the corresponding time.
    double dist_min = 9999, dist_min_t = 0.0;
    /* 从 last_progress_time_ 开始遍历，可以节省不必要的花费 */
    for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist = (pos_t - start_pt_).norm();

      // 当前点距离起点很远,但是时间却非常接近上次更新 last_progress_time_ 的时间。这可能意味着 last_progress_time_ 的更新存在问题。
      if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizen_)
      {
        // todo
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        return;
      }
      if (dist < dist_min)
      {
        dist_min = dist;  // dist_min对应得到距离当前局部轨迹起点start_pt_最近的点
        dist_min_t = t;   // 进而存储last_progress_time_为其对应全局轨迹时间，方便下一次从当前时刻开始遍历全局轨迹，避免不必要的时间耗费
      }
      /* 
        注意，本身规划视界通常也都是被设置为感知视界的 1.5 倍,这能进一步确保机器人提前发现并规划避障等动作。
        当 dist >= planning_horizen_ 时,表示机器人已经移动到了规划视界之外,也就是说机器人已经看到了足够远的全局轨迹。
        在这种情况下,代码更新 local_target_pt_ 为当前位置 pos_t
       */
      if (dist >= planning_horizen_) 
      {
        local_target_pt_ = pos_t;
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        break;
      }
    }
    if (t > planner_manager_->global_data_.global_duration_) // Last global point
    {
      local_target_pt_ = end_pt_;
    }

    /* 
      终点和局部目标点之间的距离 小于 无人机从最大速度减速至静止所需的最小距离,那么就需要无人机在当前局部目标点位置处停下来
    */
    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      // local_target_vel_ = (end_pt_ - init_pt_).normalized() * planner_manager_->pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((planner_manager_->pp_.max_vel_*planner_manager_->pp_.max_vel_)/(2*planner_manager_->pp_.max_acc_)));
      // cout << "A" << endl;
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(t);
      // cout << "AA" << endl;
    }
  }

} // namespace ego_planner
