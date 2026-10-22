// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>
#include <cmath>

namespace ego_planner
{

  // SECTION interfaces for setup and query

  EGOPlannerManager::EGOPlannerManager() {}

  EGOPlannerManager::~EGOPlannerManager() { std::cout << "des manager" << std::endl; }

  /* 完成 规划模块的初始化工作 */
  void EGOPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
  {
    /* read algorithm parameters */
    nh.param("manager/min_goal_distance", min_goal_distance_, 0.2);
    if (!std::isfinite(min_goal_distance_) || min_goal_distance_ <= 0.0)
      min_goal_distance_ = 0.2;
    nh.param("manager/close_goal_speed", close_goal_speed_, 0.0);
    if (!std::isfinite(close_goal_speed_) || close_goal_speed_ < 0.0)
      close_goal_speed_ = 0.0;
    nh.param("manager/max_vel", pp_.max_vel_, -1.0);
    nh.param("manager/max_acc", pp_.max_acc_, -1.0);
    nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
    nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
    nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);    // 相邻控制点之间的距离 0.4
    nh.param("manager/planning_horizon", pp_.planning_horizen_, 5.0);

    // 初始化轨迹 ID 和网格地图
    local_data_.traj_id_ = 0;
    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);

    // 创建一个 BsplineOptimizer 对象 bspline_optimizer_rebound_。
    // 并通过 setParam() 函数设置优化器的参数，这些参数也是从 ROS 参数服务器中读取的。
    bspline_optimizer_rebound_.reset(new BsplineOptimizer);
    bspline_optimizer_rebound_->setParam(nh);
    // 通过 setEnvironment() 函数设置优化器使用的网格地图对象 grid_map_。
    bspline_optimizer_rebound_->setEnvironment(grid_map_);
    // 创建并初始化一个 A* 算法对象 a_star_，它将被用于优化器中。
    bspline_optimizer_rebound_->a_star_.reset(new AStar);
    bspline_optimizer_rebound_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));

    visualization_ = vis;
  }

  // !SECTION Corresponding to the main content of the paper

  // SECTION rebond replanning 反弹式重规划

  bool EGOPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                                        Eigen::Vector3d local_target_pt, Eigen::Vector3d local_target_vel,
                                        bool flag_polyInit, bool flag_randomPolyTraj) {
    // flag_polyInit: have_new_target_ || flag_use_poly_init
    static int count = 0; // 声明并初始化一个静态整数变量count，用于记录函数调用的次数。
    std::cout << endl
              << "[rebo replan]: -------------------------------------" << count++ << std::endl;
    cout.precision(3);  // 设置浮点数的输出精度
    cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() 
         << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
         << endl;

    // Default 0.2 m; inspection recovery uses a tighter configurable goal-input threshold.
    // 此外，还会增加一个计数器continous_failures_count_，用于记录连续失败的次数。
    if ((start_pt - local_target_pt).norm() < min_goal_distance_)
    {
      cout << "Close to goal" << endl;
      /* continous_failures_count_ 直接影响后续生成初始 gl_traj多项式轨迹 随机中间点的偏移量。
      连续失败次数越高,偏移量越大,轨迹的随机性就越强。
      这种动态调整机制可以在简单环境下使用较平滑的轨迹,在复杂环境下增加轨迹的多样性,提高找到可行解的概率。 */
      continous_failures_count_++;  
      return false;
    }
    

    ros::Time t_start = ros::Time::now(); // 获取当前时间，用于计算路径规划所需的时间。
    // t_init: 控制点初始化用时； t_opt: 控制点优化用时； t_refine: 控制点的时间重分配用时
    ros::Duration t_init, t_opt, t_refine;  // ros::Duration为ROS中用于表示时间间隔的类

    /*** STEP 1: INIT ***/
    // 在此步骤中，首先计算起始点和目标点之间的距离((start_pt - local_target_pt).norm())。
    // 如果距离大于 0.1，说明路径点之间的距离较大，代码选择使用 乘以 1.2 是为了稍微增加时间间隔，以减缓速度和加速度的变化，以免超过系统的限制。
    // 如果距离小于等于 0.1，说明路径点之间的距离较小，这时代码选择乘以 5 是为了增加时间间隔，以确保路径段的持续时间不会过短，以满足系统对最小持续时间的要求。
    // V3 inspection can accept sub-0.1 m goals; retain the existing longer sampling interval below.
    double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; 
    // double ts = pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2; 
    // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
    // pp_.ctrl_pt_dist / pp_.max_vel_ 表示相邻B样条控制点之间的最短用时。
    // 当这个比值较小时，意味着路径点之间的 时间差 非常紧密，速度和加速度的变化会非常剧烈。

    // 接下来，定义了一些变量和标志用于存储路径点和起始/结束导数。
    // point_set用于存储路径点，start_end_derivatives用于存储起始/结束导数。
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    /* 
      需要注意，静态变量在程序启动时就会被初始化,并且只会被初始化一次。在这个例子中,flag_first_call 会被初始化为 true。
      在函数的后续调用中,这个静态变量不会被重新初始化,而是保持之前的值。
    */
    static bool flag_first_call = true, flag_force_polynomial = false;
    bool flag_regenerate = false;   // 表示是否需要重新生成或重新计算某些内容
    /* 这段代码的主要作用是生成 一系列初始化多项式轨迹点，对应得到 初始点集合(point_set) 和 起止点处的速度、加速度信息(start_end_derivatives)。
      进而方便后续通过求解最小二乘问题得到 控制点集合
      代码中主要执行了以下操作:
      根据起点、终点及其速度、加速度约束,生成一条最小抖动多项式轨迹或随机多项式轨迹 gl_traj。
      沿着生成的多项式轨迹以固定时间间隔采样,得到一系列点,存入point_set。
      如果不是第一次调用,则从上一次规划的轨迹中提取控制点,并根据期望的控制点距离进行重新采样,生成新的控制点集合point_set。
      计算并存储起点和终点处的速度、加速度信息。
      根据机器人的最大速度和加速度,动态调整采样时间间隔,以确保控制点之间的距离不会过大。
      在此部分,代码并没有考虑周围环境中可能存在的障碍物。它只是基于起点、终点以及机器人的运动学约束,生成了一条初始的曲线轨迹，并且并非真正的B样条曲线。 */
    do 
    {
      point_set.clear();
      start_end_derivatives.clear();
      flag_regenerate = false;

      // flag_polyInit <=> have_new_target_ || flag_use_poly_init  只要四个条件中的任一个条件满足即可
      /* flag_use_poly_init
         表示是否强制使用多项式轨迹作为初始轨迹。在某些情况下,可能需要放弃之前的轨迹,从头开始使用多项式轨迹生成新的初始轨迹。
         flag_force_polynomial
         表示是否强制使用多项式轨迹生成初始轨迹,而不使用之前的轨迹。这可能是由于之前的轨迹过长或其他原因导致的。 */
      if (flag_first_call || flag_polyInit || flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/) 
      {
        // Initial path generated from a min-Jerk traj by order.
        flag_first_call = false;
        flag_force_polynomial = false;    // 当 flag_force_polynomial 为 true 时，进入本条件内，因此需将其重新赋值为 false

        PolynomialTraj gl_traj;
        /* 以下这段代码根据机器人运动学（速度和加速度）初步分配了该局部路径的总时间 time */
        double dist = (start_pt - local_target_pt).norm();  // 局部目标点距离
        // pp_.max_vel_ / pp_.max_acc_ 为 机器人从 零速度 加速 到 最大速度 所需的时间，这一加速过程的平均速度是 pp_.max_vel_/2
        // 这里记为 s = pow(pp_.max_vel_, 2) / pp_.max_acc_，其约为机器人从 零速度 真正加速到 最大速度 所需的距离的2倍，即加速+减速
        // 当 dist < s 时，机器人需要先加速，再减速，因此有1/2 dist = 1/2 a t^2，进而 t = sqrt(dist / a)
        // 当 dist >= s 时，机器人需要先加速至最大速度，保持最大速度运行一段时间，再减速，
        // 因此加减速时间为 2 * pp_.max_vel_ / pp_.max_acc_；匀速时间为 dist - pow(pp_.max_vel_, 2) / pp_.max_acc_
        double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;
        // Give short inspection recovery goals a deliberately slow, smooth seed trajectory.
        // The default zero leaves the original timing unchanged in manual flight mode.
        if (close_goal_speed_ > 0.0 && dist < 0.15)
          time = std::max(time, std::max(1.0, 2.5 * dist / close_goal_speed_));
        /* 得到初始（随机）多项式轨迹 gl_traj */
        if (!flag_randomPolyTraj) // 尝试直接采用单段五次多项式生成轨迹，若成功，则可以节省轨迹规划时间
        {
          gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
        }
        else  // 采用 随机多项式 生成轨迹，即通过插入一个含有随机偏差的中间点 来将轨迹分为两段，再通过多段闭式求解 最小化Jerk生成轨迹
        {
          /* 引入随机中间点的目的是为了增加轨迹的多样性,避免每次生成的轨迹都是相同的最小抖动轨迹。
             随机中间点的偏移量会根据连续失败次数 continous_failures_count_ 动态调整,连续失败次数越高,偏移量越大,轨迹的随机性就越强 */
          // .cross(Eigen::Vector3d(0, 0, 1)) 表示对差向量与Z轴正方向进行叉乘操作。叉乘的结果是一个 与差向量和Z轴都垂直的向量。
          // .normalized() 表示将叉乘结果进行归一化，将其转换为单位向量。
          // horizon_dir与Z轴垂直，意味着horizon_dir平行于x,y平面
          Eigen::Vector3d horizon_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
          // vertical_dir 为一个 与差向量和horizon_dir都垂直的向量。
          Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizon_dir)).normalized();
          // 以上两行代码 生成一个 左手坐标系框架（计算机方向一般都是左手系），
          // 其以horizon_dir为X轴，以vertical_dir为Z轴，以start_pt - local_target_pt方向为z轴
          // 最终表现是：horizon_dir，vertical_dir 和 start_pt - local_target_pt 两两垂直
          // ((double)rand()) / RAND_MAX - 0.5 得到一个 -0.5 和 0.5 之间的随机数
          // -0.978 / (continous_failures_count_ + 0.989) + 0.989
          // 一个动态调整的系数，它与连续失败次数 continous_failures_count_ 有关。
          // 当连续失败次数增加时，该系数会增大，从而增加随机中间点的偏移量。
          /*  该代码段的作用是:
              首先计算起点和目标点的中点坐标。然后在 水平方向 和 垂直方向 （由以上求出的两个方向向量所决定） 分别添加一个随机偏移量。
              偏移量的大小由一个动态调整的系数控制，该系数与连续失败次数成正比。连续失败次数越高，偏移量越大，从而增加了随机中间点的随机性。
              水平方向的偏移量在 [-0.8 * (start_pt - local_target_pt).norm(), 0.8 * (start_pt - local_target_pt).norm()] 范围内。
              垂直方向的偏移量在 [-0.4 * (start_pt - local_target_pt).norm(), 0.4 * (start_pt - local_target_pt).norm()] 范围内。
              通过引入这个随机中间点，可以生成多样化的多段多项式轨迹，有助于在复杂环境中寻找可行解。
              同时，动态调整偏移量的机制也使得算法在简单环境下保持较高的平滑性，在复杂环境下增加随机性，提高了算法的鲁棒性。 */
          /* (start_pt - local_target_pt).norm() * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989)的范围
              应当是[0, 0.8 * (start_pt - local_target_pt).norm()]，
              然后，考虑到前面(((double)rand()) / RAND_MAX - 0.5)会生成一个(-0.5，0.5)的数，
              因此，第二项的系数范围应当是[-0.4 * (start_pt - local_target_pt).norm(), 0.4 * (start_pt - local_target_pt).norm()]   */
          Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizon_dir 
                                               * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir 
                                               * 0.4 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);
          Eigen::MatrixXd pos(3, 3);
          pos.col(0) = start_pt;
          pos.col(1) = random_inserted_pt;
          pos.col(2) = local_target_pt;
          Eigen::VectorXd t(2);
          t(0) = t(1) = time / 2;
          // 最小化Jerk轨迹 对应第五章节  参考博客：http://t.csdnimg.cn/9bRSy
          gl_traj = PolynomialTraj::minJerkTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
        }

        // 依托 gl_traj 得到初始路径轨迹上的一系列点（注意此路径考虑了动力学,但没有考虑环境障碍物，以ts为时间尺度来均匀）
        double t;
        bool flag_too_far;
        ts *= 1.5; // ts will be divided by 1.5 in the next
        do
        {
          ts /= 1.5;
          point_set.clear();  // 每一次外层循环都清空之前循环中的point_set点
          flag_too_far = false;
          Eigen::Vector3d last_pt = gl_traj.evaluate(0);  // evaluate: 求（方程式，公式，函数）在某个点的数值
          for (t = 0; t < time; t += ts)  // 针对之前生成的一整条轨迹，每隔ts时间获取轨迹上的一个点pt
          {
            Eigen::Vector3d pt = gl_traj.evaluate(t);
            if ((pt - last_pt).norm() > pp_.ctrl_pt_dist * 1.5) // 当前 间隔ts 下 某控制点与上一个控制点的距离过大，ctrl_pt_dist为一设定值
            {
              flag_too_far = true;  // 在跳过内层循环之前先标记flag_too_far为true，以便于能够再次进入外层循环，同时需要在再次进入本循环时 缩减 间隔ts 以避免 实际相邻控制点之间的距离过大
              break;  // 因此，下一次循环中首先就是将ts缩减至原来的1/1.5
            }
            last_pt = pt;
            point_set.push_back(pt);
          }
        } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
        t -= ts;  // 跳出循环说明 t > time，此时t-ts对应 小于time 的一个最大的ts的整数倍
        // 将gl_traj在0时刻的速度和加速度,以及local_target_vel和gl_traj在时间t处的加速度,加入start_end_derivatives列表。
        start_end_derivatives.push_back(gl_traj.evaluateVel(0));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
        start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
      }
      /* 局部轨迹重规划的频率通常很高,一般在10-20Hz */
      else {  // Initial path generated from previous trajectory. 从一条已有的轨迹(local_data_.position_traj_)中生成一个初始路径
        /* 执行这部分代码时 一定执行过前面的 if 部分
        从当前时间点开始,沿着之前规划的轨迹采样一系列点,并计算每个采样点到起点的伪弧长(pseudo arc length) 
        注意，在这种情况下,新生成的初始轨迹的起点位置是基于之前规划轨迹在当前时间t_cur处的位置,
        起点速度和加速度也是基于之前轨迹在t_cur时刻的速度和加速度。
        这种做法的目的是在重新规划时,尽可能利用之前规划的轨迹信息,使新轨迹的起点与之前轨迹的当前状态保持连续,
        从而避免轨迹发生突变,提高轨迹的平滑性和可执行性。 */
        double t;
        double t_cur = (ros::Time::now() - local_data_.start_time_).toSec();  // 当前时间，即相对于轨迹起始时间local_data_.start_time_的偏移量(秒)。
        // 计算当前时间t_cur,并从t_cur开始,每隔ts时间获取轨迹上的一个点,存入segment_point列表。
        // 同时计算每个点到前一个点的距离,累加得到pseudo_arc_length列表,表示每个点到起点的伪弧长
        vector<double> pseudo_arc_length; // 存储每个采样点到起点的伪弧长
        vector<Eigen::Vector3d> segment_point;  // 存储采样点的坐标
        pseudo_arc_length.push_back(0.0); // 将pseudo_arc_length的第一个元素初始化为0.0,表示起点的伪弧长为0。
        /* 使用一个循环,从t_cur开始,每隔时间ts沿着local_data_.position_traj_采样一个点,存入segment_point向量中。
        需要注意的是 这个循环没有遍历整条之前规划的轨迹,而只是从 t_cur 开始,遍历了轨迹的剩余部分 */
        // 循环条件中加上1e-3的目的是为了处理浮点数精度问题,避免由于舍入误差导致漏掉轨迹的最后一个采样点。然而,这种做法并不能完全保证一定包含轨迹的终止时刻。
        for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)  // 这里加上1e-3是为了处理浮点数计算时可能出现的小数点精度问题,确保能够包含轨迹的终止时刻。
        {
          segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
          if (t > t_cur)  // 去除第一个采样点
          {
            // 计算它与前一个采样点的欧几里得距离,并将这个距离累加到pseudo_arc_length中,从而得到该采样点到起点的伪弧长。
            // segment_point.size() - 2 得到segment_point向量中倒数第二个元素的索引。
            pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
          }
        }
        t -= ts;

        // 计算从当前轨迹上的最后一个采样点到目标点所需的时间poly_time,最后乘以2的原因是,为了给轨迹的起止点预留一些缓冲时间,使轨迹在起止点处速度为0,运动更加平滑。
        double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;
        /* 这段代码的作用是在重新规划轨迹时,生成一条新的多项式轨迹段,将之前规划轨迹的最后一段与目标点连接起来。 */
        // 如果poly_time <= ts,说明从当前轨迹的最后一个采样点到目标点的距离很短,可以直接使用之前规划的轨迹到达目标点,无需生成新的轨迹段。
        // 如果poly_time > ts,说明从当前轨迹的最后一个采样点到目标点的距离较长,之前规划的轨迹无法直接到达目标点。
        // 这种情况下,需要生成一个新的多项式轨迹段,将当前轨迹的最后一段与目标点连接起来,形成一条完整的新轨迹。
        if (poly_time > ts)
        {
          PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(local_data_.position_traj_.evaluateDeBoorT(t),
                                                                        local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                        local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                        local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

          for (t = ts; t < poly_time; t += ts)
          {
            if (!pseudo_arc_length.empty())
            {
              segment_point.push_back(gl_traj.evaluate(t));
              pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
            }
            else  // 防范某些极端情况下pseudo_arc_length被意外清空
            {
              ROS_ERROR("pseudo_arc_length is empty, return!");
              continous_failures_count_++;
              return false;
            }
          }
        }

        /* 根据之前采样得到的 segment_point 和 pseudo_arc_length 向量，重新对采样点进行参数化,生成新的控制点集 point_set。 */
        double sample_length = 0; // 初始化 sample_length 为 0,表示当前采样长度。
        // cps_dist初始化为 pp_.ctrl_pt_dist * 1.5，表示期望控制点之间的距离,它会在循环中不断缩小。
        double cps_dist = pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next
        size_t id = 0;
        do
        {
          cps_dist /= 1.5;
          point_set.clear();
          sample_length = 0;
          id = 0;
          while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())
          {
            /* 如果当前sample_length落在pseudo_arc_length[id]和pseudo_arc_length[id+1]之间,则通过线性插值计算对应的控制点坐标,
               并将其加入point_set。然后将sample_length增加cps_dist。 */
            if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
            {
              // 根据给定的采样长度sample_length,在segment_point向量中找到对应的插值点坐标
              /* pseudo_arc_length是一个存储了每个采样点到起点的累积弧长的向量。
                 segment_point是一个存储了沿着轨迹采样得到的一系列点的坐标的向量。
                 id是一个索引,用于在pseudo_arc_length和segment_point中定位当前的区间。 */
              point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                  (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
              sample_length += cps_dist;
            }
            else    // 如果sample_length没有落在当前的pseudo_arc_length[id]和pseudo_arc_length[id+1]之间,则增加id以检查下一个区间。
              id++;
          }
          point_set.push_back(local_target_pt);
        } while (point_set.size() < 7); // If the start point is very close to end point, this will help

        start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());

        // 检查生成的初始轨迹点集point_set的大小,如果点数量过多,则强制重新生成多项式轨迹。
        /* 具体来说:
          pp_.planning_horizen_是 规划的时间范围,pp_.ctrl_pt_dist是期望的控制点之间的距离。
          pp_.planning_horizen_ / pp_.ctrl_pt_dist计算了在给定的规划时间范围内,理想情况下应该有多少个控制点。
          将这个理想控制点数乘以3,就得到了一个阈值,表示控制点数量的上限。
          如果point_set.size() > pp_.planning_horizen_ / pp_.ctrl_pt_dist * 3,说明生成的初始轨迹点集point_set中的点数量过多,超出了预期的3倍。
          一旦发生这种情况,代码就会将flag_force_polynomial设置为true,并将flag_regenerate设置为true。
          flag_force_polynomial为true表示强制重新生成多项式轨迹,flag_regenerate为true则会导致外层的do-while循环继续执行。
          这么做的原因可能是:
          如果初始轨迹点集point_set中的点数量过多,可能会导致后续的B-spline优化过程计算量过大,效率低下。
          重新生成多项式轨迹可以得到一个新的、点数较少的初始轨迹,有助于提高后续优化的效率。
          因此,这段代码起到了一个"检查和调节"的作用,确保初始轨迹点集的大小在一个合理的范围内,从而为后续的B-spline优化过程提供一个良好的初始条件,提高整体算法的性能。 */
        if (point_set.size() > pp_.planning_horizen_ / pp_.ctrl_pt_dist * 3) // The initial path is unnormally too long!
        {
          flag_force_polynomial = true;
          flag_regenerate = true;
        }
      }
    } while (flag_regenerate);

    /* 首先通过求解最小二乘问题，得到 B样条 控制点 ctrl_pts */
    Eigen::MatrixXd ctrl_pts;   // ctrl_pts: B样条节点，注意前面point_set中所存储的点是从已有轨迹上采样得到的一系列点,不是B样条节点。
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

    vector<vector<Eigen::Vector3d>> a_star_pathes; 
    // 这段代码的主要作用是根据给定的初始轨迹点和障碍物信息,将初始轨迹分割成多个不同的轨迹段。
    // 并使用A*算法在每个被障碍物分割的轨迹段中寻找一条 局部避障路径。
    // 最终获得一系列 成对的 控制基准点和方向
    // 然后这里面完成控制点的初始化
    a_star_pathes = bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true);

    t_init = ros::Time::now() - t_start;

    static int vis_id = 0;
    visualization_->displayInitPathList(point_set, 0.2, 0);
    visualization_->displayAStarList(a_star_pathes, vis_id);

    t_start = ros::Time::now();

    /*** STEP 2: OPTIMIZE ***/
    bool flag_step_1_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
    cout << "first_optimize_step_success=" << flag_step_1_success << endl;
    if (!flag_step_1_success)
    {
      // visualization_->displayOptimalList( ctrl_pts, vis_id );
      continous_failures_count_++;  // 统计连续失败的次数
      return false;
    }
    //visualization_->displayOptimalList( ctrl_pts, vis_id );

    t_opt = ros::Time::now() - t_start;
    t_start = ros::Time::now();

    /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
    /* 对之前优化得到的B样条曲线轨迹进行时间重分配(refine),以确保轨迹满足机器人的动力学约束(最大速度、加速度等)。 */
    // 使用之前优化得到的控制点ctrl_pts和时间分配ts,构造一条新的B样条曲线pos。
    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
    // 调用pos.setPhysicalLimits函数,设置机器人的最大速度、加速度和可行性容差。
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

    /* 调用pos.checkFeasibility函数,检查当前轨迹是否满足动力学约束。如果不满足,会返回一个ratio值,表示需要对时间进行重新分配的比例。 */
    double ratio;
    bool flag_step_2_success = true;
    if (!pos.checkFeasibility(ratio, false))
    {
      cout << "Need to reallocate time." << endl;

      Eigen::MatrixXd optimal_control_points;
      flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
      if (flag_step_2_success)
        pos = UniformBspline(optimal_control_points, 3, ts);
    }

    if (!flag_step_2_success)
    {
      printf("\033[34mThis refined trajectory hits obstacles. It doesn't matter if appeares occasionally. But if continously appearing, Increase parameter \"lambda_fitness\".\n\033[0m");
      continous_failures_count_++;
      return false;
    }

    t_refine = ros::Time::now() - t_start;

    // save planned results
    updateTrajInfo(pos, ros::Time::now());

    cout << "total time:\033[42m" << (t_init + t_opt + t_refine).toSec() << "\033[0m,optimize:" << (t_init + t_opt).toSec() << ",refine:" << t_refine.toSec() << endl;

    // success. YoY
    continous_failures_count_ = 0;
    return true;
  }

  /* 在发生紧急情况时生成一个静止的 B-样条轨迹,并更新相关的轨迹信息。 */
  bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    /* 
      通常情况下,使用 4 个控制点就足以生成一个三次 B-样条曲线。但在代码中,为什么要使用 6 个控制点呢?

        仔细分析代码,我们可以发现这里有一些特殊的考虑:

        这个代码是在实现一个紧急停止功能,需要从当前位置平滑地停止无人机。为了实现这个平滑停止,需要更细致的曲线控制。
        使用 6 个控制点生成的三次 B-样条曲线,可以提供 3 段三次多项式曲线。
        相比于使用 4 个控制点产生 1 段B-样条曲线,3 段曲线能提供更精细的控制和更平滑的轨迹。
        所以在这个紧急停止的特殊应用场景下,使用 6 个控制点生成三次 B-样条曲线是有其合理性的。这可以帮助无人机实现更精准和更平稳的紧急停止。

        总的来说,虽然通常 4 个控制点就足以生成一段三次 B-样条曲线,但在需要更细致曲线控制的特殊场景下,使用 6 个控制点是一种值得采用的方法。
        这种技术选择体现了针对具体需求进行的权衡和优化。 
      尽管 6 个控制点在轨迹位置上与 4 个控制点相同,但它们在时间控制和加速度变化方面可能存在一些优势,这些优势在紧急停止这样的关键场景下非常有价值。
    */
    Eigen::MatrixXd control_points(3, 6); // 存储 B-样条曲线的 6 个控制点坐标
    for (int i = 0; i < 6; i++)
    {
      // 将输入参数 stop_pos (一个 3D 位置向量)赋值给 control_points 的每一列。这样就生成了 6 个相同的控制点,都位于 stop_pos 处。
      control_points.col(i) = stop_pos; 
    }

    updateTrajInfo(UniformBspline(control_points, 3, 1.0), ros::Time::now());

    return true;
  }

  bool EGOPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {
    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);

    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      points.push_back(waypoints[wp_i]);
    }

    // Calculate the total length of the trajectory
    double total_len = 0;
    total_len += (start_pos - waypoints[0]).norm();
    for (size_t i = 0; i < waypoints.size() - 1; i++)
    {
      total_len += (waypoints[i + 1] - waypoints[i]).norm();
    }

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    double dist_thresh = max(total_len / 8, 4.0);

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1; // 要插入点的个数 + 1，为了方便后续按比例计算插值坐标

        for (int j = 1; j < id_num; ++j)
        {
          // 按比例来插入中间点
          /* 
            记端点为A、B，插入 n 个点,n个点将线段AB分成了 n+1 份
            那么第 i 个点的坐标即为 A + i(B-A)/(n+1) = (1 - i/(n+1))A + (i/(n+1))B
          */
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }
    // inter_points 包含所有原有指定点和新插入点
    inter_points.push_back(points.back());

    // for ( int i=0; i<inter_points.size(); i++ )
    // {
    //   cout << inter_points[i].transpose() << endl;
    // }

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);   // pos:矩阵形式的 inter_points
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1); // 每一段轨迹所对应时间
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    // 应该是考虑第一段和最后一段需要加速和减速
    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minJerkTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = ros::Time::now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  /* 
    尝试根据 当前所订阅到的目标位置 以及 机器人当前（初始）状态 作全局轨迹规划
    此函数 跟 EGOPlannerManager::planGlobalTrajWaypoints() 有轻微区别，此函数要简单些，只涉及两个点
  */
  bool EGOPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {
    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);
    points.push_back(end_pos);

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    const double dist_thresh = 4.0;

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1; // 要插入点的个数 + 1，为了方便后续按比例计算插值坐标

        for (int j = 1; j < id_num; ++j)
        {
          // 按比例来插入中间点
          /* 
            记端点为A、B，插入 n 个点,n个点将线段AB分成了 n+1 份
            那么第 i 个点的坐标即为 A + i(B-A)/(n+1) = (1 - i/(n+1))A + (i/(n+1))B
          */
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }
    // 补充上原来的 终点
    inter_points.push_back(points.back());

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);   // 每一小段轨迹对应的时间序列
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minJerkTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = ros::Time::now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  /* 对给定的B-spline曲线轨迹traj进行时间重分配和优化,以满足动力学约束(如最大速度和加速度) */
  bool EGOPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
  {
    double t_inc; // 时间增量

    Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

    // std::cout << "ratio: " << ratio << std::endl;
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    // 构建 B样条曲线 traj 
    traj = UniformBspline(ctrl_pts, 3, ts);

    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);  // 新B-spline曲线上相邻控制点之间的时间步长t_step。
    /* ref_pts_会被用作优化目标之一,优化算法会尽量使得最终的B-spline曲线经过或接近这些采样点,从而保持优化后的轨迹与原始轨迹的形状相似。 */
    bspline_optimizer_rebound_->ref_pts_.clear();
    // ref_pts_中存储了沿着重新参数化后的B-spline曲线traj,每隔t_step时间采样得到的一系列点的坐标。
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
      bspline_optimizer_rebound_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

    bool success = bspline_optimizer_rebound_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
  }

  void EGOPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now)
  {
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.traj_id_ += 1;
  }

  // 对给定的B-spline曲线进行重新参数化(reparameterization),以满足一定的时间和速度约束。
  void EGOPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
    double time_origin = bspline.getTimeSum();  // 获取原始B-spline曲线的总时间time_origin
    int seg_num = bspline.getControlPoint().cols() - 3;   // B-spline曲线中时间区间(时间段)的数量。
    // double length = bspline.getLength(0.1);
    // int seg_num = ceil(length / pp_.ctrl_pt_dist);

    bspline.lengthenTime(ratio);  // 延长B-spline曲线的时间尺度
    double duration = bspline.getTimeSum();
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;  // 时间增量

    // 重新生成B-spline曲线的控制点
    vector<Eigen::Vector3d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      point_set.push_back(bspline.evaluateDeBoorT(time));
    }
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  }

} // namespace ego_planner
