#include "bspline_opt/bspline_optimizer.h"
#include "bspline_opt/gradient_descent_optimizer.h"
// using namespace std;

namespace ego_planner
{

  void BsplineOptimizer::setParam(ros::NodeHandle &nh)
  {
    nh.param("optimization/lambda_smooth", lambda1_, -1.0);
    nh.param("optimization/lambda_collision", lambda2_, -1.0);
    nh.param("optimization/lambda_feasibility", lambda3_, -1.0);
    nh.param("optimization/lambda_fitness", lambda4_, -1.0);

    nh.param("optimization/dist0", dist0_, -1.0);   // safe distance 参数列表赋值为 0.5
    nh.param("optimization/max_vel", max_vel_, -1.0);
    nh.param("optimization/max_acc", max_acc_, -1.0);

    nh.param("optimization/order", order_, 3);
  }

  void BsplineOptimizer::setEnvironment(const GridMap::Ptr &env)
  {
    this->grid_map_ = env;
  }

  void BsplineOptimizer::setControlPoints(const Eigen::MatrixXd &points)
  {
    cps_.points = points;
  }

  void BsplineOptimizer::setBsplineInterval(const double &ts) { bspline_interval_ = ts; }

  /* This function is very similar to check_collision_and_rebound(). 
   * It was written separately, just because I did it once and it has been running stably since March 2020.
   * But I will merge then someday.*/
  std::vector<std::vector<Eigen::Vector3d>> BsplineOptimizer::initControlPoints(Eigen::MatrixXd &init_points, bool flag_first_init /*= true*/)
  {
    // 初始化控制点cps_
    if (flag_first_init) { 
      // cps: Control points
      cps_.clearance = dist0_;          // 可能是一个阈值,用于判断是否有障碍物
      // 调整cps_.points的大小,使其列数与init_points相同。init_points是一个Eigen::MatrixXd类型的矩阵,包含了初始轨迹点。
      cps_.resize(init_points.cols());
      // 将init_points的值赋给cps_.points。这样就完成了控制点cps_的初始化,使其包含了初始轨迹点。
      cps_.points = init_points;
    }

    /*** 根据给定的初始轨迹点和障碍物信息,将初始轨迹分割成多个不同的轨迹段 ***/
    /*** Segment the initial trajectory according to obstacles 根据障碍物分割初始轨迹 ***/
    constexpr int ENOUGH_INTERVAL = 2; // 定义一个值为 2 的编译时常量，可以在编译时使用该常量，而不需要在运行时进行计算,用于判断轨迹是否发生变化
    // 计算遍历初始轨迹点时的步长step_size
    // 这个步长用于在遍历初始轨迹点时,以一定的步长在每两个相邻点之间插值,检查是否存在障碍物。步长越小,检查越精细,但计算量也越大。
    // grid_map_->getResolution() 获取栅格地图的分辨率, 即地图中每个栅格的大小，地图分辨率越高,对应的grid_map_->getResolution()的值越小。
    // 每个栅格越大，需要步长要越大。即地图分辨率越低，步长越大。
    // (init_points.col(0) - init_points.rightCols(1)).norm() 计算初始轨迹点的第一个点和最后一个点之间的欧几里得距离
    // (init_points.cols() - 1) 得到初始轨迹点的总数减1, 即 轨迹段的数量。
    // ((init_points.col(0) - init_points.rightCols(1)).norm() / (init_points.cols() - 1)) 即为每个轨迹段的平均长度。
    // 将地图分辨率除以每段平均长度,得到一个比例因子,将这个比例因子除以2,得到最终的步长step_size。
    // 即 最终的步长step_size 与 地图分辨率 和 每个轨迹段的平均长度 成反比
    /*    具体来说:
      地图分辨率越高,意味着每个栅格越小,需要检查的点越密集,因此步长应该越小,以保证检查的精度。
      每段轨迹的平均长度越长,意味着相邻点之间的距离越大,如果步长不变,可能会漏检一些障碍物。
      因此步长应该随着平均长度的增加而减小,以保证检查的全面性。   */ 
    double step_size = grid_map_->getResolution() / ((init_points.col(0) - init_points.rightCols(1)).norm() / (init_points.cols() - 1)) / 2;
    int in_id, out_id;
    vector<std::pair<int, int>> segment_ids;
    int same_occ_state_times = ENOUGH_INTERVAL + 1;
    bool occ, last_occ = false;
    bool flag_got_start = false, flag_got_end = false, flag_got_end_maybe = false;
    // 限制对初始轨迹点的遍历范围: 在计算控制点的基准点和方向时,需要用到相邻的几个控制点。对于首尾的一些点,由于没有足够的相邻点,计算会有问题。
    // 忽略最后约1/3控制点的原因可能是因为这是局部规划，后续控制点在后续的迭代中还会进行调整和优化，因此重点关注前2/3控制点的避碰，可以减少计算量
    int i_end = (int)init_points.cols() - order_ - ((int)init_points.cols() - 2 * order_) / 3; // only check closed 2/3 points.
    for (int i = order_; i <= i_end; ++i) {   // 遍历初始轨迹点连线上是否存在障碍物。
      for (double a = 1.0; a >= 0.0; a -= step_size)  // a 介于0~1，通过插值来取出 相邻两控制点所组成的线段上的 轨迹点
      {
        // 0:该位置没有障碍物;1:该位置有障碍物。
        occ = grid_map_->getInflateOccupancy(a * init_points.col(i - 1) + (1 - a) * init_points.col(i));
        // cout << setprecision(5);
        // cout << (a * init_points.col(i-1) + (1-a) * init_points.col(i)).transpose() << " occ1=" << occ << endl;

        if (occ && !last_occ) // 当前点occ被占用,而上一个点last_occ未被占用,表示遇到了障碍物的起始点。
        {
          // 如果 之前 连续ENOUGH_INTERVAL(2)个点的占用状态没有发生变化,或者当前点i等于order_(边界情况),则认为已经进入了障碍物区域,
          if (same_occ_state_times > ENOUGH_INTERVAL || i == order_)
          {
            in_id = i - 1;          // 将当前点的前一个点的索引i-1记录为in_id,并设置flag_got_start为true。
            flag_got_start = true;
          }
          same_occ_state_times = 0;   // 连续占用状态 改变
          flag_got_end_maybe = false; // terminate in advance
        }
        else if (!occ && last_occ)  // 当前点occ未被占用,而上一个点last_occ被占用,表示可能遇到了障碍物的终止点。
        {
          out_id = i;
          flag_got_end_maybe = true;
          same_occ_state_times = 0;
        }
        else // 当前点occ和 上一个点last_occ均被占用 或者 均未被占用
        {
          ++same_occ_state_times;
        }

        // 如果在flag_got_end_maybe为真的前提下连续ENOUGH_INTERVAL个点的占用状态没有发生变化,则认为已经离开了障碍物,设置flag_got_end为true
        if (flag_got_end_maybe && (same_occ_state_times > ENOUGH_INTERVAL || (i == (int)init_points.cols() - order_)))
        {   
          flag_got_end_maybe = false;
          flag_got_end = true;
        }

        last_occ = occ; // 标记下一检查时刻的上一状态

        if (flag_got_start && flag_got_end) // 找到了一个完整的轨迹段,将其索引对添加到segment_ids中,并重置相关标志位。
        {
          flag_got_start = false;
          flag_got_end = false;
          segment_ids.push_back(std::pair<int, int>(in_id, out_id));
        }
      }
    }

    /*** a star search ***/
    vector<vector<Eigen::Vector3d>> a_star_pathes;
    for (size_t i = 0; i < segment_ids.size(); ++i)
    {
      //cout << "in=" << in.transpose() << " out=" << out.transpose() << endl;
      // init_points.col(segment_ids[i].first) 获取 第i个轨迹段起始点对应的列向量,赋值给了in。
      Eigen::Vector3d in(init_points.col(segment_ids[i].first)), out(init_points.col(segment_ids[i].second));
      // 使用A*算法在每个被障碍物分割的轨迹段中寻找一条 局部避障路径。
      if (a_star_->AstarSearch(/*(in-out).norm()/10+0.05*/ 0.1, in, out))
      {
        a_star_pathes.push_back(a_star_->getPath());
      }
      else
      {
        ROS_ERROR("a star error, force return!");
        return a_star_pathes;
      }
    }

    /*** calculate bounds 为每个轨迹段计算控制点的上下界,为后续的轨迹优化提供约束条件 ***/
    int id_low_bound, id_up_bound;   // 控制点的下界和上界
    vector<std::pair<int, int>> bounds(segment_ids.size());   // 用于存储每个轨迹段的控制点上下界
    for (size_t i = 0; i < segment_ids.size(); i++) // 遍历每个轨迹段
    {

      if (i == 0) // first segment
      {
        id_low_bound = order_;
        if (segment_ids.size() > 1) { // 如果有多个轨迹段
          //第一个轨迹段的控制点上界应该在第一个轨迹段的结束点和第二个轨迹段的起始点之间。减去1.0是为了避免取到第二个轨迹段的起始点。
          id_up_bound = (int)(((segment_ids[0].second + segment_ids[1].first) - 1.0f) / 2); // id_up_bound : -1.0f fix()
        }
        else  // 只有一段轨迹
        {
          id_up_bound = init_points.cols() - order_ - 1;  // 剩余三个控制点
        }
      }
      else if (i == segment_ids.size() - 1) // last segment, i != 0 here 最后一段轨迹
      {
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        id_up_bound = init_points.cols() - order_ - 1;
      }
      else
      {
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        id_up_bound = (int)(((segment_ids[i].second + segment_ids[i + 1].first) - 1.0f) / 2);  // id_up_bound : -1.0f fix()
      }

      bounds[i] = std::pair<int, int>(id_low_bound, id_up_bound); // 将当前轨迹段的上下界(id_low_bound, id_up_bound)存储在bounds[i]中。
    }

    // cout << "+++++++++" << endl;
    // for ( int j=0; j<bounds.size(); ++j )
    // {
    //   cout << bounds[j].first << "  " << bounds[j].second << endl;
    // }

    /*** Adjust segment length 调整每个轨迹段的长度,确保每个轨迹段至少包含一定数量的控制点。 ***/
    vector<std::pair<int, int>> final_segment_ids(segment_ids.size());
    // MINIMUM_PERCENT 是一个常量,表示每个轨迹段至少应该占初始轨迹的百分比。
    constexpr double MINIMUM_PERCENT = 0.0; // Each segment is guaranteed to have sufficient points to generate sufficient thrust
    int minimum_points = round(init_points.cols() * MINIMUM_PERCENT), num_points; // round 是四舍五入
    for (size_t i = 0; i < segment_ids.size(); i++) {
      /*** Adjust segment length ***/
      num_points = segment_ids[i].second - segment_ids[i].first + 1;  // 该段轨迹所包含的 控制点数量
      //cout << "i = " << i << " first = " << segment_ids[i].first << " second = " << segment_ids[i].second << endl;
      if (num_points < minimum_points)
      {
        double add_points_each_side = (int)(((minimum_points - num_points) + 1.0f) / 2);  // 计算需要在每个轨迹段两端各添加的控制点数
        // 计算调整后的第i个轨迹段的起始点索引
        // segment_ids[i].first - add_points_each_side 意为 在原始起始点向前移动add_points_each_side个控制点的新起始点索引。
        // 如果segment_ids[i].first - add_points_each_side >= bounds[i].first,说明新的起始点索引在下界内,将其直接赋值给final_segment_ids[i].first
        // 如果segment_ids[i].first - add_points_each_side < bounds[i].first,说明新的起始点索引超出了下界,将bounds[i].first赋值给final_segment_ids[i].first,以确保在下界内。
        final_segment_ids[i].first = segment_ids[i].first - add_points_each_side >= bounds[i].first ? segment_ids[i].first - add_points_each_side : bounds[i].first;
        // 同理，计算调整后的第i个轨迹段的结束点索引
        final_segment_ids[i].second = segment_ids[i].second + add_points_each_side <= bounds[i].second ? segment_ids[i].second + add_points_each_side : bounds[i].second;
      }
      else
      {
        final_segment_ids[i].first = segment_ids[i].first;
        final_segment_ids[i].second = segment_ids[i].second;
      }

      //cout << "final:" << "i = " << i << " first = " << final_segment_ids[i].first << " second = " << final_segment_ids[i].second << endl;
    }

    /*** Assign data to each segment 为每个轨迹段分配数据,包括基准点和方向。 ***/
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      // step 1 将cps_.flag_temp数组中对应该轨迹段的元素全部设置为false。
      for (int j = final_segment_ids[i].first; j <= final_segment_ids[i].second; ++j)
        cps_.flag_temp[j] = false;

      // step 2
      int got_intersection_id = -1; // 一个标记变量,用于指示 当前轨迹段在控制点处的切平面与A*路径是否存在交点。
      // 从该轨迹段的第二个控制点开始遍历,直到倒数第二个控制点。
      for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)  
      {
        // ctrl_pts_law: 第j个控制点的切向量
        Eigen::Vector3d ctrl_pts_law(cps_.points.col(j + 1) - cps_.points.col(j - 1)), intersection_point;
        // 初始化A*路径搜索的起点,为后续计算与切向量的垂面相交的点做准备。从中间点开始搜索可以提高效率。
        int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
        double val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law), last_val = val;
        while (Astar_id >= 0 && Astar_id < (int)a_star_pathes[i].size())
        {
          last_Astar_id = Astar_id;

          if (val >= 0)   // 说明当前A*点在切向量的同侧,将Astar_id减1,朝着切向量反方向移动。
            --Astar_id;
          else
            ++Astar_id;

          val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law);

          // 如果val和last_val异号,且至少有一个不为0,说明在Astar_id和last_Astar_id之间存在与切向量的垂面相交的点。
          // 此时计算插入点 并 跳出循环
          if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
          {
            // a_star_pathes[i][Astar_id]表示第i个轨迹段对应的A*路径上的第Astar_id个点。
            // a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id] 表示线段的方向向量
            // 平面法向量：ctrl_pts_law
            // cps_.points.col(j):平面上的一个点    a_star_pathes[i][Astar_id]:线段上的一个点
            // a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]: 线段的方向向量
            /* 线段与平面的交点公式:
               交点 = 线段上一点 + 线段方向向量 * t
               其中t = (平面法向量 · (平面上一点 - 线段上一点)) / (平面法向量 · 线段方向向量) */
            intersection_point =
                a_star_pathes[i][Astar_id] +
                ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                 (ctrl_pts_law.dot(cps_.points.col(j) - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                );

            //cout << "i=" << i << " j=" << j << " Astar_id=" << Astar_id << " last_Astar_id=" << last_Astar_id << " intersection_point = " << intersection_point.transpose() << endl;

            got_intersection_id = j;
            break;
          }
        }

        // 在找到控制点切平面与A*路径的交点后,沿着交点到控制点的方向搜索,确定该控制点的基准点和方向。
        /* 为每个控制点确定合适的基准点和方向,为后续的B-spline曲线优化提供良好的初始条件。
           通过沿着交点到控制点的方向搜索,可以避免生成的曲线与障碍物发生碰撞。
           同时,由于地图分辨率的限制,当搜索点距离控制点的距离小于分辨率时,就将控制点本身作为基准点。
           这样可以确保每个控制点至少有一个基准点和方向。 */
        if (got_intersection_id >= 0)
        {
          cps_.flag_temp[j] = true;
          double length = (intersection_point - cps_.points.col(j)).norm(); // 交点到控制点的距离
          if (length > 1e-5)
          {
            // 沿着交点到控制点的方向,以地图分辨率为步长,从交点开始搜索。
            for (double a = length; a >= 0.0; a -= grid_map_->getResolution())  
            {
              // 检查从交点intersection_point到控制点cps_.points.col(j)之间的线段上的某一点是否为障碍物。
              occ = grid_map_->getInflateOccupancy((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));

              // 在从交点intersection_point到控制点cps_.points.col(j)的方向上搜索时,如果遇到障碍物或者到达了控制点,
              // 就将当前搜索点作为该控制点的基准点之一,并计算该点到控制点的方向作为控制点的方向之一。
              if (occ || a < grid_map_->getResolution())
              {
                if (occ)
                  a += grid_map_->getResolution();  // 如果当前搜索点为障碍物,将搜索点后移一个地图分辨率的距离。
                // 该控制点的基准点坐标
                cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));
                // 该控制点的基准点方向
                cps_.direction[j].push_back((intersection_point - cps_.points.col(j)).normalized());
                break;
              }
            }
          }
        }
      }

      /* 这段代码处理了一种特殊情况,即当一个轨迹段的长度为1(只有两个控制点)时,如何为该段确定基准点和方向。 */
      /* Corner case: the segment length is too short. Here the control points may outside the A* path, leading to opposite gradient direction. So I have to take special care of it */
      if (segment_ids[i].second - segment_ids[i].first == 1)  // 当前轨迹段的长度为1(只有两个控制点)
      {
        Eigen::Vector3d ctrl_pts_law(cps_.points.col(segment_ids[i].second) - cps_.points.col(segment_ids[i].first)), intersection_point;
        Eigen::Vector3d middle_point = (cps_.points.col(segment_ids[i].second) + cps_.points.col(segment_ids[i].first)) / 2;
        int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
        double val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law), last_val = val;
        while (Astar_id >= 0 && Astar_id < (int)a_star_pathes[i].size())
        {
          last_Astar_id = Astar_id;

          if (val >= 0)
            --Astar_id;
          else
            ++Astar_id;

          val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law);

          if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
          {
            intersection_point =
                a_star_pathes[i][Astar_id] +
                ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                 (ctrl_pts_law.dot(middle_point - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                );

            if ((intersection_point - middle_point).norm() > 0.01) // 1cm.
            {
              cps_.flag_temp[segment_ids[i].first] = true;
              cps_.base_point[segment_ids[i].first].push_back(cps_.points.col(segment_ids[i].first));
              cps_.direction[segment_ids[i].first].push_back((intersection_point - middle_point).normalized());

              got_intersection_id = segment_ids[i].first;
            }
            break;
          }
        }
      }

      //step 3  为每个轨迹段中未处理的控制点分配基准点和方向。 对于未处理的控制点,将它们的基准点和方向设置为相邻已处理控制点的基准点和方向。
      if (got_intersection_id >= 0)
      {
        for (int j = got_intersection_id + 1; j <= final_segment_ids[i].second; ++j)
          if (!cps_.flag_temp[j])
          {
            cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
            cps_.direction[j].push_back(cps_.direction[j - 1].back());
          }

        for (int j = got_intersection_id - 1; j >= final_segment_ids[i].first; --j)
          if (!cps_.flag_temp[j])
          {
            cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
            cps_.direction[j].push_back(cps_.direction[j + 1].back());
          }
      }
      else
      {
        // Just ignore, it does not matter ^_^.
        // ROS_ERROR("Failed to generate direction! segment_id=%d", i);
      }
    }

    return a_star_pathes;
  }

  // 这段代码定义了一个名为earlyExit的函数,它是L-BFGS优化算法的回调函数之一,用于决定是否提前终止当前的优化迭代。
  /* 如果检测到B样条曲线与障碍物发生碰撞,且无法通过A算法找到一条可行的避障路径,则会终止优化过程;
  如果A算法成功找到了一条可行路径,则会停止当前迭代,并在下一次迭代中根据新的基准点和方向重新计算代价函数和梯度。 */
  int BsplineOptimizer::earlyExit(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls)
  {
    // 将func_data指针强制转换为BsplineOptimizer对象的指针。
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);
    // cout << "k=" << k << endl;
    // cout << "opt->flag_continue_to_optimize_=" << opt->flag_continue_to_optimize_ << endl;
    // 如果force_stop_type_等于STOP_FOR_ERROR或STOP_FOR_REBOUND,则返回true,表示应该提前终止当前的优化迭代。
    // 否则,返回false,允许优化继续进行。
    return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }

  // LBFGS优化器在优化B-spline曲线控制点时调用的代价函数,它是在优化函数中具体实现时所调用的。
  double BsplineOptimizer::costFunctionRebound(void *func_data, const double *x, double *grad, const int n)
  {
    // 这里func_data是在优化函数中指向当前BsplineOptimizer对象的this指针作为void*类型的参数传递进来的,用于存储一个BsplineOptimizer对象的地址。
    // 这里将一个void*类型的指针func_data重新解释(reinterpret_cast)为BsplineOptimizer*类型的指针,并赋值给opt指针。
    // 这种通过void*指针传递对象实例地址,再在需要时重新解释为对应类型指针的做法,常见于C++中的回调函数机制。
    // 回调函数通常只能接受void*类型的参数,因此需要在函数内部进行这种指针类型的重新解释,以便访问对象的成员函数和数据成员。
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);

    double cost;
    opt->combineCostRebound(x, grad, cost, n);  // n: 变量个数

    opt->iter_num_ += 1;
    return cost;
  }

  double BsplineOptimizer::costFunctionRefine(void *func_data, const double *x, double *grad, const int n)
  {
    // 这里func_data是在优化函数中指向当前BsplineOptimizer对象的this指针作为void*类型的参数传递进来的,用于存储一个BsplineOptimizer对象的地址。
    // 这里将一个void*类型的指针func_data重新解释(reinterpret_cast)为BsplineOptimizer*类型的指针,并赋值给opt指针。
    // 这种通过void*指针传递对象实例地址,再在需要时重新解释为对应类型指针的做法,常见于C++中的回调函数机制。
    // 回调函数通常只能接受void*类型的参数,因此需要在函数内部进行这种指针类型的重新解释,以便访问对象的成员函数和数据成员。
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);

    double cost;
    opt->combineCostRefine(x, grad, cost, n);

    opt->iter_num_ += 1;
    return cost;
  }

  // 计算B样条曲线与障碍物之间的距离代价(distance cost)及其梯度。
  void BsplineOptimizer::calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost,
                                                 Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost) {
    cost = 0.0;
    int end_idx = q.cols() - order_;  // 需要考虑距离代价的最后一个控制点的索引
    double demarcation = cps_.clearance;  // a safety clearance = 0.5，对应文中的s_f
    double a = 3 * demarcation, b = -3 * pow(demarcation, 2), c = pow(demarcation, 3);

    force_stop_type_ = DONT_STOP;
    // 判断是否需要执行碰撞检测和反弹操作
    // 在迭代次数大于3之后,如果归一化的平滑性代价足够小(小于0.1),就认为B样条曲线已经足够平滑,
    // 可以执行碰撞检测和反弹操作,进一步优化曲线的形状,使其能够避开障碍物。
    if (iter_num > 3 && smoothness_cost / (cps_.size - 2 * order_) < 0.1) // 0.1 is an experimental value that indicates the trajectory is smooth enough.
    {
      check_collision_and_rebound();
    }

    /*** calculate distance cost and gradient ***/
    for (auto i = order_; i < end_idx; ++i)
    {
      for (size_t j = 0; j < cps_.direction[i].size(); ++j)
      {
        double dist = (cps_.points.col(i) - cps_.base_point[i][j]).dot(cps_.direction[i][j]);
        double dist_err = cps_.clearance - dist;            // 对应文中的c_{ij}
        Eigen::Vector3d dist_grad = cps_.direction[i][j];   // 对应文中的v_{ij}

        if (dist_err < 0)
        {
          /* do nothing */
        }
        else if (dist_err < demarcation)
        {
          cost += pow(dist_err, 3);
          gradient.col(i) += -3.0 * dist_err * dist_err * dist_grad;
        }
        else
        {
          cost += a * dist_err * dist_err + b * dist_err + c;
          gradient.col(i) += -(2.0 * a * dist_err + b) * dist_grad;
        }
      }
    }
  }

  // 计算B-spline曲线与参考点集ref_pts_之间的拟合代价函数(fitness cost)及其梯度。
  void BsplineOptimizer::calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
  {

    cost = 0.0;

    int end_idx = q.cols() - order_;

    // def: f = |x*v|^2/a^2 + |x×v|^2/b^2
    double a2 = 25, b2 = 1;
    for (auto i = order_ - 1; i < end_idx + 1; ++i)
    {
      /* 在计算B-spline曲线与参考点集ref_pts_之间的拟合代价函数时,
      代码比较的是B-spline曲线在第 i 个控制点处的近似曲线点与参考点ref_pts_[i-1],
      而不是第i-1个控制点处的近似曲线点与ref_pts_[i-1]。这是有一定原因的:
      B-spline曲线的性质决定了,每个控制点及其相邻的几个控制点共同决定了曲线在该控制点处的形状。
      具体来说,对于三次B-spline曲线,曲线在第i个控制点处的形状由第i-1、i、i+1这三个控制点共同决定。
      因此,代码中使用(q.col(i-1) + 4*q.col(i) + q.col(i+1))/6.0近似计算第i个控制点处的曲线点,是合理的。
      如果使用第i-1个控制点处的近似曲线点,那么就无法精确反映曲线在ref_pts_[i-1]处的真实形状。
      将B-spline曲线在第i个控制点处的近似曲线点与参考点ref_pts_[i-1]进行比较,
      可以更好地衡量曲线与参考轨迹在该点处的偏差,从而优化控制点以减小这种偏差。 */
      Eigen::Vector3d x = (q.col(i - 1) + 4 * q.col(i) + q.col(i + 1)) / 6.0 - ref_pts_[i - 1];
      Eigen::Vector3d v = (ref_pts_[i] - ref_pts_[i - 2]).normalized(); // 近似计算参考点ref_pts_[i-1]处的切向量。

      double xdotv = x.dot(v);
      Eigen::Vector3d xcrossv = x.cross(v);

      double f = pow((xdotv), 2) / a2 + pow(xcrossv.norm(), 2) / b2;
      cost += f;

      Eigen::Matrix3d m;  // 叉积xcrossv关于x的雅可比矩阵,即xcrossv对x的偏导数。
      m << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;  // 偏导
      Eigen::Vector3d df_dx = 2 * xdotv / a2 * v + 2 / b2 * m * xcrossv;

      /* 由于B-spline曲线在第i个控制点处的近似曲线点由第i-1、i、i+1这三个控制点共同决定,
         因此需要将df_dx分别加到这三个控制点的梯度上,权重分别为1/6、4/6和1/6。 */
      gradient.col(i - 1) += df_dx / 6;
      gradient.col(i) += 4 * df_dx / 6;
      gradient.col(i + 1) += df_dx / 6;
    }
  }

  // 计算B样条曲线的平滑性代价(smoothness cost)及其梯度
  void BsplineOptimizer::calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                                            Eigen::MatrixXd &gradient, bool flag_use_jerk /* = true*/) {
    cost = 0.0;
    /* 如果flag_use_jerk为真,则使用Jerk的平方和作为平滑性代价;
       如果flag_use_jerk为假,则使用加速度的平方和作为平滑性代价。 */
    if (flag_use_jerk) {  // 使用Jerk的平方和作为平滑性代价
      Eigen::Vector3d jerk, temp_j;

      for (int i = 0; i < q.cols() - 3; i++)  // 加加速度(jerk)是位置关于时间的三阶导数,需要通过相邻四个控制点的组合来近似计算。
      {
        /* evaluate jerk */ 
        // q是一个Eigen::MatrixXd类型的矩阵,存储了B样条曲线的所有控制点坐标。每一列对应一个3D控制点。
        jerk = q.col(i + 3) - 3 * q.col(i + 2) + 3 * q.col(i + 1) - q.col(i);
        cost += jerk.squaredNorm();
        temp_j = 2.0 * jerk;  // temp_j变量为之后计算梯度提供方便
        /* jerk gradient 梯度是一个与控制点数量相同的矩阵,每一列对应一个控制点的梯度。 */
        gradient.col(i + 0) += -temp_j;
        gradient.col(i + 1) += 3.0 * temp_j;
        gradient.col(i + 2) += -3.0 * temp_j;
        gradient.col(i + 3) += temp_j;
      }
    }
    else  // 使用加速度的平方和作为平滑性代价
    {
      Eigen::Vector3d acc, temp_acc;

      for (int i = 0; i < q.cols() - 2; i++)
      {
        /* evaluate acc */
        acc = q.col(i + 2) - 2 * q.col(i + 1) + q.col(i);
        cost += acc.squaredNorm();
        temp_acc = 2.0 * acc;
        /* acc gradient */
        gradient.col(i + 0) += temp_acc;
        gradient.col(i + 1) += -2.0 * temp_acc;
        gradient.col(i + 2) += temp_acc;
      }
    }
  }

  // 计算B样条曲线的可行性代价(feasibility cost)以及相应的梯度。
  // 这个函数的作用是评估给定控制点序列q所表示的B样条曲线在速度和加速度方面是否满足约束条件,并计算相应的代价值cost和梯度gradient。
  void BsplineOptimizer::calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                                             Eigen::MatrixXd &gradient)
  {

    //#define SECOND_DERIVATIVE_CONTINOUS 根据预编译宏SECOND_DERIVATIVE_CONTINOUS的定义,选择使用不同的代价函数形式。

// #define SECOND_DERIVATIVE_CONTINOUS

#ifdef SECOND_DERIVATIVE_CONTINOUS  // 条件编译，灰色部分不编译  二阶导数连续

    cost = 0.0;
    double demarcation = 1.0; // 1m/s, 1m/s/s  定义了一个分界值demarcation,设置为1.0。这个值用于区分速度或加速度是否接近约束值的范围。
    double ar = 3 * demarcation, br = -3 * pow(demarcation, 2), cr = pow(demarcation, 3);   // 选择以满足二阶连续，对应文中a2,b2,c2
    double al = ar, bl = -br, cl = cr;  // 对应文中a1,b1,c1

    /* abbreviation */
    double ts, ts_inv2, ts_inv3;
    ts = bspline_interval_;   // B样条时间间隔
    ts_inv2 = 1 / ts / ts;
    ts_inv3 = 1 / ts / ts / ts;

    /* velocity feasibility */
    for (int i = 0; i < q.cols() - 1; i++)
    {
      Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;

      for (int j = 0; j < 3; j++)   // 三个维度分别讨论
      {
        if (vi(j) > max_vel_ + demarcation)   // c_j = max_vel_ + demarcation
        {
          double diff = vi(j) - max_vel_;   // 乘以ts_inv3是为了使速度和加速度的代价值量级相近,便于优化,这里3次函数为主体
          cost += (ar * diff * diff + br * diff + cr) * ts_inv3; // multiply ts_inv3 to make vel and acc has similar magnitude

          // 我们的目标是优化控制点的位置,而不是优化速度本身。因此,需要将代价函数对速度的梯度转换为对控制点的梯度。
          // 利用链式法则,代价函数对控制点q.col(i)的梯度等于代价函数对速度vi(j)的梯度乘以vi(j)对q.col(i)的导数,即(2*(vi(j) - max_vel_))/ts。
          // 同理,代价函数对控制点q.col(i+1)的梯度等于(-2*(vi(j) - max_vel_))/ts。
          double grad = (2.0 * ar * diff + br) / ts * ts_inv3;
          gradient(j, i + 0) += -grad;  // 代价值对前一个控制点的梯度为负
          gradient(j, i + 1) += grad;   // 代价值对后一个控制点的梯度为正
        }
        else if (vi(j) > max_vel_)
        {
          double diff = vi(j) - max_vel_;
          cost += pow(diff, 3) * ts_inv3;
          ;

          double grad = 3 * diff * diff / ts * ts_inv3;
          ;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else if (vi(j) < -(max_vel_ + demarcation))
        {
          double diff = vi(j) + max_vel_;
          cost += (al * diff * diff + bl * diff + cl) * ts_inv3;

          double grad = (2.0 * al * diff + bl) / ts * ts_inv3;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else if (vi(j) < -max_vel_)
        {
          double diff = vi(j) + max_vel_;
          cost += -pow(diff, 3) * ts_inv3;

          double grad = -3 * diff * diff / ts * ts_inv3;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else
        {
          /* nothing happened */
        }
      }
    }

    /* acceleration feasibility */
    for (int i = 0; i < q.cols() - 2; i++)
    {
      Eigen::Vector3d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;

      for (int j = 0; j < 3; j++)
      {
        if (ai(j) > max_acc_ + demarcation)
        {
          double diff = ai(j) - max_acc_;
          cost += ar * diff * diff + br * diff + cr;

          double grad = (2.0 * ar * diff + br) * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) > max_acc_)
        {
          double diff = ai(j) - max_acc_;
          cost += pow(diff, 3);

          double grad = 3 * diff * diff * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) < -(max_acc_ + demarcation))
        {
          double diff = ai(j) + max_acc_;
          cost += al * diff * diff + bl * diff + cl;

          double grad = (2.0 * al * diff + bl) * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) < -max_acc_)
        {
          double diff = ai(j) + max_acc_;
          cost += -pow(diff, 3);

          double grad = -3 * diff * diff * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else
        {
          /* nothing happened */
        }
      }
    }

#else

    cost = 0.0;
    /* abbreviation */
    double ts, /*vm2, am2, */ ts_inv2;
    // vm2 = max_vel_ * max_vel_;
    // am2 = max_acc_ * max_acc_;

    ts = bspline_interval_;
    ts_inv2 = 1 / ts / ts;

    /* velocity feasibility */
    for (int i = 0; i < q.cols() - 1; i++)
    {
      Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;

      //cout << "temp_v * vi=" ;
      for (int j = 0; j < 3; j++)
      {
        if (vi(j) > max_vel_)
        {
          // cout << "fuck VEL" << endl;
          // cout << vi(j) << endl;   // 乘以ts_inv2是为了使速度和加速度的代价值量级相近,便于优化,这里2次函数为主体
          cost += pow(vi(j) - max_vel_, 2) * ts_inv2; // multiply ts_inv2 to make vel and acc has similar magnitude

          gradient(j, i + 0) += -2 * (vi(j) - max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) - max_vel_) / ts * ts_inv2;
        }
        else if (vi(j) < -max_vel_)
        {
          cost += pow(vi(j) + max_vel_, 2) * ts_inv2;

          gradient(j, i + 0) += -2 * (vi(j) + max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) + max_vel_) / ts * ts_inv2;
        }
        else
        {
          /* code */
        }
      }
    }

    /* acceleration feasibility */
    for (int i = 0; i < q.cols() - 2; i++)
    {
      Eigen::Vector3d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;

      //cout << "temp_a * ai=" ;
      for (int j = 0; j < 3; j++)
      {
        if (ai(j) > max_acc_)
        {
          // cout << "fuck ACC" << endl;
          // cout << ai(j) << endl;
          cost += pow(ai(j) - max_acc_, 2);

          gradient(j, i + 0) += 2 * (ai(j) - max_acc_) * ts_inv2;
          gradient(j, i + 1) += -4 * (ai(j) - max_acc_) * ts_inv2;
          gradient(j, i + 2) += 2 * (ai(j) - max_acc_) * ts_inv2;
        }
        else if (ai(j) < -max_acc_)
        {
          cost += pow(ai(j) + max_acc_, 2);

          gradient(j, i + 0) += 2 * (ai(j) + max_acc_) * ts_inv2;
          gradient(j, i + 1) += -4 * (ai(j) + max_acc_) * ts_inv2;
          gradient(j, i + 2) += 2 * (ai(j) + max_acc_) * ts_inv2;
        }
        else
        {
          /* code */
        }
      }
      //cout << endl;
    }

#endif
  }

  // 检测B样条曲线与障碍物的碰撞,并进行适当的反弹操作
  bool BsplineOptimizer::check_collision_and_rebound(void) {

    int end_idx = cps_.size - order_;

    /*** Check and segment the initial trajectory according to obstacles ***/
    int in_id, out_id;
    vector<std::pair<int, int>> segment_ids;
    bool flag_new_obs_valid = false;
    int i_end = end_idx - (end_idx - order_) / 3;
    // 忽略最后1/3控制点的原因可能是因为这是局部规划，后续控制点在后续的迭代中还会进行调整和优化，因此重点关注前2/3控制点的避碰，可以减少计算量
    for (int i = order_ - 1; i <= i_end; ++i) // 从第order_个控制点开始,遍历到距离终点约2/3的控制点。
    {
      // occ 表示当前控制点是否与膨胀后的栅格地图中的障碍物发生碰撞
      bool occ = grid_map_->getInflateOccupancy(cps_.points.col(i));

      /* 对该控制点进行进一步验证,以确定当前控制点是否真正位于一个 新的（之前未检测到的）障碍物 内，而不是之前已经检测到的障碍物。 */
      /*** check if the new collision will be valid ***/
      if (occ)  // 如果 occ 为真（即当前控制点与障碍物发生碰撞），则进入下面的循环体进行进一步验证 当前控制点是否位于与之相关的障碍物之外。
      {
        // 这里 cps_.direction[i].size() 为该控制点所对应的 所有 之前就已经检测到的障碍物 的数量
        for (size_t k = 0; k < cps_.direction[i].size(); ++k)
        {
          cout.precision(2);
          // 如果这里点积小于等于栅格地图的分辨率（grid_map_->getResolution()）
          // 则表示当前点位于 之前就已经检测到的障碍物之内
          // 因此，此障碍物 不是 一个“新”障碍物，因此需要将 occ 设置为 false
          if ((cps_.points.col(i) - cps_.base_point[i][k]).dot(cps_.direction[i][k]) < 1 * grid_map_->getResolution()) // current point is outside all the collision_points.
          {
            occ = false; // Not really takes effect, just for better hunman understanding.
            break;
          }
        }
      }

      // 在检测到当前控制点位于  一个“新检测到的”障碍物  内部时,找到该控制点前后最近的不在障碍物内的控制点,
      // 将它们的索引存储在in_id和out_id中,并将这个控制点段(in_id, out_id)加入segment_ids向量中。
      if (occ)
      {
        flag_new_obs_valid = true;

        int j;
        for (j = i - 1; j >= 0; --j)    // 在给定索引 i 的前面查找最近的一个没有发生碰撞的点
        {
          occ = grid_map_->getInflateOccupancy(cps_.points.col(j));
          if (!occ)   // 没有发生碰撞
          {
            in_id = j;
            break;
          }
        }
        if (j < 0) // fail to get the obs free point
        {
          ROS_ERROR("ERROR! the drone is in obstacle. This should not happen.");
          in_id = 0;
        }

        for (j = i + 1; j < cps_.size; ++j)   // 在给定索引 i 的后面查找最近的一个没有发生碰撞的点
        {
          occ = grid_map_->getInflateOccupancy(cps_.points.col(j));

          if (!occ)
          {
            out_id = j;
            break;
          }
        }
        if (j >= cps_.size) // fail to get the obs free point
        {
          ROS_WARN("WARN! terminal point of the current trajectory is in obstacle, skip this planning.");

          force_stop_type_ = STOP_FOR_ERROR;
          return false;
        }

        // 将变量 i 的值更新为当前循环中找到的最近的一个没有发生碰撞的点的索引加 1。
        // 这样做的目的是让下一次循环从最近的一个没有发生碰撞的点的后一个点开始继续查找。
        // 确保下一次循环中的迭代范围不会包括之前已经找到的没有发生碰撞的点，从而避免重复查找相同的点。
        i = j + 1;

        segment_ids.push_back(std::pair<int, int>(in_id, out_id));
      }
    }

    if (flag_new_obs_valid)
    {
      vector<vector<Eigen::Vector3d>> a_star_pathes;
      for (size_t i = 0; i < segment_ids.size(); ++i)
      {
        /*** a star search ***/
        Eigen::Vector3d in(cps_.points.col(segment_ids[i].first)), out(cps_.points.col(segment_ids[i].second));
        if (a_star_->AstarSearch(/*(in-out).norm()/10+0.05*/ 0.1, in, out))
        {
          a_star_pathes.push_back(a_star_->getPath());
        }
        else
        {
          // ROS_ERROR 是一个宏，用于输出错误消息，并将其记录到ROS系统的日志中。
          // 通常情况下，当出现错误时，会打印错误消息并记录日志，但不会导致程序的立即终止。
          ROS_ERROR("a star error");
          segment_ids.erase(segment_ids.begin() + i);
          i--;
        }
      }

      /*** Assign parameters to each segment ***/
      for (size_t i = 0; i < segment_ids.size(); ++i)
      {
        // step 1   将cps_.flag_temp数组中对应该轨迹段的元素全部初始化为false，此处用于标记控制点是否找到对应基准点和方向
        for (int j = segment_ids[i].first; j <= segment_ids[i].second; ++j)
          cps_.flag_temp[j] = false;

        // step 2
        int got_intersection_id = -1; // 一个标记变量,或正或-1，用于指示 当前轨迹段在控制点处的切平面与A*路径是否存在交点。
        // 从该轨迹段的第二个控制点开始遍历,直到倒数第二个控制点。
        for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
        {
          // ctrl_pts_law: 近似第j个控制点处的切向量
          Eigen::Vector3d ctrl_pts_law(cps_.points.col(j + 1) - cps_.points.col(j - 1)), intersection_point;
          // 初始化A*路径搜索的起点,为后续计算与切向量的垂面相交的点做准备。从中间点开始搜索可以提高效率。
          int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
          double val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law), last_val = val;
          while (Astar_id >= 0 && Astar_id < (int)a_star_pathes[i].size())
          {
            last_Astar_id = Astar_id;

            if (val >= 0)   // 说明当前A*点在切向量的同侧,将Astar_id减1,朝着切向量反方向移动。
              --Astar_id;
            else
              ++Astar_id;

            val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law);

            // cout << val << endl;

            // 如果val和last_val异号,且至少有一个不为0,说明在Astar_id和last_Astar_id之间存在与切向量的垂面相交的点。
            // 此时计算插入点 并 跳出循环
            if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
            {
              // a_star_pathes[i][Astar_id]表示第i个轨迹段对应的A*路径上的第Astar_id个点。
              // a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id] 表示线段的方向向量
              // 平面法向量：ctrl_pts_law
              // cps_.points.col(j):平面上的一个点    a_star_pathes[i][Astar_id]:线段上的一个点
              // a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]: 线段的方向向量
              /* 线段与平面的交点公式:
                  交点坐标 = 线段上一点 + 线段方向向量 * t
                  其中t = (平面法向量 · (平面上一点 - 线段上一点)) / (平面法向量 · 线段方向向量) */
              intersection_point =
                  a_star_pathes[i][Astar_id] +
                  ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                   (ctrl_pts_law.dot(cps_.points.col(j) - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                  );

              got_intersection_id = j;
              break;
            }
          }

          // 在找到控制点切平面与A*路径的交点后,沿着交点到控制点的方向搜索,确定该控制点的基准点和方向。
          /* 为每个控制点确定合适的基准点和方向,为后续的B-spline曲线优化提供良好的初始条件。
            通过沿着交点到控制点的方向搜索,可以避免生成的曲线与障碍物发生碰撞。
            同时,由于地图分辨率的限制,当搜索点距离控制点的距离小于分辨率时,就将控制点本身作为基准点。
            这样可以确保每个控制点至少有一个基准点和方向。 */
          if (got_intersection_id >= 0)
          {
            cps_.flag_temp[j] = true;
            double length = (intersection_point - cps_.points.col(j)).norm(); // 交点到控制点的距离
            if (length > 1e-5)
            {
              // 沿着交点到控制点的方向,以地图分辨率为步长,从交点开始搜索。
              for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
              {
                // 检查从交点intersection_point到控制点cps_.points.col(j)之间的线段上的某一点是否为障碍物。
                bool occ = grid_map_->getInflateOccupancy((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));

                // 在从交点intersection_point到控制点cps_.points.col(j)的方向上搜索时,如果遇到障碍物或者a一旦小于地图分辨率,便可确定基准点
                // 并可计算该基准点到控制点的方向作为控制点的方向之一。
                if (occ || a < grid_map_->getResolution())
                {
                  if (occ)
                    a += grid_map_->getResolution();  // 如果当前搜索点为障碍物,将搜索点后移一个地图分辨率的距离。
                  // 该控制点的基准点坐标
                  cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));
                  // 该控制点的基准点方向 （单位向量）
                  cps_.direction[j].push_back((intersection_point - cps_.points.col(j)).normalized());
                  break;
                }
              }
            }
            else
            {
              got_intersection_id = -1;
            }
          }
        }

        //step 3  为每个轨迹段中未处理的控制点分配基准点和方向。 对于未处理的控制点,将它们的基准点和方向设置为相邻已处理控制点的基准点和方向。
        if (got_intersection_id >= 0)
        {
          for (int j = got_intersection_id + 1; j <= segment_ids[i].second; ++j)
            if (!cps_.flag_temp[j])
            {
              cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
              cps_.direction[j].push_back(cps_.direction[j - 1].back());
            }

          for (int j = got_intersection_id - 1; j >= segment_ids[i].first; --j)
            if (!cps_.flag_temp[j])
            {
              cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
              cps_.direction[j].push_back(cps_.direction[j + 1].back());
            }
        }
        else
          ROS_WARN("Failed to generate direction. It doesn't matter.");
      }

      force_stop_type_ = STOP_FOR_REBOUND;  // 在遇到新障碍物时，强制重启优化过程
      return true;
    }

    return false;
  }

  bool BsplineOptimizer::BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts)
  {
    setBsplineInterval(ts);

    bool flag_success = rebound_optimize();

    optimal_points = cps_.points;

    return flag_success;
  }

  /* 这段代码的作用是对给定的B-spline曲线控制点init_points进行优化,以获得一条平滑且满足动力学约束的最优轨迹。优化后的控制点存储在optimal_points中。 */
  bool BsplineOptimizer::BsplineOptimizeTrajRefine(const Eigen::MatrixXd &init_points, const double ts, Eigen::MatrixXd &optimal_points)
  {
    setControlPoints(init_points);  // 将输入的init_points设置为当前B-spline曲线的控制点。
    setBsplineInterval(ts);         // 设置B-spline曲线的时间间隔为ts。

    bool flag_success = refine_optimize();

    optimal_points = cps_.points;

    return flag_success;
  }

  // 优化B-spline曲线,使其避开障碍物并满足平滑性约束
  bool BsplineOptimizer::rebound_optimize()
  {
    iter_num_ = 0;  // 当前优化的迭代次数
    int start_id = order_;  // 优化时控制点的起始索引 = B-spline曲线的阶数,通常取3(三次B-spline); 前order_个控制点固定,不参与优化。
    int end_id = this->cps_.size - order_;    // 优化时控制点的结束索引;最后order_个控制点也固定,不参与优化。
    variable_num_ = 3 * (end_id - start_id);  // 优化变量的数量，3个维度的变量相互独立
    double final_cost;  // 最终代价函数值

    ros::Time t0 = ros::Time::now(), t1, t2;
    int restart_nums = 0, rebound_times = 0;
    
    bool flag_force_return, flag_occ, success;
    new_lambda2_ = lambda2_;  // lambda2_ 为初始 距离权重，new_lambda2_为每次重规划 的 最新距离权重
    constexpr int MAX_RESART_NUMS_SET = 3;
    do
    {
      /* ---------- prepare ---------- */
      min_cost_ = std::numeric_limits<double>::max();   // C++标准库中的一个函数,它返回该数据类型(double型)所能表示的最大值。
      iter_num_ = 0;
      flag_force_return = false;
      flag_occ = false;
      success = false;

      /* 将需要优化的控制点坐标数据从cps_.points矩阵复制到一维数组q中,因为LBFGS优化器需要一维数组作为输入。
      通过这种方式,可以方便地将控制点坐标作为优化变量输入到LBFGS优化器中。 */
      double q[variable_num_];
      // cps_.points.data()返回cps_.points矩阵数据区的起始地址。
      // cps_.points.data() + 3 * start_id计算了需要复制的起始地址,从第start_id个控制点开始。
      // 由于每个控制点有3个坐标(x,y,z),所以起始地址需要偏移3 * start_id个元素。
      // sizeof(q)返回q数组中单个元素的字节大小。
      // memcpy是C++标准库中的内存复制函数,它的作用是从源地址复制一定字节数的数据到目标地址。
      memcpy(q, cps_.points.data() + 3 * start_id, variable_num_ * sizeof(q[0]));  // 将控制点坐标数据从cps_.points矩阵复制到q数组中

      // 设置LBFGS优化器的参数
      lbfgs::lbfgs_parameter_t lbfgs_params;  // 结构体变量，存储LBFGS优化器的参数
      lbfgs::lbfgs_load_default_parameters(&lbfgs_params);  // 将lbfgs_params初始化为默认参数值。
      // LBFGS优化器在内存中存储的 近似海森矩阵逆的内存大小,通常取值在3到20之间。值越大,近似精度越高,但计算时间也会增加。
      lbfgs_params.mem_size = 16;
      lbfgs_params.max_iterations = 200;
      lbfgs_params.g_epsilon = 0.01;

      /* ---------- optimize ---------- */
      t1 = ros::Time::now();
      /*lbfgs::lbfgs_optimize是LBFGS优化库中的一个函数,用于执行优化过程。
        variable_num_是一个整型变量,表示优化变量的数量,即需要优化的控制点坐标数。
        q是一个一维数组,存储了初始的控制点坐标,作为优化变量的初始值。
        &final_cost是一个双精度浮点型指针,用于存储优化后的最终代价函数值。
        BsplineOptimizer::costFunctionRebound是一个回调函数指针,指向计算代价函数及其梯度的函数。这个函数定义了优化的目标函数。
        NULL表示不提供任何回调函数,用于监视优化过程。
        BsplineOptimizer::earlyExit是一个回调函数指针,指向判断是否需要提前终止优化的函数。
        this是当前BsplineOptimizer对象的指针,用于在costFunctionRebound和earlyExit函数中访问对象的成员变量和函数。
        &lbfgs_params是一个lbfgs_parameter_t类型的指针,指向之前设置好的LBFGS优化器参数。
        result是一个整型变量,用于存储优化的返回状态,如收敛、达到最大迭代次数等。 */
      /* 在B-spline优化器中,有两个回调函数需要特别注意:
          1. proc_progress回调函数
            在每次L-BFGS优化迭代后被调用,用于决定是否应该终止优化过程。
            如果发生碰撞且无法通过A*算法找到可行路径,则会返回1,终止优化;否则返回0,允许优化继续进行。 
          2. proc_evaluate回调函数
            在每次优化迭代中被调用,用于计算当前控制点下的目标函数值和梯度。 */
      int result = lbfgs::lbfgs_optimize(variable_num_, q, &final_cost, BsplineOptimizer::costFunctionRebound, NULL, BsplineOptimizer::earlyExit, this, &lbfgs_params);
      t2 = ros::Time::now();
      double time_ms = (t2 - t1).toSec() * 1000;          // 纯优化用时
      double total_time_ms = (t2 - t0).toSec() * 1000;    // 优化参数初始化+优化过程总时间

      /* 这段代码是在B-spline优化器中,用于根据L-BFGS优化算法的返回结果采取相应的操作。主要功能包括:
        检查L-BFGS优化是否收敛或达到其他终止条件。
        如果优化 收敛 或 达到最大迭代次数 等,则检查生成的B-spline曲线是否与障碍物发生碰撞。
        如果没有发生碰撞,则认为优化成功,输出相关信息。
        如果发生碰撞:
        如果是前3个控制点发生碰撞,则直接返回false,优化失败。
        否则,重新初始化控制点,增加平滑性权重,继续优化。
        如果L-BFGS返回CANCELED,表示需要进行反弹操作,重新规划避开障碍物的路径。
        如果L-BFGS返回其他错误,则输出警告信息,跳过当前规划。 */
      /* ---------- success temporary, check collision again ---------- */
      if (result == lbfgs::LBFGS_CONVERGENCE ||               // 成功收敛
          result == lbfgs::LBFGSERR_MAXIMUMITERATION ||       // 最大迭代次数
          result == lbfgs::LBFGS_ALREADY_MINIMIZED ||         // 已经陷入极小化值
          result == lbfgs::LBFGS_STOP)                        // STOP
      {
        //ROS_WARN("Solver error in planning!, return = %s", lbfgs::lbfgs_strerror(result));
        flag_force_return = false;

        // 根据B样条曲线优化器中的控制点构造一条均匀B样条曲线对象traj。
        UniformBspline traj = UniformBspline(cps_.points, 3, bspline_interval_);
        double tm, tmp;  // tm 和 tmp 是表示时间的变量，用于确定插值的起始时间和结束时间。
        traj.getTimeSpan(tm, tmp);
        // 计算在 B-样条曲线上进行插值的步长（step size）
        // (traj.evaluateDeBoorT(tmp) - traj.evaluateDeBoorT(tm)).norm() 表示起始时间和结束时间的插值点之间的距离（欧几里德范数）。
        // 插值点之间的距离除以网格地图的分辨率，得到归一化的步长。
        // 时间差除以归一化的步长，得到最终的步长。
        // 综上所述，这段代码的作用是根据起始时间和结束时间之间的插值点之间的距离和网格地图的分辨率，计算出在 B-样条曲线上进行插值的步长。
        // 步长决定了在插值过程中采样的密度，使得插值点能够适应给定的分辨率。
        double t_step = (tmp - tm) / ((traj.evaluateDeBoorT(tmp) - traj.evaluateDeBoorT(tm)).norm() / grid_map_->getResolution());
        for (double t = tm; t < tmp * 2 / 3; t += t_step) // Only check the closest 2/3 partition of the whole trajectory.
        {
          flag_occ = grid_map_->getInflateOccupancy(traj.evaluateDeBoorT(t));
          if (flag_occ)
          {
            //cout << "hit_obs, t=" << t << " P=" << traj.evaluateDeBoorT(t).transpose() << endl;

            if (t <= bspline_interval_) // First 3 control points in obstacles!
            {
              cout << cps_.points.col(1).transpose() << "\n"
                   << cps_.points.col(2).transpose() << "\n"
                   << cps_.points.col(3).transpose() << "\n"
                   << cps_.points.col(4).transpose() << endl;
              ROS_WARN("First 3 control points in obstacles! return false, t=%f", t);
              return false; // 返回 false 意味着当前规划出的轨迹是无效的,程序根据状态机进行下一步操作。
            }

            break;  // 此时 break 用于 flag_occ为真 且 t > bspline_interval_ 的情况。
          }
        }

        /* 如果所有插值点均没有被障碍物占用,即!flag_occ为真,代码会执行以下操作:
        a. 使用printf函数输出当前迭代次数iter_num_、本次迭代所用时间time_ms、从开始到当前迭代所用的总时间total_time_ms以及当前代价函数值final_cost。输出的文本颜色为绿色。
        b. 将success标志设置为true,表示优化成功。 */
        if (!flag_occ)  // 所有插值点 均 没有被障碍物所占用
        {
          // \033[32m 是一个ANSI转义序列,用于设置输出文本的颜色为 绿色。
          // %5.3f是一个格式说明符,用于输出浮点数值,其中5表示输出字段的宽度为5个字符,.3表示保留3位小数。
          // \033[0m 是另一个ANSI转义序列,用于重置文本颜色和样式为 默认值。
          printf("\033[32miter(+1)=%d,time(ms)=%5.3f,total_t(ms)=%5.3f,cost=%5.3f\n\033[0m", iter_num_, time_ms, total_time_ms, final_cost);
          success = true;
        }
        else // restart 此时 flag_occ 为真 且 t > bspline_interval_，表示除前三个点之外的其他点 被障碍物占用
        {
          restart_nums++;
          // 通过重新初始化控制点,可以为优化过程提供一个全新的初始解,避免算法被困在之前的局部最优解附近。
          // 这有助于探索解空间的其他区域,增加找到更优解的可能性。
          initControlPoints(cps_.points, false);  
          // 当优化失败时,调整这个参数可以影响优化目标函数的形状,使得算法在新的初始解下有更大机会找到满意的解。
          // 重新初始化控制点可以与调整后的参数形成新的组合,为优化提供新的契机。
          // 增大这一权重会使优化过程更加侧重于避开障碍物,这在一定程度上牺牲了轨迹的平滑性和动力学可行性。
          new_lambda2_ *= 2;

          printf("\033[32miter(+1)=%d,time(ms)=%5.3f,keep optimizing\n\033[0m", iter_num_, time_ms);
        }
      }
      else if (result == lbfgs::LBFGSERR_CANCELED)  // 需要进行反弹操作,重新规划避开障碍物的路径。
      {
        flag_force_return = true;
        rebound_times++;
        cout << "iter=" << iter_num_ << ",time(ms)=" << time_ms << ",rebound." << endl;
      }
      else
      {
        ROS_WARN("Solver error. Return = %d, %s. Skip this planning.", result, lbfgs::lbfgs_strerror(result));
        // while (ros::ok());
      }

    } while ((flag_occ && restart_nums < MAX_RESART_NUMS_SET) ||
             (flag_force_return && force_stop_type_ == STOP_FOR_REBOUND && rebound_times <= 20));

    return success;
  }

  // 这段代码实现了一个基于L-BFGS算法的B-spline曲线优化过程,目的是在满足障碍物避障约束的前提下,优化B-spline曲线的平滑性。
  bool BsplineOptimizer::refine_optimize()
  {
    iter_num_ = 0;
    int start_id = order_;
    int end_id = this->cps_.points.cols() - order_;
    variable_num_ = 3 * (end_id - start_id);

    double q[variable_num_];
    double final_cost;

    // 将当前B-spline曲线的控制点拷贝到优化变量(向量)q中
    // cps_.points.data()返回cps_.points矩阵数据区的起始地址。
    // cps_.points.data() + 3 * start_id计算了需要复制的起始地址,从第start_id个控制点开始。
    // 由于每个控制点有3个坐标(x,y,z),所以起始地址需要偏移3 * start_id个元素。
    // sizeof(q)返回q数组中单个元素的字节大小。
    // memcpy是C++标准库中的内存复制函数,它的作用是从源地址复制一定字节数的数据到目标地址。
    memcpy(q, cps_.points.data() + 3 * start_id, variable_num_ * sizeof(q[0]));

    double origin_lambda4 = lambda4_;
    bool flag_safe = true;
    int iter_count = 0;
    do
    {
      lbfgs::lbfgs_parameter_t lbfgs_params;
      lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
      lbfgs_params.mem_size = 16;
      lbfgs_params.max_iterations = 200;
      lbfgs_params.g_epsilon = 0.001;

      /* 在B-spline优化器中,有两个回调函数需要特别注意:
          1. proc_progress回调函数
            在每次L-BFGS优化迭代后被调用,用于决定是否应该终止优化过程。此处为NULL
          2. proc_evaluate回调函数
            在每次优化迭代中被调用,用于计算当前控制点下的目标函数值和梯度。 */
      int result = lbfgs::lbfgs_optimize(variable_num_, q, &final_cost, BsplineOptimizer::costFunctionRefine, NULL, NULL, this, &lbfgs_params);
      if (result == lbfgs::LBFGS_CONVERGENCE ||
          result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
          result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
          result == lbfgs::LBFGS_STOP)
      {
        //pass
      }
      else
      {
        ROS_ERROR("Solver error in refining!, return = %d, %s", result, lbfgs::lbfgs_strerror(result));
      }

      /* 检查优化后的B-spline曲线轨迹是否与环境中的障碍物发生碰撞 */
      UniformBspline traj = UniformBspline(cps_.points, 3, bspline_interval_);
      double tm, tmp;
      traj.getTimeSpan(tm, tmp);
      /* grid_map_->getResolution()获取了环境栅格地图的分辨率,即每个栅格单元的边长。
         (traj.evaluateDeBoorT(tmp) - traj.evaluateDeBoorT(tm)).norm() / grid_map_->getResolution()
         计算了轨迹总长度在栅格地图中所占的栅格单元数。
         tmp - tm是traj的总时间长度。
         因此，t_step 代表了每个栅格单元对应的时间步长，保证每个栅格单元都能被筛查到。 */
      double t_step = (tmp - tm) / ((traj.evaluateDeBoorT(tmp) - traj.evaluateDeBoorT(tm)).norm() / grid_map_->getResolution()); // Step size is defined as the maximum size that can passes throgth every gird.
      for (double t = tm; t < tmp * 2 / 3; t += t_step)
      {
        if (grid_map_->getInflateOccupancy(traj.evaluateDeBoorT(t)))
        {
          // cout << "Refined traj hit_obs, t=" << t << " P=" << traj.evaluateDeBoorT(t).transpose() << endl;

          Eigen::MatrixXd ref_pts(ref_pts_.size(), 3);
          for (size_t i = 0; i < ref_pts_.size(); i++)
          {
            ref_pts.row(i) = ref_pts_[i].transpose();
          }

          flag_safe = false;
          break;
        }
      }

      if (!flag_safe)
        lambda4_ *= 2;

      iter_count++;
    } while (!flag_safe && iter_count <= 0);

    lambda4_ = origin_lambda4;

    //cout << "iter_num_=" << iter_num_ << endl;

    return flag_safe;
  }

  /* 计算B样条优化问题的综合代价函数值f_combine及其梯度grad。 */
  void BsplineOptimizer::combineCostRebound(const double *x, double *grad, double &f_combine, const int n)
  {
    // 将输入的控制点坐标x复制到cps_.points中,从第3*order_列开始。这里cps_是一个存储B样条曲线控制点的类。
    // n: The number of variables;  x: The array of variables.
    memcpy(cps_.points.data() + 3 * order_, x, n * sizeof(x[0]));

    /* ---------- evaluate cost and gradient ---------- */
    // 分别用于存储平滑性代价、距离代价和可行性代价
    double f_smoothness, f_distance, f_feasibility; 

    // 用于存储相应代价的梯度。
    Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_distance = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.size);

    // 调用calcSmoothnessCost函数计算平滑性代价f_smoothness及其梯度g_smoothness。
    calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
    // 计算B样条曲线与障碍物之间的距离代价(distance cost)及其梯度;这个函数需要传入当前迭代次数iter_num_和平滑性代价f_smoothness。
    calcDistanceCostRebound(cps_.points, f_distance, g_distance, iter_num_, f_smoothness);
    // 调用calcFeasibilityCost函数计算可行性代价f_feasibility及其梯度g_feasibility。
    calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);

    // 计算综合代价函数值f_combine,它是平滑性代价、距离代价和可行性代价的加权和。权重分别为lambda1_、new_lambda2_和lambda3_。
    f_combine = lambda1_ * f_smoothness + new_lambda2_ * f_distance + lambda3_ * f_feasibility;
    //printf("origin %f %f %f %f\n", f_smoothness, f_distance, f_feasibility, f_combine);

    // 计算综合梯度grad_3D,它也是平滑性梯度、距离梯度和可行性梯度的加权和,权重同上。
    Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + new_lambda2_ * g_distance + lambda3_ * g_feasibility;
    // 将grad_3D从第order_列开始的部分复制到输出梯度grad中。
    memcpy(grad, grad_3D.data() + 3 * order_, n * sizeof(grad[0]));
  }

  void BsplineOptimizer::combineCostRefine(const double *x, double *grad, double &f_combine, const int n)
  {

    // 将输入的控制点坐标x复制到cps_.points中,从第3*order_列开始。这里cps_是一个存储B样条曲线控制点的类。
    // n: The number of variables;  x: The array of variables.
    memcpy(cps_.points.data() + 3 * order_, x, n * sizeof(x[0]));

    /* ---------- evaluate cost and gradient ---------- */
    double f_smoothness, f_fitness, f_feasibility;

    Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    Eigen::MatrixXd g_fitness = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.points.cols());

    //time_satrt = ros::Time::now();

    calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
    calcFitnessCost(cps_.points, f_fitness, g_fitness);
    calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);

    /* ---------- convert to solver format...---------- */
    f_combine = lambda1_ * f_smoothness + lambda4_ * f_fitness + lambda3_ * f_feasibility;
    // printf("origin %f %f %f %f\n", f_smoothness, f_fitness, f_feasibility, f_combine);

    Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + lambda4_ * g_fitness + lambda3_ * g_feasibility;
    // 将grad_3D从第order_列开始的部分复制到输出梯度grad中。
    memcpy(grad, grad_3D.data() + 3 * order_, n * sizeof(grad[0]));
  }

} // namespace ego_planner