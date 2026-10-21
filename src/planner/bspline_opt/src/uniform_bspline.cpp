#include "bspline_opt/uniform_bspline.h"
#include <ros/ros.h>

namespace ego_planner
{

  UniformBspline::UniformBspline(const Eigen::MatrixXd &points, const int &order,
                                 const double &interval)
  {
    setUniformBspline(points, order, interval);
  }

  UniformBspline::~UniformBspline() {}

  void UniformBspline::setUniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval)
  {
    control_points_ = points; // 每一列为一个控制点，共有 points.cols() 个控制点
    p_ = order;
    interval_ = interval;

    n_ = points.cols() - 1;   // 这里的 n_ 即为 PPT中的 N，控制点一共有 N+1 个，也就是说控制点下标为 0~N
    m_ = n_ + p_ + 1;         // 节点向量的最大下标 M，一个有M+1个下标，也就是说节点向量下标为 0~M 

    u_ = Eigen::VectorXd::Zero(m_ + 1);   // B样条曲线的节点向量
    // 将节点向量 构造为一个等距节点向量
    for (int i = 0; i <= m_; ++i)
    {

      if (i <= p_)    // u_(0) = -3 * ts; u_(1) = -2 * ts; u_(2) = -1 * ts; u_(3) = = 0;
      {
        u_(i) = double(-p_ + i) * interval_;    
      }
      else if (i > p_ && i <= m_ - p_)  // 下标 3 之前为负，从 3 开始非负，对应B样条曲线阶数，从而可以对应这里t_pb = t_3 = 0
      {
        u_(i) = u_(i - 1) + interval_;
      }
      else if (i > m_ - p_)
      {
        u_(i) = u_(i - 1) + interval_;
      }
    }
  }

  void UniformBspline::setKnot(const Eigen::VectorXd &knot) { this->u_ = knot; }

  Eigen::VectorXd UniformBspline::getKnot() { return this->u_; }

  // 获取 B-样条曲线的时间节点向量的参数空间范围   A B-spline trajectory is parameterized by time t, where t ∈ [t_pb, t_{m-pb}].
  bool UniformBspline::getTimeSpan(double &um, double &um_p)
  {
    if (p_ > u_.rows() || m_ - p_ > u_.rows())  // 正常情况下，t_pb,t_{m-pb} < u_.rows()
      return false;

    um = u_(p_);            
    um_p = u_(m_ - p_);

    return true;
  }

  Eigen::MatrixXd UniformBspline::getControlPoint() { return control_points_; }

  // 在 B-样条曲线上进行 De Boor 算法的插值计算，给定参数 u，返回曲线上的点。
  Eigen::VectorXd UniformBspline::evaluateDeBoor(const double &u)
  {
    // 将参数 u 限制在 [u_(p_), u_(m_ - p_)] 区间内。
    // max(u_(p_), u)确保参数 u 不小于 u_(p_);min(max(u_(p_), u), u_(m_ - p_)) 确保参数 u 不大于 u_(m_ - p_)。
    double ub = min(max(u_(p_), u), u_(m_ - p_));   

    // determine which [ui,ui+1] lay in   确定参数 u 所在的节点区间 [u_k, u_k+1]。
    int k = p_;
    while (true)
    {
      if (u_(k + 1) >= ub)
        break;  // 找到满足 u_(k + 1) >= ub 的最小 k 值。这个 k 值就是参数 u 所在节点区间的起始索引。
      ++k;
    }

    /* deBoor's alg 一种高效计算 B 样条曲线点坐标的递归算法。*/
    vector<Eigen::VectorXd> d;  // 存储插值计算过程中的临时结果，注意这里存储的是坐标值而不是 B样条基函数
    for (int i = 0; i <= p_; ++i)   // 初始化向量 d,将从索引 k - p_ 开始的 p_ + 1 个控制点存储在 d 中,作为算法的初始值。
    {
      d.push_back(control_points_.col(k - p_ + i));
      // cout << d[i].transpose() << endl;
    }

    /* d[0] d[1] d[2] d[3]
          d[1] d[2] d[3]
            d[2] d[3]
               d[3] */
    for (int r = 1; r <= p_; ++r)   // 迭代计算 p_ 次，每次迭代都使用线性插值的方式,将相邻的两个点合并为一个新的点。
    {
      for (int i = p_; i >= r; --i)
      {
        double alpha = (ub - u_[i + k - p_]) / (u_[i + 1 + k - r] - u_[i + k - p_]);  // 第 i 个点的权重值
        // cout << "alpha: " << alpha << endl;
        d[i] = (1 - alpha) * d[i - 1] + alpha * d[i];   // 注意这里是直接计算的坐标值而不是 B样条基函数，因此有别于B样条基函数的计算公式
      }
    }

    return d[p_];
  }

  // Eigen::VectorXd UniformBspline::evaluateDeBoorT(const double& t) {
  //   return evaluateDeBoor(t + u_(p_));
  // }

  /* 计算控制点的导数，B样条的导数还是一个B样条，控制点数量相应少一个 */
  Eigen::MatrixXd UniformBspline::getDerivativeControlPoints()
  {
    // The derivative of a b-spline is also a b-spline, its order become p_-1
    // control point Qi = p_*(Pi+1-Pi)/(ui+p_+1-ui+1)
    Eigen::MatrixXd ctp(control_points_.rows(), control_points_.cols() - 1);
    for (int i = 0; i < ctp.cols(); ++i)
    {
      ctp.col(i) =
          p_ * (control_points_.col(i + 1) - control_points_.col(i)) / (u_(i + p_ + 1) - u_(i + 1));
    }
    return ctp;
  }

  /* 计算当前 B 样条曲线的导数 */
  UniformBspline UniformBspline::getDerivative()
  {
    // 获取当前 B 样条曲线的导数控制点矩阵 ctp。
    Eigen::MatrixXd ctp = getDerivativeControlPoints();
    UniformBspline derivative(ctp, p_ - 1, interval_);

    /* cut the first and last knot */
    /* 
      修改 derivative 对象的节点向量 u_。
      由于 B 样条曲线的导数是在内部节点上定义的,因此需要去掉首尾两个节点。
      于是使用 u_.segment(1, u_.rows() - 2) 获取中间的节点向量,并通过 setKnot() 函数将其设置为 derivative 的新节点向量。 */
    Eigen::VectorXd knot(u_.rows() - 2);
    knot = u_.segment(1, u_.rows() - 2);  // 从 u_ 的第 2 个元素开始(下标为 1),复制 u_.rows() - 2 个元素
    derivative.setKnot(knot);

    return derivative;
  }

  double UniformBspline::getInterval() { return interval_; }

  void UniformBspline::setPhysicalLimits(const double &vel, const double &acc, const double &tolerance)
  {
    limit_vel_ = vel;
    limit_acc_ = acc;
    limit_ratio_ = 1.1;
    feasibility_tolerance_ = tolerance;   // feasibility_tolerance_是一个容差参数,用于放大速度限制以提供一些容错空间。
  }

  /* 检查给定的B样条曲线轨迹是否满足速度和加速度的动力学约束,并计算出需要对时间进行重新分配的比例ratio */
  bool UniformBspline::checkFeasibility(double &ratio, bool show)
  {
    bool fea = true;

    Eigen::MatrixXd P = control_points_;
    int dimension = control_points_.rows();

    /* check vel feasibility and insert points */
    double max_vel = -1.0;  // 记录轨迹上的最大速度分量。
    double enlarged_vel_lim = limit_vel_ * (1.0 + feasibility_tolerance_) + 1e-4;
    for (int i = 0; i < P.cols() - 1; ++i)
    {
      // 估计计算B样条曲线在第i和第i+1个控制点之间的平均速度。
      Eigen::VectorXd vel = p_ * (P.col(i + 1) - P.col(i)) / (u_(i + p_ + 1) - u_(i + 1)); 

      if (fabs(vel(0)) > enlarged_vel_lim || fabs(vel(1)) > enlarged_vel_lim || fabs(vel(2)) > enlarged_vel_lim)
      {

        if (show)
          cout << "[Check]: Infeasible vel " << i << " :" << vel.transpose() << endl;
        fea = false;

        for (int j = 0; j < dimension; ++j)
        {
          max_vel = max(max_vel, fabs(vel(j)));
        }
      }
    }

    /* acc feasibility */
    double max_acc = -1.0;  // 记录轨迹上的最大加速度分量。
    double enlarged_acc_lim = limit_acc_ * (1.0 + feasibility_tolerance_) + 1e-4;
    for (int i = 0; i < P.cols() - 2; ++i)
    {

      Eigen::VectorXd acc = p_ * (p_ - 1) *
                            ((P.col(i + 2) - P.col(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
                             (P.col(i + 1) - P.col(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
                            (u_(i + p_ + 1) - u_(i + 2));

      if (fabs(acc(0)) > enlarged_acc_lim || fabs(acc(1)) > enlarged_acc_lim || fabs(acc(2)) > enlarged_acc_lim)
      {

        if (show)
          cout << "[Check]: Infeasible acc " << i << " :" << acc.transpose() << endl;
        fea = false;

        for (int j = 0; j < dimension; ++j)
        {
          max_acc = max(max_acc, fabs(acc(j)));
        }
      }
    }

    ratio = max(max_vel / limit_vel_, sqrt(fabs(max_acc) / limit_acc_));

    return fea;
  }
  
  /* 延长B-spline曲线的时间尺度，只对中间部分的节点进行时间尺度的调整。 */
  void UniformBspline::lengthenTime(const double &ratio)
  {
    /* 只对中间部分的节点进行时间尺度的调整,避免影响曲线的整体形状。 */
    int num1 = 5;   // num1和num2分别表示B-spline曲线节点向量u_中，需要调整的起始和终止节点的索引。
    int num2 = getKnot().rows() - 1 - 5;

    double delta_t = (ratio - 1.0) * (u_(num2) - u_(num1)); // 需要增加的总时间量
    double t_inc = delta_t / double(num2 - num1);  // 每个中间节点的时间增量
    for (int i = num1 + 1; i <= num2; ++i)
      u_(i) += double(i - num1) * t_inc;  // 对于 num1 + 1 到 num2 之间的节点,时间值增加 (i - num1) * t_inc。
    for (int i = num2 + 1; i < u_.rows(); ++i)
      u_(i) += delta_t;   // 对于 num2 + 1 到最后几个节点,时间值均增加 delta_t。
  }

  // void UniformBspline::recomputeInit() {}
  // B样条曲线的插值，根据已知的路径点和导数信息,计算出一条满足这些约束的B样条曲线。
  // 将给定的 点集 和 起始/终止导数信息 转换为 均匀B样条曲线 控制点
  void UniformBspline::parameterizeToBspline(const double &ts, const vector<Eigen::Vector3d> &point_set,
                                             const vector<Eigen::Vector3d> &start_end_derivative,
                                             Eigen::MatrixXd &ctrl_pts) {
    if (ts <= 0)
    {
      cout << "[B-spline]:time step error." << endl;
      return;
    }

    if (point_set.size() <= 3)    // 生成一段 3次B样条 至少需要 4个 控制点
    {
      cout << "[B-spline]:point set have only " << point_set.size() << " points." << endl;
      return;
    }

    if (start_end_derivative.size() != 4)
    {
      cout << "[B-spline]:derivatives error." << endl;
    }

    int K = point_set.size();

    // 创建行向量 prow、vrow 和 arow，分别表示均匀 B 样条曲线计算中的位置项、速度项和加速度项的系数（权重）
    // write A 
    Eigen::Vector3d prow(3), vrow(3), arow(3);  // 使用了<<运算符对prow、vrow和arow进行赋值 会将向量转换为行向量。
    prow << 1, 4, 1;  // 在计算均匀 B 样条曲线的位置约束时，中间的控制点的权重是4，而两端的控制点的权重是1。
    vrow << -1, 0, 1;
    arow << 1, -2, 1;

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K + 4, K + 2);  // 用于存储 B 样条曲线的系数矩阵。调整这些权重，可以对曲线的形状和特性进行控制。

    for (int i = 0; i < K; ++i)
      A.block(i, i, 1, 3) = (1 / 6.0) * prow.transpose(); // 1/6 + 4/6 + 1/6 = 1

    A.block(K, 0, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();  // x = v * t
    A.block(K + 1, K - 1, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();

    A.block(K + 2, 0, 1, 3) = (1 / ts / ts) * arow.transpose();     // Delta_x = a * t^2
    A.block(K + 3, K - 1, 1, 3) = (1 / ts / ts) * arow.transpose();

    //cout << "A" << endl << A << endl << endl;

    // write b
    Eigen::VectorXd bx(K + 4), by(K + 4), bz(K + 4);
    for (int i = 0; i < K; ++i)
    {
      bx(i) = point_set[i](0);
      by(i) = point_set[i](1);
      bz(i) = point_set[i](2);
    }

    for (int i = 0; i < 4; ++i)
    {
      bx(K + i) = start_end_derivative[i](0);
      by(K + i) = start_end_derivative[i](1);
      bz(K + i) = start_end_derivative[i](2);
    }

    // solve Ax = b  ————  A * px = bx
    Eigen::VectorXd px = A.colPivHouseholderQr().solve(bx);
    Eigen::VectorXd py = A.colPivHouseholderQr().solve(by);
    Eigen::VectorXd pz = A.colPivHouseholderQr().solve(bz);

    // convert to control pts 每一行对应一个维度
    ctrl_pts.resize(3, K + 2);
    ctrl_pts.row(0) = px.transpose();
    ctrl_pts.row(1) = py.transpose();
    ctrl_pts.row(2) = pz.transpose();

    // cout << "[B-spline]: parameterization ok." << endl;
  }

  // 计算 B-spline 曲线的总时间跨度。
  double UniformBspline::getTimeSum()
  {
    double tm, tmp;
    if (getTimeSpan(tm, tmp))
      return tmp - tm;
    else
      return -1.0;
  }

  double UniformBspline::getLength(const double &res)
  {
    double length = 0.0;
    double dur = getTimeSum();
    Eigen::VectorXd p_l = evaluateDeBoorT(0.0), p_n;
    for (double t = res; t <= dur + 1e-4; t += res)
    {
      p_n = evaluateDeBoorT(t);
      length += (p_n - p_l).norm();
      p_l = p_n;
    }
    return length;
  }

  double UniformBspline::getJerk()
  {
    UniformBspline jerk_traj = getDerivative().getDerivative().getDerivative();

    Eigen::VectorXd times = jerk_traj.getKnot();
    Eigen::MatrixXd ctrl_pts = jerk_traj.getControlPoint();
    int dimension = ctrl_pts.rows();

    double jerk = 0.0;
    for (int i = 0; i < ctrl_pts.cols(); ++i)
    {
      for (int j = 0; j < dimension; ++j)
      {
        jerk += (times(i + 1) - times(i)) * ctrl_pts(j, i) * ctrl_pts(j, i);
      }
    }

    return jerk;
  }

  void UniformBspline::getMeanAndMaxVel(double &mean_v, double &max_v)
  {
    UniformBspline vel = getDerivative();
    double tm, tmp;
    vel.getTimeSpan(tm, tmp);

    double max_vel = -1.0, mean_vel = 0.0;
    int num = 0;
    for (double t = tm; t <= tmp; t += 0.01)
    {
      Eigen::VectorXd vxd = vel.evaluateDeBoor(t);
      double vn = vxd.norm();

      mean_vel += vn;
      ++num;
      if (vn > max_vel)
      {
        max_vel = vn;
      }
    }

    mean_vel = mean_vel / double(num);
    mean_v = mean_vel;
    max_v = max_vel;
  }

  void UniformBspline::getMeanAndMaxAcc(double &mean_a, double &max_a)
  {
    UniformBspline acc = getDerivative().getDerivative();
    double tm, tmp;
    acc.getTimeSpan(tm, tmp);

    double max_acc = -1.0, mean_acc = 0.0;
    int num = 0;
    for (double t = tm; t <= tmp; t += 0.01)
    {
      Eigen::VectorXd axd = acc.evaluateDeBoor(t);
      double an = axd.norm();

      mean_acc += an;
      ++num;
      if (an > max_acc)
      {
        max_acc = an;
      }
    }

    mean_acc = mean_acc / double(num);
    mean_a = mean_acc;
    max_a = max_acc;
  }
} // namespace ego_planner
