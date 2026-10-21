#include <iostream>
#include <traj_utils/polynomial_traj.h>

/* 这种通过最小化Jerk准则,可以得到一条平滑轨迹。 
 对应第五章节  参考博客：http://t.csdnimg.cn/9bRSy                                    
 需要注意的是 在代码中构造选择矩阵C时,中间点的位置约束并没有被视为未知自由变量,而是被作为已知条件处理了，
 因此，所生成的多项式轨迹将强制经过所有给定的中间点位置  */

 /* 通过闭式求解最小化Jerk问题时,代码只需要计算和处理自由变量(未知导数项)对应的矩阵Rpp和Rfp,而无需直接处理所有轨迹变量。具体来说:
  1. 代码先通过选择矩阵C,从所有导数项Dx,Dy,Dz中选取出已知导数条件Dxf,Dyf,Dzf。
  2. 然后根据Dxf,Dyf,Dzf的维度,将最小Jerk矩阵R分块为Rff,Rfp,Rpf,Rpp四个子矩阵。
  3. 最后通过闭式解Dxp = -(Rpp.inverse() * Rfp.transpose()) * Dxf 直接求解出自由变量Dxp,Dyp,Dzp。
  因此,在整个求解过程中,代码只需要计算和处理自由变量对应的Rpp和Rfp矩阵,而无需直接处理所有轨迹变量。
  相比之下,如果通过构造五次多项式方程组并解析求解,则需要直接处理所有轨迹变量,包括起点终点约束和所有中间点约束,从而涉及更多的未知数和方程。
  由于自由变量的数量远小于所有轨迹变量的数量,因此闭式求解方法所需计算量也相应更小。
  这正是闭式求解方法的计算量优势所在，也是该方法被EGO-Planner所采用的重要原因之一。

 */
PolynomialTraj PolynomialTraj::minJerkTraj(const Eigen::MatrixXd &Pos, 
                                           const Eigen::Vector3d &start_vel, const Eigen::Vector3d &end_vel, 
                                           const Eigen::Vector3d &start_acc, const Eigen::Vector3d &end_acc, 
                                           const Eigen::VectorXd &Time)
{
  int seg_num = Time.size();  // number of segment
  // 存储每个分段的多项式系数,每个分段有3个多项式(x,y,z),每个多项式有6个系数
  Eigen::MatrixXd poly_coeff(seg_num, 3 * 6);   // 每个分段对应一行，每一行对应有三个维度，每一个维度对应有6个参数
  // Px、Py、Pz对应了3个维度，每个维度对应存储每个分段的参数项，每个分段都是五次多项式，有6个系数
  Eigen::VectorXd Px(6 * seg_num), Py(6 * seg_num), Pz(6 * seg_num);
  // 对于每一维度上的每个分段，有 Pm(t) = p0 + p1*t + p2*t^2 + p3*t^3 + p4*t^4 + p5*t^5;

  int num_f, num_p; // number of fixed and free variables
  int num_d;        // number of all segments' derivatives

  // 这段代码定义了一个名为 Factorial 的静态 lambda 函数，接受一个整数 x 作为输入,用于计算 x 的阶乘。
  // lambda 函数是一种匿名函数，可以在代码中定义并使用，通常用于简化代码或在需要函数作为参数的情况下使用。
  const static auto Factorial = [](int x) {
    int fac = 1;
    for (int i = x; i > 0; i--)
      fac = fac * i;
    return fac;
  };

  /* ---------- 将输入的位置、速度、加速度转换为 边界值信息（导数约束，参考 L5 P48） ---------- */
  /* ---------- end point derivative ---------- */
  Eigen::VectorXd Dx = Eigen::VectorXd::Zero(seg_num * 6);  // 轨迹具有三个维度，每个维度都有 seg_num 段
  Eigen::VectorXd Dy = Eigen::VectorXd::Zero(seg_num * 6);  // 每段 对应具有 起始点 和 终止点
  Eigen::VectorXd Dz = Eigen::VectorXd::Zero(seg_num * 6);  // 每个点对应含 位置、速度、加速度 3条信息

  for (int k = 0; k < seg_num; k++) {   // 边界约束 包括 位置、速度和加速度；中间值约束 仅有 位置
    /* position to derivative */
    Dx(k * 6) = Pos(0, k);            // 第k段轨迹 x 方向的 起始点 位置
    Dx(k * 6 + 1) = Pos(0, k + 1);    // 第k段轨迹 x 方向的 结束点 位置
    Dy(k * 6) = Pos(1, k);            // 第k段轨迹 y 方向的 起始点 位置
    Dy(k * 6 + 1) = Pos(1, k + 1);    // 第k段轨迹 y 方向的 结束点 位置
    Dz(k * 6) = Pos(2, k);            // 第k段轨迹 z 方向的 起始点 位置
    Dz(k * 6 + 1) = Pos(2, k + 1);    // 第k段轨迹 z 方向的 结束点 位置

    if (k == 0)   // 第一段轨迹
    {
      Dx(k * 6 + 2) = start_vel(0);   // x 方向的 起始点 速度
      Dy(k * 6 + 2) = start_vel(1);   // y 方向的 起始点 速度
      Dz(k * 6 + 2) = start_vel(2);   // z 方向的 起始点 速度

      Dx(k * 6 + 4) = start_acc(0);   // x 方向的 起始点 加速度
      Dy(k * 6 + 4) = start_acc(1);   // y 方向的 起始点 加速度
      Dz(k * 6 + 4) = start_acc(2);   // z 方向的 起始点 加速度
    }
    else if (k == seg_num - 1)    // 最后一段轨迹
    {
      Dx(k * 6 + 3) = end_vel(0);   // x 方向的 终止点 速度
      Dy(k * 6 + 3) = end_vel(1);   // y 方向的 终止点 速度
      Dz(k * 6 + 3) = end_vel(2);   // z 方向的 终止点 速度

      Dx(k * 6 + 5) = end_acc(0);   // x 方向的 终止点 加速度
      Dy(k * 6 + 5) = end_acc(1);   // y 方向的 终止点 加速度
      Dz(k * 6 + 5) = end_acc(2);   // z 方向的 终止点 加速度
    }
  }

  // 对于每一段轨迹，有 [p(0) p(T) v(0) v(T) a(0) a(T)]^T = Ab * [p0 p1 p2 p3 p4 p5]^T
  // Ab = [1     0     0     0      0      0;
  //       1     T     T^2   T^3    T^4    T^5;
  //       0     1     0     0      0      0;
  //       0     1     2T    3T^2   4T^3   5T^4;  
  //       0     0     2     0      0      0;
  //       0     0     2     3*2T   4*3T   5*4T]

  /* ---------- Mapping Matrix A ---------- */
  // The purpose of this code segment is to construct the mapping matrix A, 
  // which relates the coefficients of the polynomial trajectory segments to their derivatives 
  // (position, velocity, acceleration, etc.).
  Eigen::MatrixXd Ab;
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(seg_num * 6, seg_num * 6);

  for (int k = 0; k < seg_num; k++)   // 遍历每个轨迹分段k
  {
    Ab = Eigen::MatrixXd::Zero(6, 6); // 对于每个分段k,首先初始化一个6x6的零矩阵Ab。
    for (int i = 0; i < 3; i++)       // 内层循环遍历i从0到2,对应位置、速度和加速度三个导数项。
    {
      Ab(2 * i, i) = Factorial(i);    // 对于每个i,计算Ab(2*i,i)的值,它对应位置、速度和加速度导数项的系数。这里使用Factorial(i)计算i的阶乘值。
      for (int j = i; j < 6; j++)     // 内层循环遍历 j 从i到5,对应高阶导数项
        Ab(2 * i + 1, j) = Factorial(j) / Factorial(j - i) * pow(Time(k), j - i);   // Time(k) = time/2;  对应第j阶导数项的系数
    }
    A.block(k * 6, k * 6, 6, 6) = Ab;
  }

  /* ---------- Produce Selection Matrix C' 构造选择矩阵 ---------- */
  Eigen::MatrixXd Ct, C;

  // 已知导数条件包括:
    // 第一个分段的起始位置、速度和加速度 (3个)
    // 最后一个分段的终止位置、速度和加速度 (3个)
    // 所有中间点(seg_num-1个）上的位置 (seg_num-1个）
  num_f = 2 * seg_num + 4; // 3 + 3 + (seg_num - 1) * 2 = 2m + 4    已知（固定）导数项数量 = 2m + 4 (m为分段数)
  // 未知导数条件包括:
    // 所有中间点(seg_num-1个）上的速度和加速度 ((seg_num-1)*2个)
  num_p = 2 * seg_num - 2; // (seg_num - 1) * 2 = 2m - 2             未知（自由）导数项数量 = 2m - 2
  num_d = 6 * seg_num;                                              // 所有分段导数项的总数，每段轨迹对应两个端点，每个端点对应p,v,a三个值
  Ct = Eigen::MatrixXd::Zero(num_d, num_f + num_p);
  Ct(0, 0) = 1;
  Ct(2, 1) = 1;
  Ct(4, 2) = 1; // stack the start point
  Ct(1, 3) = 1;
  Ct(3, 2 * seg_num + 4) = 1;
  Ct(5, 2 * seg_num + 5) = 1;

  // 最后一段
  Ct(6 * (seg_num - 1) + 0, 2 * seg_num + 0) = 1;
  Ct(6 * (seg_num - 1) + 1, 2 * seg_num + 1) = 1; // Stack the end point
  Ct(6 * (seg_num - 1) + 2, 4 * seg_num + 0) = 1;
  Ct(6 * (seg_num - 1) + 3, 2 * seg_num + 2) = 1; // Stack the end point
  Ct(6 * (seg_num - 1) + 4, 4 * seg_num + 1) = 1;
  Ct(6 * (seg_num - 1) + 5, 2 * seg_num + 3) = 1; // Stack the end point

  for (int j = 2; j < seg_num; j++)
  {
    Ct(6 * (j - 1) + 0, 2 + 2 * (j - 1) + 0) = 1;
    Ct(6 * (j - 1) + 1, 2 + 2 * (j - 1) + 1) = 1;
    Ct(6 * (j - 1) + 2, 2 * seg_num + 4 + 2 * (j - 2) + 0) = 1;
    Ct(6 * (j - 1) + 3, 2 * seg_num + 4 + 2 * (j - 1) + 0) = 1;
    Ct(6 * (j - 1) + 4, 2 * seg_num + 4 + 2 * (j - 2) + 1) = 1;
    Ct(6 * (j - 1) + 5, 2 * seg_num + 4 + 2 * (j - 1) + 1) = 1;
  }

  C = Ct.transpose();

  // 通过将C与Dx、Dy、Dz相乘,可以从所有导数项中选取出已知的导数条件,得到Dx1、Dy1、Dz1。
  Eigen::VectorXd Dx1 = C * Dx;   // Dx1 = [p1(T0) v1(T0) a1(T0) p1(T1) p2(T1) P2(T2) v2(T2) a2(T2) 0 0]^T
  Eigen::VectorXd Dy1 = C * Dy;   // 这里矩阵末尾的两个0是为了预留出free varible的空间
  Eigen::VectorXd Dz1 = C * Dz;

  /* ---------- minimum Jerk matrix ---------- */
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(seg_num * 6, seg_num * 6);

  for (int k = 0; k < seg_num; k++)
  {
    for (int i = 3; i < 6; i++)
    {
      for (int j = 3; j < 6; j++)
      {
        Q(k * 6 + i, k * 6 + j) =
            i * (i - 1) * (i - 2) * j * (j - 1) * (j - 2) / (i + j - 5) * pow(Time(k), (i + j - 5));
      }
    }
  }

  // 该部分代码将最小Jerk问题转化为二次规划问题,并求解出未知的导数项。
  /* ---------- R matrix ---------- */
  Eigen::MatrixXd R = C * A.transpose().inverse() * Q * A.inverse() * Ct;

  Eigen::VectorXd Dxf(2 * seg_num + 4), Dyf(2 * seg_num + 4), Dzf(2 * seg_num + 4);

  Dxf = Dx1.segment(0, 2 * seg_num + 4);  // 从 索引 0 开始，提取长度为 2 * seg_num + 4 的子序列
  Dyf = Dy1.segment(0, 2 * seg_num + 4);
  Dzf = Dz1.segment(0, 2 * seg_num + 4);

  Eigen::MatrixXd Rff(2 * seg_num + 4, 2 * seg_num + 4);
  Eigen::MatrixXd Rfp(2 * seg_num + 4, 2 * seg_num - 2);
  Eigen::MatrixXd Rpf(2 * seg_num - 2, 2 * seg_num + 4);
  Eigen::MatrixXd Rpp(2 * seg_num - 2, 2 * seg_num - 2);

  // 对 R 矩阵根据 Dxf 和 Dxp 的尺寸进行分块
  Rff = R.block(0, 0, 2 * seg_num + 4, 2 * seg_num + 4);
  Rfp = R.block(0, 2 * seg_num + 4, 2 * seg_num + 4, 2 * seg_num - 2);
  Rpf = R.block(2 * seg_num + 4, 0, 2 * seg_num - 2, 2 * seg_num + 4);
  Rpp = R.block(2 * seg_num + 4, 2 * seg_num + 4, 2 * seg_num - 2, 2 * seg_num - 2);

  /* ---------- close form solution ---------- */
  /* 闭式解指的是可以通过解析表达式或公式直接得到的解，而无需进行迭代或数值计算。 */
  Eigen::VectorXd Dxp(2 * seg_num - 2), Dyp(2 * seg_num - 2), Dzp(2 * seg_num - 2); // 2 * seg_num - 2 = 2*(seg_num - 1)
  Dxp = -(Rpp.inverse() * Rfp.transpose()) * Dxf;   // 直接套用 闭式解 求解公式
  Dyp = -(Rpp.inverse() * Rfp.transpose()) * Dyf;
  Dzp = -(Rpp.inverse() * Rfp.transpose()) * Dzf;

  // 将Dxp、Dyp、Dzp补充至Dx1、Dy1、Dz1末尾
  Dx1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dxp;
  Dy1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dyp;
  Dz1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dzp;

  Px = (A.inverse() * Ct) * Dx1;
  Py = (A.inverse() * Ct) * Dy1;
  Pz = (A.inverse() * Ct) * Dz1;

  for (int i = 0; i < seg_num; i++)
  {
    // poly_coeff 每个分段对应一行，每一行对应有三个维度，每一个维度对应有6个参数
    poly_coeff.block(i, 0, 1, 6) = Px.segment(i * 6, 6).transpose();
    poly_coeff.block(i, 6, 1, 6) = Py.segment(i * 6, 6).transpose();
    poly_coeff.block(i, 12, 1, 6) = Pz.segment(i * 6, 6).transpose();
  }

  /* ---------- use polynomials ---------- */
  PolynomialTraj poly_traj;
  for (int i = 0; i < poly_coeff.rows(); ++i)
  {
    vector<double> cx(6), cy(6), cz(6);
    for (int j = 0; j < 6; ++j)
    {
      cx[j] = poly_coeff(i, j), cy[j] = poly_coeff(i, j + 6), cz[j] = poly_coeff(i, j + 12);
    }
    reverse(cx.begin(), cx.end());  // 这里 取反 操作是因为 PolynomialTraj 类中多项式系数存储顺序刚好与这里相反
    reverse(cy.begin(), cy.end());  // 这里是对应 阶数 从小到大
    reverse(cz.begin(), cz.end());  // 而 PolynomialTraj 类中多项式系数存储顺序 是 阶数 从大到小
    double ts = Time(i);
    poly_traj.addSegment(cx, cy, cz, ts);
  }

  return poly_traj;   // poly_traj中存储了每一段轨迹在每一个维度上的多项式参数
}

// 根据起始点和终止点直接生成单段五次多项式轨迹,   参考第五章PPT P32    C * [Cofx,Cofy,Cpfz] = [Bx,By,Bz]
PolynomialTraj PolynomialTraj::one_segment_traj_gen(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                    const Eigen::Vector3d &end_pt, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc,
                                                    double t) // t 为该多项式轨迹的总时间
{
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(6, 6), Crow(1, 6);
  Eigen::VectorXd Bx(6), By(6), Bz(6);

  C(0, 5) = 1;
  C(1, 4) = 1;
  C(2, 3) = 2;
  Crow << pow(t, 5), pow(t, 4), pow(t, 3), pow(t, 2), t, 1;
  C.row(3) = Crow;
  Crow << 5 * pow(t, 4), 4 * pow(t, 3), 3 * pow(t, 2), 2 * t, 1, 0;
  C.row(4) = Crow;
  Crow << 20 * pow(t, 3), 12 * pow(t, 2), 6 * t, 2, 0, 0;
  C.row(5) = Crow;

  Bx << start_pt(0), start_vel(0), start_acc(0), end_pt(0), end_vel(0), end_acc(0);
  By << start_pt(1), start_vel(1), start_acc(1), end_pt(1), end_vel(1), end_acc(1);
  Bz << start_pt(2), start_vel(2), start_acc(2), end_pt(2), end_vel(2), end_acc(2);

  Eigen::VectorXd Cofx = C.colPivHouseholderQr().solve(Bx);   // 使用了 Eigen 库中的 QR 分解算法来高效求解线性方程组,获得多项式系数
  Eigen::VectorXd Cofy = C.colPivHouseholderQr().solve(By);   // Householder QR分解为求解多项式系数的线性方程组提供了一种数值稳定的方法。
  Eigen::VectorXd Cofz = C.colPivHouseholderQr().solve(Bz);

  vector<double> cx(6), cy(6), cz(6);
  for (int i = 0; i < 6; i++)
  {
    cx[i] = Cofx(i);
    cy[i] = Cofy(i);
    cz[i] = Cofz(i);
  }

  PolynomialTraj poly_traj;
  poly_traj.addSegment(cx, cy, cz, t);

  return poly_traj;
}
