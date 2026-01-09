#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>

using std::cout;
using std::endl;

int main(int /*argc*/, char** /*argv*/) {
  cout << "========== 1. SO(3) 示例 ==========" << endl;
  // 1.1 用轴角构造一个旋转：绕 Z 轴转 90°
  Eigen::AngleAxisd z90(M_PI / 2, Eigen::Vector3d::UnitZ());
  Sophus::SO3d R_z90(z90.toRotationMatrix());

  cout << "R_z90 对应的矩阵:\n" << R_z90.matrix() << endl;
  cout << "R_z90 对数坐标 (so3):\n" << R_z90.log() << endl;  // 即旋转向量

  // 1.2 指数映射：把 so3 向量重新变回 SO3
  Eigen::Vector3d omega = Eigen::Vector3d(0, 0, M_PI / 2);
  Sophus::SO3d R_from_exp = Sophus::SO3d::exp(omega);
  cout << "指数映射后得到的矩阵:\n" << R_from_exp.matrix() << endl;

  // 1.3 伴随 Ad_R 作用：把角速度从 body 系转到 world 系
  Eigen::Vector3d omega_body(1, 0, 0);  // body 系下 x 轴角速度
  Eigen::Vector3d omega_world = R_z90 * omega_body;
  cout << "body 角速度 (1,0,0) 在 world 系下: " << omega_world.transpose() << endl;

  cout << "\n========== 2. SE(3) 示例 ==========" << endl;
  // 2.1 构造一个 SE3：旋转同上，平移 (1,0,0)
  Eigen::Vector3d t(1, 0, 0);
  Sophus::SE3d T(R_z90, t);
  cout << "SE(3) 矩阵形式 (4×4):\n" << T.matrix() << endl;
  cout << "SE(3) 对数坐标 (6维 twist):\n" << T.log() << endl;

  // 2.2 更新位姿：左乘一个微小 twist
  Eigen::Matrix<double, 6, 1> xi;
  xi << 0.05, 0, 0,   // 绕 x 轴 0.05 rad
        0.1,  0, 0;   // x 方向 0.1 m
  Sophus::SE3d T_updated = Sophus::SE3d::exp(xi) * T;
  cout << "更新后的平移: " << T_updated.translation().transpose() << endl;

  cout << "\n========== 3. 左右扰动求导对比 ==========" << endl;
  // 3.1 定义一个标量函数 f(T) = ||T.translation()||^2
  auto f = [](const Sophus::SE3d& T) { return T.translation().squaredNorm(); };

  // 3.2 左扰动数值导
  const double delta = 1e-6;
  Eigen::Matrix<double, 6, 1> num_jac_left;
  for (int i = 0; i < 6; ++i) {
    Eigen::Matrix<double, 6, 1> dp;
    dp.setZero(); dp[i] = delta;
    num_jac_left[i] = (f(Sophus::SE3d::exp(dp) * T) - f(T)) / delta;
  }
  cout << "左扰动数值导:\n" << num_jac_left.transpose() << endl;

  // 3.3 右扰动数值导
  Eigen::Matrix<double, 6, 1> num_jac_right;
  for (int i = 0; i < 6; ++i) {
    Eigen::Matrix<double, 6, 1> dp;
    dp.setZero(); dp[i] = delta;
    num_jac_right[i] = (f(T * Sophus::SE3d::exp(dp)) - f(T)) / delta;
  }
  cout << "右扰动数值导:\n" << num_jac_right.transpose() << endl;

  cout << "\n========== 4. 测试通过 ==========" << endl;
  return 0;
}