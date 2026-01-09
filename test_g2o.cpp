#include <iostream>
#include <random>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/solver.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

// 1. 待优化参数：a、b、c 三个标量
//这一类需要加EIGEN_MAKE_ALIGNED_OPERATOR_NEW宏来对齐数据
class VertexParams : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  void setToOriginImpl() override { _estimate.setZero(); }
  void oplusImpl(const double* update) override {
    _estimate += Eigen::Vector3d(update);
  }
  bool read(std::istream&) override { return false; }
  bool write(std::ostream&) const override { return false; }
};

// 2. 一元边：误差 = y_measured - (a·x² + b·x + c)
class EdgeObservation : public g2o::BaseUnaryEdge<1, double, VertexParams> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeObservation(double x, double y) : _x(x), _y(y) {}
  void computeError() override {
    const VertexParams* v = static_cast<const VertexParams*>(_vertices[0]);
    const Eigen::Vector3d& abc = v->estimate();
    _error[0] = _y - (abc[0]*_x*_x + abc[1]*_x + abc[2]);
  }
  bool read(std::istream&) override { return false; }
  bool write(std::ostream&) const override { return false; }
private:
  double _x, _y;
};

int main() {
  // 1. 生成带噪声的二次曲线观测
  const double a_gt = 2.0, b_gt = -1.0, c_gt = 5.0;
  std::vector<std::pair<double,double>> data;
  std::mt19937 rng(0);
  std::normal_distribution<double> noise(0, 0.5);
  for (double x = -5; x <= 5; x += 0.5) {
    double y = a_gt*x*x + b_gt*x + c_gt + noise(rng);
    data.emplace_back(x, y);
  }

  // 2. 构建图
  typedef g2o::BlockSolver< g2o::BlockSolverTraits<3,1> >  BS;
  auto linearSolver = std::make_unique<g2o::LinearSolverDense<BS::PoseMatrixType>>();
  auto solver = new g2o::OptimizationAlgorithmLevenberg(
                  std::make_unique<BS>(std::move(linearSolver)));

  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);

  VertexParams* v = new VertexParams();
  v->setId(0);
  v->setEstimate(Eigen::Vector3d(1,1,1));   // 初值
  optimizer.addVertex(v);

  int edge_id = 1;
  for (const auto& xy : data) {
    EdgeObservation* e = new EdgeObservation(xy.first, xy.second);
    e->setId(edge_id++);
    e->setVertex(0, v);
    e->setInformation(Eigen::Matrix<double,1,1>::Identity());
    optimizer.addEdge(e);
  }

  // 3. 优化
  optimizer.initializeOptimization();
  optimizer.setVerbose(true);
  optimizer.optimize(10);

  // 4. 输出结果
  Eigen::Vector3d abc = v->estimate();
  std::cout << "\nGround truth:  a=" << a_gt << " b=" << b_gt << " c=" << c_gt << "\n"
            << "Estimated:     a=" << abc[0] << " b=" << abc[1] << " c=" << abc[2] << std::endl;
  return 0;
}