#include <Eigen/Dense>
#include <iostream>
#include <cassert>

using namespace Eigen;
using std::cout;
using std::endl;

int main()
{
    /*----- 1. 构造 -----*/
    Matrix3d A;
    A << 1, 2, 3,
         4, 5, 6,
         7, 8, 10;          // 非奇异
    Vector3d b(3, 6, 9);

    /*----- 2. 线性求解 Ax = b -----*/
    Vector3d x = A.colPivHouseholderQr().solve(b);
    cout << "Solution x:\n" << x.transpose() << endl;

    /*----- 3. 验证残差 -----*/
    Vector3d r = A * x - b;
    cout << "residual norm: " << r.norm() << endl;
    assert(r.norm() < 1e-10 && "solve failed");

    /*----- 4. 特征值 -----*/
    SelfAdjointEigenSolver<Matrix3d> eig(A.transpose() * A);
    cout << "eigenvalues:\n" << eig.eigenvalues().transpose() << endl;
    assert(eig.info() == Success && "eig failed");

    /*----- 5. 小矩阵 SIMD 优化测试 -----*/
    Matrix4f P = Matrix4f::Random();
    Matrix4f K = P * P.transpose();
    Matrix4f L = K.llt().matrixL();
    cout << "4x4 Cholesky L(0,0) = " << L(0,0) << endl;

    cout << "\nAll Eigen tests passed!\n";
    return 0;
}