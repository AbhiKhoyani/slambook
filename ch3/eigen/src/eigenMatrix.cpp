#include <iostream>

#include <ctime>

//Eigen Core
#include <Eigen/Core>
#include <Eigen/Dense>

#define MATRIX_SIZE 50

int main(int argc, char **argv){
    Eigen::Matrix <float, 2, 3> matrix_23;

    Eigen::Vector3d v_3d;       //default double
    Eigen::Matrix <float, 3, 1> vd_3d;

    Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Zero();
    Eigen::Matrix <double, Eigen::Dynamic, Eigen::Dynamic> matrix_dynamic;
    Eigen::MatrixXd matrix_x;

    matrix_23 << 1, 2, 3, 4, 5, 6;

    std::cout << " matrix_23 from 1 to 6: " << matrix_23 << std::endl;

    v_3d << 1, 2, 3;
    vd_3d << 4, 5, 6;

    // matrix multiplication change dtype explicitely
    Eigen::Matrix <double, 2, 1> result  = matrix_23.cast<double>() * v_3d;
    std::cout << "result is: " << result.transpose() << std::endl;

    // Various Matrix Operations
    Eigen::Matrix3d mat_33 = Eigen::Matrix3d::Random();
    std::cout << "Random matrix: " << mat_33 << std::endl;
    std::cout << "Transpose: " << mat_33.transpose() << std::endl;
    std::cout << "Sum: " << mat_33.sum() << std::endl;
    std::cout << "Trace: " << mat_33.trace() << std::endl;
    std::cout << "Times 10: " << mat_33 * 10 << std::endl;
    std::cout << "Inverse: " << mat_33.inverse() << std::endl;
    std::cout << "Det: " << mat_33.determinant() << std::endl;

    // Find EigenValues of matrix
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(mat_33.transpose() * mat_33);
    std::cout << "Eigen values are: " << eigen_solver.eigenvalues() << std::endl;
    std::cout << "Eigen vectors are: " << eigen_solver.eigenvectors() << std::endl;

    // Solving Eqn
    Eigen::Matrix <double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN = Eigen::MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    matrix_NN = matrix_NN * matrix_NN.transpose();
    Eigen::Matrix <double, MATRIX_SIZE, 1> v_Nd = Eigen::MatrixXd::Random(MATRIX_SIZE, 1);

    clock_t start_time = clock();
    Eigen::Matrix <double, MATRIX_SIZE, 1> x = matrix_NN.inverse() * v_Nd;
    std::cout << "Normal time:" << 1000*(clock() - start_time)/(double)CLOCKS_PER_SEC << "ms" << std::endl;

    // Cholesky decomposition
    start_time = clock();
    x = matrix_NN.ldlt().solve(v_Nd);
    std::cout << "Normal time:" << 1000*(clock() - start_time)/(double)CLOCKS_PER_SEC << "ms" << std::endl;

    return 0;
}