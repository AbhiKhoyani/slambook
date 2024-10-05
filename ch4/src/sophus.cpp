#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/se3.hpp"

using namespace std;
using namespace Eigen;

int main(int argc, char** argv){
    
    cout << "************** Rotaion Matrix ************* " << endl;
    // Sophus use with rotation matrix
    Matrix3d r = AngleAxis( M_PI /2 , Vector3d(0, 0, 1)).toRotationMatrix();
    Quaterniond q(r);
    Sophus::SO3d SO3_R(r);
    Sophus::SO3d SO3_Q(q);

    cout << "SO(3) from Matrix: \n" << SO3_R.matrix() << endl;
    cout << "SO(3) from Quaternion: \n" << SO3_Q.matrix() << endl;

    // Lie algebra
    Vector3d so3 = SO3_R.log();
    cout << "so3: \n" << so3.transpose() << endl;
    cout << "so3 hat: \n" << Sophus::SO3d::hat(so3) << endl;
    cout << "so3 vee: \n" << Sophus::SO3d::vee(Sophus::SO3d::hat(so3)).transpose() << endl;
    
    // Add perturbation:
    Vector3d update_so3(1e-4, 0, 0);
    Sophus::SO3d so3d_updated = Sophus::SO3d::exp(update_so3)*SO3_R;
    cout << "so3 updated: \n" << so3d_updated.matrix() << endl;
    
    cout << "************** Homogeneous Matrix ************* " << endl;
    // Sophus homogeneous matrix
    Vector3d t(1, 0, 0);
    Sophus::SE3d SE3d_Rt(r, t);
    Sophus::SE3d SE3d_qt(q, t);
    cout << "SE3d R,t: \n" << SE3d_Rt.matrix() << endl;
    cout << "SE3d q,t: \n" << SE3d_qt.matrix() << endl;

    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d se3 = SE3d_Rt.log();
    cout << "SE3d log: \n" << se3.transpose() << endl;
    cout << "SE3 hat: \n" << Sophus::SE3d::hat(se3) << endl;
    cout << "SE3 vee: \n" << Sophus::SE3d::vee(Sophus::SE3d::hat(se3)).transpose() << endl;

    // Update 6d:
    Vector6d update_se3;
    update_se3.setZero();
    update_se3(0, 0) = 1e-4d;
    Sophus::SE3d SE3d_updated = Sophus::SE3d::exp(update_se3) * SE3d_Rt;
    cout << "Updated SE3d: \n" << SE3d_updated.matrix() << endl;

    return 0;
}