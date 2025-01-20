#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/SVD>
// #include <sophus/se3.hpp>
// #include <g2o/core/base_vertex.h>
// #include <g2o/core/base_unary_edge.h>
// #include <g2o/core/sparse_optimizer.h>
// #include <g2o/core/solver.h>
// #include <g2o/core/block_solver.h>
// #include <g2o/core/optimization_algorithm_gauss_newton.h>
// #include <g2o/solvers/dense/linear_solver_dense.h>
#include "get_orb.hpp"

void pose_estiamtion_3d3d(const std::vector<cv::Point3f> &points1, const std::vector<cv::Point3f> &points2,
                   cv::Mat &R, cv::Mat &t){
    
    // calculate centers
    cv::Point3f p1(0,0,0), p2(0,0,0);
    int N = points1.size();
    for(size_t i=0; i<N; i++){
        p1 += points1[i];
        p2 += points2[i];
    }
    p1 /= N;
    p2 /= N;

    // remove centers
    std::vector<cv::Point3f> q1, q2;
    for(size_t i=0; i<N; i++){
        q1.push_back(points1[i] - p1);
        q2.push_back(points2[i] - p1);
    }

    // calculate W = q1 *q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for(int i=0; i<N; i++){
        W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
    }
    std::cout << "W: \n" << W << std::endl;

    // decompose W by SVD
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    std::cout << "U: \n" << U << std::endl;
    std::cout << "V: \n" << V << std::endl;

    // R = U*V^T
    Eigen::Matrix3d _R = U * V.transpose();
    if(_R.determinant() < 0){
        _R = -_R;
    }

    // t = p1 - R*p2
    Eigen::Vector3d _t = Eigen::Vector3d(p1.x, p1.y, p1.z) - _R * Eigen::Vector3d(p2.x, p2.y, p2.z);

    // convert to cv::Mat
    R = cv::Mat_<double>(3,3) << _R(0,0), _R(0,1), _R(0,2)
                               , _R(1,0), _R(1,1), _R(1,2)
                               , _R(2,0), _R(2,1), _R(2,2); 
    t = cv::Mat_<double>(3,1) << _t(0,0), _t(1,0), _t(2,0);
    return;
}

cv::Point2d pixel2cam(cv::Point2d p, cv::Mat K){
    return cv::Point2d(
        (p.x - K.at<double>(0,0))/K.at<double>(0,2),
        (p.y - K.at<double>(1,1))/K.at<double>(1,2)
    );
}

int main(int argc , char **argv){

    // read images
    if(argc != 5){
        std::cerr << "Usage: pose_estimation_3d3d image1 depth_image1 image2 depth_image2" << std::endl;
        return 1;
    }

    // read images, get ORB keypoints, descriptors, and matches
    cv::Mat img1, depth_img1, img2, depth_img2;
    img1 = cv::imread(argv[1], cv::IMREAD_COLOR);
    depth_img1 = cv::imread(argv[2], cv::IMREAD_UNCHANGED);
    img2 = cv::imread(argv[3], cv::IMREAD_COLOR);
    depth_img2 = cv::imread(argv[4], cv::IMREAD_UNCHANGED);

    std::vector<cv::KeyPoint> kps1, kps2;
    std::vector<cv::DMatch> matches;
    extract_descriptor_matches(img1, img2, kps1, kps2, matches);
    
    // get 3D points of kps using intrinsic matrix K and read depth from depth_image:
    std::vector<cv::Point3f> points1;
    std::vector<cv::Point3f> points2;
    cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    for(size_t i=0; i<matches.size(); i++){
        ushort d1 = depth_img1.ptr<unsigned short>(int(kps1[matches[i].queryIdx].pt.y))[int(kps1[matches[i].queryIdx].pt.x)];
        ushort d2 = depth_img2.ptr<unsigned short>(int(kps2[matches[i].trainIdx].pt.y))[int(kps2[matches[i].trainIdx].pt.x)];
        if(d1==0 or d2==0)continue;     // bad depth
        float dd1 = d1/500.0, dd2=d2/500.0;

        cv::Point2d p1 = pixel2cam(kps1[matches[i].queryIdx].pt, K);
        cv::Point2d p2 = pixel2cam(kps2[matches[i].trainIdx].pt, K);
        points1.push_back(cv::Point3f(p1.x*dd1, p1.y*dd1, dd1));
        points2.push_back(cv::Point3f(p2.x*dd2, p2.y*dd2, dd2));
    }
    std::cout << "3D-3D pairs: " << points1.size() << std::endl;

    // solve 3D-3D ICP
    cv::Mat R, t;
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    pose_estiamtion_3d3d(points1, points2, R, t);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double>  time_diff =  std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
    std::cout << "R: \n" << R << std::endl;
    std::cout << "t: \n" << t << std::endl;
    std::cout << "Time take by ICP: " << time_diff.count() << std::endl;

    return 0;
}