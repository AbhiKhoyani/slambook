#include <iostream>
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/solver.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include "get_orb.hpp"


typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;
typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVector3d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

class VertexPose:public g2o::BaseVertex<6, Sophus::SE3d>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        virtual void setToOriginImpl() override {
            _estimate = Sophus::SE3d();
        }

        // implementing left multiplication of SE3d
        virtual void oplusImpl(const double *update) override{
            Eigen::Matrix<double, 6, 1> update_eigen;
            update_eigen << update[0], update[1], 
            update[2], update[3], update[4], update[5];

            _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
        }

        virtual bool read(std::istream &in) override {return true;}
        virtual bool write(std::ostream &out) const override {return true;}
};

class EdgeProjection: public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPose>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        // constructor
        EdgeProjection(const Eigen::Vector3d &pos, const Eigen::Matrix3d &K): _pos3d(pos), _K(K){}
        
        virtual void computeError() override {
            const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
            Sophus::SE3d T = v->estimate();
            Eigen::Vector3d pos_pixel = _K * (T * _pos3d);
            pos_pixel /= pos_pixel[2];
            _error = _measurement - pos_pixel.head<2>();
        }

        virtual void linearizeOplus() override {
            const VertexPose *v = static_cast<VertexPose *> (_vertices[0]);
            Sophus::SE3d T = v->estimate();
            Eigen::Vector3d pos_cam = T * _pos3d;
            double fx = _K(0, 0);
            double fy = _K(1, 1);
            double cx = _K(0, 2);
            double cy = _K(1, 2);
            double X = pos_cam[0];
            double Y = pos_cam[1];
            double Z = pos_cam[2];
            double Z2 = Z * Z;
            _jacobianOplusXi << -fx / Z, 0, fx * X / Z2, fx * X * Y / Z2, -fx - fx * X * X / Z2, fx * Y / Z,
                                0, -fy / Z, fy * Y / Z2, fy + fy * Y * Y / Z2, -fy * X * Y / Z2, -fy * X / Z;
        }
        virtual bool read(std::istream &in) override {return true;}
        virtual bool write(std::ostream &out) const override {return true;}

    private:
        Eigen::Vector3d _pos3d;
        Eigen::Matrix3d _K;
};

void bundleAdjustmentG2O(
    const VecVector3d &points3d, const VecVector2d &points2d,
    const cv::Mat &K, Sophus::SE3d &pose
){
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

    auto solver = new g2o::OptimizationAlgorithmGaussNewton(std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    // vertex
    VertexPose *vertex_pose = new VertexPose();
    vertex_pose->setId(0);
    vertex_pose->setEstimate(Sophus::SE3d());
    optimizer.addVertex(vertex_pose);

    Eigen::Matrix3d K_eigen;
    K_eigen << 
    K.at<double>(0, 0), K.at<double>(0, 1), K.at<double>(0, 2), 
    K.at<double>(1, 0), K.at<double>(1, 1), K.at<double>(1, 2), 
    K.at<double>(2, 0), K.at<double>(2, 1), K.at<double>(2, 2);

    // add edges
    int index=1;
    for(size_t i=0; i<points3d.size(); ++i){
        auto p2d = points2d[i];
        auto p3d = points3d[i];
        EdgeProjection *edge = new EdgeProjection(p3d, K_eigen);
        edge->setId(index);
        edge->setVertex(0, vertex_pose);
        edge->setMeasurement(p2d);
        edge->setInformation(Eigen::Matrix2d::Identity());
        optimizer.addEdge(edge);
        index++;
    }

    optimizer.initializeOptimization();
    optimizer.optimize(10);
    pose = vertex_pose->estimate();
    std::cout << "pose estimated by g2o =\n" << pose.matrix() << std::endl;
}

void bundleAdjustmentGaussNewton(
    const VecVector3d &points3d, const VecVector2d &points2d,
    const cv::Mat &K, Sophus::SE3d &pose
){
    const int iterations = 10;
    double cost = 0, lastCost = 0;
    double fx = K.at<double>(0,0);
    double fy = K.at<double>(1,1);
    double cx = K.at<double>(0,2);
    double cy = K.at<double>(1,2);

    for(int iter=0; iter<iterations; iter++){
        Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();
        cost = 0;
        for(size_t i=0; i<points3d.size(); i++){
            // calculate cost: based on projecting 3D points from 1st image to 2nd image
            Eigen::Vector3d pc = pose * points3d[i];
            double inv_z = 1.0 / pc[2];
            double inv_z2 = inv_z * inv_z;
            Eigen::Vector2d proj(fx*(pc[0]/pc[2])+cx, fy*(pc[1]/pc[2])+cy);
            Eigen::Vector2d e = points2d[i] - proj;
            cost += e.squaredNorm();

            Eigen::Matrix<double, 2, 6> J;
            J << -fx*inv_z, 0, fx*pc[0]*inv_z2, fx*pc[0]*pc[1]*inv_z2, -fx-(fx*pc[0]*pc[0]*inv_z2), fx*pc[1]*inv_z,
            0, -fy*inv_z, fy*pc[1]*inv_z2, fy+fy*pc[1]*pc[1]*inv_z2, -fy*pc[0]*pc[1]*inv_z2, -fy*pc[0]*inv_z;

            H += J.transpose()*J;
            b += -J.transpose()*e;
        }
        Vector6d dx;
        dx = H.ldlt().solve(b);

        if(std::isnan(dx[0])){
            std::cout << "result is nan!" << std::endl;
            break;
        }

        if(iter<0 && cost>= lastCost){
            std::cout << "cost: " << cost << "last cost: " << lastCost << std::endl;
            break;
        }

        pose = Sophus::SE3d::exp(dx)*pose;
        lastCost = cost;
        std::cout << "Iteration: " << iter << "cost= " << std::cout.precision(12) << cost << std::endl;
        if(dx.norm() < 1e-6){
            break;
        }
    }
    std::cout << "Pose by Gauss-Newton: \n" << pose.matrix() << std::endl;
}

cv::Point2d pixel2cam(cv::Point2d p, cv::Mat K){
    return cv::Point2d(
        (p.x - K.at<double>(0,0))/K.at<double>(0,2),
        (p.y - K.at<double>(1,1))/K.at<double>(1,2)
    );
}

int main(int argc , char **argv){

    // read images
    if(argc != 4){
        std::cerr << "Usage: pose_estimation_3d2d image1 depth_image1 image2" << std::endl;
        return 1;
    }

    // read images, get ORB keypoints, descriptors, and matches
    cv::Mat img1, depth_img1, img2;
    img1 = cv::imread(argv[1], cv::IMREAD_COLOR);
    depth_img1 = cv::imread(argv[2], cv::IMREAD_UNCHANGED);
    img2 = cv::imread(argv[3], cv::IMREAD_COLOR);

    std::vector<cv::KeyPoint> kps1, kps2;
    std::vector<cv::DMatch> matches;
    extract_descriptor_matches(img1, img2, kps1, kps2, matches);
    
    // get 3D points of kps using intrinsic matrix K and read depth from depth_image:
    std::vector<cv::Point3f> points3d;
    std::vector<cv::Point2f> points2d;
    cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    for(size_t i=0; i<matches.size(); i++){
        ushort d = depth_img1.ptr<unsigned short>(int(kps1[matches[i].queryIdx].pt.y))[int(kps1[matches[i].queryIdx].pt.x)];
        if(d==0)continue;     // bad depth
        float dd = d/500.0;

        cv::Point2d p1 = pixel2cam(kps1[matches[i].queryIdx].pt, K);
        points3d.push_back(cv::Point3f(p1.x*dd, p1.y*dd, dd));
        points2d.push_back(kps2[matches[i].trainIdx].pt);
    }
    std::cout << "3D-2D pairs: " << points3d.size() << std::endl;

    // calculate R,t using cv::SolvePNP
    cv::Mat R, r, t;
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    cv::solvePnP(points3d, points2d, K, cv::Mat(), r, t);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double>  time_diff =  std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
    cv::Rodrigues(r, R);    // rvec to rotation matrix
    std::cout << "R: \n" << R << std::endl;
    std::cout << "t: \n" << t << std::endl;
    std::cout << "Time take by CV:SolvePNP: " << time_diff.count() << std::endl;

    // Now solve pose by gauss-newton from scratch
    VecVector3d points3d_eigen;
    VecVector2d points2d_eigen;
    for(size_t i=0; i<points3d.size(); i++){
        points3d_eigen.push_back(Eigen::Vector3d(points3d[i].x, points3d[i].y, points3d[i].z));
        points2d_eigen.push_back(Eigen::Vector2d(points2d[i].x, points2d[i].y));
    }
    std::cout << "......Calling BundleAdjustment by Gauss-Newton method........." <<std::endl;
    Sophus::SE3d pose_gn;
    t1 = std::chrono::steady_clock::now();   
    bundleAdjustmentGaussNewton(points3d_eigen, points2d_eigen, K, pose_gn);
    t2 = std::chrono::steady_clock::now();
    time_diff = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
    std::cout << "Time take by Gauss-Newton: " << time_diff.count() << std::endl;

    // Now solve pose by g2o
    // In g2o, both image will be vertex(with ist pose), edge with relative transformation (R, t).
    std::cout << "......Calling BundleAdjustment by G2O method........." <<std::endl;
    Sophus::SE3d pose_g2o;
    t1 = std::chrono::steady_clock::now();   
    bundleAdjustmentG2O(points3d_eigen, points2d_eigen, K, pose_g2o);
    t2 = std::chrono::steady_clock::now();
    time_diff = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
    std::cout << "Time taken by G2O: " << time_diff.count() << std::endl;

    return 0;
}