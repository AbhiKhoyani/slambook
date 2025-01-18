#include <iostream>
#include "get_orb.hpp"

void pose_estimation_2d2d(std::vector<cv::KeyPoint> kps1, std::vector<cv::KeyPoint> kps2,
                            std::vector<cv::DMatch> matches,
                            cv::Mat &R, cv::Mat &t){
    // initialize camera intrinsics from TUM dataset
    cv::Mat K = (cv::Mat_<double>(3,3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

    // convert matching point from matches to vector<Point2f>
    std::vector<cv::Point2f> points1, points2;
    for(size_t i=0; i< matches.size(); i++){
        points1.push_back(kps1[matches[i].queryIdx].pt);
        points2.push_back(kps2[matches[i].trainIdx].pt);
    }

    // calculate fundamental matrix, Essential matrix, 
    // and Homogeneous Matrix(just for the sack of learning else H doesn;t makes sense in non-planner kps)
    cv::Mat F, E, H;
    F = cv::findFundamentalMat(points1, points2, cv::FM_8POINT );
    std::cout << "Fundamental matrix: \n" << F << std::endl;

    cv::Point2d principal_point(325.1, 249.7);
    double focal_length = 521;
    E = cv::findEssentialMat(points1, points2, K);
    std::cout << "Essential matrix: \n" << E << std::endl;

    H = cv::findHomography(points1, points2, cv::RANSAC);
    std::cout << "Homogeneous matrix: \n" << H << std::endl;

    // recover R and t from essential matrix
    cv::recoverPose(E, points1, points2, K, R, t);
    std::cout << "R: \n" << format(R, cv::Formatter::FMT_NUMPY) << std::endl;
    std::cout << "t: \n" << t << std::endl;
}

// pixel2cam converts 2D pixel (uv) to 3D point (xy in xyz) in camera coordinates
// uv = K @ p
cv::Point2d pixel2cam(cv::Point2d point, cv::Mat K){
    return cv::Point2d(
        (point.x - K.at<double>(0,2) / K.at<double>(0,0)),
        (point.y - K.at<double>(0,1) / K.at<double>(1,1))
    );
}

// adding triangulation part here itself.
// traignulation calculates depth of each point based on R,t
void triangulation(std::vector<cv::KeyPoint> kps1, std::vector<cv::KeyPoint> kps2,
                            std::vector<cv::DMatch> matches,
                            cv::Mat R, cv::Mat t, std::vector<cv::Point3d> &points){
    cv::Mat T1 = (cv::Mat_<float>(3,4) <<   1,0,0,0,
                                            0,1,0,0,
                                            0,0,1,0 );
    cv::Mat T2 = (cv::Mat_<float>(3,4) <<   R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0,0),
                                            R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1,0),
                                            R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2,0));
    cv::Mat K = (cv::Mat_<double>(3,3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    
    std::vector<cv::Point2f> pts1, pts2;
    for(cv::DMatch &m:matches){
        pts1.push_back(pixel2cam(kps1[m.queryIdx].pt, K));
        pts2.push_back(pixel2cam(kps2[m.trainIdx].pt, K));
    }

    cv::Mat pts_4d;
    cv::triangulatePoints(T1, T2, pts1, pts2, pts_4d);

    // convert points Mat(4,1) to Point3d(3,1)
    for(int i=0; i<pts_4d.cols; i++){
        cv::Mat x = pts_4d.col(i);
        x /= x.at<float>(3,0);
        cv::Point3d p(x.at<float>(0,0), x.at<float>(1,0), x.at<float>(2,0));
        points.push_back(p);
    }
}

inline cv::Scalar get_color(float depth) {
  float up_th = 50, low_th = 10, th_range = up_th - low_th;
  if (depth > up_th) depth = up_th;
  if (depth < low_th) depth = low_th;
  return cv::Scalar(255 * depth / th_range, 0, 255 * (1 - depth / th_range));
}

int main(int argc, char **argv){
    // check if 3 args there:
    if(argc!=3){
        std::cerr << "Usage: pose_estimation_2d2d image1 image2" << std::endl;
        return 1;
    }

    // read imgaes
    cv::Mat img1 = cv::imread(argv[1], cv::IMREAD_COLOR);
    cv::Mat img2 = cv::imread(argv[2], cv::IMREAD_COLOR);
    assert(img1.data != nullptr && img2.data != nullptr);

    std::vector<cv::KeyPoint> kps1, kps2;
    std::vector<cv::DMatch> matches;
    cv::Mat R, t;
    extract_descriptor_matches(img1, img2, kps1, kps2, matches);
    pose_estimation_2d2d(kps1, kps2, matches, R, t);

    std::vector<cv::Point3d> points;
    triangulation(kps1, kps2, matches, R, t, points);

    // draw depth points
    cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    cv::Mat img1_plot = img1.clone();
    cv::Mat img2_plot = img2.clone();
    for (int i = 0; i < matches.size(); i++) {
        float depth1 = points[i].z;
        std::cout << "depth: " << depth1 << std::endl;
        cv::Point2d pt1_cam = pixel2cam(kps1[matches[i].queryIdx].pt, K);
        cv::circle(img1_plot, kps1[matches[i].queryIdx].pt, 2, get_color(depth1), 2);

        cv::Mat pt2_trans = R * (cv::Mat_<double>(3, 1) << points[i].x, points[i].y, points[i].z) + t;
        float depth2 = pt2_trans.at<double>(2, 0);
        cv::circle(img2_plot, kps2[matches[i].trainIdx].pt, 2, get_color(depth2), 2);
    }
    cv::imshow("img 1", img1_plot);
    cv::imshow("img 2", img2_plot);
    cv::waitKey();

    return 0;
}