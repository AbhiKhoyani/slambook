
#include <opencv2/opencv.hpp>

void extract_descriptor_matches(cv::Mat img1, cv::Mat img2, 
                                std::vector<cv::KeyPoint> &kps1, std::vector<cv::KeyPoint> &kps2,
                                std::vector<cv::DMatch> &matches);