#ifndef UTILS_HPP
#define UTILS_HPP

#include <Eigen/Core>
#include <opencv2/core.hpp>

void plotDepth(const cv::Mat &gt_depth, const cv::Mat &estimate_depth);

void showEpipolarMatch(const cv::Mat &ref_img, const cv::Mat &curr_img, const Eigen::Vector2d &ref_px, const Eigen::Vector2d &curr_px);

void showEpipolarLine(const cv::Mat &ref_img, const cv::Mat &curr_img, const Eigen::Vector2d &ref_px, const Eigen::Vector2d &curr_px_min,
                      const Eigen::Vector2d &curr_px_max);

#endif // UTILS_HPP