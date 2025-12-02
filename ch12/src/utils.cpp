#include "utils.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;

void plotDepth(const cv::Mat &gt_depth, const cv::Mat &estimate_depth)
{
    imshow("gt_depth", gt_depth * 0.4);
    imshow("estimate_depth", estimate_depth * 0.4);
    imshow("depth_error", gt_depth - estimate_depth);
    waitKey(1);
}

void showEpipolarMatch(const cv::Mat &ref_img, const cv::Mat &curr_img, const Eigen::Vector2d &ref_px, const Eigen::Vector2d &curr_px)
{
    cv::Mat ref_show, curr_show;
    cv::cvtColor(ref_img, ref_show, cv::COLOR_GRAY2BGR);
    cv::cvtColor(curr_img, curr_show, cv::COLOR_GRAY2BGR);

    cv::circle(ref_show, cv::Point2f(ref_px(0, 0), ref_px(1, 0)), 5, cv::Scalar(0, 0, 250), 2);
    cv::circle(curr_show, cv::Point2f(curr_px(0, 0), curr_px(1, 0)), 5, cv::Scalar(0, 0, 250), 2);

    imshow("ref", ref_show);
    imshow("curr", curr_show);
    waitKey(1);
}

void showEpipolarLine(const cv::Mat &ref_img, const cv::Mat &curr_img, const Eigen::Vector2d &ref_px, const Eigen::Vector2d &curr_px_min,
                      const Eigen::Vector2d &curr_px_max)
{
    cv::Mat ref_show, curr_show;
    cv::cvtColor(ref_img, ref_show, cv::COLOR_GRAY2BGR);
    cv::cvtColor(curr_img, curr_show, cv::COLOR_GRAY2BGR);

    cv::circle(ref_show, cv::Point2f(ref_px(0, 0), ref_px(1, 0)), 5, cv::Scalar(0, 255, 0), 2);
    cv::circle(curr_show, cv::Point2f(curr_px_min(0, 0), curr_px_min(1, 0)), 5, cv::Scalar(0, 255, 0), 2);
    cv::circle(curr_show, cv::Point2f(curr_px_max(0, 0), curr_px_max(1, 0)), 5, cv::Scalar(0, 255, 0), 2);
    cv::line(curr_show, Point2f(curr_px_min(0, 0), curr_px_min(1, 0)), Point2f(curr_px_max(0, 0), curr_px_max(1, 0)),
             Scalar(0, 255, 0), 1);

    imshow("ref", ref_show);
    imshow("curr", curr_show);
    waitKey(1);
}
