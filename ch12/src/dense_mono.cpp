#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/highgui.hpp>
#include <sophus/se3.hpp>

#include "utils.hpp"

// using namespace Eigen;
// using namespace Sophus;
// using namespace std;
// using namespace cv;

// parameters
const int border = 20;
const int width = 640;
const int height = 480;
const double fx = 481.2f;
const double fy = -480.0f;
const double cx = 319.5f;
const double cy = 239.5f;
const int ncc_window_size = 3;
const int ncc_area = (2 * ncc_window_size + 1) * (2 * ncc_window_size + 1);
const double min_cov = 0.1;
const double max_cov = 10;

inline Eigen::Vector2d cam2px(const Eigen::Vector3d p_cam)
{
    return Eigen::Vector2d(p_cam(0, 0) * fx / p_cam(2, 0) + cx, p_cam(1, 0) * fy / p_cam(2, 0) + cy);
}

inline Eigen::Vector3d px2cam(const Eigen::Vector2d px)
{
    return Eigen::Vector3d((px(0, 0) - cx) / fx, (px(1, 0) - cy) / fy, 1);
}

inline bool inside(const Eigen::Vector2d &pt)
{
    return (pt(0, 0) >= border && pt(0, 0) <= width - border && pt(1, 0) >= border && pt(0, 0) <= height - border);
}

inline double getBilinearInterpolatedValue(const cv::Mat &img, const Eigen::Vector2d &pt)
{
    uchar *d = &img.data[int(pt(1, 0)) * img.step + int(pt(0, 0))];
    double xx = pt(0, 0) - std::floor(pt(0, 0));
    double yy = pt(1, 0) - std::floor(pt(1, 0));
    return ((1 - xx) * (1 - yy) * double(d[0]) + xx * (1 - yy) * double(d[1]) + (1 - xx) * yy * double(d[img.step]) +
            xx * yy * double(d[img.step + 1])) /
           255.0;
}

double NCC(const cv::Mat &ref_img, const cv::Mat &curr_img, const Eigen::Vector2d &ref_pt,
           const Eigen::Vector2d &curr_pt)
{
    double ref_mean = 0, curr_mean = 0;
    std::vector<double> ref_values, curr_values;
    for (int x = -ncc_window_size; x <= ncc_window_size; ++x)
        for (int y = -ncc_window_size; y <= ncc_window_size; ++y)
        {
            double ref_value = double(ref_img.ptr<uchar>(int(ref_pt(1, 0) + y))[int(ref_pt(0, 0) + x)]) / 255.0;
            ref_mean += ref_value;

            double curr_value = getBilinearInterpolatedValue(curr_img, curr_pt + Eigen::Vector2d(x, y));
            curr_mean += curr_value;
        }

    ref_mean /= ncc_area;
    curr_mean /= ncc_area;

    double numerator = 0, denominator1 = 0, denominator2 = 0;
    for (int i = 0; i < ref_values.size(); ++i)
    {
        double n = (ref_values[i] - ref_mean) * (curr_values[i] - curr_mean);
        numerator += n;
        denominator1 += (ref_values[i] - ref_mean) * (ref_values[i] - ref_mean);
        denominator2 += (curr_values[i] - curr_mean) * (curr_values[i] - curr_mean);
    }
    return numerator / std::sqrt(denominator1 * denominator2 + 1e-10);
}

bool epipolarSearch(const cv::Mat &ref_img, const cv::Mat &curr_img, const Sophus::SE3d rel_pose,
                    const Eigen::Vector2d &ref_pt, const double &depth_mean, const double &depth_std,
                    Eigen::Vector2d &curr_pt, Eigen::Vector2d &epipolar_direction)
{
    // get 3D point in reference camera
    Eigen::Vector3d f_ref = px2cam(ref_pt);
    f_ref.normalize();
    Eigen::Vector3d P_ref = f_ref * depth_mean;

    // Project reference 3D point in current camera, defining depth range to find
    // epipolar line: mean +/- 3*std
    Eigen::Vector2d curr_px_mean = cam2px(rel_pose * P_ref);
    double d_min = depth_mean - 3 * depth_std, d_max = depth_mean + 3 * depth_std;
    if (d_min < 0.1)
        d_min = 0.1;
    Eigen::Vector2d curr_pt_min = cam2px(rel_pose * (f_ref * d_min));
    Eigen::Vector2d curr_pt_max = cam2px(rel_pose * (f_ref * d_max));

    Eigen::Vector2d epipolar_line = curr_pt_max - curr_pt_min;
    epipolar_direction = epipolar_line;
    epipolar_direction.normalize();

    // this is just to reduce number of calculations
    double half_length = epipolar_line.norm() * 0.5;
    if (half_length > 100)
        half_length = 100;

    // Visualize epipolar line
    // showEpipolarLine(ref_img, curr_img, ref_pt, curr_pt_min, curr_pt_max);

    // block matching
    double best_ncc = -1.0;
    Eigen::Vector2d curr_pt_best;
    for (double l = -half_length; l <= half_length; l += 0.7)
    {
        Eigen::Vector2d temp_pt = curr_px_mean + (l * epipolar_direction);
        if (!inside(temp_pt))
            continue;

        double ncc = NCC(ref_img, curr_img, ref_pt, temp_pt);
        if (ncc > best_ncc)
        {
            best_ncc = ncc;
            curr_pt_best = temp_pt;
        }
    }

    if (best_ncc < 0.85f)
        return false;
    curr_pt = curr_pt_best;
    return true;
}

bool updateDepthFilter(const Eigen::Vector2d &ref_pt, const Eigen::Vector2d &curr_pt, const Sophus::SE3d &rel_pose,
                       const Eigen::Vector2d &epipolar_direction, cv::Mat &depth, cv::Mat &depth_cov2)
{
    Sophus::SE3d inverse_pose = rel_pose.inverse();
    Eigen::Vector3d f_ref = px2cam(ref_pt);
    f_ref.normalize();
    Eigen::Vector3d f_curr = px2cam(curr_pt);
    f_curr.normalize();

    Eigen::Vector3d t = inverse_pose.translation();
    Eigen::Vector3d f2 = inverse_pose.so3() * f_curr;
    Eigen::Vector2d b = Eigen::Vector2d(t.dot(f_ref), t.dot(f2));
    Eigen::Matrix2d A;
    A(0, 0) = f_ref.dot(f_ref);
    A(0, 1) = -f_ref.dot(f2);
    A(1, 0) = -A(0, 1);
    A(1, 1) = -f2.dot(f2);
    Eigen::Vector2d ans = A.inverse() * b;
    Eigen::Vector3d xm = ans[0] * f_ref;
    Eigen::Vector3d xn = t + ans[1] * f2;
    Eigen::Vector3d p_est = (xm + xn) / 2.0;
    double depth_estimation = p_est.norm();

    Eigen::Vector3d p = f_ref * depth_estimation;
    Eigen::Vector3d a = p - t;
    double t_norm = t.norm();
    double a_norm = a.norm();
    double alpha = std::acos(f_ref.dot(t) / t_norm);
    double beta = std::acos(-a.dot(t) / a_norm * t_norm);
    Eigen::Vector3d f_curr_prime = px2cam(curr_pt + epipolar_direction);
    f_curr_prime.normalize();
    double beta_prime = std::acos(f_curr_prime.dot(-t) / t_norm);
    double gamma = M_PI - alpha - beta_prime;
    double p_prime = t_norm * std::sin(beta_prime) / std::sin(gamma);
    double d_cov = p_prime - depth_estimation;
    double d_cov2 = d_cov * d_cov;

    double mu = depth.ptr<double>(int(ref_pt(1, 0)))[int(ref_pt(0, 0))];
    double sigma2 = depth_cov2.ptr<double>(int(ref_pt(1, 0)))[int(ref_pt(0, 0))];

    double mu_fuse = (d_cov2 * mu + sigma2 * depth_estimation) / (sigma2 + d_cov2);
    double sigma_fuse2 = (sigma2 * d_cov2) / (sigma2 + d_cov2);

    depth.ptr<double>(int(ref_pt(1, 0)))[int(ref_pt(0, 0))] = mu_fuse;
    depth_cov2.ptr<double>(int(ref_pt(1, 0)))[int(ref_pt(0, 0))] = sigma_fuse2;
    return true;
}

/**
 * Update depth for given image based on reference image & relative pose.
 * @param ref_img       reference image
 * @param curr_img      current image to calculate depth. Overlapping with
 * reference image is required for good result.
 * @param rel_pose      relative pose of current image w.r.t to reference image.
 * @param depth         calculated depth will be stored in this variable
 * @param depth_cov2    depth covariance
 */
void update(const cv::Mat &ref_img, const cv::Mat &curr_img, const Sophus::SE3d rel_pose, cv::Mat &depth,
            cv::Mat &depth_cov2)
{
    for (int x = border; x < width - border; ++x)
        for (int y = border; y < height - border; ++y)
        {
            if (depth_cov2.ptr<double>(y)[x] < min_cov || depth_cov2.ptr<double>(y)[x] > max_cov)
                continue;

            Eigen::Vector2d curr_pt;
            Eigen::Vector2d epipolar_direction;

            // find epipolar line in reference image
            bool ret = epipolarSearch(ref_img, curr_img, rel_pose, Eigen::Vector2d(x, y), depth.ptr<double>(y)[x],
                                      std::sqrt(depth_cov2.ptr<double>(y)[x]), curr_pt, epipolar_direction);

            if (ret == false)
                continue;

            showEpipolarMatch(ref_img, curr_img, Eigen::Vector2d(x, y), curr_pt);

            updateDepthFilter(Eigen::Vector2d(x, y), curr_pt, rel_pose, epipolar_direction, depth, depth_cov2);
        }
}

bool readDepthImage(const std::string &path, const std::string file_name, cv::Mat &ref_depth)
{
    std::fstream fin(path + "/depthmaps/" + file_name, std::ios::in);
    if (!fin.is_open())
        return false;
    ref_depth = cv::Mat(height, width, CV_64F);

    for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x)
        {
            double depth = 0;
            fin >> depth;
            ref_depth.ptr<double>(y)[x] = depth / 100.0;
        }

    fin.close();
    return true;
}

bool readDatasetFiles(const std::string &path, std::vector<std::string> &image_files, std::vector<Sophus::SE3d> &poses,
                      cv::Mat &ref_depth)
{
    std::fstream fin(path + "/first_200_frames_traj_over_table_input_sequence.txt", std::ios::in);

    if (!fin.is_open())
        return false;

    while (!fin.eof())
    {
        std::string image;
        fin >> image;

        double data[7];
        for (double &d : data)
            fin >> d;

        image_files.push_back(path + std::string("/images/") + image);
        poses.push_back(Sophus::SE3d(Eigen::Quaterniond(data[6], data[3], data[4], data[5]),
                                     Eigen::Vector3d(data[0], data[1], data[2])));

        if (!fin.good())
            break;
    }
    fin.close();

    return readDepthImage(path, "scene_000.depth", ref_depth);
}

void evaludateDepth(const cv::Mat &depth_truth, const cv::Mat &depth_estimate)
{
    double ave_depth_error = 0;
    double ave_depth_error_sq = 0;
    int cnt_depth_data = 0;
    for (int y = border; y < depth_truth.rows - border; ++y)
        for (int x = border; x < depth_truth.cols - border; ++x)
        {
            double error = depth_truth.ptr<double>(y)[x] - depth_estimate.ptr<double>(y)[x];
            ave_depth_error += error;
            ave_depth_error_sq += error * error;
            cnt_depth_data++;
        }
    ave_depth_error /= cnt_depth_data;
    ave_depth_error_sq /= cnt_depth_data;

    std::cout << "Average squared error = " << ave_depth_error_sq << ", average error: " << ave_depth_error
              << std::endl;
}

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cerr << "Usage: dense_mono <path_to_remode_dataset>" << std::endl;
        return -1;
    }

    // Load Dataset
    std::vector<std::string> image_files; // List of RGB images
    std::vector<Sophus::SE3d> poses;      // List of Poses
    cv::Mat ref_depth;                    // only storing current depth image, not list

    std::cout << argv[1] << std::endl;
    bool ret = readDatasetFiles(argv[1], image_files, poses, ref_depth);
    if (ret == false)
    {
        std::cerr << "Reading dataset failed.." << std::endl;
        return -1;
    }

    std::cout << "Reading total " << image_files.size() << " files." << std::endl;

    cv::Mat ref_img = cv::imread(image_files[0], 0);
    Sophus::SE3d ref_pose = poses[0];
    double init_depth = 3.0, init_cov2 = 3.0;
    cv::Mat depth(height, width, CV_64F, init_depth);
    cv::Mat depth_cov2(height, width, CV_64F, init_cov2);

    for (int index = 1; index < image_files.size(); ++index)
    {
        std::cout << "**** Loop: " << index << " ****" << std::endl;
        cv::Mat curr_img = cv::imread(image_files[index], 0);
        if (curr_img.data == nullptr)
            continue;

        Sophus::SE3d current_pose = poses[index];
        // Sophus::SE3d current_pose_ref = ref_pose.inverse() * current_pose;
        Sophus::SE3d current_pose_ref = current_pose.inverse() * ref_pose;

        // Calculate depth from both image, and relative pose
        update(ref_img, curr_img, current_pose_ref, depth, depth_cov2);
        evaludateDepth(ref_depth, depth);
        plotDepth(ref_depth, depth);
        cv::waitKey(1);
    }

    return 0;
}
