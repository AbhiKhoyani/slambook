#include <fstream>
#include <iostream>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/format.hpp>
#include <opencv2/opencv.hpp>

#include <octomap/octomap.h>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{

    if (argc != 2)
    {
        std::cerr << "Usage: pcd_mapping <data-directory>" << std::endl;
        return -1;
    }
    std::string data_directory = argv[1];

    std::ifstream fin(data_directory + "/pose.txt");
    if (!fin.is_open())
    {
        std::cerr << "No pose.txt found!" << std::endl;
        return -1;
    }

    std::vector<cv::Mat> color_imgs, depth_imgs;
    std::vector<Eigen::Isometry3d> poses;

    for (int i = 0; i < 5; i++)
    {
        boost::format fmt(data_directory + "/%s/%d.%s");
        color_imgs.push_back(cv::imread((fmt % "color" % (i + 1) % "png").str()));
        depth_imgs.push_back(cv::imread((fmt % "depth" % (i + 1) % "png").str(), -1));

        double data[7] = {0};
        for (int i = 0; i < 7; i++)
        {
            fin >> data[i];
        }
        Eigen::Quaterniond q(data[6], data[3], data[4], data[5]);
        Eigen::Isometry3d T(q);
        T.pretranslate(Eigen::Vector3d(data[0], data[1], data[2]));
        poses.push_back(T);
    }

    // merge point clouds
    double cx = 319.5;
    double cy = 239.5;
    double fx = 481.2;
    double fy = -480.0;
    double depthScale = 5000.0;

    std::cout << "Converting RGB + depth image to Octomap..." << std::endl;

    octomap::OcTree tree(0.01);
    for (int i = 0; i < 5; i++)
    {
        octomap::Pointcloud current;
        std::cout << "Image: " << i + 1 << std::endl;
        cv::Mat color = color_imgs[i];
        cv::Mat depth = depth_imgs[i];
        Eigen::Isometry3d T = poses[i];

        for (int v = 0; v < color.rows; v++)
            for (int u = 0; u < color.cols; u++)
            {
                unsigned int d = depth.ptr<unsigned short>(v)[u]; // depth value is 16bit
                if (d == 0)
                    continue;
                Eigen::Vector3d point;
                point[2] = double(d) / depthScale;
                point[0] = (u - cx) * point[2] / fx;
                point[1] = (v - cy) * point[2] / fy;
                Eigen::Vector3d pointworld = T * point;

                current.push_back(pointworld[0], pointworld[1], pointworld[2]);
            }

        tree.insertPointCloud(current, octomap::point3d(T(0, 3), T(1, 3), T(2, 3)));
    }

    tree.updateInnerOccupancy();
    std::cout << "Saving octomap..." << std::endl;
    tree.writeBinary("octomap.bt");
    return 0;
}