#include <fstream>
#include <iostream>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/format.hpp>
#include <opencv2/opencv.hpp>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std::chrono_literals;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PcdT;

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

    std::cout << "Converting RGB + depth image to Pointcloud.." << std::endl;

    PcdT::Ptr pcd(new PcdT);
    for (int i = 0; i < 5; i++)
    {
        PcdT::Ptr current(new PcdT);
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

                PointT p;
                p.x = pointworld[0];
                p.y = pointworld[1];
                p.z = pointworld[2];
                p.b = color.data[v * color.step + u * color.channels()];     // blue
                p.g = color.data[v * color.step + u * color.channels() + 1]; // green
                p.r = color.data[v * color.step + u * color.channels() + 2]; // red
                current->points.push_back(p);
            }
        // depth filter and statistical removal
        PcdT::Ptr tmp(new PcdT);
        pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
        statistical_filter.setMeanK(50);
        statistical_filter.setStddevMulThresh(1.0);
        statistical_filter.setInputCloud(current);
        statistical_filter.filter(*tmp);
        (*pcd) += *tmp;
    }

    pcd->is_dense = false;
    std::cout << "We have " << pcd->size() << " points." << std::endl;

    // voxel filter
    pcl::VoxelGrid<PointT> voxel_filter;
    double resolution = 0.03;
    voxel_filter.setLeafSize(resolution, resolution, resolution);
    PcdT::Ptr tmp(new PcdT);
    voxel_filter.setInputCloud(pcd);
    voxel_filter.filter(*tmp);
    tmp->swap(*pcd);
    std::cout << "After voxelization, we have " << pcd->size() << " points." << std::endl;

    // Visualization
    pcl::visualization::PCLVisualizer viz;
    viz.setBackgroundColor(0, 0, 0);
    viz.addPointCloud<PointT>(pcd, "point cloud");
    viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "point cloud");
    viz.addCoordinateSystem(1.0);
    viz.initCameraParameters();
    viz.resetCamera();
    viz.spin();

    pcl::io::savePCDFileBinary("map.pcd", *pcd);
    return 0;
}