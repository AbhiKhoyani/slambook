#include <iostream>
#include <fstream>
#include <boost/format.hpp>
#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>
#include <sophus/se3.hpp>
#include <unistd.h>

using namespace std;
using namespace Sophus;

typedef vector<SE3d, Eigen::aligned_allocator<SE3d>> TrajectoryClass;
typedef vector<Vector6d, Eigen::aligned_allocator<Vector6d>> pointcloud;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

void showPointcloud(pointcloud pcd){

    if (pcd.empty()) {
        cerr << "Point cloud is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("PCD Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
    pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto &p:pcd){
            glColor3d(p[3]/255.0, p[4]/255.0, p[5]/255.0);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}

int main(int argc, char** argv){

    vector<cv::Mat> colorImgs, depthImgs;
    TrajectoryClass poses;
    ifstream fin("./data/pose.txt");
    if(!fin){
        cerr << "Invalid File" << endl;
        return 1;
    }
    
    for(int i=0; i<5; i++){
        boost::format fmt("./%s/%s/%d.%s");
        colorImgs.push_back(cv::imread((fmt % "data" %"color" %(i+1) %"png").str()));
        depthImgs.push_back(cv::imread((fmt % "data" %"depth" %(i+1) %"pgm").str(), -1));       // -1 flag for laoding depth image

        double tx, ty, tz, qx, qy, qz, qw;
        fin >> tx >> ty >> tz >> qx >> qy >> qz >> qw ;
        SE3d pose(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(tx, ty, tz));
        poses.push_back(pose);
    }
    
    // compute the point cloud using camera intrinsics
    double cx = 325.5;
    double cy = 253.5;
    double fx = 518.0;
    double fy = 519.0;
    double depthScale = 1000.0;
    
    pointcloud pcd;
    pcd.reserve(1000000);

    for (int i=0; i<5; i++){
        cout << "Converting RGBD image: " << i+1 << endl;
        cv::Mat color = colorImgs[i];
        cv::Mat depth = depthImgs[i];
        SE3d pose = poses[i];
        for(int v=0; v < color.rows; v++)
        for(int u=0; u < color.cols; u++){
            unsigned int d = depth.ptr<unsigned short>(v)[u]; //depth value is 16bit
            if(d==0) continue;
            Eigen::Vector3d point;
            point[2] = double(d) / depthScale;
            point[0] = (u - cx)*point[2] / fx;
            point[1] = (v - cy)*point[2] / fy;

            point = pose*point;
            // Eigen::Vector3d pointworld = pose*point;
            
            Vector6d p;
            p.head<3>() = point;
            // p.head<3>() = pointworld;
            p[5] = color.data[ v*color.step + u*color.channels()];  // blue
            p[4] = color.data[ v*color.step + u*color.channels() + 1];  // green
            p[3] = color.data[ v*color.step + u*color.channels() + 2];  // red
            pcd.push_back(p);
        }
    }

    cout << "Total number of points: " << pcd.size() << endl;
    showPointcloud(pcd);
    return 0;
}
