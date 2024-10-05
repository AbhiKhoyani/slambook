#include <iostream>
#include <fstream>
#include <unistd.h>
#include <pangolin/pangolin.h>
#include <sophus/se3.hpp>

using namespace Sophus;
using namespace std;

typedef vector<SE3d, Eigen::aligned_allocator<SE3d>> trajectoryClass;

trajectoryClass read_poses(string filename){

    trajectoryClass poses;
    ifstream fin(filename);
    if (!fin){
        cout << "Invalid file path!: " << filename << endl;
        return poses;
    }

    while (!fin.eof()){
        double time, tx, ty, tz, qx, qy, qz, qw;
        fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        SE3d pose(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(tx, ty, tz));
        poses.push_back(pose);
    }

    return poses;
}

void show_trajectory(const trajectoryClass &gt_traj, const trajectoryClass &estim_traj){
    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
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
        glLineWidth(2);

        // connecting lines
        for (size_t i = 0; i < gt_traj.size(); i++) {
            glColor3f(0.0, 0.0, 1.0);
            glBegin(GL_LINES);
            auto p1 = gt_traj[i], p2 = gt_traj[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        for (size_t i = 0; i < estim_traj.size(); i++) {
            glColor3f(1.0, 0.0, 0.0);
            glBegin(GL_LINES);
            auto p1 = estim_traj[i], p2 = estim_traj[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}

int main(int argc, char** argv){
    // read input file, and save trajectory
    string gt_file = "./data/groundtruth.txt";
    string estim_file = "./data/estimated.txt";

    trajectoryClass gt_traj = read_poses(gt_file);
    trajectoryClass estim_traj = read_poses(estim_file);
    assert(!gt_traj.empty() && !estim_traj.empty());
    assert(gt_traj.size() == estim_traj.size());

    // Calculate error from two trajectory
    double rmse = 0;
    for (size_t i=0; i < gt_traj.size(); i++){
        SE3d p1 = estim_traj[i];
        SE3d p2 = gt_traj[i];
        double error = (p2.inverse()*p1).log().norm();
        rmse += error*error;
    }
    rmse = rmse/double(gt_traj.size());
    rmse = sqrt(rmse);
    cout << "RMSE: " << rmse << endl;

    // draw trajectory in pangolin
    show_trajectory(gt_traj, estim_traj);
    return 0;
}