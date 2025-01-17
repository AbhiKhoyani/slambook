#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>


int main(int argc, char **argv){

    // gt value and initial estimation for the curve y = exp(a*x*x + b*x + c)
    double ar = 1.0, br = 2.0, cr = 3.0;
    double ae = 2.0, be = 10.0, ce = -1.5;

    // Generate N data points by adding noise
    int N = 100;
    double w_sigma = 1.0;
    double inv_sigma = 1 / w_sigma;
    cv::RNG rng;

    std::vector<double> x_data, y_data;
    for(int i=0; i<N; i++){
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(std::exp(ar*x*x + br*x + cr) + rng.gaussian(w_sigma*w_sigma));
    }

    // Main loop for opitimization H * delta_x = b for number of iteration
    int iteration = 100;
    double cost = 0, lastCost = 0;
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

    for(int iter=0; iter<iteration; iter++){
        Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
        Eigen::Vector3d b = Eigen::Vector3d::Zero();
        cost = 0;

        // run over all N data-points
        for(int i=0; i<N; i++){
            double xi = x_data[i], yi =  y_data[i];
            double error = yi - (std::exp(ae*xi*xi + be*xi + ce));

            Eigen::Vector3d J;
            J[0] = -xi*xi*std::exp(ae*xi*xi + be*xi + ce);  // de/da
            J[1] = -xi*std::exp(ae*xi*xi + be*xi + ce);     // de/db
            J[2] = -std::exp(ae*xi*xi + be*xi + ce);        // de/dc

            H += inv_sigma * inv_sigma * J * J.transpose();
            b += -inv_sigma * inv_sigma * error * J;
            cost += error*error;
        }

        // solve Hx=b
        Eigen::Vector3d dx = H.ldlt().solve(b);
        if(std::isnan(dx[0])){
            std::cout << "result is Nan!" << std::endl;
            break;
        }

        // check if cost is actually decreasing
        if(iter>0 && cost >= lastCost){
            std::cout << "Cost: " << cost << " ,lastCost: " << lastCost << std::endl << "Break Iteration." << std::endl;
            break;
        }

        // update estimated value x = x+ delta_x lastcost
        lastCost = cost;
        ae += dx[0];
        be += dx[1];
        ce += dx[2];
        std::cout << "Total cost at Iteration: " << iter << " : " << cost << "\tUpdate: " << dx.transpose() << "\tEstimation: " \
        << ae << "," << be << "," << ce << std::endl;
    }

    // print final solution and time taken
    std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
    // std::chrono::duration<double> time_diff = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
    std::chrono::duration<double> time_diff = end_time - start_time;
    std::cout << "Solved in time: " << time_diff.count() <<std::endl;
    std::cout << "Estimation ABC: " << ae << "," << be << "," << ce << std::endl;
    return 0;
}