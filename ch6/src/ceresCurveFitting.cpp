#include <iostream>
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>

// residual cost struct to be used in ceres problem
struct CURVE_FITTING_COST{
    CURVE_FITTING_COST(double x, double y): _x(x), _y(y) {}

    // this function will compute the error. So, basically cost function "e = y - f(x)"
    template <typename T>
    bool operator()(const T *const abc, T *residual) const {
        // input params: abc: predicted values, residual: cost memory
        residual[0] = T(_y) - ceres::exp(abc[0]*T(_x)*T(_x) + abc[1]*T(_x) + abc[2]);
        return true;
    }

    const double _x, _y;
};

int main(int argc, char **argv){

    // initialize curve gt, estimation
    double ar = 1.0, br = 2.0, cr = 3.0;
    double ae = 2.0, be = 10.0, ce = -1.5;
    int N = 100;
    double w_sigma = 1.0;

    // generate datapoints
    cv::RNG rng;
    std::vector<double> x_data, y_data;
    for(int i=0; i<N; i++){
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(std::exp(ar*x*x + br*x + cr) + rng.gaussian(w_sigma * w_sigma));
    }

    // formulate ceres problem, in problem block add each datapoints
    double abc[3] = {ae, be, ce};
    ceres::Problem problem;
    for(int i=0; i<N; i++){
        // residualBock pattern autoDiffusion<tempate params: residual type, output-dimension, input-dimension>
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(new CURVE_FITTING_COST(x_data[i], y_data[i])),
            nullptr,
            abc
        );
    }

    // set options & summary
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;

    // call solve() to optimize
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    ceres::Solve(options, &problem, &summary);
    std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();

    // report summary
    std::chrono::duration<double> time_diff = end_time - start_time;
    std::cout << "Time taken: " << time_diff.count() << std::endl;
    std::cout << "Report: " << summary.BriefReport() << std::endl;
    std::cout << "Estimation: ";
    for(auto a:abc) std::cout << a << " " << std::endl;

    return 0;
}