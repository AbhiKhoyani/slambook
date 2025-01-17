#include <iostream>
#include <opencv2/opencv.hpp>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>

// define verices and edges
class CurveFittingNode: public g2o::BaseVertex<3, Eigen::Vector3d>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        // override reset function
        virtual void setToOriginImpl() override {
            _estimate << 0, 0, 0;
        }

        // override plus operator
        virtual void oplusImpl(const double *update) override {
            _estimate += Eigen::Vector3d(update);
        }

        // dummy read/write function
        virtual bool read(std::istream &in){return true;}
        virtual bool write(std::ostream &out) const{return true;}
};

// Edge: 1D Error term. Connected to single node only
class CurveFittingEdge: public g2o::BaseUnaryEdge<1, double, CurveFittingNode>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        CurveFittingEdge(double x): BaseUnaryEdge(), _x(x){}
        
        // define error term computation.
        virtual void computeError() override {
            const CurveFittingNode *n = static_cast<const CurveFittingNode *> (_vertices[0]);
            const Eigen::Vector3d abc = n->estimate();
            _error(0, 0) = _measurement - std::exp(abc(0,0)*_x*_x + abc(1,0)*_x + abc(2,0));
        }

        // jacobians
        virtual void linearizeOplus() override {
            const CurveFittingNode *n = static_cast<const CurveFittingNode *> (_vertices[0]);
            const Eigen::Vector3d abc = n->estimate();
            double y = std::exp(abc[0]*_x*_x + abc[1]*_x + abc[2]);
            _jacobianOplusXi[0] = -_x*_x*y;
            _jacobianOplusXi[1] = -_x*y;
            _jacobianOplusXi[2] = -y;
        }

        virtual bool read(std::istream &in){return true;}
        virtual bool write(std::ostream &out) const {return true;}
        double _x;
};

int main(int argc, char** argv){

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

    // define g2o classes
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3,1>> BlockSolveType;
    typedef g2o::LinearSolverDense<BlockSolveType::PoseMatrixType> LinearSolverType;

    // initialize GN optimizer
    auto solver = new g2o::OptimizationAlgorithmGaussNewton(
        std::make_unique<BlockSolveType>(std::make_unique<LinearSolverType>())
    );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    // add vertex -> single node
    CurveFittingNode *n = new CurveFittingNode();
    n->setEstimate(Eigen::Vector3d(ae, be, ce));
    n->setId(0);
    optimizer.addVertex(n);

    // add edges based on datapoints
    for(int i=0; i<N; i++){
        CurveFittingEdge *edge = new CurveFittingEdge(x_data[i]);
        edge->setId(i);
        edge->setVertex(0, n);
        edge->setMeasurement(y_data[i]);
        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1 / (w_sigma * w_sigma));  // information matrix
        optimizer.addEdge(edge);
    }

    // start optimization
    std::cout << "Opimization starts... "  << std::endl; 
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    optimizer.initializeOptimization();
    optimizer.optimize(100);
    std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_diff = end_time - start_time;
    std::cout << "Time taken: " << time_diff.count() << std::endl;

    // print results
    Eigen::Vector3d abc_estimate = n->estimate();
    std::cout << "Estimation: " << abc_estimate.transpose() << std::endl;

    return 0;
}