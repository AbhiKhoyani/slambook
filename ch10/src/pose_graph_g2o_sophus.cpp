#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Core>
#include <sophus/se3.hpp>

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

Matrix6d JRInv(const Sophus::SE3d &e)
{
    Matrix6d J;
    J.block(0, 0, 3, 3) = Sophus::SO3d::hat(e.so3().log());
    J.block(0, 3, 3, 3) = Sophus::SO3d::hat(e.translation());
    J.block(3, 0, 3, 3) = Eigen::Matrix3d::Zero(3, 3);
    J.block(3, 3, 3, 3) = Sophus::SO3d::hat(e.so3().log());
    J = J * 0.5 + Matrix6d::Identity();
    return J;
}

class VertexSE3 : public g2o::BaseVertex<6, Sophus::SE3d>
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual bool read(std::istream &is) override
    {
        double data[7];
        for (int i = 0; i < 7; ++i)
            is >> data[i];
        Eigen::Quaterniond q = Eigen::Quaterniond(data[6], data[3], data[4], data[5]);
        setEstimate(Sophus::SE3d(q, Eigen::Vector3d(data[0], data[1], data[2])));
    }

    virtual bool write(std::ostream &os) const override
    {
        os << id() << " ";
        Eigen::Quaterniond q = _estimate.unit_quaternion();
        os << _estimate.translation().transpose() << " ";
        os << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs()[3] << std::endl;
        return true;
    }

    virtual void setToOriginImpl() override
    {
        _estimate = Sophus::SE3d();
    }

    virtual void oplusImpl(const double *update) override
    {
        Vector6d upd;
        upd << update[0], update[1], update[2], update[3], update[4], update[5];
        _estimate = Sophus::SE3d::exp(upd) * _estimate;
    }
};

class EdgeSE3 : public g2o::BaseBinaryEdge<6, Sophus::SE3d, VertexSE3, VertexSE3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual bool read(std::istream &is) override
    {
        double data[7];
        for (int i = 0; i < 7; ++i)
            is >> data[i];
        Eigen::Quaterniond q(data[6], data[3], data[4], data[5]);
        q.normalize();
        setMeasurement(Sophus::SE3d(q, Eigen::Vector3d(data[0], data[1], data[2])));
        for (int i = 0; i < information().rows() && is.good(); ++i)
            for (int j = i; j < information().cols() && is.good(); ++j)
            {
                is >> information()(i, j);
                if (i != j)
                    information()(j, i) = information()(i, j);
            }
        return true;
    }

    virtual bool write(std::ostream &os) const override
    {
        VertexSE3 *v1 = static_cast<VertexSE3 *>(_vertices[0]);
        VertexSE3 *v2 = static_cast<VertexSE3 *>(_vertices[1]);
        os << v1->id() << " " << v2->id() << " ";
        Sophus::SE3d m = _measurement;
        Eigen::Quaterniond q = m.unit_quaternion();
        os << m.translation().transpose() << " ";
        os << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs()[3] << " ";

        // write information matrix
        for (int i = 0; i < information().rows(); ++i)
            for (int j = i; j < information().cols(); ++j)
                os << information()(i, j) << " ";

        os << std::endl;
        return true;
    }

    virtual void computeError() override
    {
        Sophus::SE3d v1 = (static_cast<VertexSE3 *>(_vertices[0]))->estimate();
        Sophus::SE3d v2 = (static_cast<VertexSE3 *>(_vertices[1]))->estimate();
        _error = (_measurement.inverse() * v1.inverse() * v2).log();
    }

    // Commenting out this function will force to use g2o specific jacobian implementation which is taking long time
    // as it may be detailed one and not approximation.
    // This implementation take ~3.6sec per iteration
    // G2O implementation take ~8.5sec per iteration
    // Making J = Identity matrix takes ~3.8 to 4 sec per iteration!
    // Overall results look very similar in all case, may be considering easy case!
    virtual void linearizeOplus() override
    {
        Sophus::SE3d v1 = (static_cast<VertexSE3 *>(_vertices[0]))->estimate();
        Sophus::SE3d v2 = (static_cast<VertexSE3 *>(_vertices[1]))->estimate();
        Matrix6d J = JRInv(Sophus::SE3d::exp(_error));
        // Matrix6d J = Matrix6d::Identity();
        _jacobianOplusXi = -J * v2.inverse().Adj();
        _jacobianOplusXj = J * v2.inverse().Adj();
    }
};

int main(int argc, char **argv)
{

    if (argc != 2)
    {
        std::cerr << "Usage: pose_graph_g2o_sophus sphere.g2o" << std::endl;
        return 1;
    }

    std::ifstream fin(argv[1]);
    if (!fin)
    {
        std::cerr << "file" << argv[1] << " does not exist." << std::endl;
        return 1;
    }

    // Build g2o problem
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>> BlockSolverType;
    typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    int no_vertex = 0, no_edge = 0;
    std::vector<VertexSE3 *> vertices;
    std::vector<EdgeSE3 *> edges;
    while (!fin.eof())
    {
        std::string name;
        fin >> name;

        if (name == "VERTEX_SE3:QUAT")
        {
            VertexSE3 *v = new VertexSE3();
            int index = 0;
            fin >> index;
            v->setId(index);
            v->read(fin);
            optimizer.addVertex(v);
            vertices.push_back(v);
            no_vertex++;
            if (index == 0)
                v->setFixed(true);
        }

        else if (name == "EDGE_SE3:QUAT")
        {
            EdgeSE3 *e = new EdgeSE3();
            int idx1, idx2;
            fin >> idx1 >> idx2;
            e->setId(no_edge++);
            e->setVertex(0, optimizer.vertices()[idx1]);
            e->setVertex(1, optimizer.vertices()[idx2]);
            e->read(fin);
            optimizer.addEdge(e);
            edges.push_back(e);
        }

        if (!fin.good())
            break;
    }

    std::cout << "read total " << no_vertex << " vertices, " << no_edge << " edges." << std::endl;
    std::cout << "Opimizing...." << std::endl;
    optimizer.initializeOptimization();
    optimizer.optimize(10);

    std::cout << "Saving Opimization result..." << std::endl;
    std::ofstream fout("result_sophus.g2o");
    for (VertexSE3 *v : vertices)
    {
        fout << "VERTEX_SE3:QUAT" << " ";
        v->write(fout);
    }

    for (EdgeSE3 *e : edges)
    {
        fout << "EDGE_SE3:QUAT" << " ";
        e->write(fout);
    }

    fout.close();
    return 0;
}