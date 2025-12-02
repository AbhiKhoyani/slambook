#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/surfel_smoothing.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/impl/mls.hpp>

// typedefs
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointXYZRGBNormal SurfelT;
typedef pcl::PointCloud<SurfelT> SurfelCloudT;


SurfelCloudT::Ptr reconstructSurface(const PointCloudT::Ptr &input, float radius, int polynomial_order){
    
    SurfelCloudT::Ptr output(new SurfelCloudT);
    pcl::MovingLeastSquares<PointT, SurfelT> mls;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);

    mls.setSearchMethod(tree);
    mls.setSearchRadius(radius);
    mls.setComputeNormals(true);
    mls.setSqrGaussParam(radius * radius);
    mls.setPolynomialOrder(polynomial_order);
    mls.setInputCloud(input);
    mls.process(*output);
    return output;
}

pcl::PolygonMeshPtr triangulateMesh(const SurfelCloudT::Ptr &input){

    pcl::search::KdTree<SurfelT>::Ptr tree(new pcl::search::KdTree<SurfelT>);
    // tree->setInputCloud(input);          // this line is not necessary as such

    pcl::GreedyProjectionTriangulation<SurfelT> gp3;
    pcl::PolygonMeshPtr triangles(new pcl::PolygonMesh);

    // maximum distance between edge
    gp3.setSearchRadius(0.05);
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI/4);
    gp3.setMinimumAngle(M_PI/18);
    gp3.setMaximumAngle(M_PI/3);
    gp3.setNormalConsistency(true);

    gp3.setInputCloud(input);
    gp3.setSearchMethod(tree);
    gp3.reconstruct(*triangles);
    return triangles;
}


int main(int argc, char** argv){

    PointCloudT::Ptr cloud(new PointCloudT);
    if (argc != 2 || pcl::io::loadPCDFile(argv[1], *cloud)){
        std::cerr << "Failed to load point cloud!" << std::endl;
        return -1;
    }

    std::cout << "Loaded point cloud size: " << cloud->points.size() << std::endl;

    // Compute surface elements
    std::cout << "Computing normals..... " << std::endl;
    double mls_radius = 0.05, polynomial_order = 2;
    SurfelCloudT::Ptr surfels = reconstructSurface(cloud, mls_radius, polynomial_order);

    // Compute a greedy surface triangulation
    std::cout << "Computing mesh....." << std::endl;
    pcl::PolygonMeshPtr mesh = triangulateMesh(surfels);

    pcl::visualization::PCLVisualizer viz;
    viz.addPolylineFromPolygonMesh(*mesh, "mesh frame");
    viz.addPolygonMesh(*mesh, "mesh");
    viz.resetCamera();
    viz.spin();
    return 0;
}