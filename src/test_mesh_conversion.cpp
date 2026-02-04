#include "nurbs.h"
#include <iostream>
#include <cmath>

pcl::PointCloud<pcl::PointXYZ>::Ptr generateCylinderPointCloud(double radius, double height, int numPointsCircle, int numHeightLevels)
{
    auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->points.reserve(numPointsCircle * numHeightLevels);

    for (int h = 0; h < numHeightLevels; ++h)
    {
        double z = height * static_cast<double>(h) / (numHeightLevels - 1);
        for (int i = 0; i < numPointsCircle; ++i)
        {
            double theta = 2.0 * M_PI * static_cast<double>(i) / numPointsCircle;
            double x = radius * cos(theta);
            double y = radius * sin(theta);
            cloud->points.emplace_back(x, y, z);
        }
    }

    return cloud;
}

int main()
{
    std::cout << "Generating cylinder point cloud..." << std::endl;
    auto cloud = generateCylinderPointCloud(1.0, 2.0, 30, 20);
    std::cout << "Point cloud size: " << cloud->points.size() << std::endl;

    std::cout << "Creating Nurbs object..." << std::endl;
    surface_reconstructor::Nurbs nurbs(cloud);
    nurbs.setFittingParams(0.1, 1.0, 0.1, 0.0);

    std::cout << "Fitting surface..." << std::endl;
    nurbs.fitSurface(Eigen::Vector3d(0, 0, 1));

    std::cout << "Converting to mesh..." << std::endl;
    int result = nurbs.convertToMesh(50);
    if (result != 0)
    {
        std::cerr << "Failed to convert to mesh!" << std::endl;
        return -1;
    }

    Eigen::MatrixXd triangles = nurbs.getMeshTriangles();
    std::cout << "Mesh triangles matrix size: " << triangles.rows() << " x " << triangles.cols() << std::endl;
    std::cout << std::endl;

    std::cout << "First 3 triangles:" << std::endl;
    for (int i = 0; i < std::min(3, static_cast<int>(triangles.rows())); ++i)
    {
        std::cout << "Triangle " << i << ": ";
        for (int j = 0; j < triangles.cols(); ++j)
        {
            std::cout << triangles(i, j) << " ";
        }
        std::cout << std::endl;
    }

    std::cout << std::endl;
    std::cout << "Last 3 triangles:" << std::endl;
    for (int i = std::max(0, static_cast<int>(triangles.rows()) - 3); i < triangles.rows(); ++i)
    {
        std::cout << "Triangle " << i << ": ";
        for (int j = 0; j < triangles.cols(); ++j)
        {
            std::cout << triangles(i, j) << " ";
        }
        std::cout << std::endl;
    }

    return 0;
}
