#include "invkin.h"
#include "nurbs.h"
#include <pcl/io/pcd_io.h>
#include <random>
#include <iostream>

namespace dp = dynamic_planning;
namespace sr = surface_reconstructor;

int main()
{
    // Define file paths
    std::string root_path = PROJECT_ROOT_DIR; 
    std::string pcdFile = root_path + "/data/scans.pcd";
    std::string file_name = root_path + "/data/point_normal_sparse_S2.csv";

    // Load the point cloud
    const auto nurbs = new sr::Nurbs(pcdFile);
    nurbs->fitSurface();
    auto ik = new dp::InvKin(nurbs);
    ik->setLinkLength(1.0);

    std::ofstream file(file_name);
    file << "x,y,z,nx,ny,nz" << std::endl;

    // double res = 100.0;
    double res = 100.0;

    for (int i = 0; i < res; i++)
    {
        for (int j = 0; j < res; j++)
        {
            Eigen::Vector3d point;
            nurbs->getPos(i/res, j/res, point);
            Eigen::Vector3d normal, tangent;
            // nurbs->getNormal(i/res, j/res, normal);
            nurbs->getPosDeriv(i/res, j/res, normal, tangent);
            file << point.x() << "," << point.y() << "," << point.z() << ","
            << normal.x() << "," << normal.y() << "," << normal.z() << std::endl;
        }
    }
    return 0;
}