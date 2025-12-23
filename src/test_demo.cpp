#include "invkin.h"
#include "nurbs.h"
#include <pcl/io/pcd_io.h>
#include <random>
#include <iostream>

namespace dp = dynamic_planning;
namespace sr = surface_reconstructor;

int main()
{
    // Load the point cloud
    // std::string pcdFile = "/home/wsl/proj/pcl/test/milk.pcd";
    // std::string pcdFile = "/home/wsl/proj/skyvortex_mujoco/assets/NURBS.pcd";
    // std::string pcdFile = "/home/wsl/proj/planning_ws/src/surface_reconstructor/data/blade_segment.pcd";
    //    std::string pcdFile = "/home/wsl/proj/planning_ws/src/surface_reconstructor/data/pointcloud_blade_trans_02.pcd";
    // std::string pcdFile = "/home/wsl/proj/planning_ws/src/surface_reconstructor/data/pointcloud_blade_trans_02.pcd";
    // std::string pcdFile = "/home/wsl/proj/planning_ws/src/surface_reconstructor/data/surf.pcd";
    std::string pcdFile = "/home/wsl/proj/planning_ws/src/surface_reconstructor/data/scans_S2.pcd";
    const auto nurbs = new sr::Nurbs(pcdFile);
    nurbs->fitSurface();
    auto ik = new dp::InvKin(nurbs);
    ik->setLinkLength(1.0);
    // auto qs = ik->xToQs(0.2,0.2);
    // std::cout << "qs: " << qs.transpose() << std::endl;
    // return 0;

    std::string file_name = "/home/wsl/proj/planning_ws/src/surface_reconstructor/data/point_normal_sparse_S2.csv";
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