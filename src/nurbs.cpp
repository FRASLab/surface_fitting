#include "nurbs.h"
#include <iostream>
#include <utility>
#include <pcl/io/pcd_io.h>

using namespace surface_reconstructor;

Nurbs::Nurbs(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    cloud_ = std::move(cloud);
    setFittingParams(0.1, 1.0, 0.1, 0.0); // Default parameters
}

Nurbs::Nurbs(const std::string &pcdFile) {
    cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile(pcdFile, *cloud_) == -1) {
        std::cout << "Error loading point cloud file: " << pcdFile << std::endl;
        return;
    }
    setFittingParams(0.1, 1.0, 0.1, 0.0); // Default parameters
}

void Nurbs::updatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    cloud_ = std::move(cloud);
}

int Nurbs::getPos(double u, double v, ON_3dPoint& pos) const {
    if (!is_fitted_)
    {
        std::cout << "NURBS not fitted yet!" << std::endl;
        return -1;
    }

    surface_.EvPoint(u, v, pos);
    return 0;
}

int Nurbs::getPos(double u, double v, Eigen::Vector3d& pos) const {
    ON_3dPoint pos_t;
    getPos(u, v, pos_t);
    pos << pos_t.x, pos_t.y, pos_t.z;
    return 0;
}

int Nurbs::getNormal(double u, double v, ON_3dVector& n) const
{
    if (!is_fitted_)
    {
        std::cout << "NURBS not fitted yet!" << std::endl;
        return -1;
    }
    
    surface_.EvNormal(u, v, n);
    return 0;
}

int Nurbs::getNormal(double u, double v, Eigen::Vector3d& n) const
{
    ON_3dVector n_t;
    getNormal(u, v, n_t);
    n << n_t.x, n_t.y, n_t.z;
    return 0;
}

int Nurbs::getPosDeriv(double u, double v, ON_3dVector& dp_u,
    ON_3dVector& dp_v) const
{
    if (!is_fitted_)
    {
        std::cout << "NURBS not fitted yet!" << std::endl;
        return -1;
    }

    ON_3dPoint point;
    surface_.Ev1Der(u, v, point, dp_u, dp_v);
    return 0;
}

int Nurbs::getPosDeriv(double u, double v, Eigen::Vector3d& dp_u,
    Eigen::Vector3d& dp_v) const
{
    ON_3dVector du_t, dv_t;
    getPosDeriv(u, v, du_t, dv_t);
    dp_u << du_t.x, du_t.y, du_t.z;
    dp_v << dv_t.x, dv_t.y, dv_t.z;
    return 0;
}

int Nurbs::getPos2Deriv(double u, double v, ON_3dVector& dp_u, ON_3dVector& dp_v,
    ON_3dVector& dp_uu, ON_3dVector& dp_uv, ON_3dVector& dp_vv) const {
    if (!is_fitted_)
    {
        std::cout << "NURBS not fitted yet!" << std::endl;
        return -1;
    }

    ON_3dPoint point;
    surface_.Ev2Der(u, v, point, dp_u, dp_v, dp_uu, dp_uv, dp_vv);
    return 0;
}

int Nurbs::getPos2Deriv(double u, double v, Eigen::Vector3d& dp_u, Eigen::Vector3d& dp_v,
    Eigen::Vector3d& dp_uu, Eigen::Vector3d& dp_uv, Eigen::Vector3d& dp_vv) const
{
    ON_3dVector du_t, dv_t, duu_t, duv_t, dvv_t;
    getPos2Deriv(u, v, du_t, dv_t, duu_t, duv_t, dvv_t);
    dp_u << du_t.x, du_t.y, du_t.z;
    dp_v << dv_t.x, dv_t.y, dv_t.z;
    dp_uu << duu_t.x, duu_t.y, duu_t.z;
    dp_uv << duv_t.x, duv_t.y, duv_t.z;
    dp_vv << dvv_t.x, dvv_t.y, dvv_t.z;
    return 0;
}

int Nurbs::getCurvature(const double u, const double v, double& curvature) const {
    if (!is_fitted_)
    {
        std::cout << "NURBS not fitted yet!" << std::endl;
        return -1;
    }

    ON_3dVector du, dv, duu, duv, dvv, normal, K1, K2;
    double gauss, mean, kappa1, kappa2;
    getPos2Deriv(u, v, du, dv, duu, duv, dvv);
    getNormal(u, v, normal);
    ON_EvPrincipalCurvatures(du, dv, duu, duv, dvv, normal, &gauss, &mean, &kappa1, &kappa2, K1, K2); // mean curvature
    curvature = gauss;

    return 0;
}

int Nurbs::setFittingParams(double interior_smoothness, double interior_weight, 
                            double boundary_smoothness, double boundary_weight)
{
    params_.interior_smoothness = interior_smoothness;
    params_.interior_weight = interior_weight;
    params_.boundary_smoothness = boundary_smoothness;
    params_.boundary_weight = boundary_weight;
    return 0;
}

int Nurbs::getDNormal(double u, double v, Eigen::Vector3d& dn_u, Eigen::Vector3d& dn_v) const {
    Eigen::Vector3d n;
    getNormal(u, v, n);
    Eigen::Vector3d du, dv, duu, duv, dvv;
    getPos2Deriv(u, v, du, dv, duu, duv, dvv);

    // Compute the first fundamental form the NURBS
    double E = du.dot(du);
    double F = du.dot(dv);
    double G = dv.dot(dv);

    // Compute the second fundamental form the NURBS
    double e = duu.dot(n);
    double f = duv.dot(n);
    double g = dvv.dot(n);

    dn_u = (f*F-e*G)/(E*G-F*F)*du + (e*F-f*E)/(E*G-F*F)*dv;
    dn_v = (g*F-f*G)/(E*G-F*F)*du + (f*F-g*E)/(E*G-F*F)*dv;

    return 0;
}

int Nurbs::fitSurface(const Eigen::Vector3d& refNormal)
{   
    // Initialize NURBS surface
    pcl::on_nurbs::NurbsDataSurface data;
    // Convert cloud to data
    for (const auto& point : cloud_->points)
    {
        data.interior.emplace_back(point.x, point.y, point.z);
    }
    refNormal_ = refNormal;
    surface_ = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox(4, &data, refNormal);
    pcl::on_nurbs::FittingSurface fit (&data, surface_);

    // NURBS refinement
    for (unsigned i = 0; i < refinement_; i++)
    {
        fit.refine(0);
        fit.refine(1);
    }

    // Fitting iteraions
    for (unsigned i = 0; i < iterations_; i++)
    {
        fit.assemble(params_); // Assemble system of equations
        fit.solve();
    }

    surface_ = fit.m_nurbs;

    is_fitted_ = true;
    return 0;
}

//int Nurbs::convertToMarker(visualization_msgs::Marker& marker) const {
//    // Reset the marker's properties
//    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
//    marker.action = visualization_msgs::Marker::ADD;
//
//    // Convert the NURBS surface to a triangle mesh
//    pcl::PolygonMesh mesh;
//    pcl::on_nurbs::Triangulation::convertSurface2PolygonMesh(surface_, mesh, 128);
//
//    // Convert PCLPolygonMesh's cloud to pcl::PointCloud<Point>
//    const pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::fromPCLPointCloud2(mesh.cloud, *mesh_cloud);
//
//    // Add the points to the marker
//    marker.points.reserve(mesh.polygons.size() * 3); // Reserve space for the points
//    for (const auto& polygon : mesh.polygons)
//    {
//        for (int i = 0; i < 3; i++)
//        {
//            const pcl::PointXYZ& point = mesh_cloud->at(polygon.vertices[i]);
//            geometry_msgs::Point g_point;
//            g_point.x = point.x;
//            g_point.y = point.y;
//            g_point.z = point.z;
//            marker.points.push_back(g_point);
//        }
//    }
//
//    marker.color.r = 1.0f;
//    marker.color.g = 0.0f;
//    marker.color.b = 0.0f;
//    marker.color.a = 1.0f; // Red color with full opacity
//    marker.scale.x = 1.0; // Scale doesn't matter for TRIANGLE_LIST
//    marker.scale.y = 1.0;
//    marker.scale.z = 1.0;
//
//    return 0;
//}

int Nurbs::getClosestPoint(const Eigen::Vector3d& p, double& u, double& v) const {
    if (!is_fitted_)
    {
        std::cout << "NURBS not fitted yet!" << std::endl;
        return -1;
    }

    // Find the hint point on the NURBS surface
    const Eigen::Vector2d hintPoint = pcl::on_nurbs::FittingSurface::findClosestElementMidPoint(surface_, p);

    // Find the closest point on the NURBS surface
    Eigen::Vector3d closestPoint;
    auto ret = pcl::on_nurbs::FittingSurface::inverseMapping(surface_, p, hintPoint, closestPoint, 100, 1e-6, true);

    u = ret(0);
    v = ret(1);

    return 0;
}

bool Nurbs::saveSurfaceAsStl(const std::string& filename, const double resolution) const
{
    // If not fitted, return false
    if (!is_fitted_)
    {
        std::cout << "NURBS not fitted yet!" << std::endl;
        return false;
    }

    // Convert the NURBS surface to a triangle mesh
    pcl::PolygonMesh mesh;
    pcl::on_nurbs::Triangulation::convertSurface2PolygonMesh(surface_, mesh, resolution);

    // Save the mesh as STL file
    return pcl::io::savePolygonFileSTL(filename, mesh);
}


