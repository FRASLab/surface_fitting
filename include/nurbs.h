#ifndef NURBS_H_
#define NURBS_H_

#include <pcl/point_cloud.h>
#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/3rdparty/opennurbs/opennurbs.h>
#include <pcl/surface/on_nurbs/triangulation.h>
#include <pcl/io/vtk_lib_io.h>
#include <Eigen/Dense>

namespace surface_reconstructor {

class Nurbs
{
public:
    explicit Nurbs(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    explicit Nurbs(const std::string &pcdFile);
    ~Nurbs() = default;

    /**
    * @brief Update the point cloud
    * @param cloud the new point cloud
    */
    void updatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    /**
    * @brief Get the position of the surface at a given point [u,v].
    * @param u The parameter value on the u-direction of the surface.
    * @param v The parameter value on the v-direction of the surface.
    * @param pos The output position value.
    * @return 0 if successful
    */
    int getPos(double u, double v, ON_3dPoint& pos) const;

    /**
    * @brief Get the position of the surface at a given point [u,v].
    * @param u The parameter value on the u-direction of the surface.
    * @param v The parameter value on the v-direction of the surface.
    * @param pos The output position value.
    * @return 0 if successful
    */
    int getPos(double u, double v, Eigen::Vector3d& pos) const;

    /**
    * @brief Get the normal of the surface at a given point [u,v].
    * @param u The parameter value on the u-direction of the surface.
    * @param v The parameter value on the v-direction of the surface.
    * @param n The output normal value.
    * @return 0 if successful
    */
    int getNormal(double u, double v, ON_3dVector& n) const;

    /**
    * @brief Get the normal of the surface at a given point [u,v].
    * @param u The parameter value on the u-direction of the surface.
    * @param v The parameter value on the v-direction of the surface.
    * @param n The output normal value.
    * @return 0 if successful
    */
    int getNormal(double u, double v, Eigen::Vector3d& n) const;

    /**
    * @brief Get the derivative of the normal of the surface at a given point [u,v].
    * @param u The parameter value on the u-direction of the surface.
    * @param v The parameter value on the v-direction of the surface.
    * @param dn_u The output derivative of the normal with respect to u.
    * @param dn_v The output derivative of the normal with respect to v.
    * @return 0 if successful
    */
    int getDNormal(double u, double v, ON_3dVector& dn_u, ON_3dVector& dn_v) const;

    /**
    * @brief Get the derivative of the normal of the surface at a given point [u,v].
    * @param u The parameter value on the u-direction of the surface.
    * @param v The parameter value on the v-direction of the surface.
    * @param dn_u The output derivative of the normal with respect to u.
    * @param dn_v The output derivative of the normal with respect to v.
    * @return 0 if successful
    */
    int getDNormal(double u, double v, Eigen::Vector3d& dn_u, Eigen::Vector3d& dn_v) const;

    /**
    * @brief Get the derivative of the position of the surface at a given point [u,v].
    * @param u The parameter value on the u-direction of the surface.
    * @param v The parameter value on the v-direction of the surface.
    * @param dp_u The output derivative of the position with respect to u.
    * @param dp_v The output derivative of the position with respect to v.
    * @return 0 if successful
    */
    int getPosDeriv(double u, double v, ON_3dVector& dp_u, ON_3dVector& dp_v) const;

    /**
    * @brief Get the derivative of the position of the surface at a given point [u,v].
    * @param u The parameter value on the u-direction of the surface.
    * @param v The parameter value on the v-direction of the surface.
    * @param dp_u The output derivative of the position with respect to u.
    * @param dp_v The output derivative of the position with respect to v.
    * @return 0 if successful
    */
    int getPosDeriv(double u, double v, Eigen::Vector3d& dp_u, Eigen::Vector3d& dp_v) const;

    /**
    * @brief Get the second derivative of the position of the surface at a given point [u,v].
    * @param u The parameter value on the u-direction of the surface.
    * @param v The parameter value on the v-direction of the surface.
    * @param dp_u The output second derivative of the position with respect to u.
    * @param dp_v The output second derivative of the position with respect to v.
    * @param dp_uu The output second derivative of the position with respect to u^2.
    * @param dp_uv The output second derivative of the position with respect to uv.
    * @param dp_vv The output second derivative of the position with respect to v^2.
    * @return 0 if successful
    */
    int getPos2Deriv(double u, double v, ON_3dVector& dp_u, ON_3dVector& dp_v,
                ON_3dVector& dp_uu, ON_3dVector& dp_uv, ON_3dVector& dp_vv) const;

    /**
    * @brief Get the second derivative of the position of the surface at a given point [u,v].
    * @param u The parameter value on the u-direction of the surface.
    * @param v The parameter value on the v-direction of the surface.
    * @param dp_u The output derivative of the position with respect to u.
    * @param dp_v The output derivative of the position with respect to v.
    * @param dp_uu The output second derivative of the position with respect to u^2.
    * @param dp_uv The output second derivative of the position with respect to uv.
    * @param dp_vv The output second derivative of the position with respect to v^2.
    * @return 0 if successful
    */
    int getPos2Deriv(double u, double v, Eigen::Vector3d& dp_u, Eigen::Vector3d& dp_v,
                Eigen::Vector3d& dp_uu, Eigen::Vector3d& dp_uv, Eigen::Vector3d& dp_vv) const;

    /**
     * @brief Get the curvature of the surface at a given point [u,v].
     * @param u The parameter value on the u-direction of the surface.
     * @param v The parameter value on the v-direction of the surface.
     * @param curvature The output curvature value.
     * @return 0 if successful
     */
    int getCurvature(double u, double v, double& curvature) const;

    /**
    * @brief Set the parameters for fitting the surface.
    * @param interior_smoothness The smoothness parameter for the interior of the surface. The default value is 0.1.
    * @param interior_weight The weight parameter for the interior of the surface. The default value is 1.0.
    * @param boundary_smoothness The smoothness parameter for the boundary of the surface. The default value is 0.1.
    * @param boundary_weight The weight parameter for the boundary of the surface. The default value is 0.0.
    * @return 0 if successful
    */
    int setFittingParams(
        double interior_smoothness = 0.1,
        double interior_weight = 1.0,
        double boundary_smoothness = 0.1,
        double boundary_weight = 0.0
    );

    /**
    * @brief Fit the surface to the input point cloud.
    * @return 0 if successful
    */
    int fitSurface(const Eigen::Vector3d& refNormal = -Eigen::Vector3d::UnitZ());

//    /**
//    * @brief Convert the fitted surface to a marker for visualization.
//    * @param marker The reference to the output marker.
//    * @return 0 if successful
//    */
//    int convertToMarker(visualization_msgs::Marker& marker) const;


    /**
    * @brief Get the closest point on the surface to a given point [p].
    * @param p The given point [p = (x,y,z)].
    * @param u The parameter value on the u-direction of the surface that corresponds to the closest point.
    * @param v The parameter value on the v-direction of the surface that corresponds to the closest point.
    * @return 0 if successful
    */
    int getClosestPoint(const Eigen::Vector3d& p, double& u, double& v) const;

    /**
     * @brief Set the reference normal for fitting the surface.
     * The orientation of the surface face will be consistent with the reference normal vector.
     * @param refNormal The reference normal for fitting the surface.
     */
    void setRefNormal(const Eigen::Vector3d& refNormal)
    {
        refNormal_ = refNormal;
    }

    /**
     * @brief Get the reference normal for fitting the surface.
     * @return The reference normal for fitting the surface.
     */
    const Eigen::Vector3d& getRefNormal() const
    {
        return refNormal_;
    }

    /**
    * @brief check if the surface has been fitted or not.
    * @return true if the surface has been fitted, false otherwise.
    */
    bool isFitted() const { return is_fitted_; }

    /**
    * @brief Get the fitted NURBS surface.
    * @return the fitted NURBS surface.
    */
    ON_NurbsSurface& getSurface() { return surface_; }

    /**
     * @brief Save the fitted surface as a STL file.
     * @param filename The name of the output STL file.
     * @param resolution The resolution of the output STL file, the higher the resolution, the smoother the surface.
     * The default value is 50.
     * @return true if successful, false otherwise.
     */
    bool saveSurfaceAsStl(const std::string& filename, double resolution = 50) const;


private:
    bool is_fitted_ = false; // whether the surface has been fitted or not
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_; // the input point cloud
    ON_NurbsSurface surface_; // the fitted NURBS surface
    pcl::on_nurbs::FittingSurface::Parameter params_; ON_NurbsSurface surface_params_;
    Eigen::Vector3d refNormal_ = -Eigen::Vector3d::UnitZ(); // the reference normal for fitting the surface
    unsigned refinement_ = 2;
    unsigned iterations_ = 10;
};

}
#endif // NURBS_H_