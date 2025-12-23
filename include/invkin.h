#ifndef DYNAMIC_PLANNING_INVKIN_H
#define DYNAMIC_PLANNING_INVKIN_H

#include "nurbs.h"
#include <Eigen/Dense>


namespace dynamic_planning {

using Vector5d = Eigen::Matrix<double, 5, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
namespace sr = surface_reconstructor;

    class InvKin {
    /**
     * @brief This class provides the inverse kinematics for the UAM manipulator.
     * The inverse kinematics chain is composed of the following steps:
     * 1. Convert the surface params [x = (u,v)] to surface config [q_s = (p_s,n)].
     * 2. Convert the surface config [q_s = (p_s,n)] to the end effector config [q_e = (x_e,y_e,z_e,psi_e,theta_e)].
     * 3. Convert the end effector config [q_e = (x_e,y_e,z_e,psi_e,theta_e)] to the UAM config [q = (x,y,z,psi,theta)].
     * */
    public:
        explicit InvKin(sr::Nurbs* nurbs);
        ~InvKin() = default;

        /**
         * @brief Set the link length of the manipulator
         * @param link_length The length of the manipulator's links
         */
        void setLinkLength(double link_length);

        /**
         * @brief Updates the surface.
         *
         * This function takes a NurbsClass object and updates the member variable.
         *
         * @param surface The NurbsClass object that will be updated.
         */
        void updateSurface(sr::Nurbs* surface);

        /**
         * @brief Converts the surface params [x = (u,v)] to surface config [q_s = (p_s,n)].
         * @param u The first parameter for the surface.
         * @param v The second parameter of the surface.
         * @return The corresponding point on the surface [q_s], consisting of the contact
         * point [p_s] and the normal vector [n].
         */
        Vector6d xToQs(double u, double v) const;

        /**
         * @brief Converts the surface config [q_s] to the end effector config [q_e].
         * @param qs The surface config [q_s], consisting of the contact point and the normal vector.
         * @return The config of the end effector [q_e], consisting of the position[p_e = (x_e,y_e,z_e)],
         *         yaw[psi_e] and pitch[theta_e].
         */
        Vector5d qsToQe(const Vector6d& qs);

        /**
         * @brief Converts the surface config [q_s = (p_s,n)] to the UAM config [q].
         * @param qs The surface config [q_s], consisting of the contact point and the normal vector.
         * @return The config of the UAM [q], consisting of the position[p = (x,y,z)], yaw[psi] and joint angles[theta].
         */
        Vector5d qsToQ(const Vector6d& qs);

        /**
         * @brief Converts the surface params [x = (u,v)] to the UAM config [q].
         * @param u The first parameter for the surface.
         * @param v The second parameter of the surface.
         * @return The config of the UAM [q], consisting of the position[p = (x,y,z)], yaw[psi] and joint angles[theta].
         */
        Vector5d xToQ(double u, double v);

        /**
         * @brief get the change of position of the end effector [dp_e] respect to the
         *        surface params [(u,v)] with the change of surface params [(du,dv)].
         * @param u The first parameter for the surface.
         * @param v The second parameter of the surface.
         * @param du The change of u.
         * @param dv The change of v.
         * @return The change of the position of the end effector [dp_e].
         */
        Eigen::Vector3d getDpos(double u, double v, double du, double dv) const;

        /**
         * @brief Get the change of the yaw of the end effector [dpsi_e] respect to
         *        the surface params [(u,v)] with the change of surface params [(du,dv)].
         * @param n The normal vector of the surface at [(u,v)].
         * @param dn_u The partial derivative of the normal vector with respect to u.
         * @param dn_v The partial derivative of the normal vector with respect to v.
         * @param du The change of u.
         * @param dv The change of v.
         * @return The change of the yaw of the end effector [dpsi_e].
         */
        static double getDpsi(Eigen::Vector3d& n, Eigen::Vector3d& dn_u, Eigen::Vector3d& dn_v,
                              double du, double dv);

        /**
         * @brief Get the change of the pitch of the end effector [dtheta_e]
         *         respect to the surface params [(u,v)] with the change of surface params [(du,dv)].
         * @param n The normal vector of the surface at [(u,v)].
         * @param dn_u The partial derivative of the normal vector with respect to u.
         * @param dn_v The partial derivative of the normal vector with respect to v.
         * @param du The change of u.
         * @param dv The change of v.
         * @return The change of the pitch of the end effector [dtheta_e].
         */
        static double getDtheta(Eigen::Vector3d& n, const Eigen::Vector3d& dn_u, const Eigen::Vector3d& dn_v,
                         double du, double dv) ;

        /**
         * @brief Convert the change of the surface params [(du,dv)] to the change of the end effector config [dq_e].
         * @param u The first parameter for the surface.
         * @param v The second parameter of the surface.
         * @param du The change of u.
         * @param dv The change of v.
         * @return The change of the end effector config [dq_e].
         */
        Vector5d dxToDqe(double u, double v, double du, double dv) const;

        /**
         * @brief Convert the change of the surface params [(du,dv)] to the change of the config [dq].
         * @param u The first parameter for the surface.
         * @param v The second parameter of the surface.
         * @param du The change of u.
         * @param dv The change of v.
         * @return The change of the end effector config [dq].
         */
        Vector5d dxToDq(double u, double v, double du, double dv);

        /**
        * @brief Get the tangent vectors of the end effector config [dq_e] with respect to the surface params [(u,v)].
        * @param u The first parameter for the surface.
        * @param v The second parameter of the surface.
        * @return A matrix containing the tangent vectors [dqe_u] and [dqe_v].
        */
        std::pair<Vector5d,Vector5d> getTangentVec(double u, double v);

        /**
        * @brief Get the tangent vectors of the end effector config [dq_e] with respect to the surface params [(u,v)].
        * @param u The first parameter for the surface.
        * @param v The second parameter of the surface.
        * @return A matrix containing the tangent vectors [dqe_u] and [dqe_v].
        */
        std::pair<Vector5d,Vector5d> getTangentVecQe(double u, double v) const;

        /**
         * @brief Get the link length of the manipulator.
         * @return The length of the manipulator's links.
         */
        double getLinkLength() const;

        /**
         * @brief Convert the UAM config [q] to the end effector config [q_e], i.e., forward kinematics.
         * @param q The config of the UAM [q], consisting of the position[p = (x,y,z)], yaw[psi] and joint angles[theta].
         * @return The config of the end effector [q_e], consisting of the position[p_e = (x_e,y_e,z_e)],
         */
        Vector5d qToQe(const Vector5d& q) const;

        /**
         * @brief Jacobian of the end effector config [q_e] with respect to the UAM config [q].
         * @param q The config of the UAM [q], consisting of the position[p = (x,y,z)], yaw[psi] and joint angles[theta].
         * @return The Jacobian matrix [J] of the end effector config [q_e] with respect to the UAM config [q].
         */
        Eigen::Matrix<double, 5, 5> getJacobian(const Vector5d& q) const;

        /** @brief Get the nurbs object. */
        sr::Nurbs* getNurbs() const;

        /** @brief Set the clip bound of the surface. */
        void setClipBound(const std::vector<double>& clipBound)
        {
            clipBound_ = clipBound;
            isClipped_ = true;
        }

        /** @brief Set the reference vector of the UAM.
         * The drone's heading  will be consistent with the reference vector.
         *
         */
        void setRefVector(const Eigen::Vector3d& refVector)
        {
            refVector_ = refVector;
        }

        /** @brief Get the reference vector of the UAM. */
        const Eigen::Vector3d& getRefVector() const
        {
            return refVector_;
        }

    private:
        /** @brief Scale the surface params [u,v] to the clip bound [u_min, u_max, v_min, v_max]. */
        std::tuple<double, double> clipParams(double u, double v) const;

        sr::Nurbs* nurbs_;
        Eigen::Vector3d refVector_ = Eigen::Vector3d::UnitX();
        double normEps_ = 0.12; // the projection length of the unit vector at 83.10°
        double prevPsi_ = 0.0;
        double link_length_ = 0.95;
        bool isClipped_{false};
        // The clip bound of the surface, [u_min, u_max, v_min, v_max]
        std::vector<double> clipBound_{0.2, 0.8, 0.2, 0.8};
    };

} // namespace dynamic_planning

#endif // DYNAMIC_PLANNING_INVKIN_H