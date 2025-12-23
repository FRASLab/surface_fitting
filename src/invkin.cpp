#include "invkin.h"
#include <cmath>

using namespace dynamic_planning;
namespace sr = surface_reconstructor;

InvKin::InvKin(sr::Nurbs* nurbs) : nurbs_(nurbs) {}

void InvKin::setLinkLength(const double link_length)
{
    link_length_ = link_length;
}

void InvKin::updateSurface(sr::Nurbs* surface)
{
    nurbs_ = surface;
}

Vector6d InvKin::xToQs(double u, double v) const
{
    if (isClipped_)
    {
        auto [uClipped, vClipped] = clipParams(u, v);
        u = uClipped;
        v = vClipped;
    }
    Vector6d qe;
    Eigen::Vector3d pos;
    Eigen::Vector3d normal;

    nurbs_->getPos(u, v, pos);
    nurbs_->getNormal(u, v, normal);
    qe << pos, normal;
    return qe;
} 

Vector5d InvKin::qsToQe(const Vector6d& qs)
{
    // calculate the yaw and the joint angle of the UAM
    const Eigen::Vector3d ps = qs.head<3>();
    Eigen::Vector3d n = qs.tail<3>();

    double psi_e, theta_e;

    // the surface normal is oriented opposite to the reference vector (general situation)
    if ( std::signbit(refVector_(0)) == std::signbit(-n(0)) )
    {
        psi_e = std::atan2(-n(1), -n(0));
        theta_e = std::atan2(-n(2), n.head<2>().norm());
    }
    else // the surface normal is oriented in the same direction as the reference vector (special case)
    {
        psi_e = std::atan2(n(1), n(0));
        theta_e = std::atan2(n(2), n.head<2>().norm()) - M_PI;
    }

    if (n.head<2>().norm() <= normEps_) // the surface normal is almost perpendicular to the reference vector
    {
        psi_e = prevPsi_;
    }

    prevPsi_ = psi_e;

    // calculate the joint angle of the UAM
    Eigen::VectorXd state(5);
    state << ps, psi_e, theta_e; 

    return state;
}

Vector5d InvKin::qsToQ(const Vector6d& qs)
{
    Vector5d q = qsToQe(qs);
    q(0) = q(0) + qs(3) * link_length_;
    q(1) = q(1) + qs(4) * link_length_;
    q(2) = q(2) + qs(5) * link_length_;
    return q;
}

Vector5d InvKin::xToQ(double u, double v)
{
    const Vector6d qs = xToQs(u, v);
    return qsToQ(qs);
}

Eigen::Vector3d InvKin::getDpos(double u, double v, double du, double dv) const
{
    if (isClipped_)
    {
        auto [uClipped, vClipped] = clipParams(u, v);
        u = uClipped;
        v = vClipped;
    }
    Eigen::Vector3d dps_u, dps_v;
    nurbs_->getPosDeriv(u, v, dps_u, dps_v);
    Eigen::Vector3d dps = dps_u * du + dps_v * dv;
    return dps;
}

double InvKin::getDpsi(Eigen::Vector3d& n, Eigen::Vector3d& dn_u, Eigen::Vector3d& dn_v,
                                double du, double dv)
{
    Eigen::Vector3d dn = dn_u * du + dn_v * dv;
    const double dpsi = n(0) / (n(0) * n(0) + n(1) * n(1)) * dn(1) -
                        n(1) / (n(0) * n(0) + n(1) * n(1)) * dn(0);
    return dpsi;
}

double InvKin::getDtheta(Eigen::Vector3d& n, const Eigen::Vector3d& dn_u, const Eigen::Vector3d& dn_v,
                                double du, double dv)
{
    Eigen::Vector3d dn = dn_u * du + dn_v * dv;
    const double n_t = std::sqrt(n(0) * n(0) + n(1) * n(1));

    double dtheta = -n_t / (n(0) * n(0) + n(1) * n(1) + n(2) * n(2)) * dn(2) +
                    n(2) / (n(0) * n(0) + n(1) * n(1) + n(2) * n(2)) *
                    (n(0) / n_t * dn(0) + n(1) / n_t * dn(1));

    return dtheta;
}

Vector5d InvKin::dxToDqe(double u, double v, double du, double dv) const
{
    if (isClipped_)
    {
        auto [uClipped, vClipped] = clipParams(u, v);
        u = uClipped;
        v = vClipped;
    }
    Eigen::Vector3d n;
    nurbs_->getNormal(u, v, n);
    Eigen::Vector3d dn_u, dn_v;
    nurbs_->getDNormal(u, v, dn_u, dn_v);

    const Eigen::Vector3d dpe = getDpos(u, v, du, dv);
    const double dpsi = getDpsi(n, dn_u, dn_v, du, dv);
    const double dtheta = getDtheta(n, dn_u, dn_v, du, dv);

    Vector5d dqe;
    dqe << dpe, dpsi, dtheta;
    return dqe;
}

Vector5d InvKin::dxToDq(double u, double v, double du, double dv)
{
    if (isClipped_)
    {
        auto [uClipped, vClipped] = clipParams(u, v);
        u = uClipped;
        v = vClipped;
    }
    Eigen::Vector3d n;
    nurbs_->getNormal(u, v, n);
    Eigen::Vector3d dn_u, dn_v;
    nurbs_->getDNormal(u, v, dn_u, dn_v);

    const Eigen::Vector3d dpe = getDpos(u, v, du, dv);
    const double dpsiE = getDpsi(n, dn_u, dn_v, du, dv);
    const double dthetaE = getDtheta(n, dn_u, dn_v, du, dv);

    Eigen::Vector3d dp;

    auto q = xToQ(u,v);
    const double psi = q(3);
    const double theta = q(4);

    //  Inverse Jacobian Matrix
    //  [ 1,   0,   0,   l*cos(theta)*sin(psi),   l*sin(theta)*cos(psi)   ]
    //  [ 0,   1,   0,  -l*cos(theta)*cos(psi),   l*sin(theta)*sin(psi)   ]
    //  [ 0,   0,   1,         0,                 l*cos(theta)            ]
    //  [ 0,   0,   0,         1,                 0                       ]
    //  [ 0,   0,   0,         0,                 1                       ]

    dp(0) = dpe(0)
            + link_length_ * cos(theta) * sin(psi) * dpsiE
            + link_length_ * cos(psi) * sin(theta) * dthetaE;

    dp(1) = dpe(1)
            - link_length_ * cos(theta) * cos(psi) * dpsiE
            + link_length_ * sin(psi) * sin(theta) * dthetaE;

    dp(2) = dpe(2)
            + link_length_ * cos(theta) * dthetaE;

    Vector5d dq;
    dq << dp, dpsiE, dthetaE;

    return dq;
}

std::pair<Vector5d,Vector5d> InvKin::getTangentVecQe(double u, double v) const
{
    if (isClipped_)
    {
        auto [uClipped, vClipped] = clipParams(u, v);
        u = uClipped;
        v = vClipped;
    }
    const auto dqe_u = InvKin::dxToDqe(u,v,1,0);
    const auto dqe_v = InvKin::dxToDqe(u,v,0,1);

    return {dqe_u, dqe_v};
}

std::pair<Vector5d,Vector5d> InvKin::getTangentVec(double u, double v)
{
    if (isClipped_)
    {
        auto [uClipped, vClipped] = clipParams(u, v);
        u = uClipped;
        v = vClipped;
    }
    const auto dqe_u = InvKin::dxToDq(u,v,1,0);
    const auto dqe_v = InvKin::dxToDq(u,v,0,1);

    return {dqe_u, dqe_v};
}

double InvKin::getLinkLength() const
{
    return link_length_;
}

Vector5d InvKin::qToQe(const Vector5d& q) const
{
    Vector5d qe;

    const double x = q(0);
    const double y = q(1);
    const double z = q(2);
    const double psi = q(3);
    const double theta = q(4);

    const double l = link_length_;
    const double xe = x + l * cos(theta) * cos(psi);
    const double ye = y + l * cos(theta) * sin(psi);
    const double ze = z + l * sin(theta);
    const double psi_e = psi;
    const double theta_e = theta;

    qe << xe, ye, ze, psi_e, theta_e;
    return qe;
}

Eigen::Matrix<double, 5, 5> InvKin::getJacobian(const Vector5d& q) const
{
    Eigen::Matrix<double, 5, 5> J;

    const double psi = q(3);
    const double theta = q(4);

    const double l = link_length_;

    J << 1, 0, 0, -l * sin(theta) * cos(psi),   -l * cos(theta) * sin(psi),
         0, 1, 0, l * cos(theta) * cos(psi),    -l * sin(theta) * sin(psi),
         0, 0, 1, 0,                            l * cos(theta),
         0, 0, 0, 1,                            0,
         0, 0, 0, 0,                            1;

    return J;
}

sr::Nurbs* InvKin::getNurbs() const
{
    return nurbs_;
}

std::tuple<double, double> InvKin::clipParams(double u, double v) const
{
    return std::make_tuple(u * (clipBound_[1] - clipBound_[0]) + clipBound_[0],
                           v * (clipBound_[3] - clipBound_[2]) + clipBound_[2]);
}
