#pragma once

#include <vector>

#include <Eigen/Eigen>

namespace msf {

class ImuProcessor {
public:
    ImuProcessor();

    Eigen::Vector3d PhiConeCompensation(const std::vector<Eigen::Vector3d>& wm) const;
    Eigen::Vector3d PhiPolynomialCompensation(const std::vector<Eigen::Vector3d>& wm);
    Eigen::Vector3d VelocityConeCompensation(const std::vector<Eigen::Vector3d>& wm,
                                             const std::vector<Eigen::Vector3d>& vm) const;
    Eigen::Vector3d VelocityPolynomialCompensation(const std::vector<Eigen::Vector3d>& wm,
                                                   const std::vector<Eigen::Vector3d>& vm);
    void Update(const std::vector<Eigen::Vector3d>& wm, const std::vector<Eigen::Vector3d>& vm);

    Eigen::Vector3d _phim = Eigen::Vector3d::Zero();
    Eigen::Vector3d _dvbm = Eigen::Vector3d::Zero();
    Eigen::Vector3d _wm_1 = Eigen::Vector3d::Zero();
    Eigen::Vector3d _vm_1 = Eigen::Vector3d::Zero();
    double _pf1 = 1.0;
    double _pf2 = 1.0;
    int _cps = 1;
};

}  // namespace msf