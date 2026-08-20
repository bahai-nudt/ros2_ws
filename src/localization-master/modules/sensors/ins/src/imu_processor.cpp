#include "../include/imu_processor.h"

namespace msf {

ImuProcessor::ImuProcessor() = default;

Eigen::Vector3d ImuProcessor::PhiConeCompensation(const std::vector<Eigen::Vector3d>& wm) const {
    Eigen::Vector3d dphim = Eigen::Vector3d::Zero();
    const int sample_count = static_cast<int>(wm.size());
    if (sample_count == 2) {
        dphim = 2.0 / 3.0 * wm[0].cross(wm[1]);
    } else if (sample_count == 3) {
        dphim = 27.0 / 40.0 * wm[1].cross(wm[2]) + 9.0 / 20.0 * wm[0].cross(wm[2]) +
                27.0 / 40.0 * wm[0].cross(wm[1]);
    } else if (sample_count == 4) {
        dphim = 232.0 / 315.0 * wm[2].cross(wm[3]) + 46.0 / 105.0 * wm[1].cross(wm[3]) +
                18.0 / 35.0 * wm[0].cross(wm[3]) + 178.0 / 315.0 * wm[1].cross(wm[2]) +
                46.0 / 105.0 * wm[0].cross(wm[2]) + 232.0 / 315.0 * wm[0].cross(wm[1]);
    } else if (sample_count == 5) {
        dphim = 18575.0 / 24192.0 * wm[3].cross(wm[4]) + 2675.0 / 6048.0 * wm[2].cross(wm[4]) +
                11225.0 / 24192.0 * wm[1].cross(wm[4]) + 125.0 / 252.0 * wm[0].cross(wm[4]) +
                2575.0 / 6048.0 * wm[2].cross(wm[3]) + 425.0 / 672.0 * wm[1].cross(wm[3]) +
                13975.0 / 24192.0 * wm[0].cross(wm[3]) + 1975.0 / 3024.0 * wm[1].cross(wm[2]) +
                325.0 / 1512.0 * wm[0].cross(wm[2]) + 21325.0 / 24192.0 * wm[0].cross(wm[1]);
    }
    return dphim;
}

Eigen::Vector3d ImuProcessor::PhiPolynomialCompensation(const std::vector<Eigen::Vector3d>& wm) {
    Eigen::Vector3d dphim = Eigen::Vector3d::Zero();
    const int sample_count = static_cast<int>(wm.size());
    if (sample_count == 1) {
        if (_pf1 == 1.0) {
            _wm_1 = wm[0];
            _pf1 = 0.0;
        }
        dphim = 1.0 / 12.0 * _wm_1.cross(wm[0]);
    } else if (sample_count == 2) {
        dphim = 2.0 / 3.0 * wm[0].cross(wm[1]);
    } else if (sample_count == 3) {
        dphim = 33.0 / 80.0 * wm[0].cross(wm[2]) + 57.0 / 80.0 * wm[1].cross(wm[2] - wm[0]);
    } else if (sample_count == 4) {
        dphim = 736.0 / 945.0 * (wm[0].cross(wm[1]) + wm[2].cross(wm[3])) +
                334.0 / 945.0 * (wm[0].cross(wm[2]) + wm[1].cross(wm[3])) +
                526.0 / 945.0 * wm[0].cross(wm[3]) + 654.0 / 945.0 * wm[1].cross(wm[2]);
    } else if (sample_count == 5) {
        dphim = 123425.0 / 145152.0 * (wm[0].cross(wm[1]) + wm[3].cross(wm[4])) +
                34875.0 / 145152.0 * (wm[0].cross(wm[2]) + wm[2].cross(wm[4])) +
                90075.0 / 145152.0 * (wm[0].cross(wm[3]) + wm[1].cross(wm[4])) +
                66625.0 / 145152.0 * wm[0].cross(wm[4]) +
                103950.0 / 145152.0 * (wm[1].cross(wm[2]) + wm[2].cross(wm[3])) +
                55400.0 / 145152.0 * wm[1].cross(wm[3]);
    }
    return dphim;
}

Eigen::Vector3d ImuProcessor::VelocityConeCompensation(const std::vector<Eigen::Vector3d>& wm,
                                                       const std::vector<Eigen::Vector3d>& vm) const {
    Eigen::Vector3d scullm = Eigen::Vector3d::Zero();
    const int sample_count = static_cast<int>(wm.size());
    if (sample_count == 2) {
        scullm = 2.0 / 3.0 * (wm[0].cross(vm[1]) + vm[0].cross(wm[1]));
    } else if (sample_count == 3) {
        scullm = 27.0 / 40.0 * (wm[1].cross(vm[2]) + vm[1].cross(wm[2])) +
                 9.0 / 20.0 * (wm[0].cross(vm[2]) + vm[0].cross(wm[2])) +
                 27.0 / 40.0 * (wm[0].cross(vm[1]) + vm[0].cross(wm[1]));
    } else if (sample_count == 4) {
        scullm = 232.0 / 315.0 * (wm[2].cross(vm[3]) + vm[2].cross(wm[3])) +
                 46.0 / 105.0 * (wm[1].cross(vm[3]) + vm[1].cross(wm[3])) +
                 18.0 / 35.0 * (wm[0].cross(vm[3]) + vm[0].cross(wm[3])) +
                 178.0 / 315.0 * (wm[1].cross(vm[2]) + vm[1].cross(wm[2])) +
                 46.0 / 105.0 * (wm[0].cross(vm[2]) + vm[0].cross(wm[2])) +
                 232.0 / 315.0 * (wm[0].cross(vm[1]) + vm[0].cross(wm[1]));
    }
    return scullm;
}

Eigen::Vector3d ImuProcessor::VelocityPolynomialCompensation(const std::vector<Eigen::Vector3d>& wm,
                                                             const std::vector<Eigen::Vector3d>& vm) {
    Eigen::Vector3d scullm = Eigen::Vector3d::Zero();
    const int sample_count = static_cast<int>(wm.size());
    if (sample_count == 1) {
        if (_pf2 == 1.0) {
            _wm_1 = wm[0];
            _vm_1 = vm[0];
            _pf2 = 0.0;
        }
        scullm = 1.0 / 12.0 * (_wm_1.cross(vm[0]) + _vm_1.cross(wm[0]));
        _wm_1 = wm[0];
        _vm_1 = vm[0];
    } else if (sample_count == 2) {
        scullm = 2.0 / 3.0 * (wm[0].cross(vm[1]) + vm[0].cross(wm[1]));
    } else if (sample_count == 3) {
        scullm = 33.0 / 80.0 * (wm[0].cross(vm[2]) + vm[0].cross(wm[2])) +
                 57.0 / 80.0 * (wm[0].cross(vm[1]) + vm[0].cross(wm[1]) +
                                wm[1].cross(vm[2]) + vm[1].cross(wm[2]));
    } else if (sample_count == 4) {
        scullm = 736.0 / 945.0 * (wm[0].cross(vm[1]) + wm[2].cross(vm[3]) + vm[0].cross(wm[1]) +
                                  vm[2].cross(wm[3])) +
                 334.0 / 945.0 * (wm[0].cross(vm[2]) + wm[1].cross(vm[3]) + vm[0].cross(wm[2]) +
                                  vm[1].cross(wm[3])) +
                 526.0 / 945.0 * (wm[0].cross(vm[3]) + vm[0].cross(wm[3])) +
                 654.0 / 945.0 * (wm[1].cross(vm[2]) + vm[1].cross(wm[2]));
    }
    return scullm;
}

void ImuProcessor::Update(const std::vector<Eigen::Vector3d>& wm, const std::vector<Eigen::Vector3d>& vm) {
    Eigen::Vector3d wmm = Eigen::Vector3d::Zero();
    Eigen::Vector3d vmm = Eigen::Vector3d::Zero();
    for (std::size_t index = 0; index < wm.size(); ++index) {
        wmm += wm[index];
        vmm += vm[index];
    }

    const Eigen::Vector3d dphim = (_cps == 0) ? PhiConeCompensation(wm) : PhiPolynomialCompensation(wm);
    const Eigen::Vector3d scullm = (_cps == 0) ? VelocityConeCompensation(wm, vm)
                                               : VelocityPolynomialCompensation(wm, vm);
    const Eigen::Vector3d rotm = 0.5 * wmm.cross(vmm);
    _phim = wmm + dphim;
    _dvbm = vmm + rotm + scullm;
}

}  // namespace msf