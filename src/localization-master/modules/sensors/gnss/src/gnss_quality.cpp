#include "gnss_quality.h"

#include <cmath>

#include "math/constants.h"

namespace msf {
namespace gnss {

bool AllowedGnssPos(const GnssPos& pos) {
    switch (pos._pos_type) {
        case 0: return false;   // no solution
        case 16: return false;  // single point
        case 34:                // float
        case 50: break;         // fixed
        default: return false;
    }

    if (!std::isfinite(pos._blh(0)) || !std::isfinite(pos._blh(1)) ||
        !std::isfinite(pos._blh(2))) {
        return false;
    }

    // lat/lon 已是弧度，检查范围（度）
    const double lat_deg = pos._blh(0) * constants::_R2D;
    const double lon_deg = pos._blh(1) * constants::_R2D;
    if (lat_deg < 1.0 || lat_deg > 60.0 || lon_deg < 60.0 || lon_deg > 180.0) {
        return false;
    }

    if (pos._blh_std(0) <= 0.0 || pos._blh_std(1) <= 0.0 || pos._blh_std(2) <= 0.0) {
        return false;
    }
    return true;
}

bool AllowedGnssVel(const GnssVel& vel) {
    switch (vel._vel_type) {
        case 0: return false;
        case 16: return false;
        case 8:
        case 34:
        case 50: break;
        default: return false;
    }

    if (!std::isfinite(vel._hor_speed) || !std::isfinite(vel._trk_gnd) ||
        !std::isfinite(vel._ver_speed)) {
        return false;
    }

    if (vel._trk_gnd < 0.0 || vel._trk_gnd > 2.0 * constants::_PI) {
        return false;
    }
    return true;
}

bool AllowedHeading(const HeadingData& heading) {
    switch (heading._pos_type) {
        case 34:
        case 50: break;
        case 0: return false;
        case 16: return false;
        default: return false;
    }

    if (!std::isfinite(heading._heading) || !std::isfinite(heading._heading_std) ||
        !std::isfinite(heading._baseline_length)) {
        return false;
    }

    if (heading._heading_std <= 0.0) {
        return false;
    }
    if (heading._baseline_length <= 0.0f) {
        return false;
    }
    return true;
}

std::string GnssVelRejectReason(const GnssVel& vel, double dt_sins_minus_meas) {
    switch (vel._vel_type) {
        case 0: return "vel_type_no_solution";
        case 16: return "vel_type_single_point";
        case 8:
        case 34:
        case 50: break;
        default: return "vel_type_invalid";
    }
    if (!std::isfinite(vel._hor_speed) || !std::isfinite(vel._trk_gnd) ||
        !std::isfinite(vel._ver_speed)) {
        return "non_finite";
    }
    if (vel._trk_gnd < 0.0 || vel._trk_gnd > 2.0 * constants::_PI) {
        return "trk_out_of_range";
    }
    if (std::fabs(dt_sins_minus_meas) > 0.03) {
        return "time_sync_failed";
    }
    return "ok";
}

}  // namespace gnss
}  // namespace msf
