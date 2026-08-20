#pragma once

namespace msf {

/**
 * 基础常量与数学小工具（core 唯一常量来源）。
 * 延续 msf_glv 命名规则（`_xxx`），仅将载体从宏 + 运行期静态对象改为编译期 constexpr。
 */
namespace constants {

// —— 数学 / 单位换算 ——
inline constexpr double _PI = 3.141592653589793238462643383279502884;  ///< π（统一为高精度）
inline constexpr double _D2R = _PI / 180.0;   ///< deg -> rad
inline constexpr double _R2D = 180.0 / _PI;   ///< rad -> deg
inline constexpr double _deg = _PI / 180.0;   ///< 1° [rad]
inline constexpr double _min = _deg / 60.0;   ///< 1′ [rad]
inline constexpr double _sec = _min / 60.0;   ///< 1″ [rad]
inline constexpr double _ppm = 1.0e-6;        ///< parts per million
inline constexpr double _hur = 3600.0;        ///< 1 h [s]
inline constexpr double _dps = _deg;          ///< 1 deg/s -> rad/s
inline constexpr double _dpss = _deg;         ///< 1 deg/sqrt(s) -> rad/sqrt(s)
inline constexpr double _dph = _deg / _hur;   ///< 1 deg/h -> rad/s
inline constexpr double _dpsh = _deg / 60.0;  ///< 1 deg/sqrt(h) -> rad/sqrt(s)（sqrt(3600)=60）
inline constexpr double _dphpsh = _dph / 60.0;
inline constexpr double _mpsh = 1.0 / 60.0;   ///< 1 m/sqrt(h) -> m/sqrt(s)
inline constexpr double _mpspsh = 1.0 / 60.0;
inline constexpr double _ppmpsh = _ppm / 60.0;
inline constexpr double _secpsh = _sec / 60.0;

// —— 物理 / 地球参数（WGS-84）——
inline constexpr double _CLIGHT = 2.99792458e8;        ///< 光速 [m/s]
inline constexpr double _CLIGHT_2 = 8.9875517873681764e16;
inline constexpr double _OMEGA = 7292115.1467e-11;     ///< 地球自转角速度 [rad/s]
inline constexpr double _Aell = 6378137.0;             ///< 长半轴 a [m]
inline constexpr double _Finv = 298.257223563;         ///< 扁率倒数 1/f
inline constexpr double _MJD_J2000 = 51544.5;          ///< J2000 对应 MJD
inline constexpr double _R_SPHERE = 6371000.0;         ///< 球近似半径 [m]
inline constexpr double _G_EQUA = 9.7803253359;        ///< 赤道重力 [m/s^2]
inline constexpr double _G_POLE = 9.8321849378;        ///< 极区重力 [m/s^2]
inline constexpr double _G_WMO = 9.80665;              ///< WMO 标准重力 [m/s^2]
inline constexpr double _Re = 6378137.0;               ///< 长半轴 [m]
inline constexpr double _f = 1.0 / 298.257;            ///< 扁率（重力公式用）
inline constexpr double _wie = 7.2921151467e-5;        ///< 地球自转角速度 [rad/s]
inline constexpr double _g0 = 9.7803267715;            ///< 赤道重力 [m/s^2]

// WGS-84 重力纬度公式系数（earth::Update 使用）
inline constexpr double _m = _Re * _wie * _wie / _g0;
inline constexpr double _beta = 5.0 / 2.0 * _m - _f - 17.0 / 14.0 * _m * _f;
inline constexpr double _beta1 = (5.0 * _m * _f - _f * _f) / 8.0;
inline constexpr double _beta2 = 3.086e-6;
inline constexpr double _beta3 = 8.08e-9;
inline constexpr double _mg = 1.0e-3 * _g0;   ///< 1 mg -> m/s^2
inline constexpr double _ug = 1.0e-6 * _g0;   ///< 1 ug -> m/s^2
inline constexpr double _mgpsHz = _mg;
inline constexpr double _ugpsHz = _ug;
inline constexpr double _mgpsh = _mg / 60.0;
inline constexpr double _ugpsh = _ug / 60.0;

inline constexpr double _iekf_convergence_threshold = 1.0e-3;  ///< IEKF 误差状态分量收敛阈值
inline constexpr double _gnss_norm_res_gate = 3.0;  ///< GNSS 位置/速度归一化新息门限（POST_MSF 3-sigma）
inline constexpr double _EPS = 1e-10;  ///< 数值判零阈值
inline constexpr double _INF = 1e100;
}  // namespace constants

}  // namespace msf
