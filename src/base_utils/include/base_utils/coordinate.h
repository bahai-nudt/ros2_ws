#include <iostream>
#include <proj.h>
#include <vector>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry> 

class CoordinateTransformer {
private:
    PJ_CONTEXT* m_ctx;
    PJ* m_transform;

public:
    CoordinateTransformer(int utm_zone, bool is_north = true) {
        // 创建PROJ上下文
        m_ctx = proj_context_create();
        
        // 构建目标CRS字符串 (UTM)
        std::string target_crs;
        if (is_north) {
            target_crs = "+proj=utm +zone=" + std::to_string(utm_zone) + " +datum=WGS84 +units=m +no_defs";
        } else {
            target_crs = "+proj=utm +zone=" + std::to_string(utm_zone) + " +south +datum=GS84 +units=m +no_defs";
        }
        
        // 创建转换
        m_transform = proj_create_crs_to_crs(
            m_ctx,
            "+proj=longlat +datum=WGS84",  // 源CRS: WGS84经纬度
            target_crs.c_str(),            // 目标CRS: UTM
            nullptr
        );
        
        if (!m_transform) {
            std::cerr << "错误: 无法创建坐标转换" << std::endl;
        }
    }
    
    ~CoordinateTransformer() {
        if (m_transform) {
            proj_destroy(m_transform);
        }
        if (m_ctx) {
            proj_context_destroy(m_ctx);
        }
    }
    
    // 计算UTM带号 (静态方法)
    static int calculateUTMZone(double longitude, double latitude) {
        // 处理特殊区域
        if (latitude >= 56.0 && latitude < 64.0 && longitude >= 3.0 && longitude < 12.0) {
            return 32;
        }
        
        // 处理挪威区域
        if (latitude >= 72.0 && latitude < 84.0) {
            if (longitude >= 0.0 && longitude < 9.0) return 31;
            if (longitude >= 9.0 && longitude < 21.0) return 33;
            if (longitude >= 21.0 && longitude < 33.0) return 35;
            if (longitude >= 33.0 && longitude < 42.0) return 37;
        }
        
        // 标准UTM带号计算
        return static_cast<int>((longitude + 180.0) / 6.0) + 1;
    }
    
    // 单点转换
    bool transformLLAtoUTM(double lon, double lat, double alt, 
                          double& easting, double& northing, double& height) {
        if (!m_transform) {
            return false;
        }
        
        // 创建坐标 (顺序: 经度, 纬度, 高程)
        PJ_COORD src_coord = proj_coord(lon, lat, alt, 0);
        
        // 执行转换
        PJ_COORD dst_coord = proj_trans(m_transform, PJ_FWD, src_coord);
        
        // 检查转换是否成功
        if (proj_errno(m_transform) != 0) {
            std::cerr << "转换错误: " << proj_errno_string(proj_errno(m_transform)) << std::endl;
            return false;
        }
        
        easting = dst_coord.xy.x;
        northing = dst_coord.xy.y;
        height = alt;
        
        return true;
    }
    
    // 批量转换
    bool transformBatch(const std::vector<std::tuple<double, double, double>>& lla_coords,
                       std::vector<std::tuple<double, double, double>>& utm_coords) {
        if (!m_transform) {
            return false;
        }
        
        utm_coords.clear();
        utm_coords.reserve(lla_coords.size());
        
        for (const auto& coord : lla_coords) {
            double lon = std::get<0>(coord);
            double lat = std::get<1>(coord);
            double alt = std::get<2>(coord);
            
            double easting, northing, height;
            if (transformLLAtoUTM(lon, lat, alt, easting, northing, height)) {
                utm_coords.emplace_back(easting, northing, height);
            } else {
                return false;
            }
        }
        
        return true;
    }
};


double azimuthToENUYaw(double azimuth_deg) {
    // 北云azimuth: 从北向东为正 (0°=北, 90°=东, 180°=南, 270°=西)
    // ENU航向角: 从东向北为正 (0°=东, 90°=北, 180°=西, 270°=南)
    // 转换公式: yaw = 90° - azimuth
    double yaw = 90.0 - azimuth_deg;
    
    // 规范化到 [0, 360) 范围
    while (yaw < 0.0) yaw += 360.0;
    while (yaw >= 360.0) yaw -= 360.0;
    
    return yaw;
}


Eigen::Quaterniond anglesToENUQuaternion(double roll_deg, double pitch_deg, double azimuth_deg) {
    // 转换为弧度
    double roll_rad = roll_deg / 180 * M_PI;
    double pitch_rad = pitch_deg / 180 * M_PI;
    double yaw_rad = azimuthToENUYaw(azimuth_deg) / 180 * M_PI;
    
    // 使用Eigen的欧拉角到四元数转换 (ZYX顺序 - 航向, 俯仰, 横滚)
    // Eigen使用相反的顺序: 横滚(X), 俯仰(Y), 航向(Z)
    Eigen::Quaterniond quat;
    quat = Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ())   // 航向 (Z轴)
         * Eigen::AngleAxisd(pitch_rad, Eigen::Vector3d::UnitY()) // 俯仰 (Y轴)
         * Eigen::AngleAxisd(roll_rad, Eigen::Vector3d::UnitX()); // 横滚 (X轴)
    
    return quat.normalized();
}


Eigen::Quaterniond toQuaternionZYX(double roll, double pitch, double yaw) {

    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(M_PI/2- yaw , Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    q.normalize();
    return q;
}


