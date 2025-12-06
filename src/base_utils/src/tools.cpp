#include "base_utils/tools.h"

namespace BaseTools {

Pose interpolatePose(const Pose& pose_before, const Pose& pose_after, double timestamp) {        
    // 线性插值
    double t_before = pose_before.timestamp;
    double t_after = pose_after.timestamp;
    double alpha = (timestamp - t_before) / (t_after - t_before);
    
    // 位置插值
    Eigen::Vector3d interp_pos = 
        (1 - alpha) * pose_before.position + alpha * pose_after.position;
    
    // 姿态插值（球面线性插值）
    Eigen::Quaterniond interp_orient = 
        pose_before.orientation.slerp(alpha, pose_after.orientation);
    
    return Pose(timestamp, interp_pos, interp_orient);
}

};