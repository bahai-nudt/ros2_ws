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

Eigen::Matrix3d GetSkewMatrix(const Eigen::Vector3d& v) {
    Eigen::Matrix3d w;
    w <<  0.,   -v(2),  v(1),
          v(2),  0.,   -v(0),
         -v(1),  v(0),  0.;

    return w;
}

bool get_before_after_pose(Pose& pose_before, Pose& pose_after, double timestamp, const std::vector<Ins>& vector_ins) {

    int size = static_cast<int>(vector_ins.size());

    if (size == 0) {
        return false;
    }

    if (timestamp > vector_ins[size - 1]._timestamp || timestamp < vector_ins[0]._timestamp) {
        return false;
    }

    double timestamp_first = vector_ins[0]._timestamp;
    int index = (timestamp - timestamp_first) * 100;

    if (index > size - 2) {
        return false;
    }
    pose_before = vector_ins[index]._pose;
    pose_after = vector_ins[index + 1]._pose;

    return true;

}



};