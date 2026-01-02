#include "base_utils/tools.h"
#include <iomanip>

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

bool get_before_after_pose(Pose& pose_before, Pose& pose_after, double timestamp, const std::deque<Ins>& deque_ins) {

    
    int size = static_cast<int>(deque_ins.size());

    if (size == 0) {
        return false;
    }

    std::cout << "timestamp:   " << std::setprecision(16) << timestamp << std::endl;
    std::cout << "deque_ins[size - 1]._timestamp:  " << std::setprecision(16) << deque_ins[size - 1]._timestamp << std::endl;
    std::cout << "deque_ins[size - 1]._timestamp:  " << std::setprecision(16) << deque_ins[0]._timestamp << std::endl;


    if (timestamp > deque_ins[size - 1]._timestamp || timestamp < deque_ins[0]._timestamp) {
        return false;
    }

    for (int i = size - 1; i > 0; i--) {
        if (deque_ins[i]._timestamp > timestamp) {
            continue;
        }

        if (i >= size - 1) {
            return false;
        }
        if (deque_ins[i+1]._timestamp - deque_ins[i]._timestamp > 18 * 1e-3 || deque_ins[i+1]._timestamp < timestamp ) {
            return false;
        }

        pose_before = deque_ins[i]._pose;
        pose_after = deque_ins[i+1]._pose;

        return true;
    }
}

};