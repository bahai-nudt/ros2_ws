#ifndef LOCALIZATION_BASE_TOOLS_H 
#define LOCALIZATION_BASE_TOOLS_H

#include "data_message.h"
#include <deque>

namespace BaseTools {

Pose interpolatePose(const Pose& pose_before, const Pose& pose_after, double timestamp);
Eigen::Matrix3d GetSkewMatrix(const Eigen::Vector3d& v);

bool get_before_after_pose(Pose& pose_before, Pose& pose_after, double timestamp, const std::deque<Ins>& deque_ins);
bool get_before_after_pose(Pose& pose_before, Pose& pose_after, double timestamp, const std::vector<Pose>& deque_ins);

};



#endif