#pragma once

#include "Logger.h"
#include "Frame.h"
#include "utils/Metrics.h"

#include <gtsam/geometry/Pose3.h>

namespace vdo {

template<>
inline void Formatter<gtsam::Pose3>::header(std::ofstream& stream) {
    stream << "x,y,z,qw,qx,qy,qz";
}

template<>
inline void Formatter<gtsam::Pose3>::format(std::ofstream& stream, const gtsam::Pose3& value) {
    const auto& translation = value.translation();
    const auto& quat = value.rotation().quaternion();

    stream << translation.x() << ","
           << translation.y() << ","
           << translation.z() << ","
           << quat.w() << ","
           << quat.x() << ","
           << quat.y() << ","
           << quat.z();
}

template<>
inline void Formatter<Frame>::header(std::ofstream& stream) {
    //frame id, timestamp, estimated pose, grount truth pose
    // stream << "frame_id,timestamp,x,y,z,qw,qx,qy,qz,gt_x,gt_y,gt_z,gt_qw,gt_qx,gt_qy,gt_qz";
    stream << "frame_id,timestamp,t_error,r_error";
}

template<>
inline void Formatter<Frame>::format(std::ofstream& stream, const Frame& value) {
    stream << value.frame_id << ","
           << value.timestamp << ",";
    // Formatter<gtsam::Pose3>::format(stream, value.pose);
    // stream << ",";

    double t_error, r_error;
    calculatePoseError(value.pose, value.ground_truth->X_wc, t_error, r_error);
    stream << t_error << "," << r_error;
 
}

}