#pragma once

#include "Logger.h"
#include "Frame.h"

#include <gtsam/geometry/Pose3.h>

namespace vdo {

template<>
inline void Formatter<gtsam::Pose3>::header(std::ofstream& stream) {
    stream << "x,y,z,w,x,y,z";
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
    stream << "frame_id,timestamp,";
    Formatter<gtsam::Pose3>::header(stream);
    stream << ",";
    Formatter<gtsam::Pose3>::header(stream);
}

template<>
inline void Formatter<Frame>::format(std::ofstream& stream, const Frame& value) {
    stream << value.frame_id << ","
           << value.timestamp << ",";
    Formatter<gtsam::Pose3>::format(stream, value.pose);
    stream << ",";
    Formatter<gtsam::Pose3>::format(stream, value.ground_truth->X_wc);
}

}