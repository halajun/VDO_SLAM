#pragma once

#include <opencv2/core/core.hpp>

#include <Eigen/Dense>
#include "dependencies/g2o/g2o/types/types_six_dof_expmap.h"
#include "dependencies/g2o/g2o/types/types_seven_dof_expmap.h"

#include <gtsam/geometry/Pose3.h>

namespace vdo {

namespace utils {


g2o::SE3Quat toSE3Quat(const cv::Mat& cvT);
// g2o::SE3Quat toSE3Quat(const g2o::Sim3& gSim3);

gtsam::Pose3 toGtsamPose3(const g2o::SE3Quat& se3_quat);


}

}