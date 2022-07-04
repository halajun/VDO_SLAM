/**
 * @file AltitudeFactor.h
 * @brief Altitde factor
 * @date July 4, 2022
 * @author Jesse Morris
 */
 

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace VDO_SLAM {

class AltitudeFactor: public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3> {

private:
  // measurement information
  double measured_;

public:

    AltitudeFactor(gtsam::Key poseKey, gtsam::Key pointKey, double measured, const gtsam::SharedNoiseModel& model)
        :   gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3>(model, poseKey, pointKey),
            measured_(measured) {}
  
    gtsam::Vector evaluateError(const gtsam::Pose3& pose, const gtsam::Point3& point,
            boost::optional<gtsam::Matrix&> H1, boost::optional<gtsam::Matrix&> H2) const {
        double hx = pose.z() - point.z();

        if (H1) {
            *H1 = gtsam::Matrix::Zero(1,6);
            // Use bottom row of rotation matrix for derivative of translation
            (*H1)(0, 3) = pose.rotation().r1().z();
            (*H1)(0, 4) = pose.rotation().r2().z();
            (*H1)(0, 5) = pose.rotation().r3().z();
        }

        if (H2) {
            *H2 = gtsam::Matrix::Zero(1,3);
            (*H2)(0, 2) = -1.0;
        }
        return (gtsam::Vector(1) << hx - measured_).finished();
    }

    inline const double measured() const { return  measured_; }

};

} // namespace VDO_SLAM

