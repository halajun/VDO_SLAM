/**
 * @file Point3DFactor.h
 * @brief 3D projection factor
 * @date Mar 25, 2019
 * @author Mina Henein
 * @author Jesse Morris
 */

//  A landmark 3D projection factor
//  The factor contains the difference between the expecetd landmark measurement
//  and the measured ^k l^i_k
//  The error vector will be ^0 X_k^-1 * ^0 l^i_k - ^k l^i_k

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>

// you can custom namespace (if needed for your project)
namespace vdo
{
class Point3DFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3>
{
private:
  // measurement information
  // double mx_, my_, mz_;
  gtsam::Point3 measured_; //in camera coordinates

public:
  /**
   * Constructor
   * @param pose     camera pose transformation
   * @param model    noise model for ternary factor
   * @param m        Vector measurement
   */
  Point3DFactor(gtsam::Key poseKey, gtsam::Key pointKey, const gtsam::Point3& m, gtsam::SharedNoiseModel model)
    : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3>(model, poseKey, pointKey)
    ,
    // mx_(m.x()), my_(m.y()), mz_(m.z()) {}
    measured_(m.x(), m.y(), m.z())
  {
  }

  // error function
  //L is landmark in world coordinates
  gtsam::Vector evaluateError(const gtsam::Pose3& X, const gtsam::Point3& l,
                              boost::optional<gtsam::Matrix&> J1 = boost::none,
                              boost::optional<gtsam::Matrix&> J2 = boost::none) const
  {
    gtsam::Matrix H1, H2;

    gtsam::Vector expected = X.transformTo(l, H1, H2);

    if (J1)
      *J1 = (gtsam::Matrix36() << H1).finished();

    if (J2)
      *J2 = (gtsam::Matrix33() << H2).finished();

    // return error vector
    return (gtsam::Vector3() << expected - measured_).finished();
  }

  const gtsam::Point3& measured() const
  {
    return measured_;
  }
};

}  // namespace vdo
