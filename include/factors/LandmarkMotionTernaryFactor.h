/**
 * @file LandmarkMotionTernaryFactor.h
 * @brief motion ternary factor
 * @date Feb 13, 2019
 * @author Mina Henein
 */

// A landmark motion ternary factor
// The factor contains the difference between l_{k+1} and the motion vertex _k H_{k+1} applied to l_k
// The error vector will be zeros(3,1) - [l_k - _k H_{k+1}^-1 l_{k+1}]'

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>

// you can custom namespace (if needed for your project)
namespace vdo
{
class LandmarkMotionTernaryFactor : public gtsam::NoiseModelFactor3<gtsam::Point3, gtsam::Point3, gtsam::Pose3>
{
private:
  // measurement information
  double mx_, my_, mz_;

public:
  /**
   * Constructor
   * @param motion     pose transformation (motion)
   * @param model      noise model for ternary factor
   * @param m          Vector measurement
   */
  LandmarkMotionTernaryFactor(gtsam::Key previousPointKey, gtsam::Key currentPointKey, gtsam::Key motionKey,
                              const gtsam::Point3& m, gtsam::SharedNoiseModel model)
    : gtsam::NoiseModelFactor3<gtsam::Point3, gtsam::Point3, gtsam::Pose3>(model, previousPointKey, currentPointKey,
                                                                           motionKey)
    , mx_(m.x())
    , my_(m.y())
    , mz_(m.z())
  {
  }

  // error function
  // l1 is the previouus point
  gtsam::Vector evaluateError(const gtsam::Point3& previousPoint, const gtsam::Point3& currentPoint,
                              const gtsam::Pose3& H, boost::optional<gtsam::Matrix&> J1 = boost::none,
                              boost::optional<gtsam::Matrix&> J2 = boost::none,
                              boost::optional<gtsam::Matrix&> J3 = boost::none) const
  {
    gtsam::Matrix H2, H3;
    gtsam::Vector l2H = H.transformTo(currentPoint, H3, H2);
    gtsam::Vector3 expected = previousPoint - l2H;

    if (J1)
      *J1 = (gtsam::Matrix33() << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0).finished();

    if (J2)
      *J2 = (gtsam::Matrix33() << -H2).finished();

    if (J3)
      *J3 = (gtsam::Matrix36() << -H3).finished();
    // return error vector
    return (gtsam::Vector3() << expected.x() - mx_, expected.y() - my_, expected.z() - mz_).finished();
  }

  inline gtsam::Key getPreviousPointKey() const
  {
    return key1();
  }
  inline gtsam::Key getCurrentPointKey() const
  {
    return key2();
  }
  inline gtsam::Key getMotionKey() const
  {
    return key3();
  }
};

}  // namespace vdo
