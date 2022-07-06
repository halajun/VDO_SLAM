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
namespace VDO_SLAM {

class LandmarkMotionTernaryFactor: public gtsam::NoiseModelFactor3<gtsam::Point3,gtsam::Point3,gtsam::Pose3> {

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
  LandmarkMotionTernaryFactor(gtsam::Key point1Key,gtsam::Key point2Key,
          gtsam::Key motionKey,const gtsam::Point3& m,gtsam::SharedNoiseModel model) :
            gtsam::NoiseModelFactor3<gtsam::Point3,gtsam::Point3,gtsam::Pose3>(model,point1Key,point2Key,motionKey),
                    mx_(m.x()), my_(m.y()), mz_(m.z()) {}

  
  // error function
  //l1 is the current point
  gtsam::Vector evaluateError(const gtsam::Point3& l1,const gtsam::Point3& l2,const gtsam::Pose3& H,
          boost::optional<gtsam::Matrix&> J1 = boost::none,
          boost::optional<gtsam::Matrix&> J2 = boost::none,
          boost::optional<gtsam::Matrix&> J3 = boost::none) const {
        
  
    // note that use boost optional like a pointer
    // only calculate jacobian matrix when non-null pointer exists                                    
    
    gtsam::Matrix H2, H3;
    gtsam::Vector l2H = H.transformTo(l2, H3, H2);
    //gtsam::Matrix H1, H3;
    //gtsam::Vector Hl1 = H.transform_from(l1, H3, H1);
    
    gtsam::Vector3 expected = l1 - l2H;
    //gtsam::Vector3 expected = l2.vector() - Hl1;
                                                                        
    if (J1) *J1 = (gtsam::Matrix33() << 1.0, 0.0, 0.0, 
                                        0.0, 1.0, 0.0,
                                        0.0, 0.0, 1.0).finished();
   
    if (J2) *J2 = (gtsam::Matrix33() << -H2).finished();
    
    if (J3) *J3 = (gtsam::Matrix36() << -H3).finished();  
    
    
    //if (J1) *J1 = (gtsam::Matrix33() << -H1).finished();
    
    //if (J2) *J2 = (gtsam::Matrix33() << 1.0, 0.0, 0.0, 
    //                                    0.0, 1.0, 0.0,
    //                                    0.0, 0.0, 1.0).finished();
    
    
    //if (J3) *J3 = (gtsam::Matrix36() << -H3).finished();
                                      
    // return error vector
    return (gtsam::Vector3() << expected.x()-mx_, expected.y()-my_, expected.z()-mz_).finished();
  }

  //I think this is the correct ordering
  inline gtsam::Key getPreviousPointKey() const { return key1(); }
  inline gtsam::Key getCurrentPointKey() const { return key2(); }
  inline gtsam::Key getMotionKey() const { return key3(); }

};

} // namespace VDO_SLAM

