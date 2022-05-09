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
namespace VDO_SLAM {

  class Point3DFactor: public gtsam::NoiseModelFactor2<gtsam::Pose3,gtsam::Point3> {

private:
  // measurement information
  double mx_, my_, mz_;

public:

  /**
   * Constructor
   * @param pose     camera pose transformation
   * @param model    noise model for ternary factor
   * @param m        Vector measurement
   */
  Point3DFactor(gtsam::Key poseKey, gtsam::Key pointKey,
          const gtsam::Point3& m, gtsam::SharedNoiseModel model) :
            gtsam::NoiseModelFactor2<gtsam::Pose3,gtsam::Point3>(model,poseKey,pointKey),
                    mx_(m.x()), my_(m.y()), mz_(m.z()) {}

  // error function
  gtsam::Vector evaluateError(const gtsam::Pose3& X, const gtsam::Point3& l,
          boost::optional<gtsam::Matrix&> J1 = boost::none,
          boost::optional<gtsam::Matrix&> J2 = boost::none) const {
      
      
      
    gtsam::Matrix H1, H2;
    gtsam::Vector expected = X.transformTo(l, H1, H2);

/*
    gtsam::Matrix33 Rt = (X.rotation().inverse()).matrix();
    gtsam::Vector3 expected = Rt*l.vector() - Rt*X.translation().vector();
    
    int dof_edge = 3;
    int dof_pose = 6;
    double eps_2 = 1e-10;
    gtsam::Matrix6 Eps = eye(dof_pose,dof_pose)*eps_2;

    gtsam::Vector3 edgeValue = Rt*l.vector() - Rt*X.translation().vector();
    gtsam::Matrix J1Numerical = zeros(dof_edge,dof_pose);
    
    gtsam::Matrix44 screwOldFrame = X.matrix();
    gtsam::Vector6 screwRelative = zero(dof_pose);
        
    for (int j=0; j<dof_pose; j++) {
        for (int k=0; k<dof_pose; k++){
            screwRelative(k,0) = Eps(k,j);
        }
        
        gtsam::Vector3 screwRelativeAA = screwRelative.block(3,0,3,1);  
        gtsam::Point3 screwRelativeT = screwRelative.block(0,0,3,1);        
        gtsam::Rot3 mat;
        if (screwRelativeAA.norm() != 0){
            mat = gtsam::Rot3::AxisAngle(screwRelativeAA/(screwRelativeAA.norm()),screwRelativeAA.norm());
            }
        else{
            mat = gtsam::Rot3::AxisAngle(screwRelativeAA,screwRelativeAA.norm());
            }
        
        gtsam::Pose3 screwRelativePose = gtsam::Pose3(mat,screwRelativeT);
        gtsam::Matrix44 perturbedX = screwOldFrame * screwRelativePose.matrix();
        gtsam::Rot3 perturbedXR = gtsam::Rot3(perturbedX.block(0,0,3,3));
        gtsam::Point3 perturbedXt = gtsam::Point3(perturbedX.block(0,3,3,1));
        gtsam::Vector3 d1 = perturbedXR.inverse()*l - perturbedXR.inverse()*perturbedXt;
        gtsam::Vector3 numJ = (d1-edgeValue)/(eps_2);
        
        for (int k=0; k<dof_edge; k++){
            J1Numerical(k,j) = numJ(k,0); 
        }
    }
    
    if (J1) *J1 = (gtsam::Matrix36() << J1Numerical).finished();
    
    if (J2) *J2 = (gtsam::Matrix33() << Rt).finished();
*/  

    if (J1) *J1 = (gtsam::Matrix36() << H1).finished();
    
    if (J2) *J2 = (gtsam::Matrix33() << H2).finished();  
    
    // return error vector
    return (gtsam::Vector3() << expected.x()-mx_, expected.y()-my_, expected.z()-mz_).finished();
  }

};

} // namespace VDO_SLAM

