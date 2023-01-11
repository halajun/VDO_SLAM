#pragma once

#include "Macros.h"
#include "Types.h"
#include "Frame.h"

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>

namespace vdo {
class Camera;

namespace tracking_tools {

gtsam::Vector3 obtainFlowDepth(const Frame& frame, TrackletId tracklet_id);
void determineOutlierIds(const TrackletIds& inliers, const TrackletIds& tracklets, TrackletIds& outliers);

void updateMotionModel(const Frame::Ptr& previous_frame, Frame::Ptr current_frame);
void calculateSceneFlow(const Frame::Ptr& previous_frame, Frame::Ptr current_frame, const Camera& camera);

//using the previous frame updates the mask of the upcoming frame
void updateFrameMask(const Frame::Ptr& previous_frame, Frame::Ptr current_frame);


class MotionSolver {

public:

struct Options {

    enum Solver {
        //if selected the motion will be solved in the camera frame with the resulting solution T_prev_curr
        CAMERA,
        //if elected the motion will be solved in the world frame with the resulting pose T_w_c
        WORLD
    };

    double reprojection_error = 0.4; //used for both pnpRansac and reprojection error via motion model

    //options for pnp ransac solver
    int pnp_iterations = 500;
    double pnp_confidence = 0.98;
    Solver solver{ Solver::CAMERA };

};


struct Result {

    enum Selection {
        PNP,
        MOTION_MODEL
    };

    //vector of tracklet id's corresponding to features which have been classified as inliers
    //this will be taken from which ever model is used to generate the solution
    TrackletIds inliers;
    TrackletIds outliers;
    //either in T_w_c or T_prev_curr depending on the motion options
    gtsam::Pose3 pose;
    Options options; //the options used to generate this solution
    Selection model; //which model was ultimately selected to generate the pose solution
    bool success { false };

    //motion model metadata
    double motion_model_error = 0.0; //the total reprojection error using the existing motion model
    double motion_model_inlier_error = 0.0; //the reprojection error using only the inliers generated from the motion model
    
    //the number of points that we are to be reprojected into the camera frame. 
    //if all successful this should be the same length as the number if input features
    int successful_reprojections = 0;
    
};

/**
 * @brief Attemps to solve for pose given a set of 2d observations in the current frame
 * and 3d points in the previous frame. The 3d poins must be in the camera frame of the previous pose.
 * 
 * We solve for pose (either relative pose, ie ^C{t-1}_T_{t} or pose in world frame ^X_wc) depending on the provided
 * motion options, using PnP with RANSAC. We compare the number of outliers from PnP with the number of outliers
 * generated using the existing motion model and select the set of inliers/outliers that has the smallest reprojection error.
 * 
 * @param current_uv 
 * @param previous_3d_cam 
 * @param tracklet_ids 
 * @param previous_pose T_wc at t-1
 * @param motion_model 
 * @param camera 
 * @return Result 
 */
static Result solvePnpOrMotion(const KeypointsCV& current_uv,
                              const Landmarks& previous_3d_cam,
                              const TrackletIds& tracklet_ids,
                              const gtsam::Pose3& previous_pose,
                              const gtsam::Pose3& motion_model,
                              const Camera& camera,
                              const Options& options = Options());

};


} //tracking_tools
} //vdo