#include "backend/VdoSlamBackend.h"
#include "utils/UtilsGTSAM.h"
#include "utils/UtilsOpenCv.h"

#include <glog/logging.h>
#include <fstream>
#include <memory>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/geometry/Pose3.h>

#include "utils/macros.h"
#include "utils/timing.h"
#include "factors/Point3DFactor.h"
#include "factors/LandmarkMotionTernaryFactor.h"
#include "factors/AltitudeFactor.h"
#include "utils/dataset.h"
#include "Map.h"
#include "Optimizer.h"
// #include "Converter.h"

namespace VDO_SLAM {

VdoSlamBackend::VdoSlamBackend(Map* map_, const cv::Mat& Calib_K_, BackendParams::Ptr params_) 
    :   map(CHECK_NOTNULL(map_)),
        K(Calib_K_),
        params(CHECK_NOTNULL(params_)),
        count_unique_id(1),
        pre_camera_pose_vertex(-1),
        curr_camera_pose_vertex(0),
        current_frame(-1),
        key_to_unique_vertices(),
        cameraPosePrior(nullptr),
        odometryNoiseModel(nullptr),
        point3DNoiseModel(nullptr),
        objectMotionSmootherNoiseModel(nullptr),
        dynamicPoint3DNoiseModel(nullptr),
        objectMotionNoiseModel(nullptr) {

        gtsam::ISAM2Params parameters;
        // parameters.relinearizeThreshold = 0.01;
        // parameters.relinearizeSkip = 1;

        isam = VDO_SLAM::make_unique<gtsam::ISAM2>(parameters);
        K_calib =  utils::cvMat2Cal3_S2(K);

        //parsed in as float but we want it as a double... cuz gtsam
        //will this mess up all our other data types?
        cv::Mat K_d;
        K.convertTo(K_d, CV_64F);
        camera = std::make_shared<Camera>(K_d);
        camera->print();

        setupNoiseModels();        

        // //TODO: for 3d Points only. See optimzier.
        // robust_noise_model =  gtsam::noiseModel::Robust::Create(
        //     gtsam::noiseModel::mEstimator::Huber::Create(0.0001), point3DNoiseModel);  // robust

        CHECK_NOTNULL(cameraPosePrior);
        CHECK_NOTNULL(odometryNoiseModel);
        CHECK_NOTNULL(point3DNoiseModel);
        CHECK_NOTNULL(objectMotionSmootherNoiseModel);
        CHECK_NOTNULL(dynamicPoint3DNoiseModel);
        CHECK_NOTNULL(objectMotionNoiseModel);


        cameraPosePrior->print("Camera Pose Prior ");
        odometryNoiseModel->print("Odometry Noise Model ");
        point3DNoiseModel->print("3D Point Noise Model ");
        objectMotionSmootherNoiseModel->print("Object motion smoother ");
        dynamicPoint3DNoiseModel->print("Dynamic 3d point ");
        objectMotionNoiseModel->print("Object Motion model: ");
        CHECK_NOTNULL(K_calib);

        do_manager = VDO_SLAM::make_unique<DynamicObjectManager>(map_);
    }

void VdoSlamBackend::process() {
    const int N = map->vpFeatSta.size(); // Number of Frames
    LOG(INFO) << "Running incremental update on frame " << N;

    //first vector is number of static tracklets
    const std::vector<std::vector<std::pair<int, int>>>& StaTracks = map->TrackletSta;
    //length of DynTracks is ALL the features which are clasififed as dynamic  and then the corresponding
    //vector (ef. DynTracks[i] gives what frame and what feature Id it appeared in)
    //not sure how this vector changes -> once an object is no longer tracked will we ever go back 
    //and change the track or can I assume that the map either operatos on the current tracks or ONLY
    //new tracks with a greater ID?
    const std::vector<std::vector<std::pair<int, int>>>& DynTracks = map->TrackletDyn;
    LOG(INFO) << "Number of objects tracked is " << DynTracks.size();

    current_frame = N - 1;

    std::vector<int>  vnFLS_tmp(map->vpFeatSta[current_frame].size(),-1);
    vnFeaLabSta.push_back(vnFLS_tmp);
    vnFeaMakSta.push_back(vnFLS_tmp);

    std::vector<int> vnFLD_tmp(map->vpFeatDyn[current_frame].size(),-1);
    vnFeaLabDyn.push_back(vnFLD_tmp);
    vnFeaMakDyn.push_back(vnFLD_tmp);


    CHECK_EQ(vnFeaLabSta.size(), N);
    CHECK_EQ(vnFeaMakSta.size(), N);
    CHECK_EQ(vnFeaLabDyn.size(), N);
    CHECK_EQ(vnFeaMakDyn.size(), N);



    int valid_sta = 0, valid_dyn = 0;

    //this is actually going back over the ENTIRE map. I dont think the labels change ever
    //so I might not have to do this
    // label static feature
    for (int i = 0; i < StaTracks.size(); ++i)
    {
        // filter the tracklets via threshold
        if (StaTracks[i].size()<3) { // 3 the length of track on background.
            continue;
        }
        valid_sta++;
        // label them
        for (int j = 0; j < StaTracks[i].size(); ++j) {
            //first -> frameId, second -> feature Id
            vnFeaLabSta[StaTracks[i][j].first][StaTracks[i][j].second] = i;
        }
    }
    // label dynamic feature
    for (int i = 0; i < DynTracks.size(); ++i)
    {
        // filter the tracklets via threshold
        if (DynTracks[i].size()<3) { // 3 the length of track on objects.
            continue;
        }
        valid_dyn++;
        // label them
        for (int j = 0; j < DynTracks[i].size(); ++j){
            //first -> frameId, second -> feature Id
            vnFeaLabDyn[DynTracks[i][j].first][DynTracks[i][j].second] = i;
        }
    }
    LOG(INFO) << "Tagged dynamic and static features";


    // check if objects has the required tracking length in current window
    // const int ObjLength = WINDOW_SIZE-1;
    //only do this i
    // std::vector<std::vector<bool> > ObjCheck(N-1);
    // for (int i = 0; i < N-1; ++i)
    // {
    //     std::vector<bool>  ObjCheck_tmp(pMap->vnRMLabel[i].size(),false);
    //     ObjCheck[i] = ObjCheck_tmp;
    // }
    CHECK_EQ(map->vnRMLabel.size(), current_frame);
    if(map->vnRMLabel.size() < 1) {
        // std::vector<bool> ObjCheck_tmp(1,false);
        // objCheck.push_back(ObjCheck_tmp);
        
    } 
    else {
        //current frame should start to be valid
        std::vector<bool> ObjCheck_tmp(map->vnRMLabel[current_frame-1].size(),false);
        objCheck.push_back(ObjCheck_tmp);
    }
    // std::vector<bool> ObjCheck_tmp(map->vnRMLabel[current_frame-1].size(),false);
    //     objCheck.push_back(ObjCheck_tmp);
    
    CHECK_EQ(objCheck.size(), current_frame);
    
    //we go back the last TS frames to check if the object has been tracked over this period
    const int TRACKING_SIZE = 7;
    const int OBJ_TRACK_LENGTH = TRACKING_SIZE-1;
    LOG(INFO) << "Tracking object size: " << TRACKING_SIZE << " and track length " << OBJ_TRACK_LENGTH;

    // collect unique object label and how many times it appears
    //for current naive implementation wait till current frame > TRACKING_SIZE otherwise segfault
    if (current_frame >= TRACKING_SIZE) {

        std::vector<int> UniLab, LabCount;
         LOG(INFO) << map->vnRMLabel.size();
        for(int i = N - TRACKING_SIZE; i < current_frame; i++) {
            LOG(INFO) << i << " " << map->vnRMLabel[i].size();
            if(i == N - TRACKING_SIZE) {
                for (int j = 1; j < map->vnRMLabel[i].size(); j++) {
                    UniLab.push_back(map->vnRMLabel[i][j]);
                    LabCount.push_back(1);
                }
            }
            else {
                for (int j = 1; j < map->vnRMLabel[i].size(); j++) {
                    bool used = false;

                    for(int k = 0; k < UniLab.size(); k++) {
                        //if the unique label at index k is the same as this
                        //rigid body motion, label it it has used and inncrease count
                        if (UniLab[k] == map->vnRMLabel[i][j]) {
                            used = true;
                            LabCount[k] = LabCount[k] + 1;
                            break;
                        }
                    }

                    if (!used) {
                        UniLab.push_back(map->vnRMLabel[i][j]);
                        LabCount.push_back(1);
                    }

                }
            }
        }
        LOG(INFO) << "Done object count collection"; 

        // assign the ObjCheck ......
        //if the object has been tracked in numner of framws as windowSize
        for (int i = N-TRACKING_SIZE; i < current_frame; ++i) {
            for (int j = 1; j < map->vnRMLabel[i].size(); ++j) {
                for (int k = 0; k < UniLab.size(); ++k) {
                    if (UniLab[k] == map->vnRMLabel[i][j] && LabCount[k] >= OBJ_TRACK_LENGTH) {
                        LOG(INFO) << "Tracked obj " << UniLab[k] << " for " << LabCount[k] << " frames";
                        objCheck[i][j] = true;
                        break;
                    }
                }
            }
        }

        LOG(INFO) << "Done object check assignment";

    }



    if(current_frame == 0) {
        CHECK_EQ(N, 1);
        std::vector<gtsam::Key> v_id_tmp(1,-1);
        unique_vertices.push_back(v_id_tmp);
    }
    else {
        //careful about casting -> vnRMLabel is int and we're casting to uint32...?
        std::vector<gtsam::Key> v_id_tmp(map->vnRMLabel[current_frame-1].size(),-1);
        unique_vertices.push_back(v_id_tmp);
    }

    CHECK_EQ(unique_vertices.size(), N);

    // const GtsamAccesType sigma2_cam = 0.0001; // 0.005 0.001 0.0001
    // const GtsamAccesType sigma2_obj_smo = 0.1; // 0.1
    // const GtsamAccesType sigma2_obj = 20; // 0.5 1 10 20
    // const GtsamAccesType sigma2_3d_dyn = 16; // 50 100 16
    // const GtsamAccesType sigma2_alti = 1;

    // const GtsamAccesType camera_pose_prior_sigma = 0.000001;
    // auto camera_pose_prior_n = gtsam::noiseModel::Diagonal::Sigmas(
    //     (gtsam::Vector(6) << gtsam::Vector6::Constant(camera_pose_prior_sigma)).finished()
    // );


    int FeaLengthThresSta = 3;
    int FeaLengthThresDyn = 3;

     //we just have the one camera pose measurement to add
    // (1) save <VERTEX_POSE_R3_SO3>
    gtsam::Pose3 camera_pose = utils::cvMatToGtsamPose3(map->vmCameraPose[current_frame]);
    addCameraPoseToGraph(camera_pose, count_unique_id, current_frame);
   
    //is first (or current_frame == 0?)
    if(count_unique_id == 1) {
        graph.addPrior(count_unique_id, camera_pose, cameraPosePrior);
        LOG(INFO) << "Added camera prior";
    }

    //unique id for the first frame and the camera pose
    unique_vertices[current_frame][0] = count_unique_id;
    curr_camera_pose_vertex = count_unique_id;
    count_unique_id++;

    //is the first frame
    if (current_frame == 0) {
        for (int j = 0; j < vnFeaLabSta[current_frame].size(); ++j) {
            if (vnFeaLabSta[current_frame][j]==-1) {
                continue;
            }

            gtsam::Point3 X_w = utils::cvMatToGtsamPoint3(map->vp3DPointSta[current_frame][j]);
            addLandmarkToGraph(X_w, count_unique_id, current_frame, j);

            cv::Mat Xc = Optimizer::Get3DinCamera(map->vpFeatSta[current_frame][j],map->vfDepSta[current_frame][j],K);
            gtsam::Point3 X_c_point = utils::cvMatToGtsamPoint3(Xc);
            // graph.emplace_shared<Point3DFactor>(curr_camera_pose_vertex, count_unique_id, X_c_point, camera_projection_noise);
            addPoint3DFactor(X_c_point, curr_camera_pose_vertex, count_unique_id);

            vnFeaMakSta[current_frame][j] = count_unique_id;
            count_unique_id++;
        }
    }
    //not the first call
    else {
        // (2) save <EDGE_R3_SO3> 
        //pretty sure we want to optimize for the rigid motion too...? This should become a variable?
        gtsam::Pose3 rigid_motion = utils::cvMatToGtsamPose3(map->vmRigidMotion[current_frame-1][0]);
        graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            pre_camera_pose_vertex,
            curr_camera_pose_vertex,
            rigid_motion,
            odometryNoiseModel
        );
        LOG(INFO) << "Added motion constraint factor between id: " <<pre_camera_pose_vertex << " " << curr_camera_pose_vertex;

        //loop for static features
        LOG(INFO) << "Static features for current frame is " << vnFeaLabSta[current_frame].size() << " frame " << current_frame;
        for (int j = 0; j < vnFeaLabSta[current_frame].size(); ++j) {
            // check feature validation
            if (vnFeaLabSta[current_frame][j]==-1) {
                continue;
            }
            // get the TrackID of current feature
            int TrackID = vnFeaLabSta[current_frame][j];

            // if(current_frame == 1) {
                LOG(INFO) << current_frame;
            // } 

            // get the position of current feature in the tracklet
            int PositionID = -1;
            //trackets require at least one prior frame to exist to track the feature through
            //so the first frame that we can track the the features is 
            for (int k = 0; k < StaTracks[TrackID].size(); ++k) {
                // LOG(INFO) << StaTracks[TrackID][k].first;
                //if we're not looping through all the frames (eg i becomes start_frame is this going to be a problem?)
                if(StaTracks[TrackID][k].first==current_frame && current_frame == 1) {
                    LOG(INFO) << "Track at frame " << current_frame << " and k " << k; 
                }

                if(StaTracks[TrackID][k].second==j && current_frame == 1) {
                    LOG(INFO) << "Track at id " << j << " and k " << k; 
                }
                if (StaTracks[TrackID][k].first==current_frame && StaTracks[TrackID][k].second==j) {
                    // LOG(INFO) << current_frame;
                    PositionID = k;
                    break;
                }
            }
            if (PositionID==-1){
                LOG(WARNING) << "cannot find the position of current feature in the tracklet !!!";
                continue;
            }

            // check if the PositionID is 0. Yes means this static point is first seen by this frame,
            // then save both the vertex and edge, otherwise save edge only because vertex is saved before.
            //so the smallest value that exists seems to be two...?
            //this isnt a hack...
            //I think i need to wait till this is obsevred at least twice to constrain this problem
            //add to a list of variables and then once we have two factors we can add all of them?
            //list of variables which map to factors..? Need at least two so can just track the number of factors.
            //must ensure that the conditions of newFactors and newTheta are satisfied.
            // LOG(INFO) << "Position ID " << PositionID;
            CHECK(PositionID != 0);
            if (PositionID==2) {
                //then also check that it is not in the graph
                //check in isam or check in local NLFG?
                CHECK(!isam->valueExists(count_unique_id));
                // check if this feature track has the same length as the window size
                const int TrLength = StaTracks[TrackID].size();
                if ( TrLength<FeaLengthThresSta ) {
                    continue;
                }


                gtsam::Point3 X_w = utils::cvMatToGtsamPoint3(map->vp3DPointSta[current_frame][j]);
                addLandmarkToGraph(X_w, count_unique_id, current_frame, j);

                // values.insert(count_unique_id, X_w);
                // addToKeyVertexMapping(count_unique_id, current_frame, j, kSymbolPoint3Key);
                // // LOG(INFO) << "Added point3 key to vertex mapping " << count_unique_id << " " << kSymbolPoint3Key;



                //TODO: make camera class or equivalent
                cv::Mat Xc = Optimizer::Get3DinCamera(map->vpFeatSta[current_frame][j],map->vfDepSta[current_frame][j],K);
                gtsam::Point3 X_c_point = utils::cvMatToGtsamPoint3(Xc);
                // graph.emplace_shared<Point3DFactor>(curr_camera_pose_vertex, count_unique_id, X_c_point, camera_projection_noise);
                addPoint3DFactor(X_c_point, curr_camera_pose_vertex, count_unique_id);

                vnFeaMakSta[current_frame][j] = count_unique_id;
                count_unique_id++;

            }
            else if(PositionID > 2) {
                // LOG(INFO) << PositionID;
                //landmark should be in graph
                // CHECK(isam->valueExists(curr_camera_pose_vertex) || graph.at);

            // // check if this feature track has the same length as the window size
                    // // or its previous FeaMakTmp is not -1, then save it, otherwise skip.
                const int TrLength = StaTracks[TrackID].size();
                const int FeaMakTmp = vnFeaMakSta[StaTracks[TrackID][PositionID-1].first][StaTracks[TrackID][PositionID-1].second];
                // if (TrLength<FeaLengthThresSta || FeaMakTmp==-1) {
                //     continue;
                // }

                //well it might not yet?
                // CHECK(isam->valueExists(FeaMakTmp));


                //base logic checks
                // gtsam::Values::iterator it =  values.find(FeaMakTmp);
                // if(it == values.end()) { LOG(ERROR) << "Static Feature Key is not a valid variable";};

                // it == values.find(CurFrameID);
                // if(it == values.end()) { LOG(ERROR) << "Camera frame ID Key is not a valid variable";};

                //now add factor
                //TODO: make camera class or equivalent
                cv::Mat Xc = Optimizer::Get3DinCamera(map->vpFeatSta[current_frame][j],map->vfDepSta[current_frame][j],K);
                gtsam::Point3 X_c_point = utils::cvMatToGtsamPoint3(Xc);


                // graph.emplace_shared<Point3DFactor>(curr_camera_pose_vertex, FeaMakTmp, X_c_point, camera_projection_noise);
                addPoint3DFactor(X_c_point, curr_camera_pose_vertex, FeaMakTmp);
                vnFeaMakSta[current_frame][j] = FeaMakTmp;
            }

        }
    }

    //end looping for static features
    //okay need to wait for at least TRACKING_SIZE (for naive implementation)
    //eventually need to wait and then go back.
    //Lets start by just adding tracks in TRACKING_SIZE 
    if(current_frame  == 0) {
    // if(current_frame < TRACKING_SIZE) {
        LOG(INFO) << "Frame <= tracking size " << current_frame << " vs " << TRACKING_SIZE;
        //loop for dynamic features
        for(int j = 0; j < vnFeaLabDyn[current_frame].size(); j++) {

            //check for feature validation
            if(vnFeaLabDyn[current_frame][j] == -1) {
                continue;
            }

            gtsam::Point3 X_w = utils::cvMatToGtsamPoint3(map->vp3DPointDyn[current_frame][j]);
            addDynamicLandmarkToGraph(X_w, count_unique_id, current_frame, j);

            cv::Mat Xc = Optimizer::Get3DinCamera(map->vpFeatDyn[current_frame][j],map->vfDepDyn[current_frame][j],K);
            gtsam::Point3 X_c_point = utils::cvMatToGtsamPoint3(Xc);
            addDynamicPoint3DFactor(X_c_point, curr_camera_pose_vertex, count_unique_id);


            // update unique id
            vnFeaMakDyn[current_frame][j] = count_unique_id;
            count_unique_id++;

        }
    }
    else {
        // loop for object motion, and keep the unique vertex id for saving object feature edges
        std::vector<int> ObjUniqueID(map->vmRigidMotion[current_frame-1].size()-1,-1);

        for (int j = 1; j < map->vmRigidMotion[current_frame-1].size(); j++) {

            // if (!objCheck[current_frame][j]) {
            //     continue;
            // }

            gtsam::Pose3 object_motion = gtsam::Pose3::identity();

            // if(map->vbObjStat[current_frame-1][j]) {
            //     object_motion = utils::cvMatToGtsamPose3(map->vmRigidMotion[current_frame-1][j]);
            // }
            // else {
            //     object_motion = gtsam::Pose3::identity();
            // }
            //ignoreing smooth constraint here
            addMotionToGraph(object_motion, count_unique_id, current_frame, j);


            //add smoothing betweeb motion
            //guaranteed to be > 2 here due to TRACKIGN_SIZE.
            if(current_frame > 2) {
                CHECK_GT(current_frame, 2) << "Current frame must be > 2 to apply smoothing constraint. What is the tracking size?";
                int traceID = -1;
                for (int k = 0; k < map->vnRMLabel[current_frame-2].size(); k++) {
                    if(map->vnRMLabel[current_frame-2][k] == map->vnRMLabel[current_frame-1][j]) {
                        traceID = k;
                        break;
                    }
                }

                // only if the back trace exist
                int trace_key = unique_vertices[current_frame-2][traceID];
                if(traceID != -1 && trace_key != -1 ) {
                    LOG(INFO) << "trace key " << trace_key;
                    //add smoother between vertices
                    gtsam::Pose3 motion_smoother = gtsam::Pose3::identity();
                    //I guess add directly...?

                    //must wait till motions are added to the graph as we now wait till two motions
                    // graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
                    //     trace_key,
                    //     count_unique_id,
                    //     motion_smoother,
                    //     objectMotionSmootherNoiseModel
                    // );
                }
            }
            

            ObjUniqueID[j-1]=count_unique_id;
            unique_vertices[current_frame-1][j]=count_unique_id;
            count_unique_id++;
        }

        // // save for dynamic features
        for (int j = 0; j < vnFeaLabDyn[current_frame].size(); j++) {
            // check feature validation
            if (vnFeaLabDyn[current_frame][j] == -1) {
                continue;
            }

            // get the TrackID of current feature
            int TrackID = vnFeaLabDyn[current_frame][j];

            // get the position of current feature in the tracklet
            int PositionID = -1;
            for (int k = 0; k < DynTracks[TrackID].size(); ++k)
            {
                if (DynTracks[TrackID][k].first==current_frame && DynTracks[TrackID][k].second==j)
                {
                    PositionID = k;
                    break;
                }
            }
            if (PositionID==-1){
                LOG(WARNING) << "cannot find the position of current feature in the tracklet !!!";
                continue;
            }

            // get the object position id of current feature
            int ObjPositionID = -1;
            for (int k = 1; k < map->vnRMLabel[current_frame-1].size(); ++k) {
                if (map->vnRMLabel[current_frame-1][k]==map->nObjID[TrackID]){
                    ObjPositionID = ObjUniqueID[k-1];
                    break;
                }
            }
            if (ObjPositionID==-1 && PositionID!=2) {
                LOG(WARNING) << "cannot find the object association with this edge !!! WEIRD POINT !!! ";
                continue;
            }

            // check if the PositionID is 2. Yes means this dynamic point is first seen by this frame,
                    // then save both the vertex and edge, otherwise save edge only because vertex is saved before.
            CHECK_GE(PositionID, 2);
            // LOG(INFO) << "Dyna pos " << PositionID;
            if (PositionID == 2) {
                //TODO: sanity check that it is not in isam2 graph?

                 // check if this feature track has the same length as the window size
                // const int TrLength = DynTracks[TrackID].size();
                // if ( TrLength<FeaLengthThresDyn ) {
                //     continue;
                // }

                gtsam::Point3 X_w = utils::cvMatToGtsamPoint3(map->vp3DPointDyn[current_frame][j]);
                addDynamicLandmarkToGraph(X_w, count_unique_id, current_frame, j);

                cv::Mat Xc = Optimizer::Get3DinCamera(map->vpFeatDyn[current_frame][j],map->vfDepDyn[current_frame][j],K);
                gtsam::Point3 X_c_point = utils::cvMatToGtsamPoint3(Xc);
                addDynamicPoint3DFactor(X_c_point, curr_camera_pose_vertex, count_unique_id);

                // update unique id
                vnFeaMakDyn[current_frame][j] = count_unique_id;
                count_unique_id++;

            }
            //assume we dont get anything less than 2?
            else if(PositionID > 2) {
                //then only add this feature to the existing track it belongs to.
                // const int TrLength = DynTracks[TrackID].size();
                const int FeaMakTmp = vnFeaMakDyn[DynTracks[TrackID][PositionID-1].first][DynTracks[TrackID][PositionID-1].second];
                // // LOG(INFO) << FeaMakTmp;
                // // if ( TrLength-PositionID<FeaLengthThresDyn || FeaMakTmp==-1 ) {
                // //     continue;
                // // }
                // if ( TrLength < FeaLengthThresDyn || FeaMakTmp==-1 ) {
                //     continue;
                // }
                // LOG(INFO) << "Adding ladnamrk motion factor";

                gtsam::Point3 X_w = utils::cvMatToGtsamPoint3(map->vp3DPointDyn[current_frame][j]);
                addDynamicLandmarkToGraph(X_w, count_unique_id, current_frame, j);

                cv::Mat Xc = Optimizer::Get3DinCamera(map->vpFeatDyn[current_frame][j],map->vfDepDyn[current_frame][j],K);
                gtsam::Point3 X_c_point = utils::cvMatToGtsamPoint3(Xc);
                addDynamicPoint3DFactor(X_c_point, curr_camera_pose_vertex, count_unique_id);

                //in the case of dynamic and not the first feature in the tracklet
                CHECK(ObjPositionID != -1);
                gtsam::Point3 initial_measurement(0, 0, 0);
                addLandmarkMotionFactor(initial_measurement, count_unique_id, FeaMakTmp, ObjPositionID);

                // update unique id
                vnFeaMakDyn[current_frame][j] = count_unique_id;
                count_unique_id++;



            }
            else {
                LOG(ERROR) << "Should never get here";
            }

        }
        
    }

    pre_camera_pose_vertex = curr_camera_pose_vertex;
    optimize();
    // 

}

void VdoSlamBackend::calculateError() {
    //first disp comparision to GT
    //go over all the frames?
    double t_sum_refined = 0, r_sum_refined  = 0;
    double t_sum_original = 0, r_sum_original  = 0;
    const size_t N = getMapSize();
    for (int i = 0; i < N; ++i) {
        //get camera poses
        gtsam::Key pose_key = unique_vertices[i][0];
        gtsam::Pose3 refined_camera_pose = isam->calculateEstimate<gtsam::Pose3>(pose_key);

        gtsam::Pose3 original_camera_pose = utils::cvMatToGtsamPose3(map->vmCameraPose[i]);
        gtsam::Pose3 gt_camera_pose = utils::cvMatToGtsamPose3(map->vmCameraPose_GT[i]);

        std::pair<double, double> errors_original = utils::computeRotationAndTranslationErrors(
                                                        gt_camera_pose, original_camera_pose);

        r_sum_original += errors_original.first;
        t_sum_original += errors_original.second;

        std::pair<double, double> errors_refined = utils::computeRotationAndTranslationErrors(
                                                        gt_camera_pose, refined_camera_pose);

        r_sum_refined += errors_refined.first;
        t_sum_refined += errors_refined.second;
    }

    r_sum_original /= (N);
    t_sum_original /= (N);
    r_sum_refined /= (N);
    t_sum_refined /= (N);

    LOG(INFO) << "Average camera error GTSAM\n"
              << "Original R: " << r_sum_original << " T: " << t_sum_original << "\n"
              << "Refined R: " << r_sum_refined << " T: " << t_sum_refined;
}

void VdoSlamBackend::updateMapFromIncremental() {
    

    const gtsam::KeySet& observed_keys = result.markedKeys;
    int num_poses_update = 0;
    int num_points_updated = 0;

    LOG(INFO) << "Starting incremental map udpate with keys size " << observed_keys.size();

    for(const gtsam::Key& key : observed_keys) {
        IJSymbol vertex_pair = key_to_unique_vertices[key];
        updateMapFromSymbol(key, vertex_pair);

    }

    

    // LOG(INFO) << "Num poses updated " << num_poses_update << " and num points updated " << num_points_updated;

}


void VdoSlamBackend::updateMapFull() {
    LOG(INFO) << "Starting full map update with " << key_to_unique_vertices.size() << " keys";
    const auto& total_start_time = utils::Timer::tic();

    for(const auto& pair : key_to_unique_vertices) {
        const gtsam::Key& key = pair.first;
        const IJSymbol& vertex_pair = pair.second;

        //issue with the addToKeyVertexMapping function being called before
        //we add to isam. FOr now just do a check
        if(isam->valueExists(key)) {
            updateMapFromSymbol(key, vertex_pair);
        }

    }

    auto end_time = utils::Timer::toc<std::chrono::milliseconds>(total_start_time).count();
    LOG(INFO) << "Full map update took: " << end_time << " milliseconds";

}

gtsam::Values VdoSlamBackend::calculateCurrentEstimate() const {
    return isam->calculateEstimate();

}

const size_t VdoSlamBackend::getMapSize() const { return map->vpFeatSta.size(); }

void VdoSlamBackend::setupNoiseModels() {

    //robust noise model
    //this is on the odom noise model
    gtsam::noiseModel::Base::shared_ptr huberObjectMotion;
    gtsam::noiseModel::Base::shared_ptr huberPoint3D;
    CHECK_NOTNULL(params);
    LOG(INFO) << "Setting up noise models for backend";
    cameraPosePrior = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << gtsam::Vector6::Constant(params->var_camera_prior)).finished());



    point3DNoiseModel = gtsam::noiseModel::Diagonal::Sigmas(
                    (gtsam::Vector(3) << gtsam::Vector3::Constant(params->var_3d_static)).finished());

    dynamicPoint3DNoiseModel = gtsam::noiseModel::Diagonal::Sigmas(
                    (gtsam::Vector(3) << gtsam::Vector3::Constant(params->var_3d_dyn)).finished());

    odometryNoiseModel = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << gtsam::Vector6::Constant(params->var_camera)).finished());

    objectMotionNoiseModel = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(3) << gtsam::Vector3::Constant(params->var_obj)).finished());

    objectMotionSmootherNoiseModel = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << gtsam::Vector6::Constant(params->var_obj_smooth)).finished());



    if(params->use_robust_kernel) {
        LOG(INFO) << "Using robust kernal";
        //assuming that this doesnt mess with with original pointer as we're reassigning the member ptrs
        auto pose3dNoiseModelTemp = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(params->k_huber_3d_points), point3DNoiseModel);

        point3DNoiseModel =pose3dNoiseModelTemp;
        //TODO: not using dynamic points yet
        // huberObjectMotion = gtsam::noiseModel::Robust::Create(
        //     gtsam::noiseModel::mEstimator::Huber::Create(params->k_huber_obj_motion), odometryNoiseModel);
    }


    
}

void VdoSlamBackend::addToKeyVertexMapping(const gtsam::Key& key, FrameId curr_frame, FeatureId feature_id, unsigned char symbol) {
    auto it = key_to_unique_vertices.find(key);
    CHECK(it == key_to_unique_vertices.end()) << "gtsam key is not unique in the key to vertex map";

    IJSymbol ij_symbol;
    ij_symbol.i = curr_frame;
    ij_symbol.j = feature_id;
    ij_symbol.symbol = symbol;
    key_to_unique_vertices.insert({key, ij_symbol});
}

void VdoSlamBackend::addCameraPoseToGraph(const gtsam::Pose3& pose, gtsam::Key key, FrameId curr_frame) {
    new_camera_poses.insert(key, pose);
    all_values.insert(key, pose);
    addToKeyVertexMapping(key, curr_frame, 0, kSymbolCameraPose3Key);
}

void VdoSlamBackend::addMotionToGraph(const gtsam::Pose3& motion, gtsam::Key key, FrameId curr_frame, FeatureId object_id) {
    new_object_motions.insert(key, motion);
    all_values.insert(key, motion);
    addToKeyVertexMapping(key, curr_frame, object_id, kSymbolMotion3Key);
}



void VdoSlamBackend::addLandmarkToGraph(const gtsam::Point3& landmark, gtsam::Key key, FrameId curr_frame, FeatureId feature_id) {
    auto it = observed_landmarks.find(key);
    all_values.insert(key, landmark);
    //if not in map then first time observed here so we set observation to 0.
    //also assume that if its never been seen here then it is not in new_values
    if (it == observed_landmarks.end()) {
        //sanity check. if not in observed should definitely not be in isam
        //removing only for testing with g2o files as we dont add to the isam2
        // CHECK(!isam->valueExists(key));

        observed_landmarks[key] = {};
        CHECK(!new_lmks.exists(key));
        new_lmks.insert(key, landmark);
        // all_values.insert(key, landmark);
        //TODO: THIS IS BAD: we're not actually adding the lmk to isam2 yet (it may not have enough measurements)
        //so when we try and opt this key is actually not in the map
        //for now we'll add a check in update map full but this needs to be changed
        addToKeyVertexMapping(key, curr_frame, feature_id, kSymbolPoint3Key);
    }
    else {
        CHECK(new_lmks.exists(key));

    }
}

void VdoSlamBackend::addDynamicLandmarkToGraph(const gtsam::Point3& landmark, gtsam::Key key, FrameId curr_frame, FeatureId feature_id) {
    //straight up add everything as it should be unique...?
    new_dynamic_lmks.insert(key, landmark);
    all_values.insert(key, landmark);
    addToKeyVertexMapping(key, curr_frame, feature_id, kSymbolDynamicPoint3Key);
}


void VdoSlamBackend::addPoint3DFactor(const gtsam::Point3& measurement, gtsam::Key pose_key, gtsam::Key landmark_key) {
    auto it = observed_landmarks.find(landmark_key);
    //must be added prior to the first factor? I guess?
    // CHECK(it != observed_landmarks.end()) << "Landmark measurment must be added before adding a point 3D factor";    //must be added prior to the first factor? I guess?
    observed_landmarks[landmark_key].push_back(
        boost::make_shared<Point3DFactor>(
            pose_key,
            landmark_key,
            measurement,
            point3DNoiseModel
        )
    );

    graph.emplace_shared<Point3DFactor>(
        pose_key,
        landmark_key,
        measurement,
        point3DNoiseModel
    );

}

void VdoSlamBackend::addDynamicPoint3DFactor(const gtsam::Point3& measurement, gtsam::Key pose_key, gtsam::Key landmark_key) {
    CHECK(new_dynamic_lmks.exists(landmark_key) || isam->valueExists(landmark_key)) << "Dyanmic point landmark must be added";
    observed_dyn_landmarks[landmark_key].push_back(
        boost::make_shared<Point3DFactor>(
            pose_key,
            landmark_key,
            measurement,
            dynamicPoint3DNoiseModel
        )
    );

    graph.emplace_shared<Point3DFactor>(
        pose_key,
        landmark_key,
        measurement,
        dynamicPoint3DNoiseModel
    );
}


void VdoSlamBackend::addLandmarkMotionFactor(const gtsam::Point3& measurement, gtsam::Key current_point_key, 
            gtsam::Key previous_point_key, gtsam::Key motion_key) {
                //
    //check observed landmarks to ensure that the point is actually in the set of points
    auto it = observed_dyn_landmarks.find(current_point_key);
    CHECK(it != observed_dyn_landmarks.end()) << "Current landmark key must be added before adding landmark motion factor";
    //I guess also can check for the previous one?
    // LOG(INFO) << "Added motion between "
    //Super unclear in implementation if current or previous key is first
    observed_motions[motion_key].push_back(
        boost::make_shared<LandmarkMotionTernaryFactor>(
            current_point_key,
            previous_point_key,
            motion_key,
            measurement,
            objectMotionNoiseModel
        )
    );

    graph.emplace_shared<LandmarkMotionTernaryFactor>(
        current_point_key,
            previous_point_key,
            motion_key,
            measurement,
            objectMotionNoiseModel
    );


}

gtsam::Values VdoSlamBackend::collectValuesToAdd() {
    //go through all observed values and see which ones have been observed two.
    //if yes, remove them from the new_values and add then to the values to add
    gtsam::Values values_to_add;

    //now also add camera poses
    //I mean... this will always be one.. presumably
    LOG(INFO) << "Adding camera poses";
    const gtsam::KeyVector& cam_pose_keys = new_camera_poses.keys();
    for(const gtsam::Key& key : cam_pose_keys) {
        gtsam::Pose3 pose = new_camera_poses.at<gtsam::Pose3>(key);
        values_to_add.insert(key, pose);
        debug_info.num_new_poses++;
    }

    new_camera_poses.clear();


    LOG(INFO) << "Adding observed_landmarks";

    const int kMinObservations = 2;
    //yeah thhis search grows expontially until we start to marginalize out variables
    //becuase we look through all the observed variables. 
    //Dont add them unless they are already in the map and clear the factors
    debug_info.graph_size_before_collection = graph.size();
    const gtsam::KeyVector& lmk_keys = new_lmks.keys();
    for(const gtsam::Key& landmark_key : lmk_keys) {
        // gtsam::Key landmark_key = 
    // for(auto& x : observed_landmarks) {
        // const gtsam::Key& landmark_key = x.first;
        // LOG(INFO) << "looking at " << landmark_key;
        Point3DFactors& factors = observed_landmarks[landmark_key];
        //now get all Point3DFactors
        // Point3DFactors& factors = x.second;
        //if we have at least n factors or the key is already in the graph
        // if(factors.size() >= kMinObservations) {
            //removing only for testing with g2o files as we dont add to the isam2
            // CHECK(!isam->valueExists(landmark_key));

            // LOG(INFO) << "Adding key: " << landmark_key << " to isam 2";
            //for now eveything will be a Point3
        if (factors.size() > 0) {
            gtsam::Point3 landmark = new_lmks.at<gtsam::Point3>(landmark_key);

            new_lmks.erase(landmark_key);
            values_to_add.insert(landmark_key, landmark);
            debug_info.num_new_static_points++;

            // LOG(INFO) << "Adding " << factors.size() << " Point3D Factors";

            graph.push_back(factors.begin(), factors.end());
        //i guess clear the factors?
            // factors.clear();
            observed_landmarks[landmark_key].clear();
        }
        else {
            LOG(WARNING) << "Should never get here unless we dont remove a new lmk?";
        }

        // }
        //imlicit logic here -> if the landmark is in isam 
        //then we assume that at some point prior it must have had at least two landmarks and
        //we have added those factors and cleared the array. 

        // if(all_values.exists(landmark_key) && factors.size()> 0) {

        // // if(isam->valueExists(landmark_key)) {
        //     // LOG(INFO) << "key: " << landmark_key << " exists.";
        //     // LOG(INFO) << "Adding " << factors.size() << " Point3D Factors";
        //     //sanity check that this landmark does not exist in new lmsks?
        //     graph.push_back(factors.begin(), factors.end());
        //     factors.clear();

        // }
    }
    // observed_landmarks.clear();


    //////////// DYNAMIC STUFF ////////////////////
    //just add all the dynamic lmks and assume we got our data associtaion right in the process step?
     //I think we want to add all the motions every time?
    // LOG(INFO) << "Adding new_dynamic_lmks";

    // //Add dynamic landmarks as soon as we have them
    // const gtsam::KeyVector& dynamic_lmk_keys = new_dynamic_lmks.keys();
    // for(const gtsam::Key& key : dynamic_lmk_keys) {
    //     Point3DFactors& factors = observed_dyn_landmarks[key];

    //     if (factors.size() > 0) {
    //         gtsam::Point3 point = new_dynamic_lmks.at<gtsam::Point3>(key);
    //         values_to_add.insert(key, point);

    //         new_dynamic_lmks.erase(key);
    //         graph.push_back(factors.begin(), factors.end());
    //         observed_dyn_landmarks[key].clear();
    //     }
        
    // }
  
    // LOG(INFO) << "Adding observed_motions";
    // for(auto& x : observed_motions) {
    //     LandmarkMotionTernaryFactors& factors = x.second;
    //     const gtsam::Key& motion_key = x.first;


    //     //sanity checks!
    //     for(auto& factor : factors) {
    //         const gtsam::Key& previous_point_key = factor->getPreviousPointKey();
    //         const gtsam::Key& current_point_key = factor->getCurrentPointKey();
    //         // CHECK(isam->valueExists(previous_point_key) || values_to_add.exists(previous_point_key));
    //         // CHECK(isam->valueExists(current_point_key) || values_to_add.exists(current_point_key));

    //         // factor->print();

    //     }

    //     if (factors.size() > 0) {
    //         gtsam::Pose3 motion = new_object_motions.at<gtsam::Pose3>(motion_key);
    //         new_object_motions.erase(motion_key);
    //         values_to_add.insert(motion_key, motion);

    //         graph.push_back(factors.begin(), factors.end());
    //         //i guess clear the factors?
    //         factors.clear();
    //     } 

    //     //motion should already be in isam2 or about to be if its in values
    //     CHECK(all_values.exists(motion_key) || values_to_add.exists(motion_key)) << " With " << factors.size() << " factors";


    // }
    // LOG(INFO) << "Added motions: " << observed_motions.size();


    debug_info.graph_size_before_collection = graph.size();


    debug_info.total_new_values = values_to_add.size();
    //also update debug frame info here...?
    debug_info.frame = current_frame;
    return values_to_add;
}

void VdoSlamBackend::optimizeLM() {
     LOG(INFO) << "Num gtsam vertex " << all_values.size();
    LOG(INFO) << "Num gtsam edges " << graph.size();
    try {
        // boost::shared_ptr<gtsam::GaussianFactorGraph> linearization = graph.linearize(all_values);
        // gtsam::Matrix hessian = linearization->augmentedHessian();
        // gtsam::Matrix jacobian = linearization->augmentedJacobian();

        // std::ofstream file("hessian.txt");
        // if (file.is_open()) {
        //     file << hessian;
        //     file.close();
        // }

        LOG(INFO) << "Begin LM OPT";
        Values result = gtsam::LevenbergMarquardtOptimizer(graph, all_values).optimize();
        LOG(INFO) << "Done LM OPT";
    }
    catch (const gtsam::ValuesKeyDoesNotExist& e) {
            LOG(WARNING) << e.what();
            auto it = key_to_unique_vertices.find(e.key());
            if (it == key_to_unique_vertices.end()) {
                LOG(INFO) << "key was also not in key vertex mapping";
            }
            else {
                LOG(INFO) << key_to_unique_vertices[e.key()];
            }
            throw e;

        }
        catch(const gtsam::IndeterminantLinearSystemException& e) {
            LOG(WARNING) << e.what();
            auto it = key_to_unique_vertices.find(e.nearbyVariable());
            if (it == key_to_unique_vertices.end()) {
                LOG(INFO) << "key was also not in key vertex mapping";
            }
            else {
                LOG(INFO) << key_to_unique_vertices[e.nearbyVariable()];
            }
            throw e;
        }
}

void VdoSlamBackend::optimize() {
    ///failure is always on previous camera pose
    if(current_frame >= 0) {
        //will throw KeyAlreadyExists<J> so a good test to see if the front end -> backend processing is working
        try {
            // gtsam::Values values = collectValuesToAdd();
            // all_values.insert(values);
            // result = isam->update(graph, values);
            // isam->update();
            // isam->update();
            // isam->update();
            // graph.resize(0);

            debug_info.print();
            debug_info.reset();
            // updateMapFromIncremental();
        }
        catch (const gtsam::ValuesKeyDoesNotExist& e) {
            LOG(WARNING) << e.what();
            auto it = key_to_unique_vertices.find(e.key());
            if (it == key_to_unique_vertices.end()) {
                LOG(INFO) << "key was also not in key vertex mapping";
            }
            else {
                LOG(INFO) << key_to_unique_vertices[e.key()];
            }
            throw e;

        }
        catch(const gtsam::IndeterminantLinearSystemException& e) {
            LOG(WARNING) << e.what();
            auto it = key_to_unique_vertices.find(e.nearbyVariable());
            if (it == key_to_unique_vertices.end()) {
                LOG(INFO) << "key was also not in key vertex mapping";
            }
            else {
                LOG(INFO) << key_to_unique_vertices[e.nearbyVariable()];
            }
            throw e;
        }
        
    }
    

}

void VdoSlamBackend::writeG2o(const std::string& file_name) {
    VDO_SLAM::utils::writeG2o(graph, all_values, file_name);
}

void VdoSlamBackend::updateMapFromSymbol(const gtsam::Key& key, const IJSymbol& vertex_symbol) {
    const int frame_id = vertex_symbol.i;
    const int feature_id = vertex_symbol.j;


    //we have updated a pose
    if(vertex_symbol.symbol == kSymbolCameraPose3Key) {
        gtsam::Pose3 camea_pose_refined = isam->calculateEstimate<gtsam::Pose3>(key);
        map->vmCameraPose[frame_id] = utils::gtsamPose3ToCvMat(camea_pose_refined);
        // num_poses_update++;
        
        if(frame_id > 0) {

            // CHECK_MAT_TYPES(Converter::toInvMatrix(
            //     map->vmCameraPose[frame_id-1]), map->vmCameraPose[frame_id]);

            // map->vmRigidMotion[frame_id-1][0] = Converter::toInvMatrix(
            //     map->vmCameraPose[frame_id-1])*map->vmCameraPose[frame_id];
        }
    }

    if(vertex_symbol.symbol == kSymbolPoint3Key) {
        // num_points_updated++;
        gtsam::Point3 point = isam->calculateEstimate<gtsam::Point3>(key);
        if(vnFeaMakSta[frame_id][feature_id] != -1) {
            map->vp3DPointSta[frame_id][feature_id] = utils::gtsamPoint3ToCvMat(point);
            // num_points_updated++;
        }
        else {
            //log warning?
        }
    }
}




}