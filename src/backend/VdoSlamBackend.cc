#include "backend/VdoSlamBackend.h"
#include "utils/UtilsGTSAM.h"
#include "utils/UtilsOpenCv.h"

#include <glog/logging.h>
#include <memory>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>

#include "utils/macros.h"
#include "utils/timing.h"
#include "factors/Point3DFactor.h"
#include "Map.h"
#include "Optimizer.h"
#include "Converter.h"

namespace VDO_SLAM {

VdoSlamBackend::VdoSlamBackend(Map* map_, const cv::Mat& Calib_K_) 
    :   map(CHECK_NOTNULL(map_)),
        K(Calib_K_),
        count_unique_id(1),
        pre_camera_pose_vertex(-1),
        curr_camera_pose_vertex(0),
        current_frame(-1),
        key_to_unique_vertices() {

        gtsam::ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.01;
        parameters.relinearizeSkip = 1;

        isam = VDO_SLAM::make_unique<gtsam::ISAM2>(parameters);
        K_calib =  utils::cvMat2Cal3_S2(K);

        point3DNoiseModel = gtsam::noiseModel::Diagonal::Sigmas(
                    (gtsam::Vector(3) << gtsam::Vector3::Constant(1.0/sigma2_3d_sta)).finished()
                );


        //TODO: for 3d Points only. See optimzier.
        robust_noise_model =  gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(0.0001), point3DNoiseModel);  // robust
            
        CHECK_NOTNULL(K_calib);
    }

void VdoSlamBackend::process() {
    const int N = map->vpFeatSta.size(); // Number of Frames
    LOG(INFO) << "Running incremental update on frame " << N;

    const std::vector<std::vector<std::pair<int, int>>>& StaTracks = map->TrackletSta;
    const std::vector<std::vector<std::pair<int, int>>>& DynTracks = map->TrackletDyn;


    current_frame = N - 1;


    std::vector<int>  vnFLS_tmp(map->vpFeatSta[current_frame].size(),-1);
    vnFeaLabSta.push_back(vnFLS_tmp);
    vnFeaMakSta.push_back(vnFLS_tmp);

    std::vector<int> vnFLD_tmp(map->vpFeatDyn[current_frame].size(),-1);
    vnFeaLabDyn.push_back(vnFLD_tmp);
    vnFeaMakDyn.push_back(vnFLD_tmp);

    // std::vector<bool> ObjCheck_tmp(pMap->vnRMLabel[i].size(),false);
    // obj_check.push_back(ObjCheck_tmp);

    CHECK_EQ(vnFeaLabSta.size(), N);
    CHECK_EQ(vnFeaMakSta.size(), N);
    CHECK_EQ(vnFeaLabDyn.size(), N);
    CHECK_EQ(vnFeaMakDyn.size(), N);

    
    //they are not N, they are N-1
    // CHECK_EQ(StaTracks.size(), N);
    // CHECK_EQ(DynTracks.size(), N);
    // CHECK_EQ(obj_check.size(), N);


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

    if(current_frame == 0) {
        CHECK_EQ(N, 1);
        std::vector<int> v_id_tmp(1,-1);
        unique_vertices.push_back(v_id_tmp);
    }
    else {
        std::vector<int> v_id_tmp(map->vnRMLabel[current_frame-1].size(),-1);
        unique_vertices.push_back(v_id_tmp);
    }

    CHECK_EQ(unique_vertices.size(), N);

    const GtsamAccesType sigma2_cam = 0.0001; // 0.005 0.001 0.0001
    const GtsamAccesType sigma2_obj_smo = 0.1; // 0.1
    const GtsamAccesType sigma2_obj = 20; // 0.5 1 10 20
    const GtsamAccesType sigma2_3d_dyn = 16; // 50 100 16
    const GtsamAccesType sigma2_alti = 1;

    const GtsamAccesType camera_pose_prior_sigma = 0.000001;
    auto camera_pose_prior_n = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << gtsam::Vector6::Constant(camera_pose_prior_sigma)).finished()
    );


    int FeaLengthThresSta = 3;
    int FeaLengthThresDyn = 3;

     //we just have the one camera pose measurement to add
    // (1) save <VERTEX_POSE_R3_SO3>
    gtsam::Pose3 camera_pose = utils::cvMatToGtsamPose3(map->vmCameraPose[current_frame]);
    addCameraPoseToGraph(camera_pose, count_unique_id, current_frame);
    // new_camera_poses.insert(count_unique_id, camera_pose);
    // addToKeyVertexMapping(count_unique_id, current_frame, 0, kSymbolCameraPose3Key);

    //is first (or current_frame == 0?)
    if(count_unique_id == 1) {
        graph.addPrior(count_unique_id, camera_pose, camera_pose_prior_n);
        LOG(INFO) << "Added camera prior";
    }

    //unique id for the first frame and the camera pose
    unique_vertices[current_frame][0] = count_unique_id;
    curr_camera_pose_vertex = count_unique_id;
    count_unique_id++;

    //not the first call
    if(current_frame > 0) {
        // (2) save <EDGE_R3_SO3> 
        //like I guess?
        //pretty sure we want to optimize for the rigid motion too...? This should become a variable?
        gtsam::Pose3 rigid_motion = utils::cvMatToGtsamPose3(map->vmRigidMotion[current_frame-1][0]);
        auto edge_information = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << gtsam::Vector6::Constant(sigma2_cam)).finished()
        );

        graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            pre_camera_pose_vertex,
            curr_camera_pose_vertex,
            rigid_motion,
            edge_information
        );
        LOG(INFO) << "Added factor between id: " <<pre_camera_pose_vertex << " " << curr_camera_pose_vertex;

        //missing robust kernal
    }

    //loop for static features
    LOG(INFO) << "Static features for current frame is " << vnFeaLabSta[current_frame].size();
    for (int j = 0; j < vnFeaLabSta[current_frame].size(); ++j) {
        // check feature validation
        if (vnFeaLabSta[current_frame][j]==-1) {
            continue;
        }
        // get the TrackID of current feature
        int TrackID = vnFeaLabSta[current_frame][j];

        // get the position of current feature in the tracklet
        int PositionID = -1;
        //trackets require at least one prior frame to exist to track the feature through
        //so the first frame that we can track the the features is 
        for (int k = 0; k < StaTracks[TrackID].size(); ++k) {
            // LOG(INFO) << StaTracks[TrackID][k].first;
            //if we're not looping through all the frames (eg i becomes start_frame is this going to be a problem?)
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
        if (PositionID==2) {
            //then also check that it is not in the graph
            //check in isam or check in local NLFG?
            CHECK(!isam->valueExists(count_unique_id));
            // check if this feature track has the same length as the window size
            const int TrLength = StaTracks[TrackID].size();
            if ( TrLength<FeaLengthThresSta ) {
                continue;
            }

            // //but there is also at least one track we dont have? becuase we only add the third track
            // int prev_1_frame = StaTracks[TrackID][PositionID-1].first;
            // int prev_1_lmk_id = StaTracks[TrackID][PositionID-1].second;
            // gtsam::Point3 X_w_prev1 = utils::cvMatToGtsamPoint3(
            //     map->vp3DPointSta[prev_1_frame][prev_1_lmk_id]);
            // //get the camera pose vertex at this time
            // addLandmarkToGraph(X_w_prev1, count_unique_id, prev_1_frame, prev_1_lmk_id);
            // vnFeaMakSta[prev_1_frame][prev_1_lmk_id] = count_unique_id;

            // cv::Mat Xc_prev_1 = Optimizer::Get3DinCamera(map->vpFeatSta[prev_1_frame][prev_1_lmk_id],map->vfDepSta[prev_1_frame][prev_1_lmk_id],K);
            // gtsam::Point3 X_c_prev_1_point = utils::cvMatToGtsamPoint3(Xc_prev_1);
            // int camera_pose_vertex_prev1 = unique_vertices[prev_1_frame][0];
            // addPoint3DFactor(X_c_prev_1_point, camera_pose_vertex_prev1, count_unique_id);
            // count_unique_id++;


            // int prev_2_frame = StaTracks[TrackID][PositionID-2].first;
            // int prev_2_lmk_id = StaTracks[TrackID][PositionID-2].second;
            // gtsam::Point3 X_w_prev2 = utils::cvMatToGtsamPoint3(
            //     map->vp3DPointSta[prev_2_frame][prev_2_lmk_id]);

            // addLandmarkToGraph(X_w_prev2, count_unique_id, prev_2_frame, prev_2_lmk_id);
            // vnFeaMakSta[prev_1_frame][prev_1_lmk_id] = count_unique_id;

            // cv::Mat Xc_prev_2 = Optimizer::Get3DinCamera(map->vpFeatSta[prev_2_frame][prev_2_lmk_id],map->vfDepSta[prev_2_frame][prev_2_lmk_id],K);
            // gtsam::Point3 X_c_prev_2_point = utils::cvMatToGtsamPoint3(Xc_prev_2);
            // int camera_pose_vertex_prev2 = unique_vertices[prev_2_frame][0];
            // addPoint3DFactor(X_c_prev_2_point, camera_pose_vertex_prev2, count_unique_id);
            // count_unique_id++;


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
            if (TrLength<FeaLengthThresSta || FeaMakTmp==-1) {
                continue;
            }

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
    addToKeyVertexMapping(key, curr_frame, 0, kSymbolCameraPose3Key);
}


void VdoSlamBackend::addLandmarkToGraph(const gtsam::Point3& landmark, gtsam::Key key, FrameId curr_frame, FeatureId feature_id) {
    auto it = observed_landmarks.find(key);
    //if not in map then first time observed here so we set observation to 0.
    //also assume that if its never been seen here then it is not in new_values
    if (it == observed_landmarks.end()) {
        //sanity check. if not in observed should definitely not be in isam
        CHECK(!isam->valueExists(key));
        observed_landmarks[key] = {};
        CHECK(!new_lmks.exists(key));
        new_lmks.insert(key, landmark);
        //TODO: THIS IS BAD: we're not actually adding the lmk to isam2 yet (it may not have enough measurements)
        //so when we try and opt this key is actually not in the map
        //for now we'll add a check in update map full but this needs to be changed
        addToKeyVertexMapping(key, curr_frame, feature_id, kSymbolPoint3Key);
    }
    else {
        CHECK(new_lmks.exists(key));
    }
}

void VdoSlamBackend::addPoint3DFactor(const gtsam::Point3& measurement, gtsam::Key pose_key, gtsam::Key landmark_key) {
    // graph.emplace_shared<Point3DFactor>(pose_key, landmark_key, measurement, point3DNoiseModel);
    //look for landmark key
    auto it = observed_landmarks.find(landmark_key);
    //must be added prior to the first factor? I guess?
    CHECK(it != observed_landmarks.end()) << "Landmark measurment must be added before adding a point 3D factor";
    observed_landmarks[landmark_key].push_back(
        boost::make_shared<Point3DFactor>(
            pose_key,
            landmark_key,
            measurement,
            robust_noise_model
        )
    );


}

gtsam::Values VdoSlamBackend::collectValuesToAdd() {
    //go through all observed values and see which ones have been observed two.
    //if yes, remove them from the new_values and add then to the values to add
    gtsam::Values values_to_add;
    const int kMinObservations = 2;
    //yeah thhis search grows expontially until we start to marginalize out variables
    //becuase we look through all the observed variables. 
    //Dont add them unless they are already in the map and clear the factors
    debug_info.graph_size_before_collection = graph.size();
    for(auto& x : observed_landmarks) {
        const gtsam::Key& landmark_key = x.first;
        // LOG(INFO) << "looking at " << landmark_key;

        //now get all Point3DFactors
        Point3DFactors& factors = x.second;
        //if we have at least n factors or the key is already in the graph
        if(factors.size() >= kMinObservations) {
            CHECK(!isam->valueExists(landmark_key));
            // LOG(INFO) << "Adding key: " << landmark_key << " to isam 2";
            //for now eveything will be a Point3
            gtsam::Point3 landmark = new_lmks.at<gtsam::Point3>(landmark_key);
            new_lmks.erase(landmark_key);
            values_to_add.insert(landmark_key, landmark);
            debug_info.num_new_static_points++;

            // LOG(INFO) << "Adding " << factors.size() << " Point3D Factors";

            graph.push_back(factors.begin(), factors.end());
            //i guess clear the factors?
            factors.clear();

        }
        //imlicit logic here -> if the landmark is in isam 
        //then we assume that at some point prior it must have had at least two landmarks and
        //we have added those factors and cleared the array. 
        if(isam->valueExists(landmark_key)) {
            // LOG(INFO) << "key: " << landmark_key << " exists.";
            // LOG(INFO) << "Adding " << factors.size() << " Point3D Factors";
            //sanity check that this landmark does not exist in new lmsks?
            graph.push_back(factors.begin(), factors.end());
            factors.clear();

        }
    }

    debug_info.graph_size_before_collection = graph.size();

    //now also add camera poses
    //I mean... this will always be one.. presumably
    const gtsam::KeyVector& cam_pose_keys = new_camera_poses.keys();
    for(const gtsam::Key& key : cam_pose_keys) {
        gtsam::Pose3 pose = new_camera_poses.at<gtsam::Pose3>(key);
        values_to_add.insert(key, pose);
        debug_info.num_new_poses++;
    }

    new_camera_poses.clear();

    debug_info.total_new_values = values_to_add.size();
    //also update debug frame info here...?
    debug_info.frame = current_frame;
    return values_to_add;
}

void VdoSlamBackend::optimize() {
    ///failure is always on previous camera pose
    if(current_frame > 1) {
        gtsam::Values values = collectValuesToAdd();
        try {
            result = isam->update(graph, values);
            isam->update();
            isam->update();
            isam->update();
            graph.resize(0);

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

void VdoSlamBackend::updateMapFromSymbol(const gtsam::Key& key, const IJSymbol& vertex_symbol) {
    const int frame_id = vertex_symbol.i;
    const int feature_id = vertex_symbol.j;


    //we have updated a pose
    if(vertex_symbol.symbol == kSymbolCameraPose3Key) {
        gtsam::Pose3 camea_pose_refined = isam->calculateEstimate<gtsam::Pose3>(key);
        map->vmCameraPose[frame_id] = utils::gtsamPose3ToCvMat(camea_pose_refined);
        // num_poses_update++;
        
        if(frame_id > 0) {

            CHECK_MAT_TYPES(Converter::toInvMatrix(
                map->vmCameraPose[frame_id-1]), map->vmCameraPose[frame_id]);

            map->vmRigidMotion[frame_id-1][0] = Converter::toInvMatrix(
                map->vmCameraPose[frame_id-1])*map->vmCameraPose[frame_id];
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