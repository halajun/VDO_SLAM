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

        // do_manager = VDO_SLAM::make_unique<DynamicObjectManager>(map_);
    }

void VdoSlamBackend::process() {
    const int N = map->vpFeatSta.size(); // Number of Frames
    LOG(INFO) << "Running incremental update on frame " << N;
    current_frame = N - 1;

    std::vector<std::vector<std::pair<int, int> > > StaTracks = map->TrackletSta;
    std::vector<std::vector<std::pair<int, int> > > DynTracks = map->TrackletDyn;

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


    // std::ofstream file("static_process.txt", std::ios_base::app);
    // if (file.is_open()) {
    //     //print frame number
    //     file << "Frame: " << N << std::endl;
    //     file << "Static tracks: "  << StaTracks.size() << std::endl;
    //     for(int i = 0; i < StaTracks.size(); i++) {
    //         file << "Tracklet " << i << std::endl;
    //         for(int j = 0; j < StaTracks[i].size(); j++) {
    //             file << "Frame " << StaTracks[i][j].first << " Point " << StaTracks[i][j].second << std::endl;
    //         }
    //     }
    //     file.close();
    // }
    static_tracklets.update(StaTracks);

//     std::ofstream file_s("static_track.txt", std::ios_base::app);
//     if (file_s.is_open()) {
//         //print frame number
//         file_s << "Frame: " << N << std::endl;
//         file_s << "Static tracks: "  << static_tracklets.size() << std::endl;
//         for(int i = 0; i < static_tracklets.size(); i++) {
//             file_s << "Tracklet " << i << std::endl;
//             for(int j = 0; j < static_tracklets[i].size(); j++) {
//                 file_s << "Frame " << static_tracklets[i][j]->frame_id << " Point " << static_tracklets[i][j]->point_id << std::endl;
//             }
//         }
//         file_s.close();
//     }

// // vpFeatSta

//     std::ofstream file_d("dynamic_process.txt", std::ios_base::app);
//     if (file_d.is_open()) {
//         //print frame number
//         file_d << "Frame: " << N << std::endl;
//         file_d << "Dynamic tracks: "  << DynTracks.size() << std::endl;
//         for(int i = 0; i < DynTracks.size(); i++) {
//             for(int j = 0; j < DynTracks[i].size(); j++) {
//                 file_d << "Frame " << DynTracks[i][j].first << " Point " << DynTracks[i][j].second << std::endl;
//             }
//         }
//         file_d.close();
//     }

    // std::ofstream file_ti("static_track_info.txt", std::ios_base::app);
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

    if(current_frame > 0) {
        gtsam::Pose3 rigid_motion = utils::cvMatToGtsamPose3(map->vmRigidMotion[current_frame-1][0]);
        graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            pre_camera_pose_vertex,
            curr_camera_pose_vertex,
            rigid_motion,
            odometryNoiseModel
        );
        LOG(INFO) << "Added motion constraint factor between id: " <<pre_camera_pose_vertex << " " << curr_camera_pose_vertex;
    }


    //start by adding the static points from the new frame
    for(size_t point_id = 0; point_id < map->vpFeatSta[current_frame].size(); point_id++) {
        if(static_tracklets.exists(current_frame, point_id)) {
            StaticTrackletManager::TypedTracklet tracklet = static_tracklets.getTracklet(current_frame, point_id);
            // LOG(INFO) << tracklet.TrackletId();

            if(tracklet.isWellTracked()) {
                StaticTrackletManager::Observations obs_to_add = tracklet.getNotAdded();
                LOG(INFO) << tracklet.TrackletId() << " is well tracked " << obs_to_add.size() << " and is new: "<< tracklet.isNew();

                for (StaticTrackletManager::Observation obs : obs_to_add) {
                    FrameId frame_id = obs->frame_id;
                    FeatureId feature_id = obs->point_id;
                    int track_id = obs->tracklet_id;
                    int position_id = obs->tracklet_position;

                    if (position_id == 0) {
                        gtsam::Point3 X_w = utils::cvMatToGtsamPoint3(map->vp3DPointSta[frame_id][feature_id]);
                        obs->key = static_cast<int>(count_unique_id);
                        obs->was_added = true;
                        LOG(INFO) << obs->key;
                        addLandmarkToGraph(X_w, count_unique_id, frame_id, feature_id);

                        cv::Mat Xc = Optimizer::Get3DinCamera(map->vpFeatSta[frame_id][feature_id],map->vfDepSta[frame_id][feature_id],K);
                        gtsam::Point3 X_c_point = utils::cvMatToGtsamPoint3(Xc);
                        addPoint3DFactor(X_c_point, curr_camera_pose_vertex, count_unique_id);

                        // update unique id
                        //better to use track id as Symbol but then will need to make all variables use differnt symbols
                        vnFeaMakSta[frame_id][feature_id] = count_unique_id;
                        count_unique_id++;
                    }
                    else {
                        LOG(INFO) << "Not new with " << obs_to_add.size();
                        for (StaticTrackletManager::Observation o : tracklet.Observations()) {
                            LOG(INFO) << o << " " << o->frame_id << " " << o->point_id << " " << o->tracklet_position << " " << o->was_added << " " << o->key;
                        }

                        StaticTrackletManager::Observation first_obs = tracklet[0];
                        int tracklet_key = first_obs->key;
                        //lets do a sanity check
                        //previous obs key has been set
                        CHECK(first_obs->was_added);

                        CHECK(tracklet_key != -1) << "Previous key was " << tracklet_key;
                        cv::Mat Xc = Optimizer::Get3DinCamera(map->vpFeatSta[frame_id][feature_id],map->vfDepSta[frame_id][feature_id],K);
                        gtsam::Point3 X_c_point = utils::cvMatToGtsamPoint3(Xc);
                        addPoint3DFactor(X_c_point, curr_camera_pose_vertex, tracklet_key);
                    }
                    
                }

                // if(tracklet.isNew()) {
                //     //create vertex and edge
                //     for (StaticTrackletManager::Observation obs : obs_to_add) {
                //         FrameId frame_id = obs->frame_id;
                //         FeatureId feature_id = obs->point_id;
                //         int track_id = obs->tracklet_id;
                //         LOG(INFO) << obs->tracklet_position;
                //         gtsam::Point3 X_w = utils::cvMatToGtsamPoint3(map->vp3DPointSta[frame_id][feature_id]);

                //         obs->key = static_cast<int>(count_unique_id);
                //         LOG(INFO) << obs->key;
                //         addLandmarkToGraph(X_w, count_unique_id, frame_id, feature_id);

                //         cv::Mat Xc = Optimizer::Get3DinCamera(map->vpFeatSta[frame_id][feature_id],map->vfDepSta[frame_id][feature_id],K);
                //         gtsam::Point3 X_c_point = utils::cvMatToGtsamPoint3(Xc);
                //         addPoint3DFactor(X_c_point, curr_camera_pose_vertex, count_unique_id);

                //         // update unique id
                //         vnFeaMakSta[frame_id][feature_id] = count_unique_id;
                //         count_unique_id++;
                //     }
                // }
                // else {
                //     LOG(INFO) << "Not new with " << obs_to_add.size();
                //     for (StaticTrackletManager::Observation obs : tracklet.Observations()) {
                //         LOG(INFO) << obs->frame_id << " " << obs->point_id << " " << obs->tracklet_position << " " << obs->was_added << " " << obs->key;
                //     }

                //     for (StaticTrackletManager::Observation obs : obs_to_add) {
                //         LOG(INFO) << obs->frame_id << " " << obs->point_id << " " << obs->tracklet_position;
                //         StaticTrackletManager::Observation previous_obs = tracklet.getPreviousObservation(obs);
                //         // int previous_key = previous_obs->key;
                //         LOG(INFO) << previous_obs->frame_id << " " << previous_obs->point_id;
                //         //lets do a sanity check
                //         //previous obs key has been set
                //         CHECK(previous_obs->was_added);
                //         // CHECK(previous_key != -1) << "Previous key was " << previous_key;
                //         //lets also check that the previous logic we used was correct
                //         CHECK(previous_obs->frame_id == obs->frame_id -1);
                //         int previous_key = vnFeaMakSta[previous_obs->frame_id][previous_obs->point_id];
                //         // int previous_key = static_tracklets[previous_obs->frame_id][previous_obs->point_id];
                //         CHECK(previous_key != -1) << "Previous key was " << previous_key;

                //     }
                // }

                tracklet.markAsAdded(obs_to_add);
            }
            
        }
        else {
            // LOG(WARNING) << "tracks dont exist for current frame " << current_frame << " point id " << point_id; 
        }
    }

    // file_ti.close();

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