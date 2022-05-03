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
#include "factors/Projection3DFactor.h"
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
    const GtsamAccesType sigma2_3d_sta = 16; // 50 80 16
    const GtsamAccesType sigma2_obj_smo = 0.1; // 0.1
    const GtsamAccesType sigma2_obj = 20; // 0.5 1 10 20
    const GtsamAccesType sigma2_3d_dyn = 16; // 50 100 16
    const GtsamAccesType sigma2_alti = 1;

    const GtsamAccesType camera_pose_prior_sigma = 0.0000001;
    // gtsam::noiseModel::Diagonal::shared_ptr camera_pose_prior_n = 
    auto camera_pose_prior_n = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << gtsam::Vector6::Constant(camera_pose_prior_sigma)).finished()
    );

    //make optimizer using gtsam
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;

    int FeaLengthThresSta = 3;
    int FeaLengthThresDyn = 3;

     //we just have the one camera pose measurement to add
    // (1) save <VERTEX_POSE_R3_SO3>
    gtsam::Pose3 camera_pose = utils::cvMatToGtsamPose3(map->vmCameraPose[current_frame]);
    values.insert(count_unique_id, camera_pose);
    addToKeyVertexMapping(count_unique_id, current_frame, 0, kSymbolCameraPose3Key);

    //is first
    if(count_unique_id == 1) {
        graph.addPrior(count_unique_id, camera_pose, camera_pose_prior_n);
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
        for (int k = 0; k < StaTracks[TrackID].size(); ++k)
        {
            //if we're not looping through all the frames (eg i becomes start_frame is this going to be a problem?)
            if (StaTracks[TrackID][k].first==current_frame && StaTracks[TrackID][k].second==j)
            {
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
        if (PositionID==0) {
            //then also check that it is not in the graph
            //check in isam or check in local NLFG?
            CHECK(!isam->valueExists(count_unique_id));
            // check if this feature track has the same length as the window size
            const int TrLength = StaTracks[TrackID].size();
            if ( TrLength<FeaLengthThresSta ) {
                continue;
            }

            gtsam::Point3 X_w = utils::cvMatToGtsamPoint3(map->vp3DPointSta[current_frame][j]);
            values.insert(count_unique_id, X_w);
            addToKeyVertexMapping(count_unique_id, current_frame, j, kSymbolPoint3Key);

             auto camera_projection_noise = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(3) << gtsam::Vector2::Constant(1.0/sigma2_3d_sta)).finished()
            );

            //TODO: make camera class or equivalent
            cv::Mat Xc = Optimizer::Get3DinCamera(map->vpFeatSta[current_frame][j],map->vfDepSta[current_frame][j],K);
            gtsam::Point3 X_c_point = utils::cvMatToGtsamPoint3(Xc);
            graph.emplace_shared<Projection3DFactor>(curr_camera_pose_vertex, count_unique_id, X_c_point, camera_projection_noise);

            vnFeaMakSta[current_frame][j] = count_unique_id;
            count_unique_id++;

        }
        else {
            //landmark should be in graph
            // CHECK(isam->valueExists(curr_camera_pose_vertex) || graph.at);

           // // check if this feature track has the same length as the window size
                // // or its previous FeaMakTmp is not -1, then save it, otherwise skip.
            const int TrLength = StaTracks[TrackID].size();
            const int FeaMakTmp = vnFeaMakSta[StaTracks[TrackID][PositionID-1].first][StaTracks[TrackID][PositionID-1].second];
            if (TrLength<FeaLengthThresSta || FeaMakTmp==-1) {
                continue;
            }

           auto camera_projection_noise = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(3) << gtsam::Vector2::Constant(1.0/sigma2_3d_sta)).finished()
            );

            //base logic checks
            // gtsam::Values::iterator it =  values.find(FeaMakTmp);
            // if(it == values.end()) { LOG(ERROR) << "Static Feature Key is not a valid variable";};

            // it == values.find(CurFrameID);
            // if(it == values.end()) { LOG(ERROR) << "Camera frame ID Key is not a valid variable";};

            //now add factor
             //TODO: make camera class or equivalent
            cv::Mat Xc = Optimizer::Get3DinCamera(map->vpFeatSta[current_frame][j],map->vfDepSta[current_frame][j],K);
            gtsam::Point3 X_c_point = utils::cvMatToGtsamPoint3(Xc);
            graph.emplace_shared<Projection3DFactor>(curr_camera_pose_vertex, FeaMakTmp, X_c_point, camera_projection_noise);

            vnFeaMakSta[current_frame][j] = FeaMakTmp;
        }

    }

    pre_camera_pose_vertex = curr_camera_pose_vertex;

    result = isam->update(graph, values);
    isam->update();
    // updateMap();


}

void VdoSlamBackend::updateMap() {
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

    const gtsam::KeyVector& observed_keys = result.observedKeys;

    LOG(INFO) << "Observed keys size " << observed_keys.size();

    for(const gtsam::Key& key : observed_keys) {
        IJSymbol vertex_pair = key_to_unique_vertices[key];
        const int frame_id = vertex_pair.i;
        const int feature_id = vertex_pair.j;
        //we have updated a pose
        if(vertex_pair.symbol == kSymbolCameraPose3Key) {
            gtsam::Pose3 camea_pose_refined = isam->calculateEstimate<gtsam::Pose3>(key);
            map->vmCameraPose[frame_id] = utils::gtsamPose3ToCvMat(camea_pose_refined);
            
            if(frame_id > 0) {

                CHECK_MAT_TYPES(Converter::toInvMatrix(
                    map->vmCameraPose[frame_id-1]), map->vmCameraPose[frame_id]);

                map->vmRigidMotion[frame_id-1][0] = Converter::toInvMatrix(
                    map->vmCameraPose[frame_id-1])*map->vmCameraPose[frame_id];
            }
        }

        if(vertex_pair.symbol == kSymbolPoint3Key) {
            gtsam::Point3 point = isam->calculateEstimate<gtsam::Point3>(key);
            if(vnFeaMakSta[frame_id][feature_id] != -1) {
                map->vp3DPointSta[frame_id][feature_id] = utils::gtsamPoint3ToCvMat(point);
            }
            else {
                //log warning?
            }
        }


    }




}

gtsam::Values VdoSlamBackend::calculateCurrentEstimate() const {
    return isam->calculateEstimate();

}

const size_t VdoSlamBackend::getMapSize() const { return map->vpFeatSta.size(); }

void VdoSlamBackend::addToKeyVertexMapping(const gtsam::Key& key, int i, int j, unsigned char symbol) {
    auto it = key_to_unique_vertices.find(key);
    CHECK(it == key_to_unique_vertices.end()) << "gtsam key is not unique in the key to vertex map";

    IJSymbol ij_symbol;
    ij_symbol.i = i;
    ij_symbol.j = j;
    ij_symbol.symbol = symbol;
    key_to_unique_vertices.insert({key, ij_symbol});
}

}