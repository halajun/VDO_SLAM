#pragma once 

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/StereoCamera.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>

#include <glog/logging.h>
#include <opencv2/opencv.hpp>

#include <map>
#include <memory>

#include "factors/LandmarkMotionTernaryFactor.h"
#include "utils/macros.h"
#include "utils/types.h"
#include "utils/camera.h"
#include "backend/Tracklet.h"
#include "backend/Tracklet-Definitions.h"
#include "backend/VdoSlamBackend-types.h"
#include "backend/VdoSlamBackendParams.h"
#include "backend/DynamicObjectManager.h"
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>


#include "Map.h"

// using gtsam::symbol_shorthand::H;
// using gtsam::symbol_shorthand::L;  // Dynamic point
// using gtsam::symbol_shorthand::M; //static ppint
// using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)


namespace VDO_SLAM {

class VdoSlamBackend {

    public:
        VDO_SLAM_POINTER_TYPEDEFS(VdoSlamBackend);
        VdoSlamBackend(Map* map_, const cv::Mat& Calib_K_, BackendParams::Ptr params_);
        ~VdoSlamBackend() = default;

        using KeyTimestampMap = gtsam::FixedLagSmoother::KeyTimestampMap;


        //probably should be called process frontend of something like that
        void process(bool run_as_incremental = true);

        gtsam::Values calculateCurrentEstimate() const;

        void calculateError();
        // //this updates the map from the last set of marked variables in the isam2 results object
        // void updateMapFromIncremental();
        // //updates the map using all values ever seen (as stored in key_to_unique_vertices)
        // void updateMapFull();

        void writeG2o(const std::string& file_name);

        cv::Mat getBestPoseEstimate();

        void optimizeLM();
        void makePlots();

    private:

        //some helper functions
        const size_t getMapSize() const;


        //initliases covariances/robust kernals used for isam2 opt
        void setupNoiseModels();

        //could totally make this templated and add the type itself to the values variable
        //but there is some logic about incrementing the curr_camera_pose_vertex and other vertex
        //counters that may make this difficult
        void addToKeyVertexMapping(const gtsam::Key& key, FrameId curr_frame, FeatureId feature_id, unsigned char symbol);

        void addCameraPoseToGraph(const gtsam::Pose3& pose, gtsam::Key key, FrameId curr_frame);
        void addMotionToGraph(const gtsam::Pose3& motion, gtsam::Key key, FrameId curr_frame, FeatureId object_id);


        void addLandmarkToGraph(const gtsam::Point3& landmark, gtsam::Key key, FrameId curr_frame, FeatureId feature_id);

        //need to separate dyanmic and static becuase dynamic points are given new keys so we just
        //at least one motion to say that a point has been observed twice
        void addDynamicLandmarkToGraph(const gtsam::Point3& landmark, gtsam::Key key, FrameId curr_frame, FeatureId feature_id);

        //for experimentation
        void addPoint2DFactor(const gtsam::Point2& measurement, gtsam::Key pose_key, gtsam::Key landmark_key);
        void addPoint3DFactor(const gtsam::Point3& measurement, gtsam::Key pose_key, gtsam::Key landmark_key);
        void addDynamicPoint3DFactor(const gtsam::Point3& measurement, gtsam::Key pose_key, gtsam::Key landmark_key);

        void addLandmarkMotionFactor(const gtsam::Point3& measurement, gtsam::Key current_point_key, 
            gtsam::Key previous_point_key, gtsam::Key motion_key);

        void optimize();



        void updateMap(const gtsam::Values& state);
        void logObjectMotion(gtsam::Key key, int object_label);


    private:
        Map* map;
        std::unique_ptr<gtsam::ISAM2> isam;
        // std::unique_ptr<gtsam::IncrementalFixedLagSmoother> isam;
        gtsam::ISAM2Result result;
        BackendParams::Ptr params;

        // State.
        //!< current state of the system.
        gtsam::Values state_;


        // DynamicObjectManager::UniquePtr do_manager;
        StaticTrackletManager static_tracklets;
        DynamicTrackletManager dynamic_tracklets; 

        Camera::Ptr camera;
        
        std::vector<std::vector<gtsam::Key>> unique_vertices;
        gtsam::Key count_unique_id;

        //reverse loook up so given a unique key we can find the i, j pairining in the unique_vertices
        //which will tell us how tp put this back into the map... if we know the type...
        std::map<gtsam::Key, IJSymbol> key_to_unique_vertices;

        //I think this is the id of the camera pose vertex saved at the previous and current frames
        //the frame id solved for previously
        gtsam::Key pre_camera_pose_vertex;
        //the frame id to be solving for. I guess we only need one of them 
        //this starts at 0
        gtsam::Key curr_camera_pose_vertex;

        //make optimizer using gtsam
        gtsam::NonlinearFactorGraph graph;
        gtsam::Values new_camera_poses;
        gtsam::Values new_object_motions;

        //keys to update after each optimization
        std::map<gtsam::Key, FrameSlot> camera_pose_to_update;
        std::map<gtsam::Key, FrameSlot> object_motions_to_update;
        std::map<gtsam::Key, FrameSlot> static_points_to_update;
        std::map<gtsam::Key, FrameSlot> dynamic_points_to_update;

        //each vector should be the same length as the graph for evety frame, but only true for the dynamic ones
        std::vector<std::vector<bool>> dynamic_factor_indicies;


        //these will hold all the keys every had so we can reference them later
        std::map<gtsam::Key, FrameSlot> all_camera_pose_to_update;
        std::map<gtsam::Key, FrameSlot> all_object_motions_to_update;

        gtsam::Values all_values; //currently used for writing out to g2o

        //just used currently while writing to g2o so we can add values to an "optimizer"
        //and then check if so all the previous sanity checks hold
        gtsam::Values fake_isam_poses; 

        //new landmarks that have not been added to isam yet.
        gtsam::Values new_lmks;

        //new dynamic landmarks that have not been added to isam2 yet
        //or do we want this ot be a new motion...?
        gtsam::Values new_dynamic_lmks;

        //how many times a value has been observed. Ie, has a factor on it.
        //use add3DPointFactorToGraph. We collect the factors until their are at least
        //2 point3D Factors on a landmark
        std::map<gtsam::Key, Point3DFactors> observed_landmarks;

        //same as above but for dynamic
        std::map<gtsam::Key, Point3DFactors> observed_dyn_landmarks;
        //Unsure if this is necessary as if we have a motion the point has been obserted twice?
        //the Key should be on the motion as it is the key that joins all the vertices together
        std::map<gtsam::Key, LandmarkMotionTernaryFactors> observed_motions;


        //the frame count.The size of the map (N) - 1. Assuming we call update every iteration
        int current_frame;
        int last_frame_with_dynamic;

        //camera matrix
        const cv::Mat K;
        gtsam::Cal3_S2::shared_ptr K_calib;

        //measurement information that we updated every step
        // mark each feature if it is satisfied (valid) for usage
        // here we use track length as threshold, for static >=3, dynamic >=3.
        // label each feature of the position in TrackLets: -1(invalid) or >=0(TrackID);
        // size: static: (N)xM_1, M_1 is the size of features in each frame
        // size: dynamic: (N)xM_2, M_2 is the size of features in each frame
        std::vector<std::vector<int>> vnFeaLabSta;
        std::vector<std::vector<int>> vnFeaMakSta;
        std::vector<std::vector<int>> vnFeaLabDyn;
        std::vector<std::vector<int>> vnFeaMakDyn;
        std::vector<std::vector<bool>> objCheck; //same size as frames with inner vector the same size as map->vnRMLabel

        std::vector<std::vector<int>> objUniqueId;


       
        const unsigned char kSymbolCameraPose3Key = 'X';
        const unsigned char kSymbolPoint3Key = 'm';
        const unsigned char kSymbolMotion3Key = 'H';
        const unsigned char kSymbolDynamicPoint3Key = 'l';


        //Final noise models to use. Set when calling setup Noise Models
        //either diagonal or robust + diagonal (depending on params)
        gtsam::noiseModel::Diagonal::shared_ptr cameraPosePrior;
        gtsam::noiseModel::Base::shared_ptr odometryNoiseModel;
        gtsam::noiseModel::Base::shared_ptr point3DNoiseModel;
        gtsam::noiseModel::Base::shared_ptr objectMotionNoiseModel;
        gtsam::noiseModel::Base::shared_ptr objectMotionSmootherNoiseModel;
        gtsam::noiseModel::Base::shared_ptr dynamicPoint3DNoiseModel;

        BackendDebugInfo debug_info;

        //plotting stuff
        std::vector<double> error_before_v;
        std::vector<double> error_after_v;
        std::vector<double> time_to_optimize;
        std::vector<int> no_factors;
        std::vector<int> no_variables;

        //key-> object label
        //vector of variables (via keys) for all the values associated with that particular object
        std::map<int, std::vector<gtsam::Key>> dynamic_motion_map;
        //this is keys per frame -> we clear and update the one above every tick
        std::map<int, std::vector<std::vector<gtsam::Key>>> dynamic_motion_map_total;
        //the key we first see the object .
        std::map<int, int> when_dynamic_motion_added;


};





} //VDO_SLAM