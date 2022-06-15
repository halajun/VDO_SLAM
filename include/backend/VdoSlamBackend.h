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

#include "utils/macros.h"
#include "utils/types.h"
#include "backend/VdoSlamBackend-types.h"
#include "backend/VdoSlamBackendParams.h"
#include "backend/DynamicObjectManager.h"
#include "backend/TrackedObservation.h"
#include "Map.h"


namespace VDO_SLAM {

class VdoSlamBackend {

    public:
        VDO_SLAM_POINTER_TYPEDEFS(VdoSlamBackend);


        VdoSlamBackend(Map* map_, const cv::Mat& Calib_K_, BackendParams::Ptr params_);
        ~VdoSlamBackend() = default;

        //probably should be called process frontend of something like that
        void process();

        gtsam::Values calculateCurrentEstimate() const;

        void calculateError();
        //this updates the map from the last set of marked variables in the isam2 results object
        void updateMapFromIncremental();
        //updates the map using all values ever seen (as stored in key_to_unique_vertices)
        void updateMapFull();

        void writeG2o(const std::string& file_name);

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

        void addLandmarkToGraph(const gtsam::Point3& landmark, gtsam::Key key, FrameId curr_frame, FeatureId feature_id);
        void addPoint3DFactor(const gtsam::Point3& measurement, gtsam::Key pose_key, gtsam::Key landmark_key);

        gtsam::Values collectValuesToAdd();
        void optimize();


        void updateMapFromSymbol(const gtsam::Key& key, const IJSymbol& vertex_symbol);


    private:
        Map* map;
        std::unique_ptr<gtsam::ISAM2> isam;
        gtsam::ISAM2Result result;
        BackendParams::Ptr params;

        DynamicObjectManager::UniquePtr do_manager;



        std::vector<std::vector<int>> unique_vertices;
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

        gtsam::Values all_values; //currently used for writing out to g2o

        //just used currently while writing to g2o so we can add values to an "optimizer"
        //and then check if so all the previous sanity checks hold
        gtsam::Values fake_isam_poses; 

        //new landmarks that have not been added to isam yet.
        gtsam::Values new_lmks;

        //how many times a value has been observed. Ie, has a factor on it.
        //use add3DPointFactorToGraph. We collect the factors until their are at least
        //2 point3D Factors on a landmark
        std::map<gtsam::Key, Point3DFactors> observed_landmarks;


        //the frame count.The size of the map (N) - 1. Assuming we call update every iteration
        int current_frame;

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

       
        std::vector<std::vector<bool>> obj_check;

        const unsigned char kSymbolCameraPose3Key = 'X';
        const unsigned char kSymbolPoint3Key = 'm';


        //Final noise models to use. Set when calling setup Noise Models
        //either diagonal or robust + diagonal (depending on params)
        gtsam::noiseModel::Diagonal::shared_ptr cameraPosePrior;
        gtsam::noiseModel::Base::shared_ptr odometryNoiseModel;
        gtsam::noiseModel::Base::shared_ptr point3DNoiseModel;

        BackendDebugInfo debug_info;

};





} //VDO_SLAM