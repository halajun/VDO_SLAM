// #pragma once

// #include <glog/logging.h>
// #include <memory>

// #include <gtsam/nonlinear/ISAM2.h>
// #include <gtsam/slam/ProjectionFactor.h>
// #include <gtsam/slam/BetweenFactor.h>
// #include <gtsam/base/Vector.h>
// #include <gtsam/geometry/Pose3.h>

// #include "factors/Point3DFactor.h"
// #include "backend/VdoSlamBackend-types.h"

// namespace VDO_SLAM {

// class FactorGraphManager {

//     public:
//         FactorGraphManager() = default;
//         virtual ~FactorGraphManager() = default;

//         void addCameraPoseToGraph(const gtsam::Pose3& pose, int key, FrameId current_frame);

//         void addLandmarkToGraph(const gtsam::Point3& landmark, int key, FrameId current_frame, FeatureId feature_id);
//         void addPoint3DFactor(const gtsam::Point3& measurement, int pose_key, int landmark_key);

//     protected:
//         //could totally make this templated and add the type itself to the values variable
//         //but there is some logic about incrementing the curr_camera_pose_vertex and other vertex
//         //counters that may make this difficult
//         void addToKeyVertexMapping(const gtsam::Key& key, int i, int j, unsigned char symbol);

//     protected:
//         std::vector<std::vector<int>> unique_vertices;
//         int count_unique_id;

//         //reverse loook up so given a unique key we can find the i, j pairining in the unique_vertices
//         //which will tell us how tp put this back into the map... if we know the type...
//         std::map<gtsam::Key, IJSymbol> key_to_unique_vertices;

//         //how many times a value has been observed. Ie, has a factor on it.
//         //use add3DPointFactorToGraph. We collect the factors until their are at least
//         //2 point3D Factors on a landmark
//         std::map<gtsam::Key, Point3DFactors> observed_landmarks;

//         const unsigned char kSymbolCameraPose3Key = 'X';
//         const unsigned char kSymbolPoint3Key = 'm';

//     private:
//         //make optimizer using gtsam
//         gtsam::NonlinearFactorGraph graph;
//         gtsam::Values new_camera_poses;

//         // gtsam::NonlinearFactorGraph point_3d_factors;
//         gtsam::Values new_lmks;

// };

// } //VDO_SLAM