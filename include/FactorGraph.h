#pragma once

#include "Map.h"

#include "dependencies/g2o/g2o/core/block_solver.h"
#include "dependencies/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "dependencies/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "dependencies/g2o/g2o/core/optimization_algorithm_dogleg.h"
#include "dependencies/g2o/g2o/solvers/linear_solver_eigen.h"
#include "dependencies/g2o/g2o/types/types_six_dof_expmap.h"
#include "dependencies/g2o/g2o/core/robust_kernel_impl.h"
#include "dependencies/g2o/g2o/solvers/linear_solver_dense.h"
#include "dependencies/g2o/g2o/types/types_seven_dof_expmap.h"

#include "dependencies/g2o/g2o/types/types_dyn_slam3d.h"
#include "dependencies/g2o/g2o/types/vertex_se3.h"
#include "dependencies/g2o/g2o/types/vertex_pointxyz.h"
#include "dependencies/g2o/g2o/types/edge_se3.h"
#include "dependencies/g2o/g2o/types/edge_se3_pointxyz.h"
#include "dependencies/g2o/g2o/types/edge_se3_prior.h"
#include "dependencies/g2o/g2o/types/edge_xyz_prior.h"
#include "dependencies/g2o/g2o/core/sparse_optimizer_terminate_action.h"
#include "dependencies/g2o/g2o/solvers/linear_solver_csparse.h"

#include "dependencies/g2o/g2o/incremental/graph_optimizer_sparse_incremental.h"


#include <glog/logging.h>
#include <memory>

namespace VDO_SLAM {

class FactorGraph {

    public:
        typedef std::shared_ptr<FactorGraph> Ptr;
        typedef std::unique_ptr<FactorGraph> UniquePtr;
        typedef std::shared_ptr<const FactorGraph> ConstPtr;

        FactorGraph(Map* map_, const cv::Mat& Calib_K_);
        ~FactorGraph();

        void stepAndOptimize();
        void step();
        void optimize();
        void printGraphState();
        void printStats();

    
        Map* map;
        const cv::Mat Calib_K;

        inline int getMapSize() const { return map->vpFeatSta.size(); }

        cv::Mat getLatestCameraPose() const;

    private:
        void updateMap();


        //the current frame Id we are incrementally adding
        int start_frame;
        //should always one less than the size of the map (ie. N - 1)
        int steps;

        //I think this is the id of the camera pose vertex saved at the previous and current frames
        //the frame id solved for previously
        int pre_camera_pose_vertex;
        //the frame id to be solving for. I guess we only need one of them 
        //this starts at 0
        int curr_camera_pose_vertex;

        int batch_size;
        int update_size;


        //g2o stuff
        g2o::SparseOptimizer optimizer;
        // g2o::SparseOptimizerIncremental optimizer;
        g2o::BlockSolverX::LinearSolverType* linearSolver;
        g2o::BlockSolverX* solver_ptr;
        g2o::OptimizationAlgorithmLevenberg* solver;
        g2o::SparseOptimizerTerminateAction* terminateAction;
        g2o::ParameterSE3Offset* cameraOffset;

        std::vector<std::vector<int>> unique_vertices;
        int count_unique_id;


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


        g2o::HyperGraph::VertexSet verticesAdded;
        g2o::HyperGraph::EdgeSet edgesAdded;


};


} //VDO_SLAM