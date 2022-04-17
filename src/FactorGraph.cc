#include "FactorGraph.h"
#include "Converter.h"
#include "Optimizer.h"


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

#include <glog/logging.h>
#include <vector>

#include <string>  
#include <iostream> 
#include <sstream>    


namespace VDO_SLAM {

FactorGraph::FactorGraph(Map* map_, const cv::Mat& Calib_K_)
    :   map(CHECK_NOTNULL(map_)),
        Calib_K(Calib_K_),
        steps(0),
        pre_frame_id(-1),
        curr_frame_id(0),
        count_unique_id(1),
        start_frame(0),
        vnFeaLabSta(),
        vnFeaMakSta(),
        vnFeaLabDyn(),
        vnFeaMakDyn() {

    linearSolver = new g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>();
    solver_ptr = new g2o::BlockSolverX(linearSolver);
    solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);


    optimizer.setAlgorithm(solver);

    terminateAction = new g2o::SparseOptimizerTerminateAction;
    CHECK_NOTNULL(terminateAction)->setGainThreshold(1e-3);
    optimizer.addPostIterationAction(terminateAction);

    cameraOffset = new g2o::ParameterSE3Offset();
    CHECK_NOTNULL(cameraOffset)->setId(0);
    optimizer.addParameter(cameraOffset);
}

FactorGraph::~FactorGraph() {
    delete cameraOffset;
    delete terminateAction;
    delete solver;
    delete solver_ptr;
    delete linearSolver;
}



//basically taking a step size of 1
void FactorGraph::step() {
    const int N = map->vpFeatSta.size(); // Number of Frames

    PCHECK(N >= steps) << "Number of frames cannot exceed the number of step calls";
    CHECK_EQ(N, steps + 1); //if called every time

    start_frame = N -1;
    LOG(INFO) << "Solving from start frame  " << start_frame;



    std::vector<std::vector<std::pair<int, int>>> StaTracks = map->TrackletSta;
    std::vector<std::vector<std::pair<int, int>>> DynTracks = map->TrackletDyn;

    //This is all copied from batch optimization 
    //Now we're doing it incrementally we dont need to recauclate it it every time
    //but for now I will 
    std::vector<int>  vnFLS_tmp(map->vpFeatSta[start_frame].size(),-1);
    vnFeaLabSta.push_back(vnFLS_tmp);
    vnFeaMakSta.push_back(vnFLS_tmp);

    std::vector<int> vnFLD_tmp(map->vpFeatDyn[start_frame].size(),-1);
    vnFeaLabDyn.push_back(vnFLD_tmp);
    vnFeaMakDyn.push_back(vnFLD_tmp);

    CHECK_EQ(vnFeaLabSta.size(), N);
    CHECK_EQ(vnFeaMakSta.size(), N);
    CHECK_EQ(vnFeaLabDyn.size(), N);
    CHECK_EQ(vnFeaMakDyn.size(), N);

    // // initialize
    // for (int i = 0; i < N; ++i)
    // {
    //     std::vector<int>  vnFLS_tmp(map->vpFeatSta[i].size(),-1);
    //     vnFeaLabSta[i] = vnFLS_tmp;
    //     vnFeaMakSta[i] = vnFLS_tmp;
    // }
    // for (int i = 0; i < N; ++i)
    // {
    //     std::vector<int>  vnFLD_tmp(map->vpFeatDyn[i].size(),-1);
    //     vnFeaLabDyn[i] = vnFLD_tmp;
    //     vnFeaMakDyn[i] = vnFLD_tmp;
    // }
    int valid_sta = 0, valid_dyn = 0;
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


    //for now we're just going to opt for camera poses

    // // check if objects has the required tracking length in current window
    // const int ObjLength = WINDOW_SIZE-1;
    // std::vector<std::vector<bool> > ObjCheck(N-1);
    // for (int i = 0; i < N-1; ++i)
    // {
    //     std::vector<bool>  ObjCheck_tmp(map->vnRMLabel[i].size(),false);
    //     ObjCheck[i] = ObjCheck_tmp;
    // }
    // // collect unique object label and how many times it appears
    // std::vector<int> UniLab, LabCount;
    // for (int i = N-WINDOW_SIZE; i < N-1; ++i)
    // {
    //     //first 
    //     if (i == N-WINDOW_SIZE) {
    //         //start at j = 1 because 0 is camera motion and 1 ... l is rigid motion
    //         for (int j = 1; j < map->vnRMLabel[i].size(); ++j)
    //         {
    //             //guarantee that all labels are unique?
    //             //frame i and rigid body motion j
    //             //for the first one we just go through and all labels for this frame
    //             UniLab.push_back(map->vnRMLabel[i][j]);
    //             LabCount.push_back(1);
    //         }
    //     }
    //     else
    //     {
    //         for (int j = 1; j < map->vnRMLabel[i].size(); ++j)
    //         {
    //             bool used = false;
    //             for (int k = 0; k < UniLab.size(); ++k)
    //             {
    //                 //if the unique label at index k is the same as this
    //                 //rigid body motion, label it it has used and inncrease count
    //                 if (UniLab[k]==map->vnRMLabel[i][j])
    //                 {
    //                     used = true;
    //                     LabCount[k] = LabCount[k] + 1;
    //                     break;
    //                 }
    //             }
    //             if (used==false)
    //             {
    //                 UniLab.push_back(map->vnRMLabel[i][j]);
    //                 LabCount.push_back(1);
    //             }
    //         }
    //     }
    // }
    // // assign the ObjCheck ......
    // for (int i = N-WINDOW_SIZE; i < N-1; ++i)
    // {
    //     for (int j = 1; j < map->vnRMLabel[i].size(); ++j)
    //     {
    //         for (int k = 0; k < UniLab.size(); ++k)
    //         {
    //             if (UniLab[k]==map->vnRMLabel[i][j] && LabCount[k]>=ObjLength)
    //             {
    //                 ObjCheck[i][j]= true;
    //                 break;
    //             }
    //         }
    //     }
    // }

    // === set information matrix ===
    const float sigma2_cam = 0.0001; // 0.005 0.001 0.0001
    const float sigma2_3d_sta = 16; // 50 80 16
    const float sigma2_obj_smo = 0.1; // 0.1
    const float sigma2_obj = 20; // 0.5 1 10 20
    const float sigma2_3d_dyn = 16; // 50 100 16
    const float sigma2_alti = 1;

    // === identity initialization ===
    cv::Mat id_temp = cv::Mat::eye(4,4, CV_32F);

    std::vector<g2o::EdgeSE3*> vpEdgeSE3;
    std::vector<g2o::LandmarkMotionTernaryEdge*> vpEdgeLandmarkMotion;
    std::vector<g2o::EdgeSE3PointXYZ*> vpEdgeSE3PointSta;
    std::vector<g2o::EdgeSE3PointXYZ*> vpEdgeSE3PointDyn;
    std::vector<g2o::EdgeSE3Altitude*> vpEdgeSE3Altitude;
    std::vector<g2o::EdgeSE3*> vpEdgeSE3Smooth;

    int FeaLengthThresSta = 3, FeaLengthThresDyn = 3;
    bool ROBUST_KERNEL = true, ALTITUDE_CONSTRAINT = false, SMOOTH_CONSTRAINT = true, STATIC_ONLY = true;
    float deltaHuberCamMot = 0.0001, deltaHuberObjMot = 0.0001, deltaHuber3D = 0.0001;

    //init vertex id's
    //or just is the first frame (N==1)

    if(start_frame == 0) {
        CHECK_EQ(N, 1);
        std::vector<int> v_id_tmp(1,-1);
        unique_vertices.push_back(v_id_tmp);
    }
    else {
        std::vector<int> v_id_tmp(map->vnRMLabel[start_frame-1].size(),-1);
        unique_vertices.push_back(v_id_tmp);
    }


    CHECK_EQ(unique_vertices.size(), N);

    //the first time this is called: ie. curr_frame_id == 0 as N = 1
    if(start_frame == 0) {
        LOG(INFO) << "The first frame. N: " << N;
        CHECK_EQ(count_unique_id, 1);

        //we just have the one camera pose measurement to add
        // (1) save <VERTEX_POSE_R3_SO3>
        g2o::VertexSE3 *v_se3 = new g2o::VertexSE3();
        v_se3->setId(count_unique_id);
        v_se3->setEstimate(Converter::toSE3Quat(map->vmCameraPose[start_frame]));
        optimizer.addVertex(v_se3);

        //what about the prior?

        //unique id for the first frame and the camera pose
        unique_vertices[start_frame][0] = count_unique_id;
        curr_frame_id = count_unique_id;
        count_unique_id++;
        
    }
    //not the first call
    else {
        // (2) save <EDGE_R3_SO3>
        g2o::EdgeSE3 * ep = new g2o::EdgeSE3();
        ep->setVertex(0, optimizer.vertex(pre_frame_id));
        ep->setVertex(1, optimizer.vertex(curr_frame_id));
        ep->setMeasurement(Converter::toSE3Quat(map->vmRigidMotion[start_frame-1][0]));
        ep->information() = Eigen::MatrixXd::Identity(6, 6)/sigma2_cam;
        if (ROBUST_KERNEL)
        {
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber();
            ep->setRobustKernel(rk);
            ep->robustKernel()->setDelta(deltaHuberCamMot);
        }
        optimizer.addEdge(ep);
        vpEdgeSE3.push_back(ep);
    }

    CHECK(curr_frame_id != 0);

    //now do for static features
    LOG(INFO) << "Static features for current frame is " << vnFeaLabSta[start_frame].size();
    for (int j = 0; j < vnFeaLabSta[start_frame].size(); ++j) {
        // check feature validation
        if (vnFeaLabSta[start_frame][j]==-1) {
            continue;
        }

        // get the TrackID of current feature
        int TrackID = vnFeaLabSta[start_frame][j];

        // get the position of current feature in the tracklet
        int PositionID = -1;
        for (int k = 0; k < StaTracks[TrackID].size(); ++k)
        {
            //if we're not looping through all the frames (eg i becomes start_frame is this going to be a problem?)
            if (StaTracks[TrackID][k].first==start_frame && StaTracks[TrackID][k].second==j)
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
            // check if this feature track has the same length as the window size
            const int TrLength = StaTracks[TrackID].size();
            if ( TrLength<FeaLengthThresSta ) {
                continue;
            }


            // (3) save <VERTEX_POINT_3D>
            g2o::VertexPointXYZ *v_p = new g2o::VertexPointXYZ();
            v_p->setId(count_unique_id);
            cv::Mat Xw = map->vp3DPointSta[start_frame][j];
            v_p->setEstimate(Converter::toVector3d(Xw));
            optimizer.addVertex(v_p);

            // (4) save <EDGE_3D>
            g2o::EdgeSE3PointXYZ * e = new g2o::EdgeSE3PointXYZ();
            e->setVertex(0, optimizer.vertex(curr_frame_id));
            e->setVertex(1, optimizer.vertex(count_unique_id));
            cv::Mat Xc = Optimizer::Get3DinCamera(map->vpFeatSta[start_frame][j],map->vfDepSta[start_frame][j],Calib_K);
            e->setMeasurement(Converter::toVector3d(Xc));
            e->information() = Eigen::Matrix3d::Identity()/sigma2_3d_sta;
            if (ROBUST_KERNEL)
            {
                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber();
                e->setRobustKernel(rk);
                e->robustKernel()->setDelta(deltaHuber3D);
            }
            e->setParameterId(0, 0);
            optimizer.addEdge(e);
            vpEdgeSE3PointSta.push_back(e);

            // update unique id
            vnFeaMakSta[start_frame][j] = count_unique_id;
            count_unique_id++;
        }
        else
        {
            // check if this feature track has the same length as the window size
            // or its previous FeaMakTmp is not -1, then save it, otherwise skip.
            const int TrLength = StaTracks[TrackID].size();
            const int FeaMakTmp = vnFeaMakSta[StaTracks[TrackID][PositionID-1].first][StaTracks[TrackID][PositionID-1].second];
            if (TrLength<FeaLengthThresSta || FeaMakTmp==-1) {
                continue;
            }

            // (4) save <EDGE_3D>
            g2o::EdgeSE3PointXYZ * e = new g2o::EdgeSE3PointXYZ();
            e->setVertex(0, optimizer.vertex(curr_frame_id));
            e->setVertex(1, optimizer.vertex(FeaMakTmp));
            cv::Mat Xc = Optimizer::Get3DinCamera(map->vpFeatSta[start_frame][j],map->vfDepSta[start_frame][j],Calib_K);
            e->setMeasurement(Converter::toVector3d(Xc));
            e->information() = Eigen::Matrix3d::Identity()/sigma2_3d_sta;
            if (ROBUST_KERNEL)
            {
                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber();
                e->setRobustKernel(rk);
                e->robustKernel()->setDelta(deltaHuber3D);
            }
            e->setParameterId(0, 0);
            optimizer.addEdge(e);
            vpEdgeSE3PointSta.push_back(e);

            // update unique id
            vnFeaMakSta[start_frame][j] = FeaMakTmp;
        }

    }


    // update frame ID
    pre_frame_id = curr_frame_id;

    //now steps should be the same as N
    steps++;


}

void FactorGraph::optimize() {
    //for now dont do any object motion
    //start optimize
    optimizer.initializeOptimization();
    optimizer.setVerbose(true);

    LOG(INFO) << "Running incremental opt for start frame " << start_frame;
    optimizer.optimize(100);

}

void FactorGraph::updateMap() {
    LOG(INFO) << "Updating pose and motion for increment";

    //we use this to check that the map is updated immediately after a step and an optimization
    CHECK_EQ(steps, getMapSize());
    CHECK_EQ(start_frame, getMapSize() - 1);

    g2o::VertexSE3* vSE3 = static_cast<g2o::VertexSE3*>(optimizer.vertex(unique_vertices[start_frame][0]));
    CHECK_NOTNULL(vSE3);

    // convert
    double optimized[7];
    vSE3->getEstimateData(optimized);
    Eigen::Quaterniond q(optimized[6],optimized[3],optimized[4],optimized[5]);
    Eigen::Matrix<double,3,3> rot = q.matrix();
    Eigen::Matrix<double,3,1> tra;
    tra << optimized[0],optimized[1],optimized[2];

    // camera pose
    map->vmCameraPose[start_frame] = Converter::toCvSE3(rot,tra);

    //we should have motion
    if(start_frame > 0) {
        map->vmRigidMotion[start_frame-1][0] = Converter::toInvMatrix(
            map->vmCameraPose[start_frame-1])*map->vmCameraPose[start_frame];
    }

    LOG(INFO) << "Updating 3D points!!";
    for (int j = 0; j < vnFeaMakSta[start_frame].size(); ++j) {
        if (vnFeaMakSta[start_frame][j] != -1) {

            g2o::VertexPointXYZ* vPoint = static_cast<g2o::VertexPointXYZ*>(optimizer.vertex(vnFeaMakSta[start_frame][j]));
            double optimized[3];
            vPoint->getEstimateData(optimized);
            Eigen::Matrix<double,3,1> tmp_3d;
            tmp_3d << optimized[0],optimized[1],optimized[2];
            map->vp3DPointSta[start_frame][j] = Converter::toCvMat(tmp_3d);
        }
    }
    

    LOG(INFO) << "Completed Update";
}

void FactorGraph::printGraphState() {
    std::stringstream ss;
    ss << "Current start frame " << start_frame << "\n";
    ss << "No. active edges " << optimizer.activeEdges().size() << "\n";
    ss << "No. active vertices " << optimizer.activeVertices().size() << "\n";
    LOG(INFO) <<  ss.str();
}

void FactorGraph::stepAndOptimize() {
    step();
    printGraphState();

    if(start_frame > 0) {
        optimize();
        updateMap();
    }
    
}

};