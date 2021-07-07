/**
* This file is part of VDO-SLAM.
*
* Copyright (C) 2019-2020 Jun Zhang <jun doc zhang2 at anu dot edu doc au> (The Australian National University)
* For more information see <https://github.com/halajun/VDO_SLAM>
*
**/

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

#include<Eigen/StdVector>

#include "Converter.h"

#include<mutex>

namespace VDO_SLAM
{

using namespace std;

void Optimizer::PartialBatchOptimization(Map* pMap, const cv::Mat Calib_K, const int WINDOW_SIZE)
{
    const int N = pMap->vpFeatSta.size(); // Number of Frames
    std::vector<std::vector<std::pair<int, int> > > StaTracks = pMap->TrackletSta;
    std::vector<std::vector<std::pair<int, int> > > DynTracks = pMap->TrackletDyn;

    // =======================================================================================

    // mark each feature if it is satisfied (valid) for usage
    // here we use track length as threshold, for static >=3, dynamic >=3.
    // label each feature of the position in TrackLets: -1(invalid) or >=0(TrackID);
    // size: static: (N)xM_1, M_1 is the size of features in each frame
    // size: dynamic: (N)xM_2, M_2 is the size of features in each frame
    std::vector<std::vector<int> > vnFeaLabSta(N),vnFeaMakSta(N),vnFeaLabDyn(N),vnFeaMakDyn(N);
    // initialize
    for (int i = 0; i < N; ++i)
    {
        std::vector<int>  vnFLS_tmp(pMap->vpFeatSta[i].size(),-1);
        vnFeaLabSta[i] = vnFLS_tmp;
        vnFeaMakSta[i] = vnFLS_tmp;
    }
    for (int i = 0; i < N; ++i)
    {
        std::vector<int>  vnFLD_tmp(pMap->vpFeatDyn[i].size(),-1);
        vnFeaLabDyn[i] = vnFLD_tmp;
        vnFeaMakDyn[i] = vnFLD_tmp;
    }
    int valid_sta = 0, valid_dyn = 0;
    // label static feature
    for (int i = 0; i < StaTracks.size(); ++i)
    {
        // filter the tracklets via threshold
        if (StaTracks[i].size()<3) // 3 the length of track on background.
            continue;
        valid_sta++;
        // label them
        for (int j = 0; j < StaTracks[i].size(); ++j)
            vnFeaLabSta[StaTracks[i][j].first][StaTracks[i][j].second] = i;
    }
    // label dynamic feature
    for (int i = 0; i < DynTracks.size(); ++i)
    {
        // filter the tracklets via threshold
        if (DynTracks[i].size()<3) // 3 the length of track on objects.
            continue;
        valid_dyn++;
        // label them
        for (int j = 0; j < DynTracks[i].size(); ++j){
            vnFeaLabDyn[DynTracks[i][j].first][DynTracks[i][j].second] = i;

        }
    }

    // save vertex ID in the graph
    std::vector<std::vector<int> > VertexID(N);
    // initialize
    for (int i = 0; i < N; ++i)
    {
        if (i==0)
        {
            std::vector<int> v_id_tmp(1,-1);
            VertexID[i] = v_id_tmp;
        }
        else
        {
            std::vector<int> v_id_tmp(pMap->vnRMLabel[i-1].size(),-1);
            VertexID[i] = v_id_tmp;
        }
    }

    // check if objects has the required tracking length in current window
    const int ObjLength = WINDOW_SIZE-1;
    std::vector<std::vector<bool> > ObjCheck(N-1);
    for (int i = 0; i < N-1; ++i)
    {
        std::vector<bool>  ObjCheck_tmp(pMap->vnRMLabel[i].size(),false);
        ObjCheck[i] = ObjCheck_tmp;
    }
    // collect unique object label and how many times it appears
    std::vector<int> UniLab, LabCount;
    for (int i = N-WINDOW_SIZE; i < N-1; ++i)
    {
        if (i == N-WINDOW_SIZE)
        {
            for (int j = 1; j < pMap->vnRMLabel[i].size(); ++j)
            {
                UniLab.push_back(pMap->vnRMLabel[i][j]);
                LabCount.push_back(1);
            }
        }
        else
        {
            for (int j = 1; j < pMap->vnRMLabel[i].size(); ++j)
            {
                bool used = false;
                for (int k = 0; k < UniLab.size(); ++k)
                {
                    if (UniLab[k]==pMap->vnRMLabel[i][j])
                    {
                        used = true;
                        LabCount[k] = LabCount[k] + 1;
                        break;
                    }
                }
                if (used==false)
                {
                    UniLab.push_back(pMap->vnRMLabel[i][j]);
                    LabCount.push_back(1);
                }
            }
        }
    }
    // assign the ObjCheck ......
    for (int i = N-WINDOW_SIZE; i < N-1; ++i)
    {
        for (int j = 1; j < pMap->vnRMLabel[i].size(); ++j)
        {
            for (int k = 0; k < UniLab.size(); ++k)
            {
                if (UniLab[k]==pMap->vnRMLabel[i][j] && LabCount[k]>=ObjLength)
                {
                    ObjCheck[i][j]= true;
                    break;
                }
            }
        }
    }

    // =======================================================================================

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;
    linearSolver = new g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);
    // g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg(solver_ptr);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
    optimizer.setAlgorithm(solver);

    g2o::SparseOptimizerTerminateAction* terminateAction = new g2o::SparseOptimizerTerminateAction;
    terminateAction->setGainThreshold(1e-3);
    optimizer.addPostIterationAction(terminateAction);

    g2o::ParameterSE3Offset* cameraOffset = new g2o::ParameterSE3Offset;
    cameraOffset->setId(0);
    optimizer.addParameter(cameraOffset);

    // === set information matrix ===
    const float sigma2_cam = 0.0001; // 0.005 0.001 0.0001
    const float sigma2_3d_sta = 16; // 50 80 16
    const float sigma2_obj_smo = 0.1; // 0.1
    const float sigma2_obj = 20; // 0.5 1 10 20
    const float sigma2_3d_dyn = 16; // 50 100 16
    const float sigma2_alti = 1;

    // === identity initialization ===
    cv::Mat id_temp = cv::Mat::eye(4,4, CV_32F);

    vector<g2o::EdgeSE3*> vpEdgeSE3;
    vector<g2o::LandmarkMotionTernaryEdge*> vpEdgeLandmarkMotion;
    vector<g2o::EdgeSE3PointXYZ*> vpEdgeSE3PointSta;
    vector<g2o::EdgeSE3PointXYZ*> vpEdgeSE3PointDyn;
    vector<g2o::EdgeSE3Altitude*> vpEdgeSE3Altitude;
    vector<g2o::EdgeSE3*> vpEdgeSE3Smooth;

    // ---------------------------------------------------------------------------------------
    // ---------=============!!!=- Main Loop for input data -=!!!=============----------------
    // ---------------------------------------------------------------------------------------
    int count_unique_id = 1, FeaLengthThresSta = 3, FeaLengthThresDyn = 3, StaticStartFrame = N-WINDOW_SIZE;
    bool ROBUST_KERNEL = true, ALTITUDE_CONSTRAINT = false, SMOOTH_CONSTRAINT = true, STATIC_ONLY = true;
    // float deltaHuberCamMot = 0.1, deltaHuberObjMot = 0.25, deltaHuber3D = 0.25;
    float deltaHuberCamMot = 0.0001, deltaHuberObjMot = 0.0001, deltaHuber3D = 0.0001;
    int PreFrameID, CurFrameID;

    // ===========================================================================
    // =================== FOR static points and camera poses ====================
    // ===========================================================================
    for (int i = StaticStartFrame; i < N; ++i)
    {
        // (1) save <VERTEX_POSE_R3_SO3>
        g2o::VertexSE3 *v_se3 = new g2o::VertexSE3();
        v_se3->setId(count_unique_id);
        v_se3->setEstimate(Converter::toSE3Quat(pMap->vmCameraPose[i]));
        // v_se3->setEstimate(Converter::toSE3Quat(id_temp));
        optimizer.addVertex(v_se3);
        if (count_unique_id==1 && N==WINDOW_SIZE)
        {
            // cout << "the very first frame: " << N << " " << WINDOW_SIZE << endl;
            // add prior edges
            g2o::EdgeSE3Prior * pose_prior = new g2o::EdgeSE3Prior();
            pose_prior->setVertex(0, optimizer.vertex(count_unique_id));
            pose_prior->setMeasurement(Converter::toSE3Quat(pMap->vmCameraPose[i]));
            pose_prior->information() = Eigen::MatrixXd::Identity(6, 6)/0.0000001;
            pose_prior->setParameterId(0, 0);
            optimizer.addEdge(pose_prior);
        }
        VertexID[i][0] = count_unique_id;
        // record the ID of current frame saved in graph file
        CurFrameID = count_unique_id;
        count_unique_id++;

        // ****** save camera motion if it is not the first frame ******
        if (i!=StaticStartFrame)
        {
            // (2) save <EDGE_R3_SO3>
            g2o::EdgeSE3 * ep = new g2o::EdgeSE3();
            ep->setVertex(0, optimizer.vertex(PreFrameID));
            ep->setVertex(1, optimizer.vertex(CurFrameID));
            ep->setMeasurement(Converter::toSE3Quat(pMap->vmRigidMotion[i-1][0]));
            ep->information() = Eigen::MatrixXd::Identity(6, 6)/sigma2_cam;
            if (ROBUST_KERNEL)
            {
                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                ep->setRobustKernel(rk);
                ep->robustKernel()->setDelta(deltaHuberCamMot);
            }
            optimizer.addEdge(ep);
            vpEdgeSE3.push_back(ep);
            // cout << " (1) save camera motion " << endl;
        }

        // loop for static features
        for (int j = 0; j < vnFeaLabSta[i].size(); ++j)
        {
            // check feature validation
            if (vnFeaLabSta[i][j]==-1)
                continue;

            // get the TrackID of current feature
            int TrackID = vnFeaLabSta[i][j];

            // get the position of current feature in the tracklet
            int PositionID = -1;
            for (int k = 0; k < StaTracks[TrackID].size(); ++k)
            {
                if (StaTracks[TrackID][k].first==i && StaTracks[TrackID][k].second==j)
                {
                    PositionID = k;
                    break;
                }
            }
            if (PositionID==-1){
                cout << "cannot find the position of current feature in the tracklet !!!" << endl;
                continue;
            }

            // check if the PositionID is 0. Yes means this static point is first seen by this frame,
            // then save both the vertex and edge, otherwise save edge only because vertex is saved before.
            if (PositionID==0)
            {
                // check if this feature track has the same length as the window size
                const int TrLength = StaTracks[TrackID].size();
                if ( TrLength<FeaLengthThresSta )
                    continue;

                // (3) save <VERTEX_POINT_3D>
                g2o::VertexPointXYZ *v_p = new g2o::VertexPointXYZ();
                v_p->setId(count_unique_id);
                cv::Mat Xw = pMap->vp3DPointSta[i][j];
                v_p->setEstimate(Converter::toVector3d(Xw));
                optimizer.addVertex(v_p);

                // (4) save <EDGE_3D>
                g2o::EdgeSE3PointXYZ * e = new g2o::EdgeSE3PointXYZ();
                e->setVertex(0, optimizer.vertex(CurFrameID));
                e->setVertex(1, optimizer.vertex(count_unique_id));
                cv::Mat Xc = Optimizer::Get3DinCamera(pMap->vpFeatSta[i][j],pMap->vfDepSta[i][j],Calib_K);
                e->setMeasurement(Converter::toVector3d(Xc));
                e->information() = Eigen::Matrix3d::Identity()/sigma2_3d_sta;
                if (ROBUST_KERNEL)
                {
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    e->robustKernel()->setDelta(deltaHuber3D);
                }
                e->setParameterId(0, 0);
                optimizer.addEdge(e);
                vpEdgeSE3PointSta.push_back(e);

                // update unique id
                vnFeaMakSta[i][j] = count_unique_id;
                count_unique_id++;
            }
            else
            {
                // check if this feature track has the same length as the window size
                // or its previous FeaMakTmp is not -1, then save it, otherwise skip.
                const int TrLength = StaTracks[TrackID].size();
                const int FeaMakTmp = vnFeaMakSta[StaTracks[TrackID][PositionID-1].first][StaTracks[TrackID][PositionID-1].second];
                if (TrLength<FeaLengthThresSta || FeaMakTmp==-1)
                    continue;

                // (4) save <EDGE_3D>
                g2o::EdgeSE3PointXYZ * e = new g2o::EdgeSE3PointXYZ();
                e->setVertex(0, optimizer.vertex(CurFrameID));
                e->setVertex(1, optimizer.vertex(FeaMakTmp));
                cv::Mat Xc = Optimizer::Get3DinCamera(pMap->vpFeatSta[i][j],pMap->vfDepSta[i][j],Calib_K);
                e->setMeasurement(Converter::toVector3d(Xc));
                e->information() = Eigen::Matrix3d::Identity()/sigma2_3d_sta;
                if (ROBUST_KERNEL)
                {
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    e->robustKernel()->setDelta(deltaHuber3D);
                }
                e->setParameterId(0, 0);
                optimizer.addEdge(e);
                vpEdgeSE3PointSta.push_back(e);

                // update unique id
                vnFeaMakSta[i][j] = FeaMakTmp;
            }

        }

        // cout << " (2) save static features " << endl;

        // update frame ID
        PreFrameID = CurFrameID;
    }

    // **********************************************************************
    // ************** save object motion, then dynamic features *************
    // **********************************************************************
    for (int i = N-WINDOW_SIZE; i < N; ++i)
    {
        // cout << "current processing frame: " << i << endl;

        if (STATIC_ONLY==false)
        {
            if (i==N-WINDOW_SIZE)
            {
                // loop for dynamic features
                for (int j = 0; j < vnFeaLabDyn[i].size(); ++j)
                {
                    // check feature validation
                    if (vnFeaLabDyn[i][j]==-1)
                        continue;

                    // get the TrackID of current feature
                    int TrackID = vnFeaLabDyn[i][j];

                    // get the position of current feature in the tracklet
                    int PositionID = -1;
                    for (int k = 0; k < DynTracks[TrackID].size(); ++k)
                    {
                        if (DynTracks[TrackID][k].first==i && DynTracks[TrackID][k].second==j)
                        {
                            PositionID = k;
                            break;
                        }
                    }
                    if (PositionID==-1){
                        cout << "cannot find the position of current feature in the tracklet !!!" << endl;
                        continue;
                    }

                    // check if this feature track has the same length as the window size
                    const int TrLength = DynTracks[TrackID].size();
                    if ( TrLength-PositionID<FeaLengthThresDyn )
                        continue;

                    // (3) save <VERTEX_POINT_3D>
                    g2o::VertexPointXYZ *v_p = new g2o::VertexPointXYZ();
                    v_p->setId(count_unique_id);
                    cv::Mat Xw = pMap->vp3DPointDyn[i][j];
                    v_p->setEstimate(Converter::toVector3d(Xw));
                    optimizer.addVertex(v_p);
                    // (4) save <EDGE_3D>
                    g2o::EdgeSE3PointXYZ * e = new g2o::EdgeSE3PointXYZ();
                    e->setVertex(0, optimizer.vertex(VertexID[i][0]));
                    e->setVertex(1, optimizer.vertex(count_unique_id));
                    cv::Mat Xc = Optimizer::Get3DinCamera(pMap->vpFeatDyn[i][j],pMap->vfDepDyn[i][j],Calib_K);
                    e->setMeasurement(Converter::toVector3d(Xc));
                    e->information() = Eigen::Matrix3d::Identity()/sigma2_3d_dyn;
                    if (ROBUST_KERNEL){
                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        e->robustKernel()->setDelta(deltaHuber3D);
                    }
                    e->setParameterId(0, 0);
                    optimizer.addEdge(e);
                    vpEdgeSE3PointDyn.push_back(e);

                    // update unique id
                    vnFeaMakDyn[i][j] = count_unique_id;
                    count_unique_id++;
                }
            }
            else
            {
                // loop for object motion, and keep the unique vertex id for saving object feature edges
                std::vector<int> ObjUniqueID(pMap->vmRigidMotion[i-1].size(),-1);
                // (5) save <VERTEX_SE3Motion>
                for (int j = 1; j < pMap->vmRigidMotion[i-1].size(); ++j)
                {
                    if (ObjCheck[i-1][j]==false)
                        continue;

                    g2o::VertexSE3 *m_se3 = new g2o::VertexSE3();
                    m_se3->setId(count_unique_id);
                    if (pMap->vbObjStat[i-1][j])
                        m_se3->setEstimate(Converter::toSE3Quat(pMap->vmRigidMotion[i-1][j]));
                    else
                        m_se3->setEstimate(Converter::toSE3Quat(id_temp));
                    // m_se3->setEstimate(Converter::toSE3Quat(id_temp));
                    optimizer.addVertex(m_se3);
                    if (ALTITUDE_CONSTRAINT)
                    {
                        g2o::EdgeSE3Altitude * ea = new g2o::EdgeSE3Altitude();
                        ea->setVertex(0, optimizer.vertex(count_unique_id));
                        ea->setMeasurement(0);
                        Eigen::Matrix<double, 1, 1> altitude_information(1.0/sigma2_alti);
                        ea->information() = altitude_information;
                        optimizer.addEdge(ea);
                        vpEdgeSE3Altitude.push_back(ea);
                    }
                    if (SMOOTH_CONSTRAINT && i>N-WINDOW_SIZE+2)
                    {
                        // trace back the previous id in vnRMLabel
                        int TraceID = -1;
                        for (int k = 0; k < pMap->vnRMLabel[i-2].size(); ++k)
                        {
                            if (pMap->vnRMLabel[i-2][k]==pMap->vnRMLabel[i-1][j])
                            {
                                // cout << "what is in the label: " << pMap->vnRMLabel[i-2][k] << " " << pMap->vnRMLabel[i-1][j] << " " << VertexID[i-2][k] << endl;
                                TraceID = k;
                                break;
                            }
                        }
                        // only if the back trace exist
                        if (TraceID!=-1)
                        {
                            // add smooth constraint
                            g2o::EdgeSE3 * ep = new g2o::EdgeSE3();
                            ep->setVertex(0, optimizer.vertex(VertexID[i-1][TraceID]));
                            ep->setVertex(1, optimizer.vertex(count_unique_id));
                            ep->setMeasurement(Converter::toSE3Quat(cv::Mat::eye(4,4,CV_32F)));
                            ep->information() = Eigen::MatrixXd::Identity(6, 6)/sigma2_obj_smo;
                            if (ROBUST_KERNEL){
                                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                                ep->setRobustKernel(rk);
                                ep->robustKernel()->setDelta(deltaHuberCamMot);
                            }
                            optimizer.addEdge(ep);
                            vpEdgeSE3Smooth.push_back(ep);
                        }
                    }
                    ObjUniqueID[j]=count_unique_id;
                    VertexID[i][j]=count_unique_id;
                    count_unique_id++;
                }

                // cout << " (3) save object motion " << endl;

                // // save for dynamic features
                for (int j = 0; j < vnFeaLabDyn[i].size(); j++)
                {
                    // check feature validation
                    if (vnFeaLabDyn[i][j]==-1)
                        continue;

                    // get the TrackID of current feature
                    int TrackID = vnFeaLabDyn[i][j];

                    // get the position of current feature in the tracklet
                    int PositionID = -1;
                    for (int k = 0; k < DynTracks[TrackID].size(); ++k)
                    {
                        if (DynTracks[TrackID][k].first==i && DynTracks[TrackID][k].second==j)
                        {
                            PositionID = k;
                            break;
                        }
                    }
                    if (PositionID==-1){
                        cout << "cannot find the position of current feature in the tracklet !!!" << endl;
                        continue;
                    }

                    // get the object position id of current feature
                    int ObjPositionID = -1;
                    for (int k = 1; k < pMap->vnRMLabel[i-1].size(); ++k)
                    {
                        if (pMap->vnRMLabel[i-1][k]==pMap->nObjID[TrackID]){
                            ObjPositionID = ObjUniqueID[k];
                            break;
                        }
                    }
                    if (ObjPositionID==-1 && PositionID!=0){
                        // cout << "cannot find the object association with this edge !!! WEIRD POINT !!! " << endl;
                        continue;
                    }


                    // check if the PositionID is 0. Yes means this dynamic point is first seen by this frame,
                    // then save both the vertex and edge, otherwise save edge only because vertex is saved before.
                    if (PositionID==0)
                    {

                        // check if this feature track has the same length as the window size
                        const int TrLength = DynTracks[TrackID].size();
                        if ( TrLength<FeaLengthThresDyn )
                            continue;

                        // (3) save <VERTEX_POINT_3D>
                        g2o::VertexPointXYZ *v_p = new g2o::VertexPointXYZ();
                        v_p->setId(count_unique_id);
                        cv::Mat Xw = pMap->vp3DPointDyn[i][j];
                        v_p->setEstimate(Converter::toVector3d(Xw));
                        optimizer.addVertex(v_p);
                        // (4) save <EDGE_3D>
                        g2o::EdgeSE3PointXYZ * e = new g2o::EdgeSE3PointXYZ();
                        e->setVertex(0, optimizer.vertex(VertexID[i][0]));
                        e->setVertex(1, optimizer.vertex(count_unique_id));
                        cv::Mat Xc = Optimizer::Get3DinCamera(pMap->vpFeatDyn[i][j],pMap->vfDepDyn[i][j],Calib_K);
                        e->setMeasurement(Converter::toVector3d(Xc));
                        e->information() = Eigen::Matrix3d::Identity()/sigma2_3d_dyn;
                        if (ROBUST_KERNEL)
                        {
                            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                            e->setRobustKernel(rk);
                            e->robustKernel()->setDelta(deltaHuber3D);
                        }
                        e->setParameterId(0, 0);
                        optimizer.addEdge(e);
                        vpEdgeSE3PointDyn.push_back(e);

                        // update unique id
                        vnFeaMakDyn[i][j] = count_unique_id;
                        count_unique_id++;
                    }
                    // if no, then only add this feature to the existing track it belongs to.
                    else
                    {
                        // check if this feature track has the same length as the window size
                        const int TrLength = DynTracks[TrackID].size();
                        const int FeaMakTmp = vnFeaMakDyn[DynTracks[TrackID][PositionID-1].first][DynTracks[TrackID][PositionID-1].second];
                        if ( TrLength-PositionID<FeaLengthThresDyn && FeaMakTmp==-1 )
                            continue;

                        // (3) save <VERTEX_POINT_3D>
                        g2o::VertexPointXYZ *v_p = new g2o::VertexPointXYZ();
                        v_p->setId(count_unique_id);
                        cv::Mat Xw = pMap->vp3DPointDyn[i][j];
                        v_p->setEstimate(Converter::toVector3d(Xw));
                        optimizer.addVertex(v_p);
                        // (4) save <EDGE_3D>
                        g2o::EdgeSE3PointXYZ * e = new g2o::EdgeSE3PointXYZ();
                        e->setVertex(0, optimizer.vertex(VertexID[i][0]));
                        e->setVertex(1, optimizer.vertex(count_unique_id));
                        cv::Mat Xc = Optimizer::Get3DinCamera(pMap->vpFeatDyn[i][j],pMap->vfDepDyn[i][j],Calib_K);
                        e->setMeasurement(Converter::toVector3d(Xc));
                        e->information() = Eigen::Matrix3d::Identity()/sigma2_3d_dyn;
                        if (ROBUST_KERNEL)
                        {
                            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                            e->setRobustKernel(rk);
                            e->robustKernel()->setDelta(deltaHuber3D);
                        }
                        e->setParameterId(0, 0);
                        optimizer.addEdge(e);
                        vpEdgeSE3PointDyn.push_back(e);

                        // only in the case of dynamic and it's not the first feature in tracklet
                        // we save the dynamic point ID association.
                        // (6) save <EDGE_2POINTS_SE3MOTION>
                        g2o::LandmarkMotionTernaryEdge * em = new g2o::LandmarkMotionTernaryEdge();
                        em->setVertex(0, optimizer.vertex(FeaMakTmp));
                        em->setVertex(1, optimizer.vertex(count_unique_id));
                        em->setVertex(2, optimizer.vertex(ObjPositionID));
                        em->setMeasurement(Eigen::Vector3d(0,0,0));
                        em->information() = Eigen::Matrix3d::Identity()/sigma2_obj;
                        if (ROBUST_KERNEL)
                        {
                            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                            em->setRobustKernel(rk);
                            em->robustKernel()->setDelta(deltaHuberObjMot);
                        }
                        optimizer.addEdge(em);
                        vpEdgeLandmarkMotion.push_back(em);

                        // update unique id
                        vnFeaMakDyn[i][j] = count_unique_id;
                        count_unique_id++;
                    }
                }
            }
        }

        // cout << " (4) save dynamic features " << endl;
    }


    // start optimize
    optimizer.initializeOptimization();
    optimizer.setVerbose(true);

    bool check_before_opt=true, check_after_opt=true;
    if (check_before_opt)
    {
        // ****** check the chi2 error stats ******
        cout << endl << "(" << vpEdgeSE3.size() << ") " << "EdgeSE3 chi2: " << endl;
        for(size_t i=0, iend=vpEdgeSE3.size(); i<iend; i++)
        {
            g2o::EdgeSE3* e = vpEdgeSE3[i];
            e->computeError();
            const float chi2 = e->chi2();
            cout << chi2 << " ";
        }
        cout << endl;

        std::vector<int> range(12,0);
        cout << "(" << vpEdgeSE3PointSta.size() << ") " << "EdgeSE3PointSta chi2: " << endl;
        for(size_t i=0, iend=vpEdgeSE3PointSta.size(); i<iend; i++)
        {
            g2o::EdgeSE3PointXYZ* e = vpEdgeSE3PointSta[i];
            e->computeError();
            const float chi2 = e->chi2();
            {
                if (0.0<=chi2 && chi2<0.01)
                    range[0] = range[0] + 1;
                else if (0.01<=chi2 && chi2<0.02)
                    range[1] = range[1] + 1;
                else if (0.02<=chi2 && chi2<0.04)
                    range[2] = range[2] + 1;
                else if (0.04<=chi2 && chi2<0.08)
                    range[3] = range[3] + 1;
                else if (0.08<=chi2 && chi2<0.1)
                    range[4] = range[4] + 1;
                else if (0.1<=chi2 && chi2<0.2)
                    range[5] = range[5] + 1;
                else if (0.2<=chi2 && chi2<0.4)
                    range[6] = range[6] + 1;
                else if (0.4<=chi2 && chi2<0.8)
                    range[7] = range[7] + 1;
                else if (0.8<=chi2 && chi2<1.0)
                    range[8] = range[8] + 1;
                else if (1.0<=chi2 && chi2<5.0)
                    range[9] = range[9] + 1;
                else if (5.0<=chi2 && chi2<10.0)
                    range[10] = range[10] + 1;
                else if (chi2>=10.0)
                    range[11] = range[11] + 1;
            }
            // cout << chi2 << " ";
        }
        // cout << endl;
        for (int j = 0; j < range.size(); ++j)
            cout << range[j] << " ";
        cout << endl;

        if (STATIC_ONLY==false)
        {
            std::vector<int> range1(12,0);
            cout << "(" << vpEdgeLandmarkMotion.size() << ") " << "LandmarkMotionTernaryEdge chi2: " << endl;
            for(size_t i=0, iend=vpEdgeLandmarkMotion.size(); i<iend; i++)
            {
                g2o::LandmarkMotionTernaryEdge* e = vpEdgeLandmarkMotion[i];
                e->computeError();
                const float chi2 = e->chi2();
                {
                    if (0.0<=chi2 && chi2<0.01)
                        range1[0] = range1[0] + 1;
                    else if (0.01<=chi2 && chi2<0.02)
                        range1[1] = range1[1] + 1;
                    else if (0.02<=chi2 && chi2<0.04)
                        range1[2] = range1[2] + 1;
                    else if (0.04<=chi2 && chi2<0.08)
                        range1[3] = range1[3] + 1;
                    else if (0.08<=chi2 && chi2<0.1)
                        range1[4] = range1[4] + 1;
                    else if (0.1<=chi2 && chi2<0.2)
                        range1[5] = range1[5] + 1;
                    else if (0.2<=chi2 && chi2<0.4)
                        range1[6] = range1[6] + 1;
                    else if (0.4<=chi2 && chi2<0.8)
                        range1[7] = range1[7] + 1;
                    else if (0.8<=chi2 && chi2<1.0)
                        range1[8] = range1[8] + 1;
                    else if (1.0<=chi2 && chi2<5.0)
                        range1[9] = range1[9] + 1;
                    else if (5.0<=chi2 && chi2<10.0)
                        range1[10] = range1[10] + 1;
                    else if (chi2>=10.0)
                        range1[11] = range1[11] + 1;
                }
                // cout << chi2 << " ";
            }
            // cout << endl;
            for (int j = 0; j < range1.size(); ++j)
                cout << range1[j] << " ";
            cout << endl;

            std::vector<int> range2(12,0);
            cout << "(" << vpEdgeSE3PointDyn.size() << ") " << "EdgeSE3PointDyn chi2: " << endl;
            for(size_t i=0, iend=vpEdgeSE3PointDyn.size(); i<iend; i++)
            {
                g2o::EdgeSE3PointXYZ* e = vpEdgeSE3PointDyn[i];
                e->computeError();
                const float chi2 = e->chi2();
                {
                    if (0.0<=chi2 && chi2<0.01)
                        range2[0] = range2[0] + 1;
                    else if (0.01<=chi2 && chi2<0.02)
                        range2[1] = range2[1] + 1;
                    else if (0.02<=chi2 && chi2<0.04)
                        range2[2] = range2[2] + 1;
                    else if (0.04<=chi2 && chi2<0.08)
                        range2[3] = range2[3] + 1;
                    else if (0.08<=chi2 && chi2<0.1)
                        range2[4] = range2[4] + 1;
                    else if (0.1<=chi2 && chi2<0.2)
                        range2[5] = range2[5] + 1;
                    else if (0.2<=chi2 && chi2<0.4)
                        range2[6] = range2[6] + 1;
                    else if (0.4<=chi2 && chi2<0.8)
                        range2[7] = range2[7] + 1;
                    else if (0.8<=chi2 && chi2<1.0)
                        range2[8] = range2[8] + 1;
                    else if (1.0<=chi2 && chi2<5.0)
                        range2[9] = range2[9] + 1;
                    else if (5.0<=chi2 && chi2<10.0)
                        range2[10] = range2[10] + 1;
                    else if (chi2>=10.0)
                        range2[11] = range2[11] + 1;
                }
                // cout << chi2 << " ";
            }
            // cout << endl;
            for (int j = 0; j < range2.size(); ++j)
                cout << range2[j] << " ";
            cout << endl;

            if (ALTITUDE_CONSTRAINT)
            {
                cout << "(" << vpEdgeSE3Altitude.size() << ") " << "vpEdgeSE3Altitude chi2: " << endl;
                for(size_t i=0, iend=vpEdgeSE3Altitude.size(); i<iend; i++)
                {
                    g2o::EdgeSE3Altitude* ea = vpEdgeSE3Altitude[i];
                    ea->computeError();
                    const float chi2 = ea->chi2();
                    cout << chi2 << " ";
                }
                cout << endl;
            }

            if (SMOOTH_CONSTRAINT)
            {
                cout << "(" << vpEdgeSE3Smooth.size() << ") " << "vpEdgeSE3Smooth chi2: " << endl;
                for(size_t i=0, iend=vpEdgeSE3Smooth.size(); i<iend; i++)
                {
                    g2o::EdgeSE3* ea = vpEdgeSE3Smooth[i];
                    ea->computeError();
                    const float chi2 = ea->chi2();
                    cout << chi2 << " ";
                }
                cout << endl;
            }
        }
        cout << endl;
        // **********************************************
    }

    optimizer.save("local_ba_before.g2o");
    optimizer.optimize(100);
    optimizer.save("local_ba_after.g2o");

    if (check_after_opt)
    {
        // ****** check the chi2 error stats ******
        cout << endl << "(" << vpEdgeSE3.size() << ") " << "EdgeSE3 chi2: " << endl;
        for(size_t i=0, iend=vpEdgeSE3.size(); i<iend; i++)
        {
            g2o::EdgeSE3* e = vpEdgeSE3[i];
            const float chi2 = e->chi2();
            cout << chi2 << " ";
        }
        cout << endl;

        std::vector<int> range(12,0);
        cout << "(" << vpEdgeSE3PointSta.size() << ") " << "EdgeSE3PointSta chi2: " << endl;
        for(size_t i=0, iend=vpEdgeSE3PointSta.size(); i<iend; i++)
        {
            g2o::EdgeSE3PointXYZ* e = vpEdgeSE3PointSta[i];
            const float chi2 = e->chi2();
            {
                if (0.0<=chi2 && chi2<0.01)
                    range[0] = range[0] + 1;
                else if (0.01<=chi2 && chi2<0.02)
                    range[1] = range[1] + 1;
                else if (0.02<=chi2 && chi2<0.04)
                    range[2] = range[2] + 1;
                else if (0.04<=chi2 && chi2<0.08)
                    range[3] = range[3] + 1;
                else if (0.08<=chi2 && chi2<0.1)
                    range[4] = range[4] + 1;
                else if (0.1<=chi2 && chi2<0.2)
                    range[5] = range[5] + 1;
                else if (0.2<=chi2 && chi2<0.4)
                    range[6] = range[6] + 1;
                else if (0.4<=chi2 && chi2<0.8)
                    range[7] = range[7] + 1;
                else if (0.8<=chi2 && chi2<1.0)
                    range[8] = range[8] + 1;
                else if (1.0<=chi2 && chi2<5.0)
                    range[9] = range[9] + 1;
                else if (5.0<=chi2 && chi2<10.0)
                    range[10] = range[10] + 1;
                else if (chi2>=10.0)
                    range[11] = range[11] + 1;
            }
            // cout << chi2 << " ";
        }
        for (int j = 0; j < range.size(); ++j)
            cout << range[j] << " ";
        cout << endl;

        if (STATIC_ONLY==false)
        {
            std::vector<int> range1(12,0);
            cout << "(" << vpEdgeLandmarkMotion.size() << ") " << "LandmarkMotionTernaryEdge chi2: " << endl;
            for(size_t i=0, iend=vpEdgeLandmarkMotion.size(); i<iend; i++)
            {
                g2o::LandmarkMotionTernaryEdge* e = vpEdgeLandmarkMotion[i];
                e->computeError();
                const float chi2 = e->chi2();
                {
                    if (0.0<=chi2 && chi2<0.01)
                        range1[0] = range1[0] + 1;
                    else if (0.01<=chi2 && chi2<0.02)
                        range1[1] = range1[1] + 1;
                    else if (0.02<=chi2 && chi2<0.04)
                        range1[2] = range1[2] + 1;
                    else if (0.04<=chi2 && chi2<0.08)
                        range1[3] = range1[3] + 1;
                    else if (0.08<=chi2 && chi2<0.1)
                        range1[4] = range1[4] + 1;
                    else if (0.1<=chi2 && chi2<0.2)
                        range1[5] = range1[5] + 1;
                    else if (0.2<=chi2 && chi2<0.4)
                        range1[6] = range1[6] + 1;
                    else if (0.4<=chi2 && chi2<0.8)
                        range1[7] = range1[7] + 1;
                    else if (0.8<=chi2 && chi2<1.0)
                        range1[8] = range1[8] + 1;
                    else if (1.0<=chi2 && chi2<5.0)
                        range1[9] = range1[9] + 1;
                    else if (5.0<=chi2 && chi2<10.0)
                        range1[10] = range1[10] + 1;
                    else if (chi2>=10.0)
                        range1[11] = range1[11] + 1;
                }
                // cout << chi2 << " ";
            }
            // cout << endl;
            for (int j = 0; j < range1.size(); ++j)
                cout << range1[j] << " ";
            cout << endl;

            std::vector<int> range2(12,0);
            cout << "(" << vpEdgeSE3PointDyn.size() << ") " << "EdgeSE3PointDyn chi2: " << endl;
            for(size_t i=0, iend=vpEdgeSE3PointDyn.size(); i<iend; i++)
            {
                g2o::EdgeSE3PointXYZ* e = vpEdgeSE3PointDyn[i];
                e->computeError();
                const float chi2 = e->chi2();
                {
                    if (0.0<=chi2 && chi2<0.01)
                        range2[0] = range2[0] + 1;
                    else if (0.01<=chi2 && chi2<0.02)
                        range2[1] = range2[1] + 1;
                    else if (0.02<=chi2 && chi2<0.04)
                        range2[2] = range2[2] + 1;
                    else if (0.04<=chi2 && chi2<0.08)
                        range2[3] = range2[3] + 1;
                    else if (0.08<=chi2 && chi2<0.1)
                        range2[4] = range2[4] + 1;
                    else if (0.1<=chi2 && chi2<0.2)
                        range2[5] = range2[5] + 1;
                    else if (0.2<=chi2 && chi2<0.4)
                        range2[6] = range2[6] + 1;
                    else if (0.4<=chi2 && chi2<0.8)
                        range2[7] = range2[7] + 1;
                    else if (0.8<=chi2 && chi2<1.0)
                        range2[8] = range2[8] + 1;
                    else if (1.0<=chi2 && chi2<5.0)
                        range2[9] = range2[9] + 1;
                    else if (5.0<=chi2 && chi2<10.0)
                        range2[10] = range2[10] + 1;
                    else if (chi2>=10.0)
                        range2[11] = range2[11] + 1;
                }
                // cout << chi2 << " ";
            }
            // cout << endl;
            for (int j = 0; j < range2.size(); ++j)
                cout << range2[j] << " ";
            cout << endl;

            if (ALTITUDE_CONSTRAINT)
            {
                cout << "(" << vpEdgeSE3Altitude.size() << ") " << "vpEdgeSE3Altitude chi2: " << endl;
                for(size_t i=0, iend=vpEdgeSE3Altitude.size(); i<iend; i++)
                {
                    g2o::EdgeSE3Altitude* ea = vpEdgeSE3Altitude[i];
                    ea->computeError();
                    const float chi2 = ea->chi2();
                    cout << chi2 << " ";
                }
                cout << endl;
            }

            if (SMOOTH_CONSTRAINT)
            {
                cout << "(" << vpEdgeSE3Smooth.size() << ") " << "vpEdgeSE3Smooth chi2: " << endl;
                for(size_t i=0, iend=vpEdgeSE3Smooth.size(); i<iend; i++)
                {
                    g2o::EdgeSE3* ea = vpEdgeSE3Smooth[i];
                    ea->computeError();
                    const float chi2 = ea->chi2();
                    cout << chi2 << " ";
                }
                cout << endl;
            }
        }
        cout << endl;
        // **********************************************
    }

    bool show_result_before_opt=false, show_result_after_opt=false;
    if (show_result_before_opt)
    {
        cout << "Pose and Motion BEFORE Local BA ......" << endl;
        // absolute trajectory error for CAMERA (RMSE)
        cout << "=================================================" << endl;

        cout << "CAMERA:" << endl;
        float t_sum = 0, r_sum = 0;
        for (int i = StaticStartFrame; i < N; ++i)
        {
            // cv::Mat T_lc_inv = CamPose[i]*Converter::toInvMatrix(CamPose[i-1]);
            // cv::Mat T_lc_gt = CamPose_gt[i-1]*Converter::toInvMatrix(CamPose_gt[i]);
            // cv::Mat ate_cam = T_lc_inv*T_lc_gt;
            cv::Mat ate_cam = pMap->vmCameraPose[i]*Converter::toInvMatrix(pMap->vmCameraPose_GT[i]);

            // translation
            float t_ate_cam = std::sqrt(ate_cam.at<float>(0,3)*ate_cam.at<float>(0,3) + ate_cam.at<float>(1,3)*ate_cam.at<float>(1,3) + ate_cam.at<float>(2,3)*ate_cam.at<float>(2,3));
            t_sum = t_sum + t_ate_cam;

            // rotation
            float trace_ate = 0;
            for (int j = 0; j < 3; ++j)
            {
                if (ate_cam.at<float>(j,j)>1.0)
                    trace_ate = trace_ate + 1.0-(ate_cam.at<float>(j,j)-1.0);
                else
                    trace_ate = trace_ate + ate_cam.at<float>(j,j);
            }
            float r_ate_cam = acos( (trace_ate -1.0)/2.0 )*180.0/3.1415926;
            r_sum = r_sum + r_ate_cam;
            // cout << " t: " << t_ate_cam << " R: " << r_ate_cam << endl;
        }
        // t_mean = std::sqrt(t_sum/(CamPose.size()-1));
        t_sum = t_sum/(N-StaticStartFrame);
        r_sum = r_sum/(N-StaticStartFrame);
        cout << "average error (Camera):" << " t: " << t_sum << " R: " << r_sum << endl;

        if (STATIC_ONLY==false)
        {
            cout << "OBJECTS:" << endl;
            // all motion error for objects (mean error)
            float r_rpe_sum = 0, t_rpe_sum = 0, obj_count = 0;
            for (int i = N-WINDOW_SIZE; i < N-1; ++i)
            {
                if (pMap->vmRigidMotion[i].size()>1)
                {
                    for (int j = 1; j < pMap->vmRigidMotion[i].size(); ++j)
                    {
                        if (ObjCheck[i][j]==false)
                            continue;

                        cv::Mat rpe_obj = Converter::toInvMatrix(pMap->vmRigidMotion[i][j])*pMap->vmRigidMotion_GT[i][j];

                        // translation error
                        float t_rpe_obj = std::sqrt( rpe_obj.at<float>(0,3)*rpe_obj.at<float>(0,3) + rpe_obj.at<float>(1,3)*rpe_obj.at<float>(1,3) + rpe_obj.at<float>(2,3)*rpe_obj.at<float>(2,3) );
                        t_rpe_sum = t_rpe_sum + t_rpe_obj;

                        // rotation error
                        float trace_rpe = 0;
                        for (int k = 0; k < 3; ++k)
                        {
                            if (rpe_obj.at<float>(k,k)>1.0)
                                trace_rpe = trace_rpe + 1.0-(rpe_obj.at<float>(k,k)-1.0);
                            else
                                trace_rpe = trace_rpe + rpe_obj.at<float>(k,k);
                        }
                        float r_rpe_obj = acos( ( trace_rpe -1.0 )/2.0 )*180.0/3.1415926;
                        r_rpe_sum = r_rpe_sum + r_rpe_obj;

                        // cout << "(" << j << ")" << " t: " << t_rpe_obj << " R: " << r_rpe_obj << endl;
                        obj_count++;
                    }
                }
            }
            t_rpe_sum = t_rpe_sum/obj_count;
            r_rpe_sum = r_rpe_sum/obj_count;
            cout << "average error (Objects):" << " t: " << t_rpe_sum << " R: " << r_rpe_sum << endl;
        }
        cout << "=================================================" << endl << endl;
    }


    // *** save optimized motion and pose results ***
    cout << "UPDATE POSE and MOTION ......" << endl;
    // (1) camera
    for (int i = StaticStartFrame; i < N; ++i)
    {
        g2o::VertexSE3* vSE3 = static_cast<g2o::VertexSE3*>(optimizer.vertex(VertexID[i][0]));

        // convert
        double optimized[7];
        vSE3->getEstimateData(optimized);
        Eigen::Quaterniond q(optimized[6],optimized[3],optimized[4],optimized[5]);
        Eigen::Matrix<double,3,3> rot = q.matrix();
        Eigen::Matrix<double,3,1> tra;
        tra << optimized[0],optimized[1],optimized[2];

        // camera pose
        pMap->vmCameraPose[i] = Converter::toCvSE3(rot,tra);

        // camera motion
        if (i>StaticStartFrame)
        {
            pMap->vmRigidMotion[i-1][0] = Converter::toInvMatrix(pMap->vmCameraPose[i-1])*pMap->vmCameraPose[i];
        }
    }
    // (2) object
    for (int i = N-WINDOW_SIZE; i < N; ++i)
    {
        for (int j = 1; j < VertexID[i].size(); ++j)
        {
            if (STATIC_ONLY)
                continue;

            if (VertexID[i][j]==-1)
                continue;

            g2o::VertexSE3* vSE3 = static_cast<g2o::VertexSE3*>(optimizer.vertex(VertexID[i][j]));

            // convert
            double optimized[7];
            vSE3->getEstimateData(optimized);
            Eigen::Quaterniond q(optimized[6],optimized[3],optimized[4],optimized[5]);
            Eigen::Matrix<double,3,3> rot = q.matrix();
            Eigen::Matrix<double,3,1> tra;
            tra << optimized[0],optimized[1],optimized[2];

            // assign
            pMap->vmRigidMotion[i-1][j] = Converter::toCvSE3(rot,tra);
        }
    }


    // *** save optimized 3d point results ***
    cout << "UPDATE 3D POINTS ......" << endl << endl;
    // (1) static points
    for (int i = StaticStartFrame; i < N; ++i)
    {
        for (int j = 0; j < vnFeaMakSta[i].size(); ++j)
        {
            if (vnFeaMakSta[i][j]!=-1)
            {
                g2o::VertexPointXYZ* vPoint = static_cast<g2o::VertexPointXYZ*>(optimizer.vertex(vnFeaMakSta[i][j]));
                double optimized[3];
                vPoint->getEstimateData(optimized);
                Eigen::Matrix<double,3,1> tmp_3d;
                tmp_3d << optimized[0],optimized[1],optimized[2];
                pMap->vp3DPointSta[i][j] = Converter::toCvMat(tmp_3d);
            }
        }
    }
    // (2) dynamic points
    for (int i = N-WINDOW_SIZE; i < N; ++i)
    {
        if (STATIC_ONLY==false)
        {
            for (int j = 0; j < vnFeaMakDyn[i].size(); ++j)
            {
                if (vnFeaMakDyn[i][j]!=-1)
                {
                    g2o::VertexPointXYZ* vPoint = static_cast<g2o::VertexPointXYZ*>(optimizer.vertex(vnFeaMakDyn[i][j]));
                    double optimized[3];
                    vPoint->getEstimateData(optimized);
                    Eigen::Matrix<double,3,1> tmp_3d;
                    tmp_3d << optimized[0],optimized[1],optimized[2];
                    // cout << "dynamic before: " << pMap->vp3DPointDyn[i][j] << endl;
                    // cout << "dynamic after: " << tmp_3d << endl;
                    pMap->vp3DPointDyn[i][j] = Converter::toCvMat(tmp_3d);
                }
            }
        }
    }

    if (show_result_after_opt)
    {
        cout << "Pose and Motion AFTER Local BA ......" << endl;
        // absolute trajectory error for CAMERA (RMSE)
        cout << "=================================================" << endl;

        cout << "CAMERA:" << endl;
        float t_sum = 0, r_sum = 0;
        for (int i = StaticStartFrame; i < N; ++i)
        {
            // cv::Mat T_lc_inv = CamPose[i]*Converter::toInvMatrix(CamPose[i-1]);
            // cv::Mat T_lc_gt = CamPose_gt[i-1]*Converter::toInvMatrix(CamPose_gt[i]);
            // cv::Mat ate_cam = T_lc_inv*T_lc_gt;
            cv::Mat ate_cam = pMap->vmCameraPose[i]*Converter::toInvMatrix(pMap->vmCameraPose_GT[i]);

            // translation
            float t_ate_cam = std::sqrt(ate_cam.at<float>(0,3)*ate_cam.at<float>(0,3) + ate_cam.at<float>(1,3)*ate_cam.at<float>(1,3) + ate_cam.at<float>(2,3)*ate_cam.at<float>(2,3));
            t_sum = t_sum + t_ate_cam;

            // rotation
            float trace_ate = 0;
            for (int j = 0; j < 3; ++j)
            {
                if (ate_cam.at<float>(j,j)>1.0)
                    trace_ate = trace_ate + 1.0-(ate_cam.at<float>(j,j)-1.0);
                else
                    trace_ate = trace_ate + ate_cam.at<float>(j,j);
            }
            float r_ate_cam = acos( (trace_ate -1.0)/2.0 )*180.0/3.1415926;
            r_sum = r_sum + r_ate_cam;
            // cout << " t: " << t_ate_cam << " R: " << r_ate_cam << endl;
        }
        // t_mean = std::sqrt(t_sum/(CamPose.size()-1));
        t_sum = t_sum/(N-StaticStartFrame);
        r_sum = r_sum/(N-StaticStartFrame);
        cout << "average error (Camera):" << " t: " << t_sum << " R: " << r_sum << endl;

        if (STATIC_ONLY==false)
        {
            cout << "OBJECTS:" << endl;
            // all motion error for objects (mean error)
            float r_rpe_sum = 0, t_rpe_sum = 0, obj_count = 0;
            for (int i = N-WINDOW_SIZE; i < N-1; ++i)
            {
                if (pMap->vmRigidMotion[i].size()>1)
                {
                    for (int j = 1; j < pMap->vmRigidMotion[i].size(); ++j)
                    {
                        if (ObjCheck[i][j]==false)
                            continue;

                        cv::Mat rpe_obj = Converter::toInvMatrix(pMap->vmRigidMotion[i][j])*pMap->vmRigidMotion_GT[i][j];

                        // translation error
                        float t_rpe_obj = std::sqrt( rpe_obj.at<float>(0,3)*rpe_obj.at<float>(0,3) + rpe_obj.at<float>(1,3)*rpe_obj.at<float>(1,3) + rpe_obj.at<float>(2,3)*rpe_obj.at<float>(2,3) );
                        t_rpe_sum = t_rpe_sum + t_rpe_obj;

                        // rotation error
                        float trace_rpe = 0;
                        for (int k = 0; k < 3; ++k)
                        {
                            if (rpe_obj.at<float>(k,k)>1.0)
                                trace_rpe = trace_rpe + 1.0-(rpe_obj.at<float>(k,k)-1.0);
                            else
                                trace_rpe = trace_rpe + rpe_obj.at<float>(k,k);
                        }
                        float r_rpe_obj = acos( ( trace_rpe -1.0 )/2.0 )*180.0/3.1415926;
                        r_rpe_sum = r_rpe_sum + r_rpe_obj;

                        // cout << "(" << j << ")" << " t: " << t_rpe_obj << " R: " << r_rpe_obj << endl;
                        obj_count++;
                    }
                }
            }
            t_rpe_sum = t_rpe_sum/obj_count;
            r_rpe_sum = r_rpe_sum/obj_count;
            cout << "average error (Objects):" << " t: " << t_rpe_sum << " R: " << r_rpe_sum << endl;
        }
        cout << "=================================================" << endl << endl;
    }

    // =========================================================================================================
    // ==================================== GET METRIC ERROR ===================================================
    // =========================================================================================================
}

void Optimizer::FullBatchOptimization(Map* pMap, const cv::Mat Calib_K)
{
    const int N = pMap->vpFeatSta.size(); // Number of Frames
    std::vector<std::vector<std::pair<int, int> > > StaTracks = pMap->TrackletSta;
    std::vector<std::vector<std::pair<int, int> > > DynTracks = pMap->TrackletDyn;

    // // show feature mark index
    // int count = 0;
    // for (int i = 0; i < DynTracks.size(); ++i)
    // {
    //     if (DynTracks[i].size()==2)
    //         continue;
    //     count = count + DynTracks[i].size();
    //     for (int j = 0; j < DynTracks[i].size(); ++j)
    //     {
    //         cout << DynTracks[i][j].first << " " << DynTracks[i][j].second << " / ";
    //     }
    //     cout << endl;
    // }

    // =======================================================================================

    // mark each feature if it is satisfied (valid) for usage
    // here we use track length as threshold, for static >=3, dynamic >=3.
    // label each feature of the position in TrackLets: -1(invalid) or >=0(TrackID);
    // size: static: (N)xM_1, M_1 is the size of features in each frame
    // size: dynamic: (N)xM_2, M_2 is the size of features in each frame
    std::vector<std::vector<int> > vnFeaLabSta(N),vnFeaMakSta(N),vnFeaLabDyn(N),vnFeaMakDyn(N);
    // initialize
    for (int i = 0; i < N; ++i)
    {
        std::vector<int>  vnFLS_tmp(pMap->vpFeatSta[i].size(),-1);
        vnFeaLabSta[i] = vnFLS_tmp;
        vnFeaMakSta[i] = vnFLS_tmp;
    }
    for (int i = 0; i < N; ++i)
    {
        std::vector<int>  vnFLD_tmp(pMap->vpFeatDyn[i].size(),-1);
        vnFeaLabDyn[i] = vnFLD_tmp;
        vnFeaMakDyn[i] = vnFLD_tmp;
    }
    int valid_sta = 0, valid_dyn = 0;
    // label static feature
    for (int i = 0; i < StaTracks.size(); ++i)
    {
        // filter the tracklets via threshold
        if (StaTracks[i].size()<3) // 3 the length of track on background.
            continue;
        valid_sta++;
        // label them
        for (int j = 0; j < StaTracks[i].size(); ++j)
            vnFeaLabSta[StaTracks[i][j].first][StaTracks[i][j].second] = i;
    }
    // label dynamic feature
    for (int i = 0; i < DynTracks.size(); ++i)
    {
        // filter the tracklets via threshold
        if (DynTracks[i].size()<3) // 3 the length of track on objects.
            continue;
        valid_dyn++;
        // label them
        for (int j = 0; j < DynTracks[i].size(); ++j){
            vnFeaLabDyn[DynTracks[i][j].first][DynTracks[i][j].second] = i;

        }
    }

    // cout << "Valid Static and Dynamic Tracks:((( " << valid_sta << " / " << valid_dyn << " )))" << endl << endl;

    // save vertex ID (camera pose and object motion) in the graph
    std::vector<std::vector<int> > VertexID(N-1);
    // initialize
    for (int i = 0; i < N-1; ++i)
    {
        std::vector<int> v_id_tmp(pMap->vnRMLabel[i].size(),-1);
        VertexID[i] = v_id_tmp;
    }

    // =======================================================================================

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;
    linearSolver = new g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);
    // g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg(solver_ptr);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
    optimizer.setAlgorithm(solver);

    g2o::SparseOptimizerTerminateAction* terminateAction = new g2o::SparseOptimizerTerminateAction;
    terminateAction->setGainThreshold(1e-4);
    optimizer.addPostIterationAction(terminateAction);

    g2o::ParameterSE3Offset* cameraOffset = new g2o::ParameterSE3Offset;
    cameraOffset->setId(0);
    optimizer.addParameter(cameraOffset);

    // === set information matrix ===
    const float sigma2_cam = 0.001; // 0.005 0.001 (ox:) 0.0001
    const float sigma2_3d_sta = 80; // 50 (ox:) 80 16
    const float sigma2_obj_smo = 0.001; // 0.1 0.5 (ox:) 0.001
    const float sigma2_obj = 100; // 0.5 1 10 20 50 (ox:) 100
    const float sigma2_3d_dyn = 80; // 50 100 16 (ox:) 80
    const float sigma2_alti = 0.1;

    // === identity initialization ===
    cv::Mat IDENTITY_TMP = cv::Mat::eye(4,4, CV_32F);

    vector<g2o::EdgeSE3*> vpEdgeSE3;
    vector<g2o::LandmarkMotionTernaryEdge*> vpEdgeLandmarkMotion;
    vector<g2o::EdgeSE3PointXYZ*> vpEdgeSE3PointSta;
    vector<g2o::EdgeSE3PointXYZ*> vpEdgeSE3PointDyn;
    vector<g2o::EdgeSE3Altitude*> vpEdgeSE3Altitude;
    vector<g2o::EdgeSE3*> vpEdgeSE3Smooth;

    // ---------------------------------------------------------------------------------------
    // ---------=============!!!=- Main Loop for input data -=!!!=============----------------
    // ---------------------------------------------------------------------------------------
    int count_unique_id = 1;
    bool ROBUST_KERNEL = true, ALTITUDE_CONSTRAINT = false, SMOOTH_CONSTRAINT = true, STATIC_ONLY = false;
    float deltaHuberCamMot = 0.0001, deltaHuberObjMot = 0.0001, deltaHuber3D = 0.0001;
    int PreFrameID;
    for (int i = 0; i < N; ++i)
    {
        // cout << "current processing frame: " << i << endl;

        // (1) save <VERTEX_POSE_R3_SO3>
        g2o::VertexSE3 *v_se3 = new g2o::VertexSE3();
        v_se3->setId(count_unique_id);
        v_se3->setEstimate(Converter::toSE3Quat(pMap->vmCameraPose[i]));
        // v_se3->setEstimate(Converter::toSE3Quat(IDENTITY_TMP));
        optimizer.addVertex(v_se3);
        if (count_unique_id==1)
        {
            // add prior edges
            g2o::EdgeSE3Prior * pose_prior = new g2o::EdgeSE3Prior();
            pose_prior->setVertex(0, optimizer.vertex(count_unique_id));
            pose_prior->setMeasurement(Converter::toSE3Quat(pMap->vmCameraPose[i]));
            pose_prior->information() = Eigen::MatrixXd::Identity(6, 6)*100000;
            pose_prior->setParameterId(0, 0);
            optimizer.addEdge(pose_prior);
        }
        if (i!=0)
            VertexID[i-1][0] = count_unique_id;
        // record the ID of current frame saved in graph file
        int CurFrameID = count_unique_id;
        count_unique_id++;

        // cout << " (0) save camera pose " << endl;

        // ****** save camera motion if it is not the first frame ******
        if (i!=0)
        {
            // (2) save <EDGE_R3_SO3>
            g2o::EdgeSE3 * ep = new g2o::EdgeSE3();
            ep->setVertex(0, optimizer.vertex(PreFrameID));
            ep->setVertex(1, optimizer.vertex(CurFrameID));
            ep->setMeasurement(Converter::toSE3Quat(pMap->vmRigidMotion[i-1][0]));
            ep->information() = Eigen::MatrixXd::Identity(6, 6)/sigma2_cam;
            if (ROBUST_KERNEL){
                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                ep->setRobustKernel(rk);
                ep->robustKernel()->setDelta(deltaHuberCamMot);
            }
            optimizer.addEdge(ep);
            vpEdgeSE3.push_back(ep);
            // cout << " (1) save camera motion " << endl;
        }


        // **************** save for static features *****************
        // **************** For frame i=0, save directly ***************
        if (i==0)
        {
            // loop for static features
            for (int j = 0; j < vnFeaLabSta[i].size(); ++j)
            {
                // check feature validation
                if (vnFeaLabSta[i][j]==-1)
                    continue;

                // (3) save <VERTEX_POINT_3D>
                g2o::VertexPointXYZ *v_p = new g2o::VertexPointXYZ();
                v_p->setId(count_unique_id);
                cv::Mat Xw = pMap->vp3DPointSta[i][j];
                v_p->setEstimate(Converter::toVector3d(Xw));
                optimizer.addVertex(v_p);

                // (4) save <EDGE_3D>
                g2o::EdgeSE3PointXYZ * e = new g2o::EdgeSE3PointXYZ();
                e->setVertex(0, optimizer.vertex(CurFrameID));
                e->setVertex(1, optimizer.vertex(count_unique_id));
                cv::Mat Xc = Optimizer::Get3DinCamera(pMap->vpFeatSta[i][j],pMap->vfDepSta[i][j],Calib_K);
                e->setMeasurement(Converter::toVector3d(Xc));
                e->information() = Eigen::Matrix3d::Identity()/sigma2_3d_sta;
                if (ROBUST_KERNEL){
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    e->robustKernel()->setDelta(deltaHuber3D);
                }
                e->setParameterId(0, 0);
                optimizer.addEdge(e);
                vpEdgeSE3PointSta.push_back(e);

                // update unique id
                vnFeaMakSta[i][j] = count_unique_id;
                count_unique_id++;
            }
        }
        // ****************** For frame i>0  *********************
        else
        {
            // loop for static features
            for (int j = 0; j < vnFeaLabSta[i].size(); ++j)
            {
                // check feature validation
                if (vnFeaLabSta[i][j]==-1)
                    continue;

                // get the TrackID of current feature
                int TrackID = vnFeaLabSta[i][j];

                // get the position of current feature in the tracklet
                int PositionID = -1;
                for (int k = 0; k < StaTracks[TrackID].size(); ++k)
                {
                    if (StaTracks[TrackID][k].first==i && StaTracks[TrackID][k].second==j)
                    {
                        PositionID = k;
                        break;
                    }
                }
                if (PositionID==-1){
                    cout << "cannot find the position of current feature in the tracklet !!!" << endl;
                    continue;
                }

                // check if the PositionID is 0. Yes means this static point is first seen by this frame,
                // then save both the vertex and edge, otherwise save edge only because vertex is saved before.
                if (PositionID==0)
                {
                    // (3) save <VERTEX_POINT_3D>
                    g2o::VertexPointXYZ *v_p = new g2o::VertexPointXYZ();
                    v_p->setId(count_unique_id);
                    cv::Mat Xw = pMap->vp3DPointSta[i][j];
                    v_p->setEstimate(Converter::toVector3d(Xw));
                    // v_p->setFixed(true);
                    optimizer.addVertex(v_p);
                    // (4) save <EDGE_3D>
                    g2o::EdgeSE3PointXYZ * e = new g2o::EdgeSE3PointXYZ();
                    e->setVertex(0, optimizer.vertex(CurFrameID));
                    e->setVertex(1, optimizer.vertex(count_unique_id));
                    cv::Mat Xc = Optimizer::Get3DinCamera(pMap->vpFeatSta[i][j],pMap->vfDepSta[i][j],Calib_K);
                    e->setMeasurement(Converter::toVector3d(Xc));
                    e->information() = Eigen::Matrix3d::Identity()/sigma2_3d_sta;
                    if (ROBUST_KERNEL){
                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        e->robustKernel()->setDelta(deltaHuber3D);
                    }
                    e->setParameterId(0, 0);
                    optimizer.addEdge(e);
                    vpEdgeSE3PointSta.push_back(e);

                    // update unique id
                    vnFeaMakSta[i][j] = count_unique_id;
                    count_unique_id++;
                }
                else
                {
                    int FeaMakTmp = vnFeaMakSta[StaTracks[TrackID][PositionID-1].first][StaTracks[TrackID][PositionID-1].second];

                    // (4) save <EDGE_3D>
                    g2o::EdgeSE3PointXYZ * e = new g2o::EdgeSE3PointXYZ();
                    e->setVertex(0, optimizer.vertex(CurFrameID));
                    e->setVertex(1, optimizer.vertex(FeaMakTmp));
                    cv::Mat Xc = Optimizer::Get3DinCamera(pMap->vpFeatSta[i][j],pMap->vfDepSta[i][j],Calib_K);
                    e->setMeasurement(Converter::toVector3d(Xc));
                    e->information() = Eigen::Matrix3d::Identity()/sigma2_3d_sta;
                    if (ROBUST_KERNEL){
                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        e->robustKernel()->setDelta(deltaHuber3D);
                    }
                    e->setParameterId(0, 0);
                    optimizer.addEdge(e);
                    vpEdgeSE3PointSta.push_back(e);

                    // update unique id
                    vnFeaMakSta[i][j] = FeaMakTmp;
                }
            }
        }

        // cout << " (2) save static features " << endl;

        // // // ************** save object motion, then dynamic features *************
        if (!STATIC_ONLY)
        {
            if (i==0)
            {
                // loop for dynamic features
                for (int j = 0; j < vnFeaLabDyn[i].size(); ++j)
                {
                    // check feature validation
                    if (vnFeaLabDyn[i][j]==-1)
                        continue;

                    // (3) save <VERTEX_POINT_3D>
                    g2o::VertexPointXYZ *v_p = new g2o::VertexPointXYZ();
                    v_p->setId(count_unique_id);
                    cv::Mat Xw = pMap->vp3DPointDyn[i][j];
                    v_p->setEstimate(Converter::toVector3d(Xw));
                    optimizer.addVertex(v_p);
                    // (4) save <EDGE_3D>
                    g2o::EdgeSE3PointXYZ * e = new g2o::EdgeSE3PointXYZ();
                    e->setVertex(0, optimizer.vertex(CurFrameID));
                    e->setVertex(1, optimizer.vertex(count_unique_id));
                    cv::Mat Xc = Optimizer::Get3DinCamera(pMap->vpFeatDyn[i][j],pMap->vfDepDyn[i][j],Calib_K);
                    e->setMeasurement(Converter::toVector3d(Xc));
                    e->information() = Eigen::Matrix3d::Identity()/sigma2_3d_dyn;
                    if (ROBUST_KERNEL){
                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        e->robustKernel()->setDelta(deltaHuber3D);
                    }
                    e->setParameterId(0, 0);
                    optimizer.addEdge(e);
                    vpEdgeSE3PointDyn.push_back(e);

                    // update unique id
                    vnFeaMakDyn[i][j] = count_unique_id;
                    count_unique_id++;
                }
                // cout << "SAVE SOME DYNAMIC POINTS IN FIRST FRAME ....." << endl;
            }
            else
            {
                // loop for object motion, and keep the unique vertex id for saving object feature edges
                std::vector<int> ObjUniqueID(pMap->vmRigidMotion[i-1].size()-1,-1);
                // (5) save <VERTEX_SE3Motion>
                for (int j = 1; j < pMap->vmRigidMotion[i-1].size(); ++j)
                {
                    g2o::VertexSE3 *m_se3 = new g2o::VertexSE3();
                    m_se3->setId(count_unique_id);
                    // if (pMap->vbObjStat[i-1][j])
                    //     m_se3->setEstimate(Converter::toSE3Quat(pMap->vmRigidMotion[i-1][j]));
                    // else
                    //     m_se3->setEstimate(Converter::toSE3Quat(IDENTITY_TMP));
                    m_se3->setEstimate(Converter::toSE3Quat(IDENTITY_TMP));
                    optimizer.addVertex(m_se3);
                    if (ALTITUDE_CONSTRAINT)
                    {
                        g2o::EdgeSE3Altitude * ea = new g2o::EdgeSE3Altitude();
                        ea->setVertex(0, optimizer.vertex(count_unique_id));
                        ea->setMeasurement(0);
                        Eigen::Matrix<double, 1, 1> altitude_information(1.0/sigma2_alti);
                        ea->information() = altitude_information;
                        optimizer.addEdge(ea);
                        vpEdgeSE3Altitude.push_back(ea);
                    }
                    if (SMOOTH_CONSTRAINT && i>2)
                    {
                        // trace back the previous id in vnRMLabel
                        int TraceID = -1;
                        for (int k = 0; k < pMap->vnRMLabel[i-2].size(); ++k)
                        {
                            if (pMap->vnRMLabel[i-2][k]==pMap->vnRMLabel[i-1][j])
                            {
                                // cout << "what is in the label: " << pMap->vnRMLabel[i-2][k] << " " << pMap->vnRMLabel[i-1][j] << " " << VertexID[i-2][k] << endl;
                                TraceID = k;
                                break;
                            }
                        }
                        // only if the back trace exist
                        if (TraceID!=-1)
                        {
                            // add smooth constraint
                            g2o::EdgeSE3 * ep = new g2o::EdgeSE3();
                            ep->setVertex(0, optimizer.vertex(VertexID[i-2][TraceID]));
                            ep->setVertex(1, optimizer.vertex(count_unique_id));
                            ep->setMeasurement(Converter::toSE3Quat(cv::Mat::eye(4,4,CV_32F)));
                            ep->information() = Eigen::MatrixXd::Identity(6, 6)/sigma2_obj_smo;
                            if (ROBUST_KERNEL){
                                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                                ep->setRobustKernel(rk);
                                ep->robustKernel()->setDelta(deltaHuberCamMot);
                            }
                            optimizer.addEdge(ep);
                            vpEdgeSE3Smooth.push_back(ep);
                        }
                    }
                    ObjUniqueID[j-1]=count_unique_id;
                    VertexID[i-1][j]=count_unique_id;
                    count_unique_id++;
                }

                // cout << " (3) save object motion " << endl;

                // // save for dynamic features
                for (int j = 0; j < vnFeaLabDyn[i].size(); j++)
                {
                    // check feature validation
                    if (vnFeaLabDyn[i][j]==-1)
                        continue;

                    // get the TrackID of current feature
                    int TrackID = vnFeaLabDyn[i][j];

                    // get the position of current feature in the tracklet
                    int PositionID = -1;
                    for (int k = 0; k < DynTracks[TrackID].size(); ++k)
                    {
                        if (DynTracks[TrackID][k].first==i && DynTracks[TrackID][k].second==j)
                        {
                            PositionID = k;
                            break;
                        }
                    }
                    if (PositionID==-1){
                        // cout << "cannot find the position of current feature in the tracklet !!!" << endl;
                        continue;
                    }

                    // get the object position id of current feature
                    int ObjPositionID = -1;
                    for (int k = 1; k < pMap->vnRMLabel[i-1].size(); ++k)
                    {
                        if (pMap->vnRMLabel[i-1][k]==pMap->nObjID[TrackID]){
                            ObjPositionID = ObjUniqueID[k-1];
                            break;
                        }
                    }
                    if (ObjPositionID==-1 && PositionID!=0){
                        // cout << "cannot find the object association with this edge !!! WEIRD POINT !!! " << endl;
                        continue;
                    }

                    // check if the PositionID is 0. Yes means this dynamic point is first seen by this frame,
                    // then save both the vertex and edge, otherwise save edge only because vertex is saved before.
                    if (PositionID==0)
                    {
                        // (3) save <VERTEX_POINT_3D>
                        g2o::VertexPointXYZ *v_p = new g2o::VertexPointXYZ();
                        v_p->setId(count_unique_id);
                        cv::Mat Xw = pMap->vp3DPointDyn[i][j];
                        v_p->setEstimate(Converter::toVector3d(Xw));
                        optimizer.addVertex(v_p);
                        // (4) save <EDGE_3D>
                        g2o::EdgeSE3PointXYZ * e = new g2o::EdgeSE3PointXYZ();
                        e->setVertex(0, optimizer.vertex(CurFrameID));
                        e->setVertex(1, optimizer.vertex(count_unique_id));
                        cv::Mat Xc = Optimizer::Get3DinCamera(pMap->vpFeatDyn[i][j],pMap->vfDepDyn[i][j],Calib_K);
                        e->setMeasurement(Converter::toVector3d(Xc));
                        e->information() = Eigen::Matrix3d::Identity()/sigma2_3d_dyn;
                        if (ROBUST_KERNEL){
                            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                            e->setRobustKernel(rk);
                            e->robustKernel()->setDelta(deltaHuber3D);
                        }
                        e->setParameterId(0, 0);
                        optimizer.addEdge(e);
                        vpEdgeSE3PointDyn.push_back(e);

                        // update unique id
                        vnFeaMakDyn[i][j] = count_unique_id;
                        count_unique_id++;
                    }
                    // if no, then only add this feature to the existing track it belongs to.
                    else
                    {
                        // (3) save <VERTEX_POINT_3D>
                        g2o::VertexPointXYZ *v_p = new g2o::VertexPointXYZ();
                        v_p->setId(count_unique_id);
                        cv::Mat Xw = pMap->vp3DPointDyn[i][j];
                        v_p->setEstimate(Converter::toVector3d(Xw));
                        optimizer.addVertex(v_p);
                        // (4) save <EDGE_3D>
                        g2o::EdgeSE3PointXYZ * e = new g2o::EdgeSE3PointXYZ();
                        e->setVertex(0, optimizer.vertex(CurFrameID));
                        e->setVertex(1, optimizer.vertex(count_unique_id));
                        cv::Mat Xc = Optimizer::Get3DinCamera(pMap->vpFeatDyn[i][j],pMap->vfDepDyn[i][j],Calib_K);
                        e->setMeasurement(Converter::toVector3d(Xc));
                        e->information() = Eigen::Matrix3d::Identity()/sigma2_3d_dyn;
                        if (ROBUST_KERNEL){
                            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                            e->setRobustKernel(rk);
                            e->robustKernel()->setDelta(deltaHuber3D);
                        }
                        e->setParameterId(0, 0);
                        optimizer.addEdge(e);
                        vpEdgeSE3PointDyn.push_back(e);

                        // only in the case of dynamic and it's not the first feature in tracklet
                        // we save the dynamic point ID association.
                        int FeaMakTmp = vnFeaMakDyn[DynTracks[TrackID][PositionID-1].first][DynTracks[TrackID][PositionID-1].second];
                        // (6) save <EDGE_2POINTS_SE3MOTION>
                        g2o::LandmarkMotionTernaryEdge * em = new g2o::LandmarkMotionTernaryEdge();
                        em->setVertex(0, optimizer.vertex(FeaMakTmp));
                        em->setVertex(1, optimizer.vertex(count_unique_id));
                        em->setVertex(2, optimizer.vertex(ObjPositionID));
                        em->setMeasurement(Eigen::Vector3d(0,0,0));
                        em->information() = Eigen::Matrix3d::Identity()/sigma2_obj;
                        if (ROBUST_KERNEL){
                            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                            em->setRobustKernel(rk);
                            em->robustKernel()->setDelta(deltaHuberObjMot);
                        }
                        optimizer.addEdge(em);
                        vpEdgeLandmarkMotion.push_back(em);

                        // update unique id
                        vnFeaMakDyn[i][j] = count_unique_id;
                        count_unique_id++;
                    }
                }
            }
        }

        // cout << " (4) save dynamic features " << endl;

        // update frame ID
        PreFrameID = CurFrameID;
    }

    // // show feature mark index
    // for (int i = 0; i < StaTracks.size(); ++i)
    // {
    //     for (int j = 0; j < StaTracks[i].size(); ++j)
    //     {
    //         cout << vnFeaMakSta[StaTracks[i][j].first][StaTracks[i][j].second] << " ";
    //     }
    //     cout << endl;
    // }

    // start optimize
    optimizer.initializeOptimization();
    optimizer.setVerbose(true);

    bool check_before_opt=true, check_after_opt=true;
    if (check_before_opt)
    {
        // ****** check the chi2 error stats ******
        cout << endl << "(" << vpEdgeSE3.size() << ") " << "EdgeSE3 chi2: " << endl;
        for(size_t i=0, iend=vpEdgeSE3.size(); i<iend; i++)
        {
            g2o::EdgeSE3* e = vpEdgeSE3[i];
            e->computeError();
            const float chi2 = e->chi2();
            cout << chi2 << " ";
        }
        cout << endl;

        std::vector<int> range(12,0);
        cout << "(" << vpEdgeSE3PointSta.size() << ") " << "EdgeSE3PointSta chi2: " << endl;
        for(size_t i=0, iend=vpEdgeSE3PointSta.size(); i<iend; i++)
        {
            g2o::EdgeSE3PointXYZ* e = vpEdgeSE3PointSta[i];
            e->computeError();
            const float chi2 = e->chi2();
            {
                if (0.0<=chi2 && chi2<0.01)
                    range[0] = range[0] + 1;
                else if (0.01<=chi2 && chi2<0.02)
                    range[1] = range[1] + 1;
                else if (0.02<=chi2 && chi2<0.04)
                    range[2] = range[2] + 1;
                else if (0.04<=chi2 && chi2<0.08)
                    range[3] = range[3] + 1;
                else if (0.08<=chi2 && chi2<0.1)
                    range[4] = range[4] + 1;
                else if (0.1<=chi2 && chi2<0.2)
                    range[5] = range[5] + 1;
                else if (0.2<=chi2 && chi2<0.4)
                    range[6] = range[6] + 1;
                else if (0.4<=chi2 && chi2<0.8)
                    range[7] = range[7] + 1;
                else if (0.8<=chi2 && chi2<1.0)
                    range[8] = range[8] + 1;
                else if (1.0<=chi2 && chi2<5.0)
                    range[9] = range[9] + 1;
                else if (5.0<=chi2 && chi2<10.0)
                    range[10] = range[10] + 1;
                else if (chi2>=10.0)
                    range[11] = range[11] + 1;
            }
            // cout << chi2 << " ";
        }
        // cout << endl;
        for (int j = 0; j < range.size(); ++j)
            cout << range[j] << " ";
        cout << endl;

        std::vector<int> range1(12,0);
        cout << "(" << vpEdgeLandmarkMotion.size() << ") " << "LandmarkMotionTernaryEdge chi2: " << endl;
        for(size_t i=0, iend=vpEdgeLandmarkMotion.size(); i<iend; i++)
        {
            g2o::LandmarkMotionTernaryEdge* e = vpEdgeLandmarkMotion[i];
            e->computeError();
            const float chi2 = e->chi2();
            {
                if (0.0<=chi2 && chi2<0.01)
                    range1[0] = range1[0] + 1;
                else if (0.01<=chi2 && chi2<0.02)
                    range1[1] = range1[1] + 1;
                else if (0.02<=chi2 && chi2<0.04)
                    range1[2] = range1[2] + 1;
                else if (0.04<=chi2 && chi2<0.08)
                    range1[3] = range1[3] + 1;
                else if (0.08<=chi2 && chi2<0.1)
                    range1[4] = range1[4] + 1;
                else if (0.1<=chi2 && chi2<0.2)
                    range1[5] = range1[5] + 1;
                else if (0.2<=chi2 && chi2<0.4)
                    range1[6] = range1[6] + 1;
                else if (0.4<=chi2 && chi2<0.8)
                    range1[7] = range1[7] + 1;
                else if (0.8<=chi2 && chi2<1.0)
                    range1[8] = range1[8] + 1;
                else if (1.0<=chi2 && chi2<5.0)
                    range1[9] = range1[9] + 1;
                else if (5.0<=chi2 && chi2<10.0)
                    range1[10] = range1[10] + 1;
                else if (chi2>=10.0)
                    range1[11] = range1[11] + 1;
            }
            // cout << chi2 << " ";
        }
        // cout << endl;
        for (int j = 0; j < range1.size(); ++j)
            cout << range1[j] << " ";
        cout << endl;

        std::vector<int> range2(12,0);
        cout << "(" << vpEdgeSE3PointDyn.size() << ") " << "EdgeSE3PointDyn chi2: " << endl;
        for(size_t i=0, iend=vpEdgeSE3PointDyn.size(); i<iend; i++)
        {
            g2o::EdgeSE3PointXYZ* e = vpEdgeSE3PointDyn[i];
            e->computeError();
            const float chi2 = e->chi2();
            {
                if (0.0<=chi2 && chi2<0.01)
                    range2[0] = range2[0] + 1;
                else if (0.01<=chi2 && chi2<0.02)
                    range2[1] = range2[1] + 1;
                else if (0.02<=chi2 && chi2<0.04)
                    range2[2] = range2[2] + 1;
                else if (0.04<=chi2 && chi2<0.08)
                    range2[3] = range2[3] + 1;
                else if (0.08<=chi2 && chi2<0.1)
                    range2[4] = range2[4] + 1;
                else if (0.1<=chi2 && chi2<0.2)
                    range2[5] = range2[5] + 1;
                else if (0.2<=chi2 && chi2<0.4)
                    range2[6] = range2[6] + 1;
                else if (0.4<=chi2 && chi2<0.8)
                    range2[7] = range2[7] + 1;
                else if (0.8<=chi2 && chi2<1.0)
                    range2[8] = range2[8] + 1;
                else if (1.0<=chi2 && chi2<5.0)
                    range2[9] = range2[9] + 1;
                else if (5.0<=chi2 && chi2<10.0)
                    range2[10] = range2[10] + 1;
                else if (chi2>=10.0)
                    range2[11] = range2[11] + 1;
            }
            // cout << chi2 << " ";
        }
        // cout << endl;
        for (int j = 0; j < range2.size(); ++j)
            cout << range2[j] << " ";
        cout << endl;

        if (ALTITUDE_CONSTRAINT)
        {
            cout << "(" << vpEdgeSE3Altitude.size() << ") " << "vpEdgeSE3Altitude chi2: " << endl;
            for(size_t i=0, iend=vpEdgeSE3Altitude.size(); i<iend; i++)
            {
                g2o::EdgeSE3Altitude* ea = vpEdgeSE3Altitude[i];
                ea->computeError();
                const float chi2 = ea->chi2();
                cout << chi2 << " ";
            }
            cout << endl;
        }

        if (SMOOTH_CONSTRAINT)
        {
            cout << "(" << vpEdgeSE3Smooth.size() << ") " << "vpEdgeSE3Smooth chi2: " << endl;
            for(size_t i=0, iend=vpEdgeSE3Smooth.size(); i<iend; i++)
            {
                g2o::EdgeSE3* ea = vpEdgeSE3Smooth[i];
                ea->computeError();
                const float chi2 = ea->chi2();
                cout << chi2 << " ";
            }
            cout << endl;
        }
        cout << endl;
        // **********************************************
    }

    optimizer.save("dynamic_slam_graph_before_opt.g2o");
    optimizer.optimize(300);
    optimizer.save("dynamic_slam_graph_after_opt.g2o");

    if (check_after_opt)
    {
        // ****** check the chi2 error stats ******
        cout << endl << "(" << vpEdgeSE3.size() << ") " << "EdgeSE3 chi2: " << endl;
        for(size_t i=0, iend=vpEdgeSE3.size(); i<iend; i++)
        {
            g2o::EdgeSE3* e = vpEdgeSE3[i];
            const float chi2 = e->chi2();
            cout << chi2 << " ";
        }
        cout << endl;

        std::vector<int> range(12,0);
        cout << "(" << vpEdgeSE3PointSta.size() << ") " << "EdgeSE3PointSta chi2: " << endl;
        for(size_t i=0, iend=vpEdgeSE3PointSta.size(); i<iend; i++)
        {
            g2o::EdgeSE3PointXYZ* e = vpEdgeSE3PointSta[i];
            const float chi2 = e->chi2();
            {
                if (0.0<=chi2 && chi2<0.01)
                    range[0] = range[0] + 1;
                else if (0.01<=chi2 && chi2<0.02)
                    range[1] = range[1] + 1;
                else if (0.02<=chi2 && chi2<0.04)
                    range[2] = range[2] + 1;
                else if (0.04<=chi2 && chi2<0.08)
                    range[3] = range[3] + 1;
                else if (0.08<=chi2 && chi2<0.1)
                    range[4] = range[4] + 1;
                else if (0.1<=chi2 && chi2<0.2)
                    range[5] = range[5] + 1;
                else if (0.2<=chi2 && chi2<0.4)
                    range[6] = range[6] + 1;
                else if (0.4<=chi2 && chi2<0.8)
                    range[7] = range[7] + 1;
                else if (0.8<=chi2 && chi2<1.0)
                    range[8] = range[8] + 1;
                else if (1.0<=chi2 && chi2<5.0)
                    range[9] = range[9] + 1;
                else if (5.0<=chi2 && chi2<10.0)
                    range[10] = range[10] + 1;
                else if (chi2>=10.0)
                    range[11] = range[11] + 1;
            }
            // cout << chi2 << " ";
        }
        for (int j = 0; j < range.size(); ++j)
            cout << range[j] << " ";
        cout << endl;

        std::vector<int> range1(12,0);
        cout << "(" << vpEdgeLandmarkMotion.size() << ") " << "LandmarkMotionTernaryEdge chi2: " << endl;
        for(size_t i=0, iend=vpEdgeLandmarkMotion.size(); i<iend; i++)
        {
            g2o::LandmarkMotionTernaryEdge* e = vpEdgeLandmarkMotion[i];
            const float chi2 = e->chi2();
            {
                if (0.0<=chi2 && chi2<0.01)
                    range1[0] = range1[0] + 1;
                else if (0.01<=chi2 && chi2<0.02)
                    range1[1] = range1[1] + 1;
                else if (0.02<=chi2 && chi2<0.04)
                    range1[2] = range1[2] + 1;
                else if (0.04<=chi2 && chi2<0.08)
                    range1[3] = range1[3] + 1;
                else if (0.08<=chi2 && chi2<0.1)
                    range1[4] = range1[4] + 1;
                else if (0.1<=chi2 && chi2<0.2)
                    range1[5] = range1[5] + 1;
                else if (0.2<=chi2 && chi2<0.4)
                    range1[6] = range1[6] + 1;
                else if (0.4<=chi2 && chi2<0.8)
                    range1[7] = range1[7] + 1;
                else if (0.8<=chi2 && chi2<1.0)
                    range1[8] = range1[8] + 1;
                else if (1.0<=chi2 && chi2<5.0)
                    range1[9] = range1[9] + 1;
                else if (5.0<=chi2 && chi2<10.0)
                    range1[10] = range1[10] + 1;
                else if (chi2>=10.0)
                    range1[11] = range1[11] + 1;
            }
            // cout << chi2 << " ";
        }
        for (int j = 0; j < range1.size(); ++j)
            cout << range1[j] << " ";
        cout << endl;

        std::vector<int> range2(12,0);
        cout << "(" << vpEdgeSE3PointDyn.size() << ") " << "EdgeSE3PointDyn chi2: " << endl;
        for(size_t i=0, iend=vpEdgeSE3PointDyn.size(); i<iend; i++)
        {
            g2o::EdgeSE3PointXYZ* e = vpEdgeSE3PointDyn[i];
            const float chi2 = e->chi2();
            {
                if (0.0<=chi2 && chi2<0.01)
                    range2[0] = range2[0] + 1;
                else if (0.01<=chi2 && chi2<0.02)
                    range2[1] = range2[1] + 1;
                else if (0.02<=chi2 && chi2<0.04)
                    range2[2] = range2[2] + 1;
                else if (0.04<=chi2 && chi2<0.08)
                    range2[3] = range2[3] + 1;
                else if (0.08<=chi2 && chi2<0.1)
                    range2[4] = range2[4] + 1;
                else if (0.1<=chi2 && chi2<0.2)
                    range2[5] = range2[5] + 1;
                else if (0.2<=chi2 && chi2<0.4)
                    range2[6] = range2[6] + 1;
                else if (0.4<=chi2 && chi2<0.8)
                    range2[7] = range2[7] + 1;
                else if (0.8<=chi2 && chi2<1.0)
                    range2[8] = range2[8] + 1;
                else if (1.0<=chi2 && chi2<5.0)
                    range2[9] = range2[9] + 1;
                else if (5.0<=chi2 && chi2<10.0)
                    range2[10] = range2[10] + 1;
                else if (chi2>=10.0)
                    range2[11] = range2[11] + 1;
            }
            // cout << chi2 << " ";
        }
        for (int j = 0; j < range2.size(); ++j)
            cout << range2[j] << " ";
        cout << endl;

        if (ALTITUDE_CONSTRAINT)
        {
            cout << "(" << vpEdgeSE3Altitude.size() << ") " << "vpEdgeSE3Altitude chi2: " << endl;
            for(size_t i=0, iend=vpEdgeSE3Altitude.size(); i<iend; i++)
            {
                g2o::EdgeSE3Altitude* ea = vpEdgeSE3Altitude[i];
                ea->computeError();
                const float chi2 = ea->chi2();
                cout << chi2 << " ";
            }
            cout << endl;
        }

        if (SMOOTH_CONSTRAINT)
        {
            cout << "(" << vpEdgeSE3Smooth.size() << ") " << "vpEdgeSE3Smooth chi2: " << endl;
            for(size_t i=0, iend=vpEdgeSE3Smooth.size(); i<iend; i++)
            {
                g2o::EdgeSE3* ea = vpEdgeSE3Smooth[i];
                ea->computeError();
                const float chi2 = ea->chi2();
                cout << chi2 << " ";
            }
            cout << endl;
        }
        cout << endl;
        // **********************************************
    }


    // *** save optimized camera pose and object motion results ***
    // cout << "UPDATE POSE and MOTION ......" << endl;
    for (int i = 0; i < N-1; ++i)
    {
        for (int j = 0; j < VertexID[i].size(); ++j)
        {
            if (j==0)  // static only
            {
                g2o::VertexSE3* vSE3 = static_cast<g2o::VertexSE3*>(optimizer.vertex(VertexID[i][j]));

                // convert
                double optimized[7];
                vSE3->getEstimateData(optimized);
                Eigen::Quaterniond q(optimized[6],optimized[3],optimized[4],optimized[5]);
                Eigen::Matrix<double,3,3> rot = q.matrix();
                Eigen::Matrix<double,3,1> tra;
                tra << optimized[0],optimized[1],optimized[2];

                // camera motion
                pMap->vmCameraPose_RF[i+1] = Converter::toCvSE3(rot,tra);
            }
            else
            {
                if (!STATIC_ONLY)
                {
                    g2o::VertexSE3* vSE3 = static_cast<g2o::VertexSE3*>(optimizer.vertex(VertexID[i][j]));

                    // convert
                    double optimized[7];
                    vSE3->getEstimateData(optimized);
                    Eigen::Quaterniond q(optimized[6],optimized[3],optimized[4],optimized[5]);
                    Eigen::Matrix<double,3,3> rot = q.matrix();
                    Eigen::Matrix<double,3,1> tra;
                    tra << optimized[0],optimized[1],optimized[2];

                    // object
                    pMap->vmRigidMotion_RF[i][j] = Converter::toCvSE3(rot,tra);
                }
            }
        }
    }

    // *** save optimized 3d point results ***
    // cout << "UPDATE 3D POINTS ......" << endl << endl;
    // (1) static points
    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < vnFeaMakSta[i].size(); ++j)
        {
            if (vnFeaMakSta[i][j]!=-1)
            {
                g2o::VertexPointXYZ* vPoint = static_cast<g2o::VertexPointXYZ*>(optimizer.vertex(vnFeaMakSta[i][j]));
                double optimized[3];
                vPoint->getEstimateData(optimized);
                Eigen::Matrix<double,3,1> tmp_3d;
                tmp_3d << optimized[0],optimized[1],optimized[2];
                pMap->vp3DPointSta[i][j] = Converter::toCvMat(tmp_3d);
            }
        }
    }
    // (2) dynamic points
    if (!STATIC_ONLY)
    {
        for (int i = 0; i < N; ++i)
        {
            for (int j = 0; j < vnFeaMakDyn[i].size(); ++j)
            {
                if (vnFeaMakDyn[i][j]!=-1)
                {
                    g2o::VertexPointXYZ* vPoint = static_cast<g2o::VertexPointXYZ*>(optimizer.vertex(vnFeaMakDyn[i][j]));
                    double optimized[3];
                    vPoint->getEstimateData(optimized);
                    Eigen::Matrix<double,3,1> tmp_3d;
                    tmp_3d << optimized[0],optimized[1],optimized[2];
                    pMap->vp3DPointDyn[i][j] = Converter::toCvMat(tmp_3d);
                }
            }
        }
    }


}

int Optimizer::PoseOptimizationNew(Frame *pCurFrame, Frame *pLastFrame, vector<int> &TemperalMatch)
{
    // cv::RNG rng((unsigned)time(NULL));

    float rp_thres = 0.01;

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    int nInitialCorrespondences=0;

    // Set Frame vertex
    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(Converter::toSE3Quat(pCurFrame->mTcw));
    vSE3->setId(0);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);

    // Set MapPoint vertices
    // const int N = pCurFrame->N_s;
    const int N = TemperalMatch.size();

    vector<g2o::EdgeSE3ProjectXYZOnlyPose*> vpEdgesMono;
    vector<size_t> vnIndexEdgeMono;
    vpEdgesMono.reserve(N);
    vnIndexEdgeMono.reserve(N);

    const float deltaMono = sqrt(rp_thres);

    bool mono = 1; // monocular
    float repro_e = 0;
    std::vector<bool> vIsOutlier(N,false);

    for(int i=0; i<N; i++)
    {
        // if (TemperalMatch[i]==-1)
        //     continue;

        if(mono)
        {
            nInitialCorrespondences++;
            vIsOutlier[i] = false;

            Eigen::Matrix<double,2,1> obs;
            const cv::KeyPoint &kpUn = pCurFrame->mvStatKeys[TemperalMatch[i]]; // i
            obs << kpUn.pt.x, kpUn.pt.y;

            g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
            e->setMeasurement(obs);
            // const float invSigma2 = pCurFrame->mvInvLevelSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix2d::Identity());

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(deltaMono);

            e->fx = pCurFrame->fx;
            e->fy = pCurFrame->fy;
            e->cx = pCurFrame->cx;
            e->cy = pCurFrame->cy;

            cv::Mat Xw = pLastFrame->UnprojectStereoStat(TemperalMatch[i],1);
            e->Xw[0] = Xw.at<float>(0);
            e->Xw[1] = Xw.at<float>(1);
            e->Xw[2] = Xw.at<float>(2);

            optimizer.addEdge(e);

            vpEdgesMono.push_back(e);
            vnIndexEdgeMono.push_back(i);

        }

    }


    if(nInitialCorrespondences<3)
        return 0;

    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    const float chi2Mono[4]={rp_thres,5.991,5.991,5.991};
    const int its[4]={100,10,10,10};

    int nBad=0;
    for(size_t it=0; it<1; it++)
    {

        vSE3->setEstimate(Converter::toSE3Quat(pCurFrame->mTcw));
        optimizer.initializeOptimization(0);
        optimizer.optimize(its[it]);

        nBad=0;
        // monocular
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
        {
            g2o::EdgeSE3ProjectXYZOnlyPose* e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];

            if(vIsOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2Mono[it])
            {
                vIsOutlier[idx]=true;
                TemperalMatch[idx] = -1;
                e->setLevel(1);
                nBad++;
            }
            else
            {
                // ++++ new added for calculating re-projection error +++
                if (it==0)
                {
                    repro_e = repro_e + std::sqrt(chi2);
                }
                vIsOutlier[idx]=false;
                e->setLevel(0);
            }

            if(it==2)
                e->setRobustKernel(0);
        }

        if(optimizer.edges().size()<5)
            break;
    }

    // Recover optimized pose and return number of inliers
    g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    cv::Mat pose = Converter::toCvMat(SE3quat_recov);
    pCurFrame->SetPose(pose);

    int inliers = nInitialCorrespondences-nBad;
    cout << "(Camera) inliers number/total numbers: " << inliers << "/" << nInitialCorrespondences << endl;
    repro_e = repro_e/inliers;
    // cout << "re-projection error from the optimization: " << repro_e << endl;

    return nInitialCorrespondences-nBad;
}

int Optimizer::PoseOptimizationFlow2Cam(Frame *pCurFrame, Frame *pLastFrame, vector<int> &TemperalMatch)
{
    float rp_thres = 0.04; // 0.01
    bool updateflow = true;

    g2o::SparseOptimizer optimizer;
    // optimizer.setVerbose(true);
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    optimizer.setAlgorithm(solver);

    int nInitialCorrespondences=0;

    // Set MapPoint vertices
    const int N = TemperalMatch.size();

    // Set Frame vertex
    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    cv::Mat Init = pCurFrame->mTcw; // initial with camera pose
    vSE3->setEstimate(Converter::toSE3Quat(Init));
    // vSE3->setEstimate(Converter::toSE3Quat(cv::Mat::eye(4,4,CV_32F)));
    vSE3->setId(0);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);

    // Set Edge info
    vector<g2o::EdgeSE3ProjectFlow2*> vpEdgesMono;
    vector<size_t> vnIndexEdgeMono;
    vpEdgesMono.reserve(N);
    vnIndexEdgeMono.reserve(N);

    // parameter for robust function
    const float deltaMono = sqrt(rp_thres);  // 5.991

    bool mono = 1; // monocular
    float repro_e = 0;
    std::vector<bool> vIsOutlier(N);

    for(int i=0; i<N; i++)
    {

        if(mono)
        {
            nInitialCorrespondences++;
            vIsOutlier[i] = false;

            // Set Flow vertices
            g2o::VertexSBAFlow* vFlo = new g2o::VertexSBAFlow();
            Eigen::Matrix<double,3,1> FloD = Converter::toVector3d(pLastFrame->ObtainFlowDepthCamera(TemperalMatch[i],0));
            vFlo->setEstimate(FloD.head(2));
            const int id = i+1;
            vFlo->setId(id);
            vFlo->setMarginalized(true);
            optimizer.addVertex(vFlo);

            Eigen::Matrix<double,2,1> obs_2d;
            const cv::KeyPoint &kpUn = pLastFrame->mvStatKeys[TemperalMatch[i]];
            obs_2d << kpUn.pt.x, kpUn.pt.y;

            // Set Binary Edges
            g2o::EdgeSE3ProjectFlow2* e = new g2o::EdgeSE3ProjectFlow2();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
            e->setMeasurement(obs_2d);
            Eigen::Matrix2d info_flow;
            info_flow << 0.1, 0.0, 0.0, 0.1;
            e->setInformation(Eigen::Matrix2d::Identity()*info_flow);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(deltaMono);

            e->fx = pCurFrame->fx;
            e->fy = pCurFrame->fy;
            e->cx = pCurFrame->cx;
            e->cy = pCurFrame->cy;

            e->depth = FloD(2);

            const cv::Mat Rlw = pLastFrame->mTcw.rowRange(0,3).colRange(0,3);
            const cv::Mat Rwl = Rlw.t();
            const cv::Mat tlw = pLastFrame->mTcw.rowRange(0,3).col(3);
            const cv::Mat twl = -Rlw.t()*tlw;
            e->Twl.setIdentity(4,4);
            e->Twl.block(0,0,3,3) = Converter::toMatrix3d(Rwl);
            e->Twl.col(3).head(3) = Converter::toVector3d(twl);

            optimizer.addEdge(e);

            vpEdgesMono.push_back(e);
            vnIndexEdgeMono.push_back(i);

            Eigen::Matrix<double,2,1> obs_flo;
            obs_flo << FloD(0), FloD(1);

            // Set Unary Edges (constraints)
            g2o::EdgeFlowPrior* e_con = new g2o::EdgeFlowPrior();
            e_con->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
            e_con->setMeasurement(obs_flo);
            Eigen::Matrix2d invSigma2_flo;
            invSigma2_flo << 0.3, 0.0, 0.0, 0.3;
            e_con->setInformation(Eigen::Matrix2d::Identity()*invSigma2_flo);
            optimizer.addEdge(e_con);

        }

    }


    if(nInitialCorrespondences<3)
        return 0;

    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    const float chi2Mono[4]={rp_thres,5.991,5.991,5.991}; // {5.991,5.991,5.991,5.991} {4,4,4,4}
    const int its[4]={100,100,100,100};

    int nBad=0;
    cout << endl;
    for(size_t it=0; it<1; it++)
    {

        vSE3->setEstimate(Converter::toSE3Quat(Init));
        optimizer.initializeOptimization(0);
        optimizer.optimize(its[it]);

        nBad=0;

        // monocular
        // cout << endl << "chi2: " << endl;
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
        {
            g2o::EdgeSE3ProjectFlow2* e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];

            if(vIsOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            // if(chi2>chi2Mono[it])
            //     cout << chi2 << " ";
            // if (i==(iend-1))
            //     cout << endl << endl;

            if(chi2>chi2Mono[it])
            {
                vIsOutlier[idx]=true;
                TemperalMatch[idx] = -1;
                e->setLevel(1);
                nBad++;
            }
            else
            {
                // ++++ new added for calculating re-projection error +++
                if (it==0)
                {
                    repro_e = repro_e + std::sqrt(chi2);
                }
                vIsOutlier[idx]=false;
                e->setLevel(0);
            }

            if(it==2)
                e->setRobustKernel(0);
        }


        if(optimizer.edges().size()<5)
            break;
    }

    // *** Recover optimized pose and return number of inliers ***
    g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    cv::Mat pose = Converter::toCvMat(SE3quat_recov);
    pCurFrame->SetPose(pose);

    // cout << "pose after update: " << endl << pose << endl;

    // *** Recover optimized optical flow ***
    for (int i = 0; i < N; ++i)
    {
        g2o::VertexSBAFlow* vFlow = static_cast<g2o::VertexSBAFlow*>(optimizer.vertex(i+1));

        if (updateflow && vIsOutlier[i]==false)
        {
            Eigen::Vector2d flow_new = vFlow->estimate();
            pCurFrame->mvStatKeys[TemperalMatch[i]].pt.x = pLastFrame->mvStatKeys[TemperalMatch[i]].pt.x + flow_new(0);
            pCurFrame->mvStatKeys[TemperalMatch[i]].pt.y = pLastFrame->mvStatKeys[TemperalMatch[i]].pt.y + flow_new(1);

        }
    }
    int inliers = nInitialCorrespondences-nBad;
    cout << "(Camera) inliers number/total numbers: " << inliers << "/" << nInitialCorrespondences << endl;
    repro_e = repro_e/inliers;
    // cout << "re-projection error from the optimization: " << repro_e << endl;

    return nInitialCorrespondences-nBad;
}

cv::Mat Optimizer::PoseOptimizationObjMot(Frame *pCurFrame, Frame *pLastFrame, const vector<int> &ObjId, std::vector<int> &InlierID)
{
    float rp_thres = 0.01;

    g2o::SparseOptimizer optimizer;
    // optimizer.setVerbose(true);
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    int nInitialCorrespondences=0;

    // Set MapPoint vertices
    const int N = ObjId.size();

    // ************************** preconditioning *****************************
    // (1) compute center location (in {o}) of the two point clouds of cur and pre frames.
    cv::Mat NewCentre = (cv::Mat_<float>(3,1) << 0, 0, 0);
    for(int i=0; i<N; i++)
    {
        cv::Mat Xp = pLastFrame->UnprojectStereoObject(ObjId[i],1);
        cv::Mat Xc = pCurFrame->UnprojectStereoObject(ObjId[i],1);
        NewCentre.at<float>(0) = NewCentre.at<float>(0) + Xp.at<float>(0) + Xc.at<float>(0);
        NewCentre.at<float>(1) = NewCentre.at<float>(1) + Xp.at<float>(1) + Xc.at<float>(1);
        NewCentre.at<float>(2) = NewCentre.at<float>(2) + Xp.at<float>(2) + Xc.at<float>(2);

    }

    // (2) construct preconditioning coordinate ^{o}T_{p}
    cv::Mat Twp = cv::Mat::eye(4,4,CV_32F);
    Twp.at<float>(0,3)=NewCentre.at<float>(0)/(2*N);
    Twp.at<float>(1,3)=NewCentre.at<float>(1)/(2*N);
    Twp.at<float>(2,3)=NewCentre.at<float>(2)/(2*N);

    // cout << "the preconditioning coordinate: " << endl << Twp << endl;

    const cv::Mat Twp_inv = Converter::toInvMatrix(Twp);
    const cv::Mat R_wp_inv = Twp_inv.rowRange(0,3).colRange(0,3);
    const cv::Mat t_wp_inv = Twp_inv.rowRange(0,3).col(3);
    // ************************************************************************

    // Set Frame vertex
    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    // cv::Mat Init = cv::Mat::eye(4,4,CV_32F); // initial with identity matrix
    cv::Mat Init = Converter::toInvMatrix(pCurFrame->mTcw)*pCurFrame->mInitModel; // initial with identity matrix
    vSE3->setEstimate(Converter::toSE3Quat(Init));
    vSE3->setId(0);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);

    // Set Edge info
    vector<g2o::EdgeSE3ProjectXYZOnlyObjMotion*> vpEdgesMono;
    vector<size_t> vnIndexEdgeMono;
    vpEdgesMono.reserve(N);
    vnIndexEdgeMono.reserve(N);

    // Set Projection Matrix
    Eigen::Matrix<double, 3, 4> KK, PP;
    KK << pCurFrame->fx, 0, pCurFrame->cx, 0, 0, pCurFrame->fy, pCurFrame->cy, 0, 0, 0, 1, 0;
    PP = KK*Converter::toMatrix4d(pCurFrame->mTcw); // *Converter::toMatrix4d(Twp)
    // cout << "PP: " << endl << PP << endl;

    // // parameter for robust function
    // const float deltaMono = sqrt(rp_thres);  // 5.991

    bool mono = 1; // monocular
    float repro_e = 0;
    std::vector<bool> vIsOutlier(N);

    for(int i=0; i<N; i++)
    {

        if(mono)
        {
            nInitialCorrespondences++;
            vIsOutlier[i] = false;

            Eigen::Matrix<double,2,1> obs;
            const cv::KeyPoint &kpUn = pCurFrame->mvObjKeys[ObjId[i]];
            obs << kpUn.pt.x, kpUn.pt.y;

            g2o::EdgeSE3ProjectXYZOnlyObjMotion* e = new g2o::EdgeSE3ProjectXYZOnlyObjMotion();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
            e->setMeasurement(obs);
            // const float invSigma2 = 1.0;
            Eigen::Matrix2d invSigma2;
            // invSigma2 << 1.0/flo_co.x, 0, 0, 1.0/flo_co.y;
            invSigma2 << 1.0, 0, 0, 1.0;
            e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

            // g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            // e->setRobustKernel(rk);
            // rk->setDelta(deltaMono);

            // add projection matrix
            e->P = PP;

            cv::Mat Xw = pLastFrame->UnprojectStereoObject(ObjId[i],0);
            // transfer to preconditioning coordinate
            // Xw = R_wp_inv*Xw+t_wp_inv;

            e->Xw[0] = Xw.at<float>(0);
            e->Xw[1] = Xw.at<float>(1);
            e->Xw[2] = Xw.at<float>(2);

            optimizer.addEdge(e);

            vpEdgesMono.push_back(e);
            vnIndexEdgeMono.push_back(i);

        }

    }


    if(nInitialCorrespondences<3)
        return cv::Mat::eye(4,4,CV_32F);

    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    const float chi2Mono[4]={rp_thres,5.991,5.991,5.991}; // {5.991,5.991,5.991,5.991} {4,4,4,4}
    const int its[4]={200,100,100,100};

    int nBad=0;
    cout << endl;
    for(size_t it=0; it<1; it++)
    {

        vSE3->setEstimate(Converter::toSE3Quat(Init));
        optimizer.initializeOptimization(0);
        optimizer.optimize(its[it]);

        nBad=0;

        // monocular
        // cout << endl << "chi2: " << endl;
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
        {
            g2o::EdgeSE3ProjectXYZOnlyObjMotion* e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];

            if(vIsOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            // if(chi2>chi2Mono[it])
            //     cout << chi2 << " ";
            // if (i==(iend-1))
            //     cout << endl << endl;

            if(chi2>chi2Mono[it])
            {
                vIsOutlier[idx]=true;
                e->setLevel(1);
                nBad++;
            }
            else
            {
                // ++++ new added for calculating re-projection error +++
                if (it==0)
                {
                    repro_e = repro_e + std::sqrt(chi2);
                }
                vIsOutlier[idx]=false;
                e->setLevel(0);
            }

            if(it==2)
                e->setRobustKernel(0);
        }


        if(optimizer.edges().size()<5)
            break;
    }

    // Recover optimized pose and return number of inliers
    g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    cv::Mat pose = Converter::toCvMat(SE3quat_recov);

    int inliers = nInitialCorrespondences-nBad;
    // cout << endl;
    cout << "(OBJ)inliers number/total numbers: " << inliers << "/" << nInitialCorrespondences << endl;
    repro_e = repro_e/inliers;
    // cout << "re-projection error from the optimization: " << repro_e << endl;

    // save inlier ID
    std::vector<int> output_inlier;
    for (int i = 0; i < vIsOutlier.size(); ++i)
    {
        if (vIsOutlier[i]==false)
            output_inlier.push_back(ObjId[i]);
        else
            pCurFrame->vObjLabel[ObjId[i]] = -1;
    }
    InlierID = output_inlier;

    return pose; // Twp*pose*Twp_inv
}

cv::Mat Optimizer::PoseOptimizationFlow2(Frame *pCurFrame, Frame *pLastFrame, const vector<int> &ObjId, std::vector<int> &InlierID)
{
    float rp_thres = 0.04;  // 0.04 0.01
    bool updateflow = true;

    g2o::SparseOptimizer optimizer;
    // optimizer.setVerbose(true);
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    optimizer.setAlgorithm(solver);

    int nInitialCorrespondences=0;

    // Set MapPoint vertices
    const int N = ObjId.size();

    // Set Frame vertex
    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    cv::Mat Init = pCurFrame->mInitModel; // initial with camera pose
    // cv::Mat Init = cv::Mat::eye(4,4,CV_32F); // initial with camera pose
    vSE3->setEstimate(Converter::toSE3Quat(Init));
    vSE3->setId(0);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);

    // Set Edge info
    vector<g2o::EdgeSE3ProjectFlow2*> vpEdgesMono;
    vector<size_t> vnIndexEdgeMono;
    vpEdgesMono.reserve(N);
    vnIndexEdgeMono.reserve(N);

    // parameter for robust function
    const float deltaMono = sqrt(rp_thres);  // 5.991

    bool mono = 1; // monocular
    float repro_e = 0;
    std::vector<bool> vIsOutlier(N);

    for(int i=0; i<N; i++)
    {

        if(mono)
        {
            nInitialCorrespondences++;
            vIsOutlier[i] = false;

            // Set Flow vertices
            g2o::VertexSBAFlow* vFlo = new g2o::VertexSBAFlow();
            Eigen::Matrix<double,3,1> FloD = Converter::toVector3d(pLastFrame->ObtainFlowDepthObject(ObjId[i],0));
            vFlo->setEstimate(FloD.head(2));
            const int id = i+1;
            vFlo->setId(id);
            vFlo->setMarginalized(true);
            optimizer.addVertex(vFlo);

            Eigen::Matrix<double,2,1> obs_2d;
            const cv::KeyPoint &kpUn = pLastFrame->mvObjKeys[ObjId[i]];
            obs_2d << kpUn.pt.x, kpUn.pt.y;

            // Set Binary Edges
            g2o::EdgeSE3ProjectFlow2* e = new g2o::EdgeSE3ProjectFlow2();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
            e->setMeasurement(obs_2d);
            Eigen::Matrix2d info_flow;
            info_flow << 0.1, 0.0, 0.0, 0.1;
            e->setInformation(Eigen::Matrix2d::Identity()*info_flow);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(deltaMono);

            e->fx = pCurFrame->fx;
            e->fy = pCurFrame->fy;
            e->cx = pCurFrame->cx;
            e->cy = pCurFrame->cy;

            e->depth = FloD(2);

            const cv::Mat Rlw = pLastFrame->mTcw.rowRange(0,3).colRange(0,3);
            const cv::Mat Rwl = Rlw.t();
            const cv::Mat tlw = pLastFrame->mTcw.rowRange(0,3).col(3);
            const cv::Mat twl = -Rlw.t()*tlw;
            e->Twl.setIdentity(4,4);
            e->Twl.block(0,0,3,3) = Converter::toMatrix3d(Rwl);
            e->Twl.col(3).head(3) = Converter::toVector3d(twl);

            optimizer.addEdge(e);

            vpEdgesMono.push_back(e);
            vnIndexEdgeMono.push_back(i);

            Eigen::Matrix<double,2,1> obs_flo;
            obs_flo << FloD(0), FloD(1);

            // Set Unary Edges (constraints)
            g2o::EdgeFlowPrior* e_con = new g2o::EdgeFlowPrior();
            e_con->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
            e_con->setMeasurement(obs_flo);
            // const float invSigma2_flo = 1.0;
            Eigen::Matrix2d invSigma2_flo;
            invSigma2_flo << 0.5, 0.0, 0.0, 0.5;
            e_con->setInformation(Eigen::Matrix2d::Identity()*invSigma2_flo);
            optimizer.addEdge(e_con);

        }

    }


    if(nInitialCorrespondences<3)
        return cv::Mat::eye(4,4,CV_32F);

    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    const float chi2Mono[4]={rp_thres,5.991,5.991,5.991}; // {5.991,5.991,5.991,5.991} {4,4,4,4}
    const int its[4]={200,100,100,100};

    int nBad=0;
    cout << endl;
    for(size_t it=0; it<1; it++)
    {

        vSE3->setEstimate(Converter::toSE3Quat(Init));
        optimizer.initializeOptimization(0);
        optimizer.optimize(its[it]);

        nBad=0;

        // monocular
        // cout << endl << "chi2: " << endl;
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
        {
            g2o::EdgeSE3ProjectFlow2* e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];

            if(vIsOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            // if(chi2>chi2Mono[it])
            //     cout << chi2 << " ";
            // if (i==(iend-1))
            //     cout << endl << endl;

            if(chi2>chi2Mono[it])
            {
                vIsOutlier[idx]=true;
                e->setLevel(1);
                nBad++;
            }
            else
            {
                // ++++ new added for calculating re-projection error +++
                if (it==0)
                {
                    repro_e = repro_e + std::sqrt(chi2);
                }
                vIsOutlier[idx]=false;
                e->setLevel(0);
            }

            if(it==2)
                e->setRobustKernel(0);
        }


        if(optimizer.edges().size()<5)
            break;
    }

    // *** Recover optimized pose and return number of inliers ***
    g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    cv::Mat pose = Converter::toCvMat(SE3quat_recov);

    // *** Recover optimized optical flow ***
    for (int i = 0; i < N; ++i)
    {
        g2o::VertexSBAFlow* vFlow = static_cast<g2o::VertexSBAFlow*>(optimizer.vertex(i+1));

        if (updateflow && vIsOutlier[i]==false)
        {
            Eigen::Vector2d flow_new = vFlow->estimate();
            pCurFrame->mvObjKeys[ObjId[i]].pt.x = pLastFrame->mvObjKeys[ObjId[i]].pt.x + flow_new(0);
            pCurFrame->mvObjKeys[ObjId[i]].pt.y = pLastFrame->mvObjKeys[ObjId[i]].pt.y + flow_new(1);

        }
    }
    int inliers = nInitialCorrespondences-nBad;
    cout << "(Object) inliers number/total numbers: " << inliers << "/" << nInitialCorrespondences << endl;
    repro_e = repro_e/inliers;
    // cout << "re-projection error from the optimization: " << repro_e << endl;

    // save inlier ID
    std::vector<int> output_inlier;
    for (int i = 0; i < vIsOutlier.size(); ++i)
    {
        if (vIsOutlier[i]==false)
            output_inlier.push_back(ObjId[i]);
        else
            pCurFrame->vObjLabel[ObjId[i]] = -1;
    }
    InlierID = output_inlier;

    return pose;
}

cv::Mat Optimizer::Get3DinWorld(const cv::KeyPoint &Feats2d, const float &Dpts, const cv::Mat &Calib_K, const cv::Mat &CameraPose)
{
    const float invfx = 1.0f/Calib_K.at<float>(0,0);
    const float invfy = 1.0f/Calib_K.at<float>(1,1);
    const float cx = Calib_K.at<float>(0,2);
    const float cy = Calib_K.at<float>(1,2);

    const float u = Feats2d.pt.x;
    const float v = Feats2d.pt.y;

    const float z = Dpts;
    const float x = (u-cx)*z*invfx;
    const float y = (v-cy)*z*invfy;

    cv::Mat x3D = (cv::Mat_<float>(3,1) << x, y, z);

    const cv::Mat mRwc = CameraPose.rowRange(0,3).colRange(0,3);
    const cv::Mat mtwc = CameraPose.rowRange(0,3).col(3);

    return mRwc*x3D+mtwc;
}

cv::Mat Optimizer::Get3DinCamera(const cv::KeyPoint &Feats2d, const float &Dpts, const cv::Mat &Calib_K)
{
    const float invfx = 1.0f/Calib_K.at<float>(0,0);
    const float invfy = 1.0f/Calib_K.at<float>(1,1);
    const float cx = Calib_K.at<float>(0,2);
    const float cy = Calib_K.at<float>(1,2);

    const float u = Feats2d.pt.x;
    const float v = Feats2d.pt.y;

    const float z = Dpts;
    const float x = (u-cx)*z*invfx;
    const float y = (v-cy)*z*invfy;

    cv::Mat x3D = (cv::Mat_<float>(3,1) << x, y, z);

    return x3D;
}


} //namespace VDO_SLAM
