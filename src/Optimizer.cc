#include "Optimizer.h"

#include "dependencies/g2o/g2o/types/vertex_se3.h"
#include "dependencies/g2o/g2o/types/vertex_pointxyz.h"
#include "dependencies/g2o/g2o/types/edge_se3.h"
#include "dependencies/g2o/g2o/types/edge_se3_pointxyz.h"
#include "dependencies/g2o/g2o/types/edge_se3_prior.h"
#include "dependencies/g2o/g2o/core/block_solver.h"
#include "dependencies/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "dependencies/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "dependencies/g2o/g2o/solvers/linear_solver_csparse.h"
#include "dependencies/g2o/g2o/solvers/linear_solver_eigen.h"
#include "dependencies/g2o/g2o/solvers/linear_solver_dense.h"



namespace vdo {

void PoseOptimizationFlow2Cam::operator()(Frame::Ptr previous_frame_, Frame::Ptr current_frame_) {
    float rp_thres = 0.04;  // 0.01
    bool updateflow = true;

    g2o::SparseOptimizer optimizer;
    // optimizer.setVerbose(true);
    g2o::BlockSolver_6_3::LinearSolverType* linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3* solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    optimizer.setAlgorithm(solver);

    int nInitialCorrespondences = 0;

    //TODO: refactor this function and structure -> repe

}


}