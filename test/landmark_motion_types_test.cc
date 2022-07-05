#include "dependencies/g2o/g2o/types/types_dyn_slam3d.h"
#include "factors/LandmarkMotionTernaryFactor.h"
#include "Converter.h"
#include "utils/UtilsGTSAM.h"
#include "utils/UtilsOpenCv.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>


using namespace VDO_SLAM;

int main() {
    // cv::Mat id_temp = cv::Mat::eye(4,4, CV_32F);
    cv::Mat id_temp = (cv::Mat_<double>(4,4) << 0, -0.2, 3.2, 118,
                                                0.02, 0.56, 1.45, 72,
                                                0.89, 1232.5, 11, 15,
                                                12.56, 86.6, 15.5, 1);

    Eigen::Matrix<double,3,1> v1;
    v1 << 10, 11, 12;
    g2o::VertexPointXYZ *v_p1 = new g2o::VertexPointXYZ();
    v_p1->setId(0);
    v_p1->setEstimate(v1);

    Eigen::Matrix<double,3,1> v2;
    v2 << 12, 15, 17;
    g2o::VertexPointXYZ *v_p2 = new g2o::VertexPointXYZ();
    v_p2->setId(1);
    v_p2->setEstimate(v2);

    g2o::VertexSE3 *m_se3 = new g2o::VertexSE3();
    m_se3->setId(2);
    m_se3->setEstimate(Converter::toSE3Quat(id_temp));

    g2o::LandmarkMotionTernaryEdge * em = new g2o::LandmarkMotionTernaryEdge();
    em->setVertex(0, v_p1);
    em->setVertex(1, v_p2);
    em->setVertex(2, m_se3);
    em->setMeasurement(Eigen::Vector3d(10,13,15));
    em->information() = Eigen::Matrix3d::Identity()/0.001;

    em->computeError();

    std::cout << "g2o error " << em->error() << std::endl;


    //now try for gtsam error
    gtsam::Point3 p1(10, 11, 12);
    gtsam::Point3 p2(12, 15, 17);
    gtsam::Pose3 H(utils::cvMatToGtsamPose3(id_temp));

    gtsam::Point3 measurement(10, 13, 15);

    gtsam::Key p1_key = 0, p2_key = 1, motion_key = 2;
    LandmarkMotionTernaryFactor lmktf(p1_key, p2_key, motion_key, measurement, nullptr);

    auto e = lmktf.evaluateError(p2, p1, H);
    std::cout << "gtsam error " << e << std::endl;    

    

    return 0;
}
