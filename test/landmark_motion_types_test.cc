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
    cv::Mat id_temp = (cv::Mat_<float>(4,4) <<  0.5145401, -0.0434989, 0.8563623, 118,
                                                0.2096710, 0.9747775, -0.0764658, 72,
                                                -0.8314365, 0.2188990, 0.5106825, 15,
                                                0, 0, 0, 1);

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
    std::cout << "g2o H " << Converter::toSE3Quat(id_temp) << std::endl; 

    auto Homo = Converter::toSE3Quat(id_temp).to_homogeneous_matrix();

    Eigen::Matrix<double,4,1> v1_4;
    v1_4 << 10, 11, 12, 1;

    Eigen::Matrix<double,4,1> v2_4;
    v2_4 << 12, 15, 17, 1;

    Eigen::Vector4d g2o_measurement_4(10,13,15, 1);
    Eigen::Vector3d g2o_measurement(10,13,15);

    auto g2o_e_manual = (v1_4 - Homo.inverse() * v2_4) - g2o_measurement_4;
    std::cout << "g2o manual error " << g2o_e_manual << std::endl; 

    g2o::LandmarkMotionTernaryEdge * em = new g2o::LandmarkMotionTernaryEdge();
    em->setVertex(0, v_p1);
    em->setVertex(1, v_p2);
    em->setVertex(2, m_se3);
    em->setMeasurement(g2o_measurement);
    em->information() = Eigen::Matrix3d::Identity()/0.001;

    em->computeError();

    std::cout << "g2o error " << em->error() << std::endl;
    // auto g2o_e_manual = v1 - Converter::toMatrix3d(id_temp) * 


    //now try for gtsam error
    gtsam::Point3 p1(10, 11, 12);
    gtsam::Point3 p2(12, 15, 17);
    gtsam::Pose3 H(utils::cvMatToGtsamPose3(id_temp));
    std::cout << "gtsam H " << H << std::endl; 

    gtsam::Point3 measurement(10, 13, 15);

    gtsam::Key p1_key = 0, p2_key = 1, motion_key = 2;
    LandmarkMotionTernaryFactor lmktf(p1_key, p2_key, motion_key, measurement, nullptr);

    auto e = lmktf.evaluateError(p1, p2, H);
    std::cout << "gtsam error " << e << std::endl; 

    //equation goes e = m_k - H * m_{k-1}
    auto e_manul =  (p1 - H * p2) - measurement;
    std::cout << "gtsam manual error " << e_manul << std::endl; 

    auto e_manul_inv = (p1 - H.inverse() * p2) - measurement;
    std::cout << "gtsam manual invert error " << e_manul_inv << std::endl; 


    

    return 0;
}
