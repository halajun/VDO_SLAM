#include "utils/UtilsGTSAM.h"

#include <glog/logging.h>
#include <opencv2/core/eigen.hpp>


namespace VDO_SLAM {
namespace utils {

//TODO: unit test
gtsam::Pose3 cvMatToGtsamPose3(const cv::Mat& H) {
    CHECK_EQ(H.rows, 4);
    CHECK_EQ(H.cols, 4);

    cv::Mat R(3, 3, H.type());
    cv::Mat T(3, 1, H.type());

    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            R.at<CvMatAccessType>(i, j) = H.at<CvMatAccessType>(i, j);
        }
    }

    for(int i = 0; i < 3; i++) {
        T.at<CvMatAccessType>(i, 0) = H.at<CvMatAccessType>(i, 3);
    }

    return cvMatsToGtsamPose3(R, T);
}

gtsam::Pose3 cvMatsToGtsamPose3(const cv::Mat& R, const cv::Mat& T) {
    return gtsam::Pose3(cvMatToGtsamRot3(R), cvMatToGtsamPoint3(T));
}

cv::Mat gtsamPose3ToCvMat(const gtsam::Pose3& pose) {
    cv::Mat RT(4, 4, CV_32F);
    cv::eigen2cv(pose.matrix(), RT);
    RT.convertTo(RT, CV_32F);
    return RT;
}

gtsam::Rot3 cvMatToGtsamRot3(const cv::Mat& R) {
    CHECK_EQ(R.rows, 3);
    CHECK_EQ(R.cols, 3);
    gtsam::Matrix rot_mat = gtsam::Matrix::Identity(3, 3);
    cv::cv2eigen(R, rot_mat);
    return gtsam::Rot3(rot_mat);
}

gtsam::Point3 cvMatToGtsamPoint3(const cv::Mat& cv_t) {
    CHECK_EQ(cv_t.rows, 3);
    CHECK_EQ(cv_t.cols, 1);
    gtsam::Point3 gtsam_t;
    gtsam_t << cv_t.at<CvMatAccessType>(0, 0), cv_t.at<CvMatAccessType>(1, 0),
        cv_t.at<CvMatAccessType>(2, 0);
    return gtsam_t;
}

cv::Mat gtsamPoint3ToCvMat(const gtsam::Point3& point) {
    cv::Mat T(3, 1, CV_32F);
    cv::eigen2cv(point, T);
    T.convertTo(T, CV_32F);
    return T.clone();
}

gtsam::Cal3_S2::shared_ptr cvMat2Cal3_S2(const cv::Mat& K) {
    CHECK_EQ(K.rows, 3);  // We expect homogeneous camera matrix.
    CHECK_GE(K.cols, 3);  // We accept extra columns (which we do not use).
    const CvMatAccessType& fx = K.at<CvMatAccessType>(0, 0);
    const CvMatAccessType& fy = K.at<CvMatAccessType>(1, 1);
    const CvMatAccessType& s = K.at<CvMatAccessType>(0, 1);
    const CvMatAccessType& u0 = K.at<CvMatAccessType>(0, 2);
    const CvMatAccessType& v0 = K.at<CvMatAccessType>(1, 2);
    return boost::make_shared<gtsam::Cal3_S2>(fx, fy, s, u0, v0);
}

std::pair<double, double> computeRotationAndTranslationErrors(
      const gtsam::Pose3& expectedPose,
      const gtsam::Pose3& actualPose,
      const bool upToScale) {
    // compute errors
    gtsam::Rot3 rotErrorMat =
        (expectedPose.rotation()).between(actualPose.rotation());
    gtsam::Vector3 rotErrorVector = gtsam::Rot3::Logmap(rotErrorMat);
    double rotError = rotErrorVector.norm();

    gtsam::Vector3 actualTranslation = actualPose.translation();
    gtsam::Vector3 expectedTranslation = expectedPose.translation();
    if (upToScale) {
        double normExpected = expectedTranslation.norm();
        double normActual = actualTranslation.norm();
        if (normActual > 1e-5) {
            actualTranslation = normExpected * actualTranslation /
                normActual;  // we manually add the scale here
        }
    }
    gtsam::Vector3 tranErrorVector = expectedTranslation - actualTranslation;
    double tranError = tranErrorVector.norm();
    return std::make_pair(rotError, tranError);
}


} //utils
} //VDO_SLAM