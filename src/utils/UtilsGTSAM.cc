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


} //utils
} //VDO_SLAM