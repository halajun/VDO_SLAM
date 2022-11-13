#include "UtilsGtsam.h"

#include <opencv2/core/eigen.hpp>


namespace vdo {
namespace utils {

//TODO: unit test
gtsam::Pose3 cvMatToGtsamPose3(const cv::Mat& H) {
    CHECK_EQ(H.rows, 4);
    CHECK_EQ(H.cols, 4);

    cv::Mat R(3, 3, H.type());
    cv::Mat T(3, 1, H.type());

    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            R.at<double>(i, j) = H.at<double>(i, j);
        }
    }

    for(int i = 0; i < 3; i++) {
        T.at<double>(i, 0) = H.at<double>(i, 3);
    }

    return cvMatsToGtsamPose3(R, T);
}

gtsam::Pose3 cvMatsToGtsamPose3(const cv::Mat& R, const cv::Mat& T) {
    return gtsam::Pose3(cvMatToGtsamRot3(R), cvMatToGtsamPoint3(T));
}

cv::Mat gtsamPose3ToCvMat(const gtsam::Pose3& pose) {
    cv::Mat RT(4, 4, CV_64F);
    cv::eigen2cv(pose.matrix(), RT);
    RT.convertTo(RT, CV_64F);
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
    gtsam_t << cv_t.at<double>(0, 0), cv_t.at<double>(1, 0),
        cv_t.at<double>(2, 0);
    return gtsam_t;
}

cv::Mat gtsamPoint3ToCvMat(const gtsam::Point3& point) {
    cv::Mat T(3, 1, CV_32F);
    cv::eigen2cv(point, T);
    T.convertTo(T, CV_32F);
    return T.clone();
}


}
}