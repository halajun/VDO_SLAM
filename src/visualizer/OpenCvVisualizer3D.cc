#include "visualizer/OpenCvVisualizer3D.h"
#include "utils/UtilsOpenCv.h"

#include <memory>

namespace VDO_SLAM {

OpenCvVisualizer3D::OpenCvVisualizer3D(Map* map_)
    : map(CHECK_NOTNULL(map_)) {}

void OpenCvVisualizer3D::process() {
    WidgetsMap widgets;
    drawCurrentCameraPose(&widgets);

    for (auto it = widgets.begin(); it != widgets.end(); ++it) {
        CHECK(it->second);
        // / This is to go around opencv issue #10829, new opencv should have this
        // fixed. cv::viz::Widget3D::getPose segfaults.
        it->second->updatePose(cv::Affine3d());
        window.showWidget(
            it->first, *(it->second), it->second->getPose());
    }

    window.spinOnce(1, true);
    removeWidgets();
}

void OpenCvVisualizer3D::drawCurrentCameraPose(WidgetsMap* widgets_map) {
    //todo add posr to trajectory?
    const int N = map->vpFeatSta.size() - 1;
    cv::Mat camera_pose_mat = map->vmCameraPose[N];
    cv::Affine3d camera_pose  = utils::matPoseToCvAffine3d(camera_pose_mat);

    CHECK_NOTNULL(widgets_map);
    std::unique_ptr<cv::viz::WCameraPosition> cam_widget_ptr = nullptr;
    cam_widget_ptr = VDO_SLAM::make_unique<cv::viz::WCameraPosition>(
        K_, 1.0, cv::viz::Color::white());
        CHECK(cam_widget_ptr);
    cam_widget_ptr->setPose(camera_pose);
    (*widgets_map)["Camera_pose"] = std::move(cam_widget_ptr);

    markWidgetForRemoval("Camera_pose");
}


void OpenCvVisualizer3D::drawTrajectory(WidgetsMap* widgets_map) {
    CHECK_NOTNULL(widgets_map);

    //this redraws it every time?
    const int N = map->vpFeatSta.size() - 1;
    // for(int i = 0; i < )


    // // Create a Trajectory widget. (argument can be PATH, FRAMES, BOTH).
    // std::vector<cv::Affine3f> trajectory;
    // trajectory.reserve(trajectory_poses_3d_.size());
    // for (const auto& pose : trajectory_poses_3d_) {
    // trajectory.push_back(pose);
    // }
    // (*widgets_map)["Trajectory"] = VIO::make_unique<cv::viz::WTrajectory>(
    // trajectory, cv::viz::WTrajectory::PATH, 1.0, cv::viz::Color::red());
}

void OpenCvVisualizer3D::markWidgetForRemoval(const std::string& widget_id) {
    //assume unique
    widgets_to_remove.push_back(widget_id);
}

void OpenCvVisualizer3D::removeWidgets() {
    for(const std::string& widget_id : widgets_to_remove) {
        window.removeWidget(widget_id);
    }

    widgets_to_remove.clear();
}


} //VDO_SLAM