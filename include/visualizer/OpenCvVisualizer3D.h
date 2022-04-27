#pragma once

#include "utils/macros.h"
#include "utils/Color.h"
#include "visualizer/Display.h"
#include "visualizer/DisplayTypes.h"
#include "Map.h"

#include <opencv2/opencv.hpp>
#include <glog/logging.h>
#include <map>

namespace VDO_SLAM { 

class OpenCvVisualizer3D : public Display {

    public:
        VDO_SLAM_POINTER_TYPEDEFS(OpenCvVisualizer3D);

        OpenCvVisualizer3D(Map* map_);
        ~OpenCvVisualizer3D() = default;

        void process() override;

    private:
        void drawCurrentCameraPose(WidgetsMap* widgets_map);
        void drawTrajectory(WidgetsMap* widgets_map);

        void markWidgetForRemoval(const std::string& widget_id);
        void removeWidgets();

        cv::viz::Viz3d window;
        Map* map;

        //! Intrinsics of the camera frustum used for visualization.
        const cv::Matx33d K_ = {458.0, 0.0, 360.0, 0.0, 458.0, 240.0, 0.0, 0.0, 1.0};

        //todo widgets to remove?
        WidgetIds widgets_to_remove;

};


}