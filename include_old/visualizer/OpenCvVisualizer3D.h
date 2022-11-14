#pragma once

#include "utils/macros.h"
#include "utils/Color.h"
#include "visualizer/Display.h"
#include "visualizer/DisplayTypes.h"
#include "Map.h"

#include <opencv2/opencv.hpp>
#include <glog/logging.h>

#include <map>
#include <deque>

namespace VDO_SLAM
{
class OpenCvVisualizer3D : public Display
{
public:
  VDO_SLAM_POINTER_TYPEDEFS(OpenCvVisualizer3D);

  OpenCvVisualizer3D(DisplayParams::Ptr params_, Map* map_);
  ~OpenCvVisualizer3D() = default;

  void process() override;

private:
  // should only draw once
  void drawWorldCoordinateSystem();
  void setupModelViewMatrix();

  void drawCurrentCameraPose(WidgetsMap* widgets_map);
  void followCurrentView();

  void addToTrajectory();
  void drawTrajectory(WidgetsMap* widgets_map);

  void drawStaticPointCloud(WidgetsMap* widgets_map);
  void drawDynamicPointClouds(WidgetsMap* widgets_map);

  void markWidgetForRemoval(const std::string& widget_id);
  void removeWidgets();

  // helpder functions
  cv::Mat getLatestPose();

  static cv::Mat ModelViewLookAt(double ex, double ey, double ez, double lx, double ly, double lz, double ux, double uy,
                                 double uz);

private:
  cv::viz::Viz3d window;
  Map* map;

  //! Intrinsics of the camera frustum used for visualization.
  const cv::Matx33d K_ = { 458.0, 0.0, 360.0, 0.0, 458.0, 240.0, 0.0, 0.0, 1.0 };

  // todo widgets to remove?
  WidgetIds widgets_to_remove;

  std::deque<cv::Affine3f> trajectory;

  // viewer params -> eventually put in params
  const double viewpointX = 0.0;
  const double viewpointY = 0.7;
  const double viewpointZ = -3.5;
  const double viewpointF = 500.0;

  cv::Mat model_view_matrix;
};

}  // namespace VDO_SLAM