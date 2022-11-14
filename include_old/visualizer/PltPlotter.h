#pragma once

#include <vector>
#include <map>
#include <string>

#include <gtsam/geometry/Pose3.h>

namespace VDO_SLAM
{
class Map;

struct PlotInfo
{
  std::string title;
  std::string x_label;
  std::string y_label;
};

class Plotter
{
public:
  static void initPlot(const PlotInfo& plot_info);

  // replaces existing data
  static void addData(const std::string& title, const std::vector<double>& data);
  static void appendData(const std::string& title, double data);
  static void makePlots(const std::string& path = "/root/data/vdo_slam/results/");
  static void makePlot(const std::string& path, const std::string& title, int fig = 1);
  static void drawDynamicSize(const std::map<int, std::vector<std::vector<gtsam::Key>>>& dynmic_motions,
                              const std::map<int, int>& when_added,
                              const std::string& path = "/root/data/vdo_slam/results/");

  static void PlotMetricError(Map* map, int max_id, const std::string& path = "/root/data/vdo_slam/results/");

  // requires not updating the motion in the map (or updating the refined motion)
  static void PlotMotionComparison(Map* map, const std::string& path = "/root/data/vdo_slam/results/");

private:
  static bool checkPlot(const std::string& title);

private:
  struct PlotData
  {
    PlotInfo info;
    std::vector<double> data;
  };

  static std::map<std::string, PlotData> plots;
  static int figure;
};

}  // namespace VDO_SLAM