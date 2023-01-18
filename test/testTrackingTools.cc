#include "Tracking-Tools.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

//totoal namespace importing
using namespace vdo;
using namespace vdo::tracking_tools;

TEST(testTrackingTools, determineOutlierIdsBasic)
{
  TrackletIds tracklets = { 1, 2, 3, 4, 5 };
  TrackletIds inliers = { 1, 2 };

  TrackletIds expected_outliers = { 3, 4, 5 };
  TrackletIds outliers;
  determineOutlierIds(inliers, tracklets, outliers);
  EXPECT_EQ(expected_outliers, outliers);
}

TEST(testTrackingTools, determineOutlierIdsUnorderd)
{
  TrackletIds tracklets = { 12, 45, 1, 85, 3, 100 };
  TrackletIds inliers = { 3, 1, 100 };

  TrackletIds expected_outliers = { 12, 45, 85 };
  TrackletIds outliers;
  determineOutlierIds(inliers, tracklets, outliers);
  EXPECT_EQ(expected_outliers, outliers);
}

TEST(testTrackingTools, determineOutlierIdsNoSubset)
{
  TrackletIds tracklets = { 12, 45, 1, 85, 3, 100 };
  TrackletIds inliers = { 12, 45, 1, 85, 3, 100 };

  TrackletIds outliers = { 4, 5, 6 };  // also add a test that outliers is cleared
  determineOutlierIds(inliers, tracklets, outliers);
  EXPECT_TRUE(outliers.empty());
}