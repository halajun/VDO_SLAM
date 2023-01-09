#include "utils/Logger.h"
#include "Frontend-Definitions.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>


using namespace vdo;


TEST(testLogger, testFrontendStatsSeralization)
{
    FrontendMetrics metric;
    metric.timestamp = 100;
    metric.frame_id = 12;

    metric.ate_after_flow.rot = 15.4;
    metric.ate_before_flow.rot = 12.0;

    saveArchiveAsXML<FrontendMetrics>("test.xml", metric);
  
}

TEST(testLogger, testFrontendStatsSeralizationVector)
{
    FrontendMetrics metric;
    metric.timestamp = 100;
    metric.frame_id = 12;

    metric.ate_after_flow.rot = 15.4;
    metric.ate_before_flow.rot = 12.0;

    FrontendMetrics metric1;

    std::vector<FrontendMetrics*> metrics = {&metric, &metric1};

    saveArchiveAsXML<std::vector<FrontendMetrics*>>("test.xml", metrics);
  
}