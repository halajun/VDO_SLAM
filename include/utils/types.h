#pragma once

#include <memory>
#include <vector>

namespace VDO_SLAM {


typedef float CvMatAccessType;
typedef double GtsamAccesType;

typedef int FrameId;
typedef int FeatureId;

// feature tracklets: pair.first = frameID; pair.second = featureID;
typedef std::pair<FrameId, FeatureId> FeatureTracklet;
typedef std::vector<FeatureTracklet> FeatureTrackletVector;
typedef std::vector<FeatureTrackletVector> FeatureTrackletMatrix;


} //VDO_SLAM