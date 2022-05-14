#pragma once

#include <memory>
#include <vector>

namespace VDO_SLAM {


typedef float CvMatAccessType;
typedef double GtsamAccesType;

// feature tracklets: pair.first = frameID; pair.second = featureID;
typedef std::pair<int, int> FeatureTracklet;
typedef std::vector<FeatureTracklet> FeatureTrackletVector;
typedef std::vector<FeatureTrackletVector> FeatureTrackletMatrix;


} //VDO_SLAM