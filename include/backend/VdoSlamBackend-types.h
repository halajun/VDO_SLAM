#pragma once

#include "utils/macros.h"
#include "utils/types.h"
#include "factors/Point3DFactor.h"

#include <glog/logging.h>
#include <iostream>
#include <vector>

namespace VDO_SLAM {

typedef boost::shared_ptr<Point3DFactor> Point3DFactorPtr;
typedef std::vector<Point3DFactorPtr> Point3DFactors;

// typedef std::vector<gtsam::Key> VertexKeys;




//quick reverse lookup for unique vertices
struct IJSymbol {
    FrameId i;
    FeatureId j;
    unsigned char symbol;
    
    IJSymbol()
        :   i(-1),
            j(-1),
            symbol('\0') {}
};


std::ostream& operator<<(std::ostream& os, const IJSymbol& symbol);

struct BackendDebugInfo {
    //from the isam2 results
    FrameId frame = 0;
    size_t num_marked_keys = 0;

    size_t num_new_poses = 0;
    size_t num_new_static_points = 0;

    //collection being finding all the new landmarks and any associated new factors
    size_t total_new_values = 0;
    size_t graph_size_before_collection = 0;
    size_t graph_size_after_collection = 0;


    void reset() {
        frame = 0;
        num_marked_keys = 0;
        num_new_poses = 0;
        num_new_static_points = 0;
        total_new_values = 0;
        graph_size_before_collection = 0;
        graph_size_after_collection = 0;
    }

    void print() const {
        LOG(INFO) << "\n-------- BackendDebugInfo: --------\n"
                  << " frame: " << frame << "\n"
                  << " num_marked_keys: " << num_marked_keys << "\n"
                  << " num_new_poses: " << num_new_poses << "\n"
                  << " num_new_static_points: " << num_new_static_points << "\n"
                  << " total_new_values: " << total_new_values << "\n"
                  << " graph_size_before_collection: " << graph_size_before_collection << "\n"
                  << " graph_size_after_collection: " << graph_size_after_collection;
    }
};


} //VDO_SLAM