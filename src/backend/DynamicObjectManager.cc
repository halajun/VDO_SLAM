#include "backend/DynamicObjectManager.h"
#include "utils/macros.h"
#include "utils/types.h"
#include "Map.h"

#include <glog/logging.h>
#include <vector>

namespace VDO_SLAM {

DynamicObjectManager::DynamicObjectManager(Map* map_, DynamicTrackletManager* tracklet_manager_)
    :   map(CHECK_NOTNULL(map_)),
        tracklet_manager(CHECK_NOTNULL(tracklet_manager_)) {}

void DynamicObjectManager::update(int current_frame) {
    for(size_t tracklet_manager = 0; point_id < map->vpFeatDyn[current_frame].size(); point_id++) {
        if(dynamic_tracklets.exists(current_frame, point_id)) {

        }
    }
}

DynamicObject::Ptr DynamicObjectManager::operator[](int unique_label) const {
    if(object_map.find(unique_label) == object_map.end()) {
        return nullptr;
    }
    return object_map[unique_label];
}


}