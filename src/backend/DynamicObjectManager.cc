#include "backend/DynamicObjectManager.h"
#include "utils/macros.h"
#include "utils/types.h"
#include "Map.h"

#include <glog/logging.h>
#include <vector>

namespace VDO_SLAM {

DynamicObjectManager::DynamicObjectManager(Map* map_)
    :   map(CHECK_NOTNULL(map_)) {}

void DynamicObjectManager::process(const FrameId frame_id) {
    LOG(INFO) << "DOM: processing frame: " << frame_id;
}

}