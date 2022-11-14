#include "backend/DynamicObjectManager.h"
#include "utils/macros.h"
#include "utils/types.h"
#include "Map.h"

#include <glog/logging.h>
#include <vector>

namespace VDO_SLAM
{
DynamicObjectManager::DynamicObjectManager(Map* map_) : map(CHECK_NOTNULL(map_))
{
}

void DynamicObjectManager::process(const FrameId frame_id)
{
  LOG(INFO) << "DOM: processing frame: " << frame_id;

  // get all labels from pMap->vnRMLabel[frame_id]:

  // make unnique?
  // if in seconod frame or more
  // loop through vnRMLabel and check if that label has been seen before
  // if so increase the count, indicating it has been seen again

  // for all in windows and for all in vnRMLabel
  // for all unique labels
  // if the count on a specific label is >= tracking length,
  // makr obj_check[frame][lables] = true, where labels j is also the number of motions
}

}  // namespace VDO_SLAM