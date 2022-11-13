#pragma once 


#include "utils/macros.h"
#include "utils/types.h"

#include <vector>

namespace VDO_SLAM {

class DynamicObject;
class Map;

class DynamicObjectManager {
    public:
        VDO_SLAM_POINTER_TYPEDEFS(DynamicObjectManager);

        DynamicObjectManager(Map* map_); 
        //a lot of this is going to assume that the tracking is never reasigned 
        //from the front end... (which we dont in the formulattion)
        void process(const FrameId frame_id);

    private:
        Map* map;
    
};

}