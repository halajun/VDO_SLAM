#pragma once 

#include "backend/Tracklet.h"
#include "backend/Tracklet-Definitions.h"
#include "backend/VdoSlamBackend-types.h"

#include <map>
#include <vector>
#include <memory>

namespace VDO_SLAM {

struct DynamicObject {
    typedef std::shared_ptr<DynamicObject> Ptr;


    int unique_label = -1;
    std::vector<Slot> associated_factors;
    std::vector<gtsam::Key> associated_values;
};


class DynamicObjectManager {
    public:
        VDO_SLAM_POINTER_TYPEDEFS(DynamicObjectManager);

        DynamicObjectManager(Map* map_, DynamicTrackletManager* tracklet_manager_); 

        void update(int current_frame);


        DynamicObject::Ptr operator[](int unique_label) const;

    private:
        Map* map;
        DynamicTrackletManager* tracklet_manager;

        std::map<DynamicObject::Ptr> object_map;
        
        //which objects (via object ID) where seen in each frame
        //first vector should be same as number of
        std::vector<std::vector<int>> object_history;

    
};

}