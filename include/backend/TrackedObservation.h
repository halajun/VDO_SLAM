#pragma once 

#include "backend/Observation.h"
#include "utils/macros.h"
#include "utils/types.h"

#include <gtsam/nonlinear/Values.h>
#include <glog/logging.h>
#include <vector>

namespace VDO_SLAM {

//only works for single "thing" observation...
//N is minimum tracking allowed (eg. if N = 4, lanmark must be seen 4 times)
template<class T, size_t N>
class TrackedObservation {
    public:
        VDO_SLAM_POINTER_TYPEDEFS(TrackedObservation);
        using TypedObservation = Observation<T>;
        using ObservationVector = std::vector<TypedObservation>;
        using TypedObservationPtr = typename TypedObservation::Ptr;


        TrackedObservation(const TypedObservationPtr& obs)
            :   is_init(true) {
                initWithFirstObservation(CHECK_NOTNULL(obs));
            }

        TrackedObservation()
            :   is_init(false) {}

        void insert(const TypedObservationPtr& obs) {
            CHECK_NOTNULL(obs);
            CHECK_EQ(obs->key, landmark_key);
            if(!is_init) {
                initWithFirstObservation(obs);
                return;
            }

            observations.push_back(obs);
        }

        size_t getTotalObservations() const {
            return observations.size();
        }

        //if there are enough measurements ... 
        //NOTE: this is currently a frame 2 frame appraoch so assume tracked means that
        //each tracked feature is seen at immediately adjacent frames (ie. frame num increases)
        //is this a good assumption to make or will it break?
        bool isWellTracked() const {
            return getTotalObservations() > N;
        }

    private:
        //if has a starting value
        bool is_init = false;
        gtsam::Key landmark_key;

        ObservationVector observations;


    private:
        void initWithFirstObservation(const TypedObservationPtr& obs) {
            is_init = true;
            landmark_key = obs->landmark_key;
            observations.push_back(obs);
        }


};

}