#pragma once 

#include "backend/Observation.h"
#include "utils/macros.h"
#include "utils/types.h"

#include <gtsam/nonlinear/Values.h>
#include <glog/logging.h>
#include <vector>
#include <unordered_map>

#include <functional>

namespace VDO_SLAM {

// template<typedef T>
// using ConversionFunc = std::function<T (Map*)>;

//only works for single "thing" observation...
//N is minimum tracking allowed (eg. if N = 4, lanmark must be seen 4 times)
template<class T, size_t N>
class Tracklet {
    public:
        VDO_SLAM_POINTER_TYPEDEFS(Tracklet);
        using TypedObservation = Observation<T>;
        using TypedObservationPtr = typename Observation<T>::Ptr;
        using ObservationVector = std::vector<TypedObservation>;
        using ObservationPtrVector = std::vector<TypedObservationPtr>;
        const size_t MinObservations = N;


        Tracklet(int tracklet_id_)
        :   tracklet_id(tracklet_id_) {}

        //assume the prior tracklets dont change so this
        //incoming tracklet should have fontend_tracks_.size() - observations.size()
        // number of tracklets 
        void update(const std::vector<std::pair<int, int>>& fontend_tracks_, int tracklet_id_) {
            CHECK_EQ(tracklet_id, tracklet_id_);
            const size_t new_observations = fontend_tracks_.size() - observations.size();
            CHECK_GE(new_observations, 0);
            if(new_observations == 0) {
                // LOG(INFO) << "No new observations for tracklet " << tracklet_id;
                return;
            }

            // //assume 0:new_observations are the same
            for(size_t i = observations.size(); i < fontend_tracks_.size(); i++) {
                const std::pair<int, int>& track = fontend_tracks_[i];
                size_t tracklet_position = observations.size();
                observations.push_back(std::make_shared<Observation<T>>(
                    track.first, track.second, tracklet_position, tracklet_id));
            }

            

            CHECK_EQ(fontend_tracks_.size(), observations.size());
        }

        const TypedObservationPtr& operator[](std::size_t idx) const {
            CHECK(idx < size());
            return observations[idx];
        }


        //access via track id and then feature in vector
        TypedObservationPtr operator[](std::size_t idx) {
            CHECK(idx < size());
            return observations[idx];
        }

        //access via track id and then feature in vector
        TypedObservationPtr getPreviousObservation(TypedObservationPtr current_obs) {
            CHECK(current_obs->tracklet_position > 0);
            CHECK(current_obs->tracklet_id == tracklet_id);
            return observations[current_obs->tracklet_position-1];
        }


        size_t size() const {
            return observations.size();
        }

        ObservationPtrVector getNotAdded() {
            ObservationPtrVector observed;
            for(TypedObservationPtr obs : observations) {
                if(!obs->was_added) {
                    observed.push_back(obs);
                }
            }
            return observed;
        }

        ObservationPtrVector getNotAddedAndMark() {
            ObservationPtrVector observed = getNotAdded();
            markAsAdded(observed);
            return observed;
        }

        void markAsAdded(ObservationPtrVector& tracks_to_add) {
            for(TypedObservationPtr obs : tracks_to_add) {
                // CHECK(!obs->was_added);
                obs->was_added = true;
            }
        }

        //if nothing has been added yet
        bool isNew() {
            ObservationPtrVector tracks = getNotAdded();
            return tracks.size() == observations.size();
        }

        //if there are enough measurements ... 
        //NOTE: this is currently a frame 2 frame appraoch so assume tracked means that
        //each tracked feature is seen at immediately adjacent frames (ie. frame num increases)
        //is this a good assumption to make or will it break?
        bool isWellTracked() const {
            return size() >= MinObservations;
        }

        const int TrackletId() const  { return tracklet_id; }

        ObservationPtrVector Observations() { return observations; }

        // T convert(FrameId frame_id, FeatureId point_id, Map* map);

    private:
        //if has a starting value
        bool is_init = false;
        int tracklet_id;

        //access should be with [j] ie pointId
        ObservationPtrVector observations;

};

struct FrameFeaturePair {
    FrameId frame_id = 0;
    FeatureId feature_id = 0;

    FrameFeaturePair() {}

    FrameFeaturePair(FrameId frame_id_, FeatureId feature_id_)
    :   frame_id(frame_id_),
        feature_id(feature_id_) {}

    bool operator==(const FrameFeaturePair &other) const {
        return (frame_id == other.frame_id
                && feature_id == other.feature_id);
  }
};

struct FrameFeatureHash {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// number was arbitrarily chosen with no good justification
  static constexpr size_t sl = 17191;
  static constexpr size_t sl2 = sl * sl;

  std::size_t operator()(const FrameFeaturePair& index) const {
    return static_cast<unsigned int>(index.frame_id * sl +
                                     index.feature_id * sl2);
  }
};

template <typename ValueType>
struct FrameFeatureHashMapType {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::unordered_map<
      FrameFeaturePair, ValueType, FrameFeatureHash>
      type;
};

template<typename T>
using FrameFeatureObservationMap = typename FrameFeatureHashMapType<typename Observation<T>::Ptr>::type;


template<class T, size_t N>
class TrackletManager {
    public:
        using TypedTracklet = Tracklet<T, N>;
        using TrackletList = std::vector<TypedTracklet>;
        using Observations = typename TypedTracklet::ObservationPtrVector;
        using Observation = typename TypedTracklet::TypedObservationPtr;

        TrackletManager() {}

        //all the tracklets from the front end
        //we should already have tracklets.size() number of them but they need to be updated
        void update(const std::vector<std::vector<std::pair<int, int>>>& frontend_tracklets_) {
            const size_t new_tracklets = frontend_tracklets_.size() - tracklets.size();
            //update for existing tracklets
            for(size_t i = 0; i < tracklets.size(); i++) {
                tracklets[i].update(frontend_tracklets_[i], i);
            }

            //sanity check
            CHECK_EQ(tracklets.size() + new_tracklets, frontend_tracklets_.size());

            //make and update new tracklets
            for(size_t i = tracklets.size(); i < frontend_tracklets_.size(); i++) {
                Tracklet<T, N> tracklet(i);
                tracklets.push_back(tracklet);
                tracklets[i].update(frontend_tracklets_[i], i);
            }

            CHECK_EQ(frontend_tracklets_.size(), tracklets.size());
            updateFremeFeatureIdMap();

        }

        bool exists(FrameId frame_id, FeatureId feature_id) {
            FrameFeaturePair key(frame_id, feature_id);
            return !(frame_feature_obs_map.find(key) == frame_feature_obs_map.end());
        }

        typename TypedTracklet::TypedObservationPtr getObservation(FrameId frame_id, FeatureId feature_id) {
            FrameFeaturePair key(frame_id, feature_id);
            if (frame_feature_obs_map.find(key) == frame_feature_obs_map.end()) {
                return nullptr;
            } 
            else {
                return frame_feature_obs_map[key];
            } 
        }


        TypedTracklet& getTracklet(FrameId frame_id, FeatureId feature_id) {
            CHECK(exists(frame_id, feature_id));
            typename TypedTracklet::TypedObservationPtr obs = getObservation(frame_id, feature_id);
            return tracklets[obs->tracklet_id];
        }


        TypedTracklet& operator[](std::size_t idx) {
            CHECK(idx < size());
            return tracklets[idx];
        }

        const TypedTracklet& operator[](std::size_t idx) const {
            CHECK(idx < size());
            return tracklets[idx];
        }

        size_t size() const {
            return tracklets.size();
        }

    private:
        void updateFremeFeatureIdMap() {
            for(TypedTracklet& track : tracklets) {
                for(size_t i = 0; i < track.size(); i++) {
                    FrameFeaturePair pair;
                    pair.frame_id = track[i]->frame_id;
                    pair.feature_id = track[i]->point_id;
                    frame_feature_obs_map[pair] = track[i];
                }
            }
        }
        

        TrackletList tracklets;
        FrameFeatureObservationMap<T> frame_feature_obs_map;

        //i index should get all the obsevrations from the same frame
        // std::vector<typename TypedTracklet::ObservationPtrVector> frame_feature_list;



};



}