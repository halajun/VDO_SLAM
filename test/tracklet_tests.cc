#include "backend/Tracklet.h"
#include <vector>
#include <glog/logging.h>


using namespace VDO_SLAM;
int main() {

    Tracklet<int, 2> tracklet(0);

    std::vector<std::pair<int, int>> frontend_tracks;
    frontend_tracks.push_back({0, 1});
    tracklet.update(frontend_tracks, 0);

    CHECK(!tracklet.isWellTracked());
    CHECK(tracklet.isNew());

    frontend_tracks.push_back({1, 2});
    tracklet.update(frontend_tracks, 0);
    CHECK(tracklet.isWellTracked());
    CHECK(tracklet.isNew());

    frontend_tracks.push_back({2, 3});
    tracklet.update(frontend_tracks, 0);
    CHECK(tracklet.isWellTracked());
    CHECK(tracklet.isNew());

    for(auto& obs : tracklet.getNotAddedAndMark()) {
        LOG(INFO) << obs->to_string();
    }

    CHECK(tracklet.getNotAdded().size() == 0);

    frontend_tracks.push_back({3, 4});
    tracklet.update(frontend_tracks, 0);
    CHECK(tracklet.isWellTracked());
    CHECK(tracklet.getNotAdded().size() == 1);

    //should just be 3,4
    for(auto& obs : tracklet.getNotAdded()) {
        LOG(INFO) << obs->to_string();
        obs->was_added = true;
    }

    CHECK(tracklet.getNotAdded().size() == 0);


    TrackletManager<int, 3> tracklet_manager;
    std::vector<std::vector<std::pair<int, int>>> tracklets;
    tracklets.push_back(frontend_tracks);
    tracklets.push_back({{0, 8}, {1, 10}});

    tracklet_manager.update(tracklets);
    CHECK(tracklet_manager[0].size() == 4);
    CHECK(tracklet_manager[0].isWellTracked());
    CHECK(!tracklet_manager[1].isWellTracked());

    tracklet_manager[0].getNotAddedAndMark();
    tracklet_manager[1].getNotAddedAndMark();

    tracklets[1].push_back({2, 10});
    tracklets[1].push_back({3, 11});
    tracklet_manager.update(tracklets);

    CHECK(tracklet_manager[1].isWellTracked());
    CHECK(tracklet_manager[0].getNotAdded().size() == 0);
    CHECK(tracklet_manager[1].getNotAdded().size() == 2) << "Size was " << tracklet_manager[1].getNotAdded().size();

    tracklets[0].push_back({4, 12});
    tracklet_manager.update(tracklets);
    CHECK(tracklet_manager[0].getNotAdded().size() == 1);

    //should just be 4,12
    for(auto& obs : tracklet_manager[0].getNotAdded()) {
        LOG(INFO) << obs->to_string();
        obs->was_added = true;
    }

    //should just be [2,10], [3, 11]
    for(auto& obs : tracklet_manager[1].getNotAdded()) {
        LOG(INFO) << obs->to_string();
    }

    auto last = tracklet_manager[1][3];
    //shoudl be [3, 11]
    LOG(INFO) << last->to_string();
    //should be 2, 10
    LOG(INFO) << tracklet_manager[1].getPreviousObservation(last)->to_string();










}