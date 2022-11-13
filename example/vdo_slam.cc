#include <glog/logging.h>
#include "DataProvider.h"
#include "System.h"

int main(int argc, char **argv)

{
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();


    PCHECK(argc == 3) << "Usage: ./vdo_slam path_to_settings path_to_sequence";

    vdo::DataProvider::UniquePtr data_provider = vdo::make_unique<vdo::KittiSequenceDataProvider>(argv[2]);

    vdo::System system(argv[1]);

    vdo::InputPacket input;
    vdo::GroundTruthInputPacket gt;
    while(data_provider->next(input, gt)) {
        // LOG(INFO) << "Spinning on FIS " << input.frame_id << " " << gt.frame_id;
        // system.TrackRGBD(input, gt);
    }

}