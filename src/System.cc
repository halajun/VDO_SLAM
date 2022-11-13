#include "System.h"

namespace vdo {

System::System(const std::string& settings_file) {
        
}

gtsam::Pose3 System::TrackRGBD(
        const InputPacket& input, 
        boost::optional<const GroundTruthInputPacket&> ground_truth) {
        
}

}