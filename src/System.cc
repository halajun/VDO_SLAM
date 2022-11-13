#include "System.h"
#include "Tracking.h"
#include "Camera.h" //for camera params
#include "utils/ParamParser.h"

namespace vdo {

System::System(const std::string& settings_file) {

	utils::ParamParser parser(settings_file);

	//load TrackingParams
	TrackingParams tracking_params;
	parser.getParam("MaxTrackPointBG", &tracking_params.max_tracking_points_bg);
	parser.getParam("MaxTrackPointOBJ", &tracking_params.max_tracking_points_obj);

	parser.getParam("SFMgThres", &tracking_params.scene_flow_magnitude);
	parser.getParam("SFDsThres", &tracking_params.scene_flow_percentage);

	parser.getParam("ThDepthBG", &tracking_params.depth_background_thresh);
	parser.getParam("ThDepthOBJ", &tracking_params.depth_obj_thresh);

	parser.getParam("DepthMapFactor", &tracking_params.depth_scale_factor);

	parser.getParam("ORBextractor.nFeatures", &tracking_params.n_features);
	parser.getParam("ORBextractor.scaleFactor", &tracking_params.scale_factor);
	parser.getParam("ORBextractor.nLevels", &tracking_params.n_levels);
	parser.getParam("ORBextractor.iniThFAST", &tracking_params.init_threshold_fast);
	parser.getParam("ORBextractor.minThFAST", &tracking_params.min_threshold_fast);

	std::stringstream ss;
	ss << "Tracking Params:" << std::endl;
	ss << "- max tracking points: " << "(1) background: " << tracking_params.max_tracking_points_bg << " (2) object: " << tracking_params.max_tracking_points_obj << std::endl;
	ss << "- Depth scale factor: " << tracking_params.depth_scale_factor << std::endl;
	ss << "- scene flow paras: " << "(1) magnitude: " << tracking_params.scene_flow_magnitude << " (2) percentage: " << tracking_params.scene_flow_percentage;
	LOG(INFO) << ss.str();


	CameraParams::Intrinsics intrinsics(4);
	parser.getParam("Camera.fx", &intrinsics[0]);
	parser.getParam("Camera.fy", &intrinsics[1]);
	parser.getParam("Camera.cx", &intrinsics[2]);
	parser.getParam("Camera.cy", &intrinsics[3]);

	CameraParams::Distortion distortion(4);
	parser.getParam("Camera.k1", &distortion[0]);
    parser.getParam("Camera.k2", &distortion[1]);
    parser.getParam("Camera.p1", &distortion[2]);
    parser.getParam("Camera.p2", &distortion[3]);

	double k3 = 0;
    parser.getParam("Camera.k3", &k3);
    if(k3!=0)
    {
        distortion.resize(5);
        distortion[4 ]= k3;
    }

	double width, height;
	parser.getParam("Camera.width", &width);
    parser.getParam("Camera.height", &height);
	cv::Size image_size(width, height);

	std::string distortion_model = "none";

	double baseline;
	parser.getParam("Camera.bf", &baseline);

	CameraParams camera_params(intrinsics, distortion, image_size, distortion_model, baseline);
	LOG(INFO) << camera_params.toString();

	Camera camera(camera_params);
	tracker = vdo::make_unique<Tracking>(tracking_params, camera);

}

gtsam::Pose3 System::TrackRGBD(
        const InputPacket& input, 
        boost::optional<const GroundTruthInputPacket&> ground_truth) {
	
	tracker->process(input, ground_truth);
	return gtsam::Pose3::identity();
        
}

}