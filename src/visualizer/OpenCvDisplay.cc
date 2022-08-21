#include "visualizer/OpenCvDisplay.h"
#include "utils/UtilsOpenCv.h"

#include "Map.h"

#include <set>

namespace VDO_SLAM {

OpenCvDisplay::OpenCvDisplay(DisplayParams::Ptr params_)
    :   Display(params_) {}

void OpenCvDisplay::addInput(const Display2DInput& input) {
    if(!params->use_2d_viz) {
        return;
    }

    if(params->display_input) {
        drawInputImages(input.frame);
    }

    if(params->display_frame) {
        drawFrame(input);
    }

}

void OpenCvDisplay::drawInputImages(const Frame& frame) {
    //draw each portion of the inputs
    cv::Mat rgb, depth, flow, mask;
    frame.rgb.copyTo(rgb);
    CHECK(rgb.channels() == 3) << "Expecting rgb in frame to gave 3 channels";

    frame.depth.copyTo(depth);
    //expect depth in float 32
    depth.convertTo(depth, CV_8UC1);

    frame.flow.copyTo(flow);

    frame.mask.copyTo(mask);

    //canot display the original ones so these needs special treatment...
    cv::Mat flow_viz, mask_viz;
    drawOpticalFlow(flow, flow_viz);
    drawSemanticInstances(rgb, mask, mask_viz);

    cv::Mat top_row = utils::concatenateImagesHorizontally(rgb, depth);
    cv::Mat bottom_row = utils::concatenateImagesHorizontally(flow_viz, mask_viz);
    cv::Mat input_images = utils::concatenateImagesVertically(top_row, bottom_row);

    //reisize images to be the original image size
    cv::resize(input_images, input_images, cv::Size(rgb.cols,rgb.rows), 0, 0, CV_INTER_LINEAR);
    addDisplayImages(input_images, "Input Images");

}

void OpenCvDisplay::drawTracklet(const cv::Mat& rgb, cv::Mat& rgb_tracks, const FeatureTrackletMatrix& static_tracks, const Map* map) {
    std::vector<std::vector<std::pair<int, int> > > StaTracks = map->TrackletSta;
    static_tracklets.update(StaTracks);
    rgb.copyTo(rgb_tracks);


    std::set<int> unique_tracks;

    const int current_frame = map->vpFeatSta.size() - 1;
    for(size_t point_id = 0; point_id < map->vpFeatSta[current_frame].size(); point_id+=3) {
        if(point_id >= map->vpFeatSta[current_frame].size()) {
            break;
        }
        // for (int point_id = 0; point_id < vnFeaLabSta[current_frame].size(); ++point_id) {
        if(static_tracklets.exists(current_frame, point_id)) {
            StaticTrackletManager::TypedTracklet tracklet = static_tracklets.getTracklet(current_frame, point_id);

            if(tracklet.isWellTracked()) {
                StaticTrackletManager::Observations obs_to_add = tracklet.getNotAdded();
                // LOG(INFO) << "obs to add " << obs_to_add.size() << " track Id " << tracklet.TrackletId();
                for (StaticTrackletManager::Observation obs : obs_to_add) {
                    FrameId frame_id = obs->frame_id;
                    FeatureId feature_id = obs->point_id;
                    int track_id = obs->tracklet_id;
                    // obs->was_added = true;
                    

                    //dont draw if for another frame
                    if (frame_id != current_frame) {
                        continue;
                    }

                    CHECK_EQ(unique_tracks.count(track_id), 0);
                    unique_tracks.insert(track_id);


                    cv::KeyPoint kp = map->vpFeatSta[frame_id][feature_id];
                    cv::Scalar colour = Display::getObjectColour(track_id);
                    utils::drawCircleInPlace(rgb_tracks, kp, colour);

                    int x= kp.pt.x;
                    int y= kp.pt.y;

                    cv::putText(rgb_tracks,std::to_string(track_id),
                        cv::Point2i(x-5, y-5),cv::FONT_HERSHEY_SIMPLEX, 0.3, colour);
                }

                tracklet.markAsAdded(obs_to_add);


            }
        }
    }


    // for(int t = 0; t < static_tracklets.size(); t++) {
    //     StaticTrackletManager::TypedTracklet tracklet = static_tracklets[t];
    //     for(int i = 0; i < tracklet.size(); i++) {
    //         StaticTrackletManager::Observation obs = tracklet[i];
    //         if(obs) {
    //             if(obs->frame_id == current_frame && obs->tracklet_position > 3) {
    //                 FrameId frame_id = obs->frame_id;
    //                 FeatureId feature_id = obs->point_id;
    //                 int track_id = obs->tracklet_id;
    //                 cv::KeyPoint kp = map->vpFeatSta[frame_id][feature_id];
    //                 cv::Scalar colour = Display::getObjectColour(track_id);
    //                 utils::drawCircleInPlace(rgb_tracks, kp, colour);

    //                 int x= kp.pt.x;
    //                 int y= kp.pt.y;

    //                 cv::putText(rgb_tracks,std::to_string(track_id),
    //                     cv::Point2i(x-5, y-5),cv::FONT_HERSHEY_SIMPLEX, 0.3, colour);
    //             }
    //         }
    //     }
        
    // }
   


}


void OpenCvDisplay::drawFrame(const Display2DInput& input) {
    const Frame& frame = input.frame;
    cv::Mat frame_viz, rgb;
    frame.rgb.copyTo(rgb);
    CHECK(rgb.channels() == 3) << "Expecting rgb in frame to gave 3 channels";

    drawTracklet(rgb, frame_viz, input.static_tracklets, input.map);

    // drawFeatures(rgb, frame, frame_viz);
    addDisplayImages(frame_viz, "Current Frame");
}

void OpenCvDisplay::drawFeatures(const cv::Mat& rgb, const Frame& frame, cv::Mat& frame_viz) {
    rgb.copyTo(frame_viz);
    //Temporal match subset is used for static objects
    //see renew frame for how the inlier outliers are used. Can categorise as inlier outliers?

    //i think i could use the nStaInlierID to index the mvStatKeys which is equivalent to mvStatKeysTmp
    for(int inlier : frame.nStaInlierID) {
        const cv::KeyPoint& kp = frame.mvStatKeys[inlier];
        utils::drawCircleInPlace(frame_viz, kp, cv::Scalar(0, 255, 0));
    }

    //draw dynamic objects
    //TODO: should be drawing object inliers?
    for (int i = 0; i < frame.vObjLabel.size(); ++i) {
        int dynamic_label = frame.vObjLabel[i];
        if(dynamic_label == -1 || dynamic_label == -2 ) {
            //log warning?
            continue;
        }

        cv::KeyPoint object_kp = frame.mvObjKeys[i];
        cv::Scalar colour = getObjectColour(dynamic_label);
        utils::drawCircleInPlace(frame_viz, object_kp, colour);
    }
}



void OpenCvDisplay::process() {
    for(const ImageToDisplay& display : display_images) {
        cv::imshow(display.title, display.image);
    }

    cv::waitKey(1);
    display_images.clear();
}

void OpenCvDisplay::drawOpticalFlow(const cv::Mat& flow, cv::Mat& flow_viz) {
    CHECK(flow.channels() == 2) << "Expecting flow in frame to have 2 channels";

    // Visualization part
    cv::Mat flow_parts[2];
    cv::split(flow, flow_parts);

    // Convert the algorithm's output into Polar coordinates
    cv::Mat magnitude, angle, magn_norm;
    cv::cartToPolar(flow_parts[0], flow_parts[1], magnitude, angle, true);
    cv::normalize(magnitude, magn_norm, 0.0f, 1.0f, cv::NORM_MINMAX);
    angle *= ((1.f / 360.f) * (180.f / 255.f));

    // Build hsv image
    cv::Mat _hsv[3], hsv, hsv8, bgr;
    _hsv[0] = angle;
    _hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
    _hsv[2] = magn_norm;
    cv::merge(_hsv, 3, hsv);
    hsv.convertTo(hsv8, CV_8U, 255.0);
    
    // Display the results
    cv::cvtColor(hsv8, flow_viz, cv::COLOR_HSV2BGR);
}

void OpenCvDisplay::drawSemanticInstances(const cv::Mat& rgb, const cv::Mat& mask, cv::Mat& mask_viz) {
    CHECK_EQ(rgb.size, mask.size) << "Input rgb and mask image must have the same size";
    rgb.copyTo(mask_viz);
    CHECK(mask.channels() == 1) << "Expecting mask input to have channels 1";
    CHECK(mask.depth() == CV_32SC1);

    for(int i=0; i< mask.rows; i++) {
        for(int j=0; j< mask.cols; j++) {
            //background is zero
            if(mask.at<int>(i,j) != 0) {
                Color color = getColourFromInstanceMask(mask.at<int>(i,j));
                //rgb or bgr?
                mask_viz.at<cv::Vec3b>(i,j)[0] = color.r;
                mask_viz.at<cv::Vec3b>(i,j)[1] = color.g;
                mask_viz.at<cv::Vec3b>(i,j)[2] = color.b;
            }
        }
    }
}





void OpenCvDisplay::addDisplayImages(const cv::Mat& image, const std::string& title) {
    display_images.push_back(ImageToDisplay(image, title));
}

Color OpenCvDisplay::getColourFromInstanceMask(int value) {
    return rainbowColorMap(value);
}

} //VDO_SLAM