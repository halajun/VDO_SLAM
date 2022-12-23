#include "viz/OpenCvDisplay.h"
#include "utils/UtilsOpenCV.h"

#include "Frame.h"

#include <set>

namespace vdo
{
OpenCvDisplay::OpenCvDisplay(DisplayParams::Ptr params_) : Display(params_)
{
}

void OpenCvDisplay::process(const VisualiserInput& viz_input)
{
  if (!params->use_2d_viz)
  {
    return;
  }

  if (params->display_input)
  {
    drawInputImages(*viz_input.frontend_output->frame_);
  }

  if (params->display_frame)
  {
    drawFrame(*viz_input.frontend_output->frame_);
  }

  for (const ImageToDisplay& display : display_images)
  {
    cv::imshow(display.title, display.image);
  }

  cv::waitKey(1);
  display_images.clear();
}

void OpenCvDisplay::drawInputImages(const Frame& frame)
{
  // draw each portion of the inputs
  cv::Mat rgb, depth, flow, mask;
  frame.images_.rgb.copyTo(rgb);
  CHECK(rgb.channels() == 3) << "Expecting rgb in frame to gave 3 channels";

  frame.images_.depth.copyTo(depth);
  // expect depth in float 32
  depth.convertTo(depth, CV_8UC1);

  frame.images_.flow.copyTo(flow);

  frame.images_.semantic_mask.copyTo(mask);

  // canot display the original ones so these needs special treatment...
  cv::Mat flow_viz, mask_viz;
  drawOpticalFlow(flow, flow_viz);
  drawSemanticInstances(rgb, mask, mask_viz);

  cv::Mat top_row = utils::concatenateImagesHorizontally(rgb, depth);
  cv::Mat bottom_row = utils::concatenateImagesHorizontally(flow_viz, mask_viz);
  cv::Mat input_images = utils::concatenateImagesVertically(top_row, bottom_row);

  // reisize images to be the original image size
  cv::resize(input_images, input_images, cv::Size(rgb.cols, rgb.rows), 0, 0, CV_INTER_LINEAR);
  addDisplayImages(input_images, "Input Images");
}

void OpenCvDisplay::drawFrame(const Frame& frame)
{
  cv::Mat frame_viz, rgb;
  frame.images_.rgb.copyTo(rgb);
  CHECK(rgb.channels() == 3) << "Expecting rgb in frame to gave 3 channels";

  // drawTracklet(rgb, frame_viz, input.static_tracklets, input.map);

  drawFeatures(rgb, frame, frame_viz);
  addDisplayImages(frame_viz, "Current Frame");
}

void OpenCvDisplay::drawFeatures(const cv::Mat& rgb, const Frame& frame, cv::Mat& frame_viz)
{
  rgb.copyTo(frame_viz);
}

void OpenCvDisplay::drawOpticalFlow(const cv::Mat& flow, cv::Mat& flow_viz)
{
  utils::flowToRgb(flow, flow_viz);
}

void OpenCvDisplay::drawSemanticInstances(const cv::Mat& rgb, const cv::Mat& mask, cv::Mat& mask_viz)
{
  CHECK_EQ(rgb.size, mask.size) << "Input rgb and mask image must have the same size";
  rgb.copyTo(mask_viz);
  CHECK(mask.channels() == 1) << "Expecting mask input to have channels 1";
  CHECK(mask.depth() == CV_32SC1);

  for (int i = 0; i < mask.rows; i++)
  {
    for (int j = 0; j < mask.cols; j++)
    {
      // background is zero
      if (mask.at<int>(i, j) != 0)
      {
        Color color = getColourFromInstanceMask(mask.at<int>(i, j));
        // rgb or bgr?
        mask_viz.at<cv::Vec3b>(i, j)[0] = color.r;
        mask_viz.at<cv::Vec3b>(i, j)[1] = color.g;
        mask_viz.at<cv::Vec3b>(i, j)[2] = color.b;
      }
    }
  }
}

void OpenCvDisplay::addDisplayImages(const cv::Mat& image, const std::string& title)
{
  display_images.push_back(ImageToDisplay(image, title));
}

Color OpenCvDisplay::getColourFromInstanceMask(int value)
{
  return rainbowColorMap(value);
}

}  // namespace vdo