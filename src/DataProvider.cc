#include "DataProvider.h"
#include "UtilsGtsam.h"

#include <opencv2/optflow.hpp>
#include <glog/logging.h>
#include <iostream>

namespace vdo
{
size_t DataProvider::next(Inputs& input)
{
  return next(input.first, input.second);
}

KittiSequenceDataProvider::KittiSequenceDataProvider(const std::string& path_to_sequence_)
{
  LOG(INFO) << "Constructing Kitti Sequence Data provider - " << path_to_sequence_;
  loadData(path_to_sequence_, data);

  LOG(INFO) << "Loaded data " << data.size();
}

size_t KittiSequenceDataProvider::size() const
{
  return data.size();
}

size_t KittiSequenceDataProvider::next(InputPacket& input_packet, GroundTruthInputPacket::Optional ground_truth)
{
  if (index >= data.size())
  {
    return 0;
  }

  input_packet = data[index].first;

  if (ground_truth)
  {
    *ground_truth = data[index].second;
  }

  index++;
  return 1;
}

bool KittiSequenceDataProvider::loadData(const std::string& path_to_sequence, InputsVector& inputs_vector)
{
  // +++ timestamps +++
  std::vector<std::string> vstrFilenamesRGB;
  std::vector<std::string> vstrFilenamesDEP;
  std::vector<std::string> vstrFilenamesSEM;
  std::vector<std::string> vstrFilenamesFLO;
  std::vector<gtsam::Pose3> vPoseGT;
  std::vector<ObjectPoseGT> vObjPoseGT;
  std::vector<Timestamp> vTimestamps;

  std::ifstream times_stream;
  std::string strPathTimeFile = path_to_sequence + "/times.txt";
  times_stream.open(strPathTimeFile.c_str());
  while (!times_stream.eof())
  {
    std::string s;
    getline(times_stream, s);
    if (!s.empty())
    {
      std::stringstream ss;
      ss << s;
      double t;
      ss >> t;
      vTimestamps.push_back(t);
    }
  }
  times_stream.close();

  LOG(INFO) << "N frame - " << vTimestamps.size();

  // +++ image, depth, semantic and moving object tracking mask +++
  std::string strPrefixImage = path_to_sequence + "/image_0/";      // image  image_0
  std::string strPrefixDepth = path_to_sequence + "/depth/";        // depth_gt  depth  depth_mono_stereo
  std::string strPrefixSemantic = path_to_sequence + "/semantic/";  // semantic_gt  semantic
  std::string strPrefixFlow = path_to_sequence + "/flow/";          // flow_gt  flow

  const int nTimes = vTimestamps.size();
  vstrFilenamesRGB.resize(nTimes);
  vstrFilenamesDEP.resize(nTimes);
  vstrFilenamesSEM.resize(nTimes);
  vstrFilenamesFLO.resize(nTimes);

  for (int i = 0; i < nTimes; i++)
  {
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(6) << i;
    vstrFilenamesRGB[i] = strPrefixImage + ss.str() + ".png";
    vstrFilenamesDEP[i] = strPrefixDepth + ss.str() + ".png";
    vstrFilenamesSEM[i] = strPrefixSemantic + ss.str() + ".txt";
    vstrFilenamesFLO[i] = strPrefixFlow + ss.str() + ".flo";
  }

  // +++ ground truth pose +++
  std::string strFilenamePose = path_to_sequence + "/pose_gt.txt";  //  pose_gt.txt  kevin_extrinsics.txt
  // vPoseGT.resize(nTimes);
  std::ifstream fPose;
  fPose.open(strFilenamePose.c_str());
  while (!fPose.eof())
  {
    std::string s;
    getline(fPose, s);
    if (!s.empty())
    {
      std::stringstream ss;
      ss << s;
      int t;
      ss >> t;
      cv::Mat Pose_tmp = cv::Mat::eye(4, 4, CV_64F);

      ss >> Pose_tmp.at<double>(0, 0) >> Pose_tmp.at<double>(0, 1) >> Pose_tmp.at<double>(0, 2) >>
          Pose_tmp.at<double>(0, 3) >> Pose_tmp.at<double>(1, 0) >> Pose_tmp.at<double>(1, 1) >>
          Pose_tmp.at<double>(1, 2) >> Pose_tmp.at<double>(1, 3) >> Pose_tmp.at<double>(2, 0) >>
          Pose_tmp.at<double>(2, 1) >> Pose_tmp.at<double>(2, 2) >> Pose_tmp.at<double>(2, 3) >>
          Pose_tmp.at<double>(3, 0) >> Pose_tmp.at<double>(3, 1) >> Pose_tmp.at<double>(3, 2) >>
          Pose_tmp.at<double>(3, 3);

      gtsam::Pose3 pose = utils::cvMatToGtsamPose3(Pose_tmp);
      vPoseGT.push_back(pose);
      // if(t==410)
      //     cout << "ground truth pose 0 (for validation):" << endl << vPoseGT[t] << endl;
    }
  }
  fPose.close();

  // +++ ground truth object pose +++
  std::string strFilenameObjPose = path_to_sequence + "/object_pose.txt";
  std::ifstream fObjPose;
  fObjPose.open(strFilenameObjPose.c_str());

  while (!fObjPose.eof())
  {
    std::string s;
    getline(fObjPose, s);
    if (!s.empty())
    {
      std::stringstream ss;
      ss << s;

      // FrameID ObjectID B1 B2 B3 B4 t1 t2 t3 r1
      // Where ti are the coefficients of 3D object location **t** in camera coordinates, and r1 is the Rotation around
      // Y-axis in camera coordinates.
      // B1-4 is 2D bounding box of object in the image, used for visualization.
      std::vector<double> ObjPose_tmp(10, 0);
      ss >> ObjPose_tmp[0] >> ObjPose_tmp[1] >> ObjPose_tmp[2] >> ObjPose_tmp[3] >> ObjPose_tmp[4] >> ObjPose_tmp[5] >>
          ObjPose_tmp[6] >> ObjPose_tmp[7] >> ObjPose_tmp[8] >> ObjPose_tmp[9];

      ObjectPoseGT object_pose;
      object_pose.frame_id = ObjPose_tmp[0];
      object_pose.object_id = ObjPose_tmp[1];
      object_pose.bounding_box.b1 = ObjPose_tmp[2];
      object_pose.bounding_box.b2 = ObjPose_tmp[3];
      object_pose.bounding_box.b3 = ObjPose_tmp[4];
      object_pose.bounding_box.b4 = ObjPose_tmp[5];
      object_pose.pose = ObjPoseParsingKT(ObjPose_tmp);
      // object_pose.translation(0) = ObjPose_tmp[6];
      // object_pose.translation(1) = ObjPose_tmp[7];
      // object_pose.translation(2) = ObjPose_tmp[8];
      // object_pose.r1 = ObjPose_tmp[9];

      // LOG(INFO) << "Object frame id  " << object_pose.frame_id << " object ID " << object_pose.object_id;

      vObjPoseGT.push_back(object_pose);
    }
  }
  fObjPose.close();

  // organise gt poses into vector of arrays
  VectorsXi vObjPoseID(vstrFilenamesRGB.size());
  LOG(INFO) << vObjPoseGT.size();
  for (size_t i = 0; i < vObjPoseGT.size(); ++i)
  {
    size_t f_id = vObjPoseGT[i].frame_id;
    vObjPoseID[f_id].push_back(i);
  }
  LOG(INFO) << vObjPoseID.size();

  // now read image image and add grount truths
  for (size_t frame_id = 0; frame_id < nTimes - 1; frame_id++)
  {
    // LOG(INFO) << "Loading data at - frame ID " << frame_id;
    Timestamp timestamp = vTimestamps[frame_id];
    GroundTruthInputPacket gt_packet;
    gt_packet.timestamp = timestamp;
    gt_packet.frame_id = frame_id;
    // add ground truths for this fid
    // gt_packet.obj_poses.resize(vObjPoseID[frame_id].size());
    for (int i = 0; i < vObjPoseID[frame_id].size(); i++)
    {
      gt_packet.obj_poses.push_back(vObjPoseGT[vObjPoseID[frame_id][i]]);
      // sanity check
      CHECK_EQ(gt_packet.obj_poses[i].frame_id, frame_id);
    }
    gt_packet.X_wc = vPoseGT[frame_id];

    // add input packets
    cv::Mat rgb, depth, flow;
    // Read imreadmage and depthmap from file
    rgb = cv::imread(vstrFilenamesRGB[frame_id], CV_LOAD_IMAGE_UNCHANGED);
    depth = cv::imread(vstrFilenamesDEP[frame_id], CV_LOAD_IMAGE_UNCHANGED);
    // / For stereo disparity input
    depth.convertTo(depth, CV_64F);

    flow = cv::optflow::readOpticalFlow(vstrFilenamesFLO[frame_id]);

    cv::Mat sem(rgb.rows, rgb.cols, CV_32SC1);
    loadSemanticMask(vstrFilenamesSEM[frame_id], sem);

    CHECK(!rgb.empty());
    CHECK(!depth.empty());
    CHECK(!flow.empty());
    CHECK(!sem.empty());

    InputPacket input(timestamp, frame_id, rgb, depth, flow, sem);

    inputs_vector.push_back(std::make_pair(std::move(input), std::move(gt_packet)));
  }

  return true;
}

void KittiSequenceDataProvider::loadSemanticMask(const std::string& strFilenamesMask, cv::Mat& mask)
{
  std::ifstream file_mask;
  file_mask.open(strFilenamesMask.c_str());

  // Main loop
  int count = 0;
  while (!file_mask.eof())
  {
    std::string s;
    getline(file_mask, s);
    if (!s.empty())
    {
      std::stringstream ss;
      ss << s;
      int tmp;
      for (int i = 0; i < mask.cols; ++i)
      {
        ss >> tmp;
        if (tmp != 0)
        {
          mask.at<int>(count, i) = tmp;
        }
        else
        {
          mask.at<int>(count, i) = 0;
        }
      }
      count++;
    }
  }

  file_mask.close();
}

gtsam::Pose3 KittiSequenceDataProvider::ObjPoseParsingKT(const std::vector<double>& obj_pose_gt)
{
  CHECK(obj_pose_gt.size() == 10);

  cv::Mat t(3, 1, CV_64FC1);
  t.at<double>(0) = obj_pose_gt[6];
  t.at<double>(1) = obj_pose_gt[7];
  t.at<double>(2) = obj_pose_gt[8];

  // from Euler to Rotation Matrix
  cv::Mat R(3, 3, CV_64FC1);

  // assign r vector
  double y = obj_pose_gt[9] + (3.1415926 / 2);  // +(3.1415926/2)
  double x = 0.0;
  double z = 0.0;

  // the angles are in radians.
  double cy = cos(y);
  double sy = sin(y);
  double cx = cos(x);
  double sx = sin(x);
  double cz = cos(z);
  double sz = sin(z);

  double m00, m01, m02, m10, m11, m12, m20, m21, m22;

  m00 = cy * cz + sy * sx * sz;
  m01 = -cy * sz + sy * sx * cz;
  m02 = sy * cx;
  m10 = cx * sz;
  m11 = cx * cz;
  m12 = -sx;
  m20 = -sy * cz + cy * sx * sz;
  m21 = sy * sz + cy * sx * cz;
  m22 = cy * cx;

  R.at<double>(0, 0) = m00;
  R.at<double>(0, 1) = m01;
  R.at<double>(0, 2) = m02;
  R.at<double>(1, 0) = m10;
  R.at<double>(1, 1) = m11;
  R.at<double>(1, 2) = m12;
  R.at<double>(2, 0) = m20;
  R.at<double>(2, 1) = m21;
  R.at<double>(2, 2) = m22;

  // construct 4x4 transformation matrix
  cv::Mat Pose = cv::Mat::eye(4, 4, CV_64F);
  Pose.at<double>(0, 0) = R.at<double>(0, 0);
  Pose.at<double>(0, 1) = R.at<double>(0, 1);
  Pose.at<double>(0, 2) = R.at<double>(0, 2);
  Pose.at<double>(0, 3) = t.at<double>(0);
  Pose.at<double>(1, 0) = R.at<double>(1, 0);
  Pose.at<double>(1, 1) = R.at<double>(1, 1);
  Pose.at<double>(1, 2) = R.at<double>(1, 2);
  Pose.at<double>(1, 3) = t.at<double>(1);
  Pose.at<double>(2, 0) = R.at<double>(2, 0);
  Pose.at<double>(2, 1) = R.at<double>(2, 1);
  Pose.at<double>(2, 2) = R.at<double>(2, 2);
  Pose.at<double>(2, 3) = t.at<double>(2);

  return utils::cvMatToGtsamPose3(Pose);
}

}  // namespace vdo