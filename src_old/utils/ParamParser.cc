#include "utils/ParamParser.h"
#include <glog/logging.h>

namespace VDO_SLAM
{
namespace utils
{
ParamParser::ParamParser(const std::string& file_path_) : file_path(file_path_), fs()
{
  openFile(file_path);
}

ParamParser::~ParamParser()
{
  closeFile();
}

void ParamParser::openFile(const std::string& file_path_)
{
  CHECK(!file_path_.empty()) << "Empty filepath!";
  try
  {
    fs.open(file_path_, cv::FileStorage::READ);
    LOG(INFO) << "Loading param settings from: " << file_path_;
  }
  catch (cv::Exception& e)
  {
    LOG(FATAL) << "Cannot open file: " << file_path_ << '\n' << "OpenCV error code: " << e.msg;
  }
  LOG_IF(FATAL, !fs.isOpened()) << "Cannot open file in parseYAML: " << file_path_
                                << " (remember that the first line should be: %YAML:1.0)";
}

void ParamParser::closeFile()
{
  fs.release();
}

}  // namespace utils
}  // namespace VDO_SLAM