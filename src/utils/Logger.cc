#include "utils/Logger.h"

DEFINE_string(output_path, "./", "Path where to store VDO's log output.");

namespace vdo
{
OfstreamWrapper::OfstreamWrapper(const std::string& filename, const bool& open_file_in_append_mode)
  : filename_(filename), output_path_(FLAGS_output_path)
{
  openLogFile(filename, open_file_in_append_mode);
}

OfstreamWrapper::~OfstreamWrapper()
{
  LOG(INFO) << "Closing output file: " << filename_.c_str();
  ofstream_.close();
}

void OfstreamWrapper::closeAndOpenLogFile()
{
  ofstream_.close();
  CHECK(!filename_.empty());
  OpenFile(output_path_ + '/' + filename_, &ofstream_, false);
}

void OfstreamWrapper::openLogFile(const std::string& output_file_name, bool open_file_in_append_mode)
{
  CHECK(!output_file_name.empty());
  LOG(INFO) << "Opening output file: " << output_file_name.c_str();
  OpenFile(output_path_ + '/' + output_file_name, &ofstream_, open_file_in_append_mode);
}

}  // namespace vdo