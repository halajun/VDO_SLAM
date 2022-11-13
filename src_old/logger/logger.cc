#include "logger/logger.h"


#include <fstream>
#include <memory>
#include <string>

#include <boost/filesystem.hpp>  // to create folders
#include <boost/foreach.hpp>

#include <gflags/gflags.h>


DEFINE_string(output_path, "./", "Path where to store VDO_SLAM's log output.");

namespace VDO_SLAM {

OfstreamWrapper::OfstreamWrapper(const std::string& output_file_, bool append_file_)
    :   output_file(output_file_),
        append_file(append_file_),
        output_folder(FLAGS_output_path) {
    openLogFile(output_file, append_file);
}

OfstreamWrapper::~OfstreamWrapper() {
    LOG(INFO) << "Closing output file: " << output_file;
    ofstream.close();
}


void OfstreamWrapper::openLogFile(const std::string& output_file_, bool append_file_) {
    CHECK(!output_file_.empty());
    LOG(INFO) << "Opening output file: " << output_file_;
    openFile(output_folder + '/' + output_file_,
        ofstream,
        append_file_);
}

} //VDO_SLAM