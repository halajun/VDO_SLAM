#pragma once

#include <glog/logging.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>

namespace VDO_SLAM {

//Taken with inspiration from https://github.com/MIT-SPARK/Kimera-VIO/blob/master/include/kimera-vio/logging/Logger.h
static inline void openFile(const std::string& output_file_, std::ofstream& output_stream_, bool append_ = false) {
    output_stream_.open(output_file_.c_str(), append_ ? std::ios_base::app : std::ios_base::out);
    output_stream_.precision(20);
    CHECK(output_stream_.is_open()) << "Cannot open file to log to: " << output_file_;
    CHECK(output_stream_.good()) << "File is not good: " << output_file_;
}

class OfstreamWrapper {
    public:
        //Constructor will open the log file directly when called
        OfstreamWrapper(const std::string& output_file_, bool append_file_ = false);
        //desctructor will explicitly call class on the ofsteam object
        virtual ~OfstreamWrapper();

        const std::string output_file;
        //the full path to the output file (not including the file name )
        const std::string output_folder;

    protected:
        void openLogFile(const std::string& output_file_, bool append_file_);

    protected:
        std::ofstream ofstream;
        const bool append_file;
};




} //VDO_SLAM