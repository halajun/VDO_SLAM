#pragma once 

#include "utils/macros.h"

#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <string>
#include <iostream>
#include <stdexcept>


namespace VDO_SLAM {
namespace utils {
class ParamParser {
    public:
        VDO_SLAM_POINTER_TYPEDEFS(ParamParser);

        ParamParser(const std::string& file_path_);
        ~ParamParser();

        template<class T>
        bool getParam(const std::string& id, T* value) const;


    private:
        void openFile(const std::string& file_path_);
        void closeFile();

        template<class T>
        inline T converter(const cv::FileNode& node) const;


    private:
        const std::string file_path;
        cv::FileStorage fs;
        bool is_valid; 

};

template<class T>
bool ParamParser::getParam(const std::string& id, T* value) const {
    CHECK(!id.empty()) << "Id cannot be empty";
    CHECK_NOTNULL(value);

    const cv::FileNode& handle = fs[id];
    if(handle.type() == cv::FileNode::NONE) {
        LOG(WARNING) << "Missing parameter: " << id
                     << " in file: " << file_path;
        
        return false;
    }
    else {
        *value = converter<T>(handle);
        return true;
    }
    
}

template<class T>
inline T ParamParser::converter(const cv::FileNode& node) const { return node; }

template<>
inline bool ParamParser::converter(const cv::FileNode& node) const { return static_cast<int>(node) != 0; }


} //utils
} //VDO_SLAM