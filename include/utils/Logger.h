#pragma once

#include "Macros.h"

#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>

#include <glog/logging.h>

namespace vdo {


// Open files with name output_filename, and checks that it is valid
static inline void OpenFile(const std::string& output_filename,
                            std::ofstream* output_file,
                            bool append_mode = false) {
  CHECK_NOTNULL(output_file);
  output_file->open(output_filename.c_str(),
                    append_mode ? std::ios_base::app : std::ios_base::out);
  output_file->precision(20);
  CHECK(output_file->is_open()) << "Cannot open file: " << output_filename;
  CHECK(output_file->good()) << "File in bad state: " << output_filename;
}

class OfstreamWrapper {
 public:
  VDO_POINTER_TYPEDEFS(OfstreamWrapper);
  VDO_DELETE_COPY_CONSTRUCTORS(OfstreamWrapper);
  OfstreamWrapper(const std::string& filename,
                  const bool& open_file_in_append_mode = false);
  virtual ~OfstreamWrapper();
  void closeAndOpenLogFile();

 public:
  std::ofstream ofstream_;
  const std::string filename_;
  const std::string output_path_;
  const bool open_file_in_append_mode = false;

 protected:
  void openLogFile(const std::string& output_file_name,
                   bool open_file_in_append_mode = false);
};


template<typename Value>
struct Formatter {

    static void header(std::ofstream& stream);
    static void format(std::ofstream& stream, const Value& value);
};


template<typename Value>
class Logger {
public:
    Logger(const std::string& file_name);
    void log(const Value& value);

protected:
    bool is_header_written { false };
    OfstreamWrapper output_stream_;
};

}

#include "Logger-inl.h"
