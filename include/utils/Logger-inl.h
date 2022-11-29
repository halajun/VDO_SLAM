#pragma once

#include "utils/Logger.h"
#include <stdlib.h>
#include <fstream>
#include <iostream>

#include "utils/Logger-Definitions.h"

namespace vdo
{
template <typename Value>
Logger<Value>::Logger(const std::string& file_name) : output_stream_(file_name)
{
}

template <typename Value>
void Logger<Value>::log(const Value& value)
{
  std::ofstream& ofstream = output_stream_.ofstream_;
  if (!is_header_written)
  {
    Formatter<Value>::header(ofstream);
    ofstream << std::endl;
    is_header_written = true;
  }

  Formatter<Value>::format(ofstream, value);
  ofstream << std::endl;
}

}  // namespace vdo