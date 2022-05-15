#pragma once

#include <chrono>
#include <memory>

namespace VDO_SLAM {
namespace utils {

class Timer {
	public:
		static std::chrono::high_resolution_clock::time_point tic() {
			return std::chrono::high_resolution_clock::now();
		}

		// Stop timer and report duration in given time.
		// Returns duration in milliseconds by default.
		// call .count() on returned duration to have number of ticks.
		template <typename T = std::chrono::milliseconds>
		static T toc(const std::chrono::high_resolution_clock::time_point& start) {
			return std::chrono::duration_cast<T>(
				std::chrono::high_resolution_clock::now() - start);
		}
};

} //utils
} //VDO_SLAM