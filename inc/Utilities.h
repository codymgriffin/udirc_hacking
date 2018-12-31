#pragma once

#include "Hexdump.hpp"
#include <chrono>

// chrono is stupidly verbose...  so we have this helper that just
// computes dt in milliseconds.
typedef std::chrono::high_resolution_clock Clock;

auto ms = [](auto&& a, auto&& b) { 
  return std::chrono::duration_cast<
    std::chrono::milliseconds>(a - b).count();
};

#define DEBUG false
#define DUMP(label, buffer) if (DEBUG) {\
    std::cerr << #label << std::endl;\
    std::cerr << Hexdump(buffer.data(), buffer.size()) << std::endl;\
}\
