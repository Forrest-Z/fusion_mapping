//
// Created by linsin on 13/03/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_TOOLS_TIC_TOC_H_
#define FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_TOOLS_TIC_TOC_H_
#include <ctime>
#include <cstdlib>
#include <chrono>

namespace FM {
class TicToc {
 public:
  TicToc() {
    tic();
  }

  void tic() {
    start = std::chrono::system_clock::now();
  }

  double toc() {
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    start = std::chrono::system_clock::now();
    return elapsed_seconds.count();
  }

 private:
  std::chrono::time_point<std::chrono::system_clock> start, end;
};
}
#endif //FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_TOOLS_TIC_TOC_H_
