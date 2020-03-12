//
// Created by linsin on 12/03/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_TOOLS_FILE_MANAGER_HPP_
#define FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_TOOLS_FILE_MANAGER_HPP_
#include <string>
#include <iostream>
#include <fstream>

namespace FM {
class FileManager{
 public:
  static bool CreateFile(std::ofstream& ofs, std::string file_path);
  static bool CreateDirectory(std::string directory_path);
};
}
#endif //FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_TOOLS_FILE_MANAGER_HPP_
