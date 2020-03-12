//
// Created by linsin on 12/03/2020.
//
#include "fusion_mapping/core/tools/file_manager.h"

#include <boost/filesystem.hpp>

namespace FM {
bool FileManager::CreateFile(std::ofstream& ofs, std::string file_path) {
  ofs.open(file_path.c_str(), std::ios::app);
  if (!ofs) {
    std::cout << "无法生成文件: " << file_path;
    return false;
  }

  return true;
}

bool FileManager::CreateDirectory(std::string directory_path) {
  if (!boost::filesystem::is_directory(directory_path)) {
    boost::filesystem::create_directory(directory_path);
  }
  if (!boost::filesystem::is_directory(directory_path)) {
    std::cout << "无法建立文件夹: " << directory_path;
    return false;
  }
  return true;
}
}
