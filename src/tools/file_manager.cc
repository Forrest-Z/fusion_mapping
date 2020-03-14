//
// Created by linsin on 12/03/2020.
//
#include "fusion_mapping/core/tools/file_manager.h"

#include <boost/filesystem.hpp>

namespace FM {
bool FileManager::CreateFile(std::ofstream& ofs, std::string file_path) {
  ofs.close();
  boost::filesystem::remove(file_path.c_str());

  ofs.open(file_path.c_str(), std::ios::out);

  if (!ofs) {
    std::cout << "无法生成文件: " << std::endl << file_path << std::endl << std::endl;
    return false;
  }

  return true;
}

bool FileManager::InitDirectory(std::string directory_path, std::string use_for) {
  if (boost::filesystem::is_directory(directory_path)) {
    boost::filesystem::remove_all(directory_path);
  }

  return CreateDirectory(directory_path, use_for);
}

bool FileManager::CreateDirectory(std::string directory_path, std::string use_for) {
  if (!boost::filesystem::is_directory(directory_path)) {
    boost::filesystem::create_directory(directory_path);
  }

  if (!boost::filesystem::is_directory(directory_path)) {
    std::cout << "无法创建文件夹: " << std::endl << directory_path << std::endl << std::endl;
    return false;
  }

  std::cout << use_for << "存放地址：" << std::endl << directory_path << std::endl << std::endl;
  return true;
}

bool FileManager::CreateDirectory(std::string directory_path) {
  if (!boost::filesystem::is_directory(directory_path)) {
    boost::filesystem::create_directory(directory_path);
  }

  if (!boost::filesystem::is_directory(directory_path)) {
    std::cout << "无法创建文件夹: " << std::endl << directory_path << std::endl << std::endl;
    return false;
  }

  return true;
}
}
