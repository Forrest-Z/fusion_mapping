cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(fusion_mapping
        LANGUAGES CXX
        VERSION 1.0.0)
add_compile_options(-fPIC)


# <添加自定义cmake模块>
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(cmake/global_defination.cmake)
include(cmake/FindG2O.cmake)
include(cmake/FindCholmod.cmake)
include(cmake/FindCSparse.cmake)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        pcl_ros
        geometry_msgs
        tf
        eigen_conversions
        tf2
        )

FIND_LIBRARY(YAML_CPP_LIBRARIES yaml-cpp)
if (NOT YAML_CPP_LIBRARIES)
    FIND_PACKAGE(yaml-cpp REQUIRED)
    INCLUDE_DIRECTORIES(${YAML_CPP_INCLUDE_DIRS})
endif (NOT YAML_CPP_LIBRARIES)

find_package(gflags REQUIRED)

find_package(Boost REQUIRED filesystem program_options)

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARIY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Eigen3 3.3.1 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

find_package(Cholmod REQUIRED)
include_directories(${CHOLMOD_INCLUDE_DIR})

include_directories(
        include
        3rdparty/GeographicLib/include
        ${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${GFLAGS_INCLUDE_DIRS}
)

add_service_files(
        FILES
        saveMap.srv
        optimizeMap.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES fusion_mapping
        CATKIN_DEPENDS roscpp nodelet velodyne_pointcloud

)

# <构建类型>
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()


# <C++标准>
set(CMAKE_CXX_STANDARD 11)          # c++标准
set(CMAKE_CXX_EXTENSIONS OFF)       # 只启用ISO C++标准的编译器标志，而不使用特定编译器的扩展
set(CMAKE_CXX_STANDARD_REQUIRED ON) # 语言标准是否必须
# ON:如果此版本不可用，CMake将报错
# OFF:如果此版本不可用,CMake将查找标准的下一个版本
#     c++17->c++14->c++11->c++98




# <构建类型>
# ON:构建动态库, OFF:构建静态库
option(BUILD_SHARED "Build shared library" ON)

# <全局变量>
#set(NAMESPACE            fusion_mapping)       # 命名空间
#set(CMAKEFILE_PREFIX     FusionMapping)   # .cmake前缀，<Namesoace><Packagename>



# <打包>
# 命令：cpack --config CPackConfig.cmake
include(CPackProcess)

# <安装>
# 安装：sudo dpkg -i xxx.deb
# 解压：dpkg -x xxx.deb <dir>


# 安装路径
#set(CMAKE_INSTALL_PREFIX /usr/local)                   # 系统目录
set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install)  # 构建目录

#include(InstallProcess)
#setup_output()
#setup_install()
#setup_uninstall()

# <构建>
option(BUILD_DOC "Build docs" OFF)
if(BUILD_DOC)
    add_subdirectory(doc)                       # 文档
endif()

add_subdirectory(3rdparty)                      # 第三方库
add_subdirectory(include)                       # 头文件
add_subdirectory(src)                           # 源文件

option(BUILD_APPS "Build tools" ON)
if(BUILD_APPS)
    add_subdirectory(apps)                      # 工具
endif()


# <测试>
# 命令：make CTEST_OUTPUT_ON_FAILURE=1 test
#      ctest -V
option(BUILD_TEST "Build test" OFF)
if(BUILD_TEST)
    include(GtestProcess)
    setup_gtest()
    enable_testing()
    add_subdirectory(test)                      # 测试
endif()
