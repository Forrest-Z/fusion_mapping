set(target_name core)

list(APPEND dir
        subscriber
        publisher
        tf_listener
        sensor_data
        models/cloud_filter
        models/registration
        models/scan_adjust
        models/graph_optimizer/g2o
        models/graph_optimizer
        models/graph_optimizer/g2o/edge
        data_preprocess
        mapping/front_end
        mapping/back_end
        mapping/viewer
        mapping/loop_closing
        matching
        param
        tools
  )

foreach(_dir ${dir})
    file(GLOB     _src_file       ${CMAKE_CURRENT_SOURCE_DIR}/${_dir}/*.cc)
    list(APPEND   src_file        ${_src_file})
endforeach()



if(BUILD_SHARED)
    add_library(${target_name} SHARED "")
else()
    add_library(${target_name} STATIC "")
endif()

target_sources(${target_name}
  PRIVATE
    ${src_file}

  )

target_include_directories(${target_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/${INSTALL_INCLUDEDIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${INSTALL_INCLUDEDIR}>
  )

target_link_libraries(${target_name}
  PUBLIC
   # <Add your libs>
        ${PCL_LIBRARIES}
        ${CERES_LIBRARIES}
        ${GFLAGS_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        ${G2O_TYPES_DATA}
        ${G2O_CORE_LIBRARY}
        ${G2O_STUFF_LIBRARY}
        ${G2O_SOLVER_PCG}
        ${G2O_SOLVER_CSPARSE}
        ${G2O_SOLVER_CHOLMOD}
        ${G2O_TYPES_SLAM3D}
        ${G2O_TYPES_SLAM3D_ADDONS}
        libGeographiccc
  )

set_target_properties(${target_name}
  PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    SOVERSION ${PROJECT_VERSION}
    OUTPUT_NAME ${NAMESPACE}_${target_name}
    DEBUG_POSTFIX "_d"
  )

target_compile_options(${target_name}
  PRIVATE
    -std=c++11
  )

#install(
#  TARGETS
#    ${target_name}
#  EXPORT
#    ${CMAKEFILE_PREFIX}Targets
#  LIBRARY
#    DESTINATION ${INSTALL_LIBDIR}
#    COMPONENT lib
#  )



