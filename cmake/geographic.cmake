add_subdirectory(${PROJECT_SOURCE_DIR}/3rdparty/GeographicLib)
include_directories(${PROJECT_SOURCE_DIR}/3rdparty/GeographicLib/include/)
list(APPEND ALL_TARGET_LIBRARIES libGeographiccc)