cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

project(template_alignment)
# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(template_alignment src/template_alignment.cpp)
target_link_libraries(template_alignment ${PCL_LIBRARIES})
rosbuild_add_executable(makehuman_alignment src/makehuman_alignment.cpp)
target_link_libraries(makehuman_alignment ${PCL_LIBRARIES})
rosbuild_add_executable(face_detection src/face_detection.cpp)
target_link_libraries(face_detection ${PCL_LIBRARIES})
#rosbuild_add_executable(align_mh_to_kinect src/align_mh_to_kinect.cpp)
#target_link_libraries(align_mh_to_kinect ${PCL_LIBRARIES})
#rosbuild_add_executable(body_align src/body_align.cpp)
#target_link_libraries(body_align ${PCL_LIBRARIES})
