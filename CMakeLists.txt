

cmake_minimum_required(VERSION 2.6)

#handling of relative paths by link_directories() is defined by CMP0015 policy
cmake_policy(SET CMP0015 NEW)
PROJECT(KinectOpenCVObjectTracking)

find_package( OpenCV REQUIRED )

LINK_DIRECTORIES(/usr/lib/x86_64-linux-gnu)
LINK_DIRECTORIES(${CMAKE_SOURCE_DIR}/lib)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES(${OPENCV_INCLUDE})
INCLUDE_DIRECTORIES(/usr/include)
add_executable(KinectOpenCVObjectTracking ObjectTracking.cpp)

target_link_libraries(KinectOpenCVObjectTracking libOpenNI2.so)
target_link_libraries( KinectOpenCVObjectTracking ${OpenCV_LIBS} )
#target_link_libraries(KinectOpenCVObjectTrackings libfreenect.so)
add_custom_command(TARGET KinectOpenCVObjectTracking PRE_BUILD
                  COMMAND ${CMAKE_COMMAND} -E copy_directory
                   ${CMAKE_SOURCE_DIR}/lib $<TARGET_FILE_DIR:KinectOpenCVObjectTracking>)
