find_package(Boost REQUIRED thread system)

# find the DepthSense SDK
set(SOFTKINETIC_PATHS "/opt/softkinetic/DepthSenseSDK")
find_library(SOFTKINETIC_LIBRARIES DepthSense ${SOFTKINETIC_PATHS}/lib)
find_library(TURBO_JPEG_LIBRARIES turbojpeg ${SOFTKINETIC_PATHS}/lib)
find_path(SOFTKINETIC_INCLUDE_DIR DepthSense.hxx
  PATHS "${SOFTKINETIC_PATHS}"
  PATH_SUFFIXES include
)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs
    image_transport
    camera_info_manager
)

include_directories(SYSTEM
                           ${OpenCV_INCLUDE_DIRS}
                           ${BOOST_INCLUDE_DIRS}
                           ${SOFTKINETIC_INCLUDE_DIR}
                           ${catkin_INCLUDE_DIRS}
)

add_library(opencv_creative SHARED creative.cpp)

target_link_libraries(opencv_creative ${Boost_LIBRARIES}
                                      ${OpenCV_LIBRARIES}
                                      ${SOFTKINETIC_LIBRARIES}
                                      ${TURBO_JPEG_LIBRARIES}
                                      ${catkin_LIBRARIES}
)

install(TARGETS opencv_creative
        DESTINATION lib
)

add_executable(creative_bringup_node main.cpp)
target_link_libraries(creative_bringup_node opencv_creative
)
