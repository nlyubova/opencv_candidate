add_subdirectory(opencv_candidate)
add_subdirectory(reconst3d)
add_subdirectory(rgbd)

find_library(SOFTKINETIC_LIBRARIES DepthSense /opt/softkinetic/DepthSenseSDK/lib)

if (NOT SOFTKINETIC_FOUND STREQUAL SOFTKINETIC_LIBRARIES-NOTFOUND)
  add_subdirectory(creative)
endif()
