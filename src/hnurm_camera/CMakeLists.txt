cmake_minimum_required(VERSION 3.22)
project(hnurm_camera)

set(CMAKE_CXX_STANDARD 17)

if (${CMAKE_SYSTEM} MATCHES "Linux")
    file(GLOB SDK_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/linux/*)
elseif (${CMAKE_SYSTEM} MATCHES "Darwin")
    file(GLOB SDK_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/darwin/*)
endif ()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(hnurm_camera SHARED src/Camera.cpp src/camera_node.cpp)
target_link_libraries(hnurm_camera ${SDK_PATH})

ament_auto_add_executable(hnurm_camera_node src/main.cpp)
target_link_libraries(hnurm_camera_node hnurm_camera)

ament_auto_package(
        INSTALL_TO_SHARE
        launch
        params
)
