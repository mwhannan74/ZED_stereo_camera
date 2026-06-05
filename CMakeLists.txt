cmake_minimum_required(VERSION 3.20)

include("${CMAKE_CURRENT_LIST_DIR}/local_paths.cmake")

project(zed_stereo_camera LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Export compile flags/includes for IntelliSense and other editor tooling.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ------------------------------------------------------------
# OpenCV
# ------------------------------------------------------------
# Find OpenCV from vcpkg BEFORE finding ZED.
# The ZED SDK has an old bundled OpenCV path that can pollute OpenCV_DIR.
find_package(OpenCV 4 REQUIRED
    COMPONENTS core imgproc highgui
    PATHS "${VCPKG_ROOT}/installed/x64-windows/share/opencv4"
    NO_DEFAULT_PATH
)

# Save the vcpkg OpenCV values immediately.
# Do not rely on OpenCV_* variables after find_package(ZED).
set(ZED_APP_OPENCV_VERSION       "${OpenCV_VERSION}")
set(ZED_APP_OPENCV_INCLUDE_DIRS  "${OpenCV_INCLUDE_DIRS}")
set(ZED_APP_OPENCV_LIBRARIES     "${OpenCV_LIBRARIES}")
set(ZED_APP_OPENCV_LIBS          "${OpenCV_LIBS}")
set(ZED_APP_OPENCV_LIBRARY_DIRS  "${OpenCV_LIBRARY_DIRS}")

# Some OpenCV configs use OpenCV_LIBS, others use OpenCV_LIBRARIES.
if (NOT ZED_APP_OPENCV_LIBS)
    set(ZED_APP_OPENCV_LIBS "${ZED_APP_OPENCV_LIBRARIES}")
endif()

# ------------------------------------------------------------
# ZED SDK + CUDA
# ------------------------------------------------------------
find_package(ZED REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} REQUIRED)

# ------------------------------------------------------------
# ZED -> OpenCV bridge library
# ------------------------------------------------------------
add_library(zed_opencv_bridge
    src/zed_opencv_camera.cpp
)

target_include_directories(zed_opencv_bridge
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        ${ZED_APP_OPENCV_INCLUDE_DIRS}
    PRIVATE
        ${ZED_INCLUDE_DIRS}
        ${CUDA_INCLUDE_DIRS}
)

target_link_directories(zed_opencv_bridge PRIVATE
    ${ZED_LIBRARY_DIR}
    ${CUDA_LIBRARY_DIRS}
    ${ZED_APP_OPENCV_LIBRARY_DIRS}
)

target_link_libraries(zed_opencv_bridge
    PUBLIC
        ${ZED_APP_OPENCV_LIBS}
    PRIVATE
        ${ZED_LIBRARIES}
        ${CUDA_CUDA_LIBRARY}
        ${CUDA_CUDART_LIBRARY}
)

# ------------------------------------------------------------
# Demo executable
# ------------------------------------------------------------
add_executable(zed_stereo_camera
    zed_stereo_camera_main.cpp
)

target_link_libraries(zed_stereo_camera PRIVATE
    zed_opencv_bridge
)

set_target_properties(zed_stereo_camera PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

message(STATUS "---- ZED / CUDA / OpenCV configuration ----")
message(STATUS "ZED include dirs:       ${ZED_INCLUDE_DIRS}")
message(STATUS "ZED library dir:        ${ZED_LIBRARY_DIR}")
message(STATUS "ZED libraries:          ${ZED_LIBRARIES}")
message(STATUS "CUDA include dirs:      ${CUDA_INCLUDE_DIRS}")
message(STATUS "CUDA library dirs:      ${CUDA_LIBRARY_DIRS}")
message(STATUS "OpenCV saved version:   ${ZED_APP_OPENCV_VERSION}")
message(STATUS "OpenCV saved includes:  ${ZED_APP_OPENCV_INCLUDE_DIRS}")
message(STATUS "OpenCV saved libs:      ${ZED_APP_OPENCV_LIBS}")
message(STATUS "------------------------------------------")
