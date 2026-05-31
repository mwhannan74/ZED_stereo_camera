# local_paths.cmake
#
# Machine-specific paths for Michael's Windows 11 workstation.
# Keep this file local.

set(VCPKG_ROOT "C:/Users/mwhan/vcpkg" CACHE PATH "vcpkg root")
set(ZED_SDK_ROOT "C:/Program Files (x86)/ZED SDK" CACHE PATH "ZED SDK root")

# vcpkg must be set before project() is called.
set(CMAKE_TOOLCHAIN_FILE
    "${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
    CACHE FILEPATH "vcpkg CMake toolchain"
)

set(VCPKG_TARGET_TRIPLET "x64-windows" CACHE STRING "vcpkg target triplet")

# Force OpenCV to come from vcpkg.
# This prevents CMake from picking up:
#   C:/Program Files (x86)/ZED SDK/dependencies/opencv_3.1.0
set(OpenCV_DIR
    "${VCPKG_ROOT}/installed/x64-windows/share/opencv4"
    CACHE PATH "vcpkg OpenCV config directory"
)

# Help CMake find the ZED SDK CMake module.
list(APPEND CMAKE_PREFIX_PATH "${ZED_SDK_ROOT}")
list(APPEND CMAKE_MODULE_PATH "${ZED_SDK_ROOT}/cmake")