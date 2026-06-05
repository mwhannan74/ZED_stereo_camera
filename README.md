# ZED Stereo Camera C++ Starter

Minimal Windows 11 C++ starter project for a Stereolabs ZED stereo camera.

This project opens the first available ZED camera, runs the ZED SDK grab/depth loop, retrieves common image/depth outputs, and displays live OpenCV windows for quick validation and early application development.

The code is split into:

- `zed_opencv_bridge`: a small library that owns the ZED SDK objects and exposes selected camera outputs as OpenCV `cv::Mat` values.
- `zed_stereo_camera`: a demo executable that uses the bridge library for camera data and keeps display, overlay, FPS, and keyboard handling logic in the app.

The project is intentionally simple:

```text
ZED_stereo_camera/
  CMakeLists.txt
  include/
    zed_opencv_camera.hpp
  local_paths.cmake
  src/
    zed_opencv_camera.cpp
  zed_stereo_camera_main.cpp
  README.md
```

No `CMakePresets.json`, `tasks.json`, or `launch.json` is required.

---

## What this program is for

Use this application as a working C++ starting point for:

- Verifying that your ZED 2 opens from your own code
- Confirming full-rate image acquisition
- Running SDK depth processing
- Viewing left/right camera images
- Viewing an 8-bit depth visualization
- Viewing the confidence map
- Reading numeric depth at a pixel
- Reading XYZ point-cloud data at a pixel
- Computing Euclidean range from XYZ
- Prototyping an OpenCV-facing camera interface for a larger application

This is not intended to be a final production architecture. It is a compact, readable baseline that proves the camera, SDK, CUDA, OpenCV, CMake setup, and library/executable boundary all work together.

---

## Project structure

### Bridge library

The bridge library target is:

```text
zed_opencv_bridge
```

Its public header is:

```text
include/zed_opencv_camera.hpp
```

This header intentionally includes OpenCV only. It exposes:

- `zed_bridge::ZedCameraConfig`
- `zed_bridge::ZedCameraInfo`
- `zed_bridge::ZedFrame`
- `zed_bridge::ZedOpenCvCamera`

The implementation is:

```text
src/zed_opencv_camera.cpp
```

This file owns the ZED SDK dependency. It includes `sl/Camera.hpp`, opens the camera, retrieves ZED outputs, and converts `sl::Mat` buffers into OpenCV `cv::Mat` views.

The `cv::Mat` values in `ZedFrame` reference bridge-owned buffers. They remain valid until the next successful `grab()` call or until the camera closes. Use `clone()` if another project needs to store a frame longer than that.

### Demo executable

The demo executable target is:

```text
zed_stereo_camera
```

Its source file is:

```text
zed_stereo_camera_main.cpp
```

The demo app includes only the bridge header and OpenCV display headers. It does not include ZED or CUDA headers directly.

---

## Requirements

### Hardware

- Stereolabs ZED camera, tested with ZED 2
- USB 3.x connection
- NVIDIA GPU supported by the installed ZED SDK
- Working NVIDIA driver

### Software

- Windows 11
- ZED SDK installed and validated
- CUDA version compatible with the installed ZED SDK
- Visual Studio 2022 or Visual Studio 2022 Build Tools with Desktop C++ workload
- CMake
- Visual Studio Code
- VS Code extensions:
  - C/C++
  - CMake Tools
- vcpkg
- OpenCV installed through vcpkg

---

## Known-good local paths

This project is currently set up for these local paths:

```text
VCPKG_ROOT   = C:/Users/mwhan/vcpkg
ZED_SDK_ROOT = C:/Program Files (x86)/ZED SDK
```

These are configured in:

```text
local_paths.cmake
```

If either path changes, update `local_paths.cmake`.

---

## Important OpenCV / ZED CMake issue

The ZED SDK installation includes an old bundled OpenCV path:

```text
C:/Program Files (x86)/ZED SDK/dependencies/opencv_3.1.0
```

That path can pollute CMake's OpenCV discovery. The symptom is that CMake finds OpenCV headers but either links no OpenCV libraries or finds the wrong OpenCV version.

A bad configure output looks like this:

```text
OpenCV version: 3.1.0
OpenCV include dirs: C:/Program Files (x86)/ZED SDK/dependencies/opencv_3.1.0/include
OpenCV libs:
```

That causes linker errors such as:

```text
unresolved external symbol cv::imshow
unresolved external symbol cv::resize
unresolved external symbol cv::putText
unresolved external symbol cv::Mat
```

The fix used by this project is:

1. Force OpenCV to come from vcpkg.
2. Find OpenCV before finding ZED.
3. Save the OpenCV include/library variables immediately.
4. Find ZED afterward.
5. Link against the saved OpenCV libraries.

A good configure output looks like this:

```text
Found OpenCV: C:/Users/mwhan/vcpkg/installed/x64-windows
OpenCV saved version:   4.11.0
OpenCV saved includes:  C:/Users/mwhan/vcpkg/installed/x64-windows/include/opencv4
OpenCV saved libs:      opencv_core;opencv_imgproc;opencv_highgui
```

---

## Verify vcpkg and OpenCV

Open PowerShell.

Check that `VCPKG_ROOT` points to the intended vcpkg installation:

```powershell
echo $env:VCPKG_ROOT
```

Expected:

```text
C:\Users\mwhan\vcpkg
```

Verify vcpkg:

```powershell
& "C:\Users\mwhan\vcpkg\vcpkg.exe" --vcpkg-root "C:\Users\mwhan\vcpkg" version
```

Verify OpenCV:

```powershell
& "C:\Users\mwhan\vcpkg\vcpkg.exe" --vcpkg-root "C:\Users\mwhan\vcpkg" list | Select-String -Pattern "opencv"
```

Expected result includes:

```text
opencv4:x64-windows
```

Verify the vcpkg CMake toolchain file:

```powershell
Test-Path "C:\Users\mwhan\vcpkg\scripts\buildsystems\vcpkg.cmake"
```

Expected:

```text
True
```

If `VCPKG_ROOT` is wrong, set it:

```powershell
[Environment]::SetEnvironmentVariable("VCPKG_ROOT", "C:\Users\mwhan\vcpkg", "User")
$env:VCPKG_ROOT = "C:\Users\mwhan\vcpkg"
```

Restart VS Code after changing environment variables.

---

## Verify the ZED SDK

Verify the install folder:

```powershell
Test-Path "C:\Program Files (x86)\ZED SDK"
```

Expected:

```text
True
```

Before debugging this C++ project, confirm the camera works in the ZED tools:

```text
ZED Explorer
ZED Diagnostic
ZED Depth Viewer
```

Close those tools before running this application. Only one process should own the camera during the test.

---

## Configure CMake

Open PowerShell in the project folder:

```powershell
cd "C:\Users\mwhan\Documents\CODE_SANDBOX\CPlusPlus\Solutions\ZED_stereo_camera"
```

Start from a clean build folder:

```powershell
Remove-Item -Recurse -Force build -ErrorAction SilentlyContinue
```

Configure with Visual Studio 2022 x64 and the vcpkg toolchain:

```powershell
cmake -S . -B build `
  -G "Visual Studio 17 2022" `
  -A x64 `
  -DCMAKE_TOOLCHAIN_FILE="C:/Users/mwhan/vcpkg/scripts/buildsystems/vcpkg.cmake" `
  -DVCPKG_TARGET_TRIPLET=x64-windows
```

Expected configure output includes:

```text
Found CUDA: C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v13.3
Found OpenCV: C:/Users/mwhan/vcpkg/installed/x64-windows
OpenCV saved version:   4.11.0
OpenCV saved includes:  C:/Users/mwhan/vcpkg/installed/x64-windows/include/opencv4
OpenCV saved libs:      opencv_core;opencv_imgproc;opencv_highgui
```

If CMake reports OpenCV under the ZED SDK dependency folder, stop and fix `local_paths.cmake` / `CMakeLists.txt` before building.

---

## Build

```powershell
cmake --build build --config RelWithDebInfo
```

Expected output includes:

```text
zed_opencv_bridge.vcxproj -> ...\build\RelWithDebInfo\zed_opencv_bridge.lib
zed_stereo_camera.vcxproj -> ...\build\bin\RelWithDebInfo\zed_stereo_camera.exe
```

---

## Run

Make sure the ZED camera is connected and all ZED GUI tools are closed.

Run:

```powershell
.\build\bin\RelWithDebInfo\zed_stereo_camera.exe
```

Expected behavior:

- Console prints startup and camera information
- OpenCV windows appear:
  - `ZED Left + Measurements`
  - `ZED Right`
  - `ZED Depth Display Only`
  - `ZED Confidence Display`
- The left image shows an overlay with:
  - Application FPS
  - Center-pixel Z depth
  - Center-pixel Euclidean range
  - Center-pixel confidence
  - Center-pixel XYZ value

Exit by clicking an OpenCV window and pressing:

```text
ESC
```

or:

```text
Q
```

---

## Run from VS Code

After the PowerShell configure/build works, VS Code is straightforward.

1. Open the project folder in VS Code.
2. Install these extensions:
   - C/C++
   - CMake Tools
3. Open the Command Palette:

```text
Ctrl+Shift+P
```

4. Run:

```text
CMake: Configure
```

5. Select the Visual Studio 2022 x64 kit if prompted.
6. Run:

```text
CMake: Build
```

7. Select target:

```text
zed_stereo_camera
```

8. Run:

```text
CMake: Run Without Debugging
```

or:

```text
CMake: Debug
```

If VS Code behaves differently from PowerShell, delete the `build` folder and reconfigure. The PowerShell commands above are the reference build path.

For IntelliSense in this project, use manual include paths through:

```text
.vscode/c_cpp_properties.json
```

This is the current known-good setup for reliable header navigation in VS Code.

---

## What the code retrieves

The bridge library can retrieve these ZED SDK outputs and expose them as OpenCV matrices in `zed_bridge::ZedFrame`.

### `VIEW::LEFT`

Left camera image.

Used for:

- Main display
- Image processing
- Overlaying measurements
- Visual debugging

### `VIEW::RIGHT`

Right camera image.

Used for:

- Stereo debugging
- Verifying both sensors are working

### `VIEW::DEPTH`

8-bit normalized depth visualization.

Used for:

- Human-readable display only

Do not use this for numeric measurement.

### `MEASURE::DEPTH`

32-bit floating-point Z-depth map.

Used for:

- Numeric depth at each pixel
- Distance along the camera Z axis

The current code uses millimeters.

### `MEASURE::CONFIDENCE`

32-bit floating-point confidence map.

Used for:

- Evaluating whether a depth value is trustworthy
- Debugging weak depth areas

Lower confidence values are better.

### `MEASURE::XYZRGBA`

32-bit floating-point point cloud with color.

Used for:

- 3D point lookup at each pixel
- Computing Euclidean range:

```text
range = sqrt(X*X + Y*Y + Z*Z)
```

### Optional outputs

These are available in the code but can be disabled for performance:

```text
MEASURE::DISPARITY
MEASURE::NORMALS
MEASURE::DEPTH_U16_MM
```

---

## User-editable settings

Demo camera and processing settings are near the top of:

```text
zed_stereo_camera_main.cpp
```

Look for:

```cpp
namespace user_settings {
```

Key settings:

```cpp
CAMERA_RESOLUTION
CAMERA_FPS
DEPTH_MODE
DEPTH_MINIMUM_DISTANCE
DEPTH_MAXIMUM_DISTANCE
CONFIDENCE_THRESHOLD
TEXTURE_CONFIDENCE_THRESHOLD
ENABLE_DEPTH_FILL_MODE
DISPLAY_SCALE
DISPLAY_EVERY_N_FRAMES
```

Output retrieval switches:

```cpp
RETRIEVE_LEFT_IMAGE
RETRIEVE_RIGHT_IMAGE
RETRIEVE_DEPTH_IMAGE_FOR_DISPLAY
RETRIEVE_DEPTH_MAP_F32
RETRIEVE_CONFIDENCE_MAP_F32
RETRIEVE_POINT_CLOUD_XYZRGBA
RETRIEVE_DISPARITY_F32
RETRIEVE_NORMALS_F32
RETRIEVE_DEPTH_U16_MM
```

These settings are copied into `zed_bridge::ZedCameraConfig` before opening the camera. The bridge currently uses millimeters and image coordinates internally.

---

## Using the bridge from another CMake target

A second executable in this CMake project can consume the bridge without including ZED headers:

```cmake
add_executable(my_consumer
    my_consumer_main.cpp
)

target_link_libraries(my_consumer PRIVATE
    zed_opencv_bridge
)
```

Consumer code should include:

```cpp
#include "zed_opencv_camera.hpp"
```

The consumer still has a runtime dependency on ZED/CUDA because `zed_opencv_bridge` calls into the ZED SDK internally. The useful separation is that ZED/CUDA types and headers stay out of consumer code.

---

## Display size

The OpenCV windows are controlled by:

```cpp
DISPLAY_SCALE
```

Examples:

```cpp
static constexpr double DISPLAY_SCALE = 1.0;   // native image size
static constexpr double DISPLAY_SCALE = 1.5;   // larger display
static constexpr double DISPLAY_SCALE = 2.25;  // about 3x larger than 0.75
```

For HD720 input, `DISPLAY_SCALE = 1.5` displays approximately 1920 x 1080.

If the OpenCV windows appear square or stretched, make sure the code uses `cv::resizeWindow()` with the scaled image width and height before `cv::imshow()`.

---

## Performance notes

The application currently prioritizes visibility and debugging over minimum CPU/GPU transfer cost.

Retrieving and displaying many outputs every frame can reduce frame rate. The most expensive output is usually the full-resolution point cloud:

```cpp
static constexpr bool RETRIEVE_POINT_CLOUD_XYZRGBA = true;
```

For higher frame rate, disable outputs not needed by the application.

A lighter real-time configuration:

```cpp
static constexpr bool RETRIEVE_LEFT_IMAGE = true;
static constexpr bool RETRIEVE_RIGHT_IMAGE = false;
static constexpr bool RETRIEVE_DEPTH_IMAGE_FOR_DISPLAY = true;
static constexpr bool RETRIEVE_DEPTH_MAP_F32 = true;
static constexpr bool RETRIEVE_CONFIDENCE_MAP_F32 = true;
static constexpr bool RETRIEVE_POINT_CLOUD_XYZRGBA = false;
```

Enable the point cloud only when XYZ/range values are needed.

Display can also become the bottleneck. If that happens, increase:

```cpp
static constexpr int DISPLAY_EVERY_N_FRAMES = 2;
```

or lower:

```cpp
static constexpr double DISPLAY_SCALE = 1.0;
```

---

## Runtime DLL issues

If the program builds but fails to start because a DLL is missing, temporarily prepend the ZED SDK and vcpkg binary folders to `PATH` in the current PowerShell session:

```powershell
$env:Path = "C:\Program Files (x86)\ZED SDK\bin;C:\Users\mwhan\vcpkg\installed\x64-windows\bin;$env:Path"

.\build\bin\RelWithDebInfo\zed_stereo_camera.exe
```

If that works, make the PATH change permanent or update CMake to copy required DLLs next to the executable.

---

## Troubleshooting checklist

### CMake cannot find OpenCV

Check:

```powershell
echo $env:VCPKG_ROOT
Test-Path "$env:VCPKG_ROOT\scripts\buildsystems\vcpkg.cmake"
& "$env:VCPKG_ROOT\vcpkg.exe" --vcpkg-root "$env:VCPKG_ROOT" list | Select-String -Pattern "opencv"
```

Expected:

```text
C:\Users\mwhan\vcpkg
True
opencv4:x64-windows
```

### CMake finds OpenCV 3.1.0 under the ZED SDK

That is the wrong OpenCV for this project. CMake should use vcpkg OpenCV 4.x.

Check that `CMakeLists.txt` finds OpenCV before ZED and saves the OpenCV variables before calling `find_package(ZED REQUIRED)`.

### OpenCV headers are found but linker fails with `cv::... unresolved external symbol`

This means the target is not linking OpenCV libraries.

The configure output should include:

```text
OpenCV saved libs: opencv_core;opencv_imgproc;opencv_highgui
```

The target must link those saved OpenCV libraries.

### ZED camera does not open

Close:

```text
ZED Explorer
ZED Diagnostic
ZED Depth Viewer
```

Then rerun the application.

If it still fails, validate the camera again using ZED Diagnostic.

### Build works but runtime says missing DLL

Use the temporary PATH command in the Runtime DLL Issues section.

---

## Clean rebuild

Use this when CMake state seems stale or after editing `CMakeLists.txt` / `local_paths.cmake`:

```powershell
Remove-Item -Recurse -Force build -ErrorAction SilentlyContinue

cmake -S . -B build `
  -G "Visual Studio 17 2022" `
  -A x64 `
  -DCMAKE_TOOLCHAIN_FILE="C:/Users/mwhan/vcpkg/scripts/buildsystems/vcpkg.cmake" `
  -DVCPKG_TARGET_TRIPLET=x64-windows

cmake --build build --config RelWithDebInfo

.\build\bin\RelWithDebInfo\zed_stereo_camera.exe
```
