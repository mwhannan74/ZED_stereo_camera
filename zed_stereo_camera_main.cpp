/*
    zed_stereo_camera_main.cpp

    Minimal ZED 2 / ZED 2i C++ starter application for Windows + VS Code.

    What this does:
      1. Opens the first available ZED camera.
      2. Runs the camera in a continuous grab loop.
      3. Enables SDK depth processing.
      4. Retrieves common ZED outputs:
           - Left image for display / image processing
           - Right image for display / stereo debugging
           - 8-bit depth visualization image for display only
           - 32-bit float depth map for numeric depth in millimeters
           - 32-bit float confidence map
           - 32-bit float XYZRGBA point cloud
           - Optional disparity, normals, and uint16 depth
      5. Displays live OpenCV windows.
      6. Prints / overlays the center-pixel depth, confidence, XYZ, and range.

    Notes:
      - This file intentionally uses constants near the top instead of CLI arguments.
      - For maximum frame rate, retrieve only the outputs your application actually needs.
      - OpenCV display and CPU retrieval of large point clouds can reduce frame rate.
      - Press ESC or Q in an OpenCV window to exit.
*/

#include <sl/Camera.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

// -----------------------------
// User-editable camera settings
// -----------------------------
namespace user_settings {

// ZED 2 practical starting points:
//   - HD720 @ 60 FPS: good first choice for higher frame rate.
//   - HD1080 @ 30 FPS: better image detail, lower frame rate.
//   - VGA   @ 100 FPS: highest camera rate, lower image/depth resolution.
static constexpr sl::RESOLUTION CAMERA_RESOLUTION = sl::RESOLUTION::HD720;
static constexpr int CAMERA_FPS = 60;

// Depth mode tradeoff:
//   - NEURAL_LIGHT: fastest neural mode; good for first real-time prototype.
//   - NEURAL:       default balanced neural mode.
//   - NEURAL_PLUS:  higher quality, heavier GPU load.
// Older PERFORMANCE / QUALITY / ULTRA modes exist but are deprecated in recent SDKs.
static constexpr sl::DEPTH_MODE DEPTH_MODE = sl::DEPTH_MODE::NEURAL_LIGHT;

// Units for depth map, point cloud, tracking, etc.
// MILLIMETER is convenient for inspection/metrology prototypes.
static constexpr sl::UNIT COORDINATE_UNITS = sl::UNIT::MILLIMETER;

// IMAGE coordinate system is intuitive when working from image pixels:
//   X right, Y down, Z forward from the left camera.
static constexpr sl::COORDINATE_SYSTEM COORDINATE_SYSTEM = sl::COORDINATE_SYSTEM::IMAGE;

// Depth range clamp. Values use COORDINATE_UNITS above.
// Set <= 0 to let the SDK use camera defaults.
static constexpr float DEPTH_MINIMUM_DISTANCE = 300.0f;    // 0.3 m
static constexpr float DEPTH_MAXIMUM_DISTANCE = 12000.0f;  // 12 m

// Runtime confidence filtering.
// Confidence values are in [1,100]; lower is better. The threshold rejects less-trusted pixels.
// Default SDK threshold is 95. Lower values reject more pixels near edges / low-texture areas.
static constexpr int CONFIDENCE_THRESHOLD = 95;
static constexpr int TEXTURE_CONFIDENCE_THRESHOLD = 100;

// Fill mode completes holes in the depth map, but it can hide invalid pixels.
// Keep false for measurement/debugging. Consider true only for display-oriented applications.
static constexpr bool ENABLE_DEPTH_FILL_MODE = false;

// Display scaling. Keep 1.0 for full-size windows; use 0.5 if display becomes a bottleneck.
static constexpr double DISPLAY_SCALE = 1.0;

// Display every frame. Increase to 2, 3, etc. if OpenCV display limits frame rate.
static constexpr int DISPLAY_EVERY_N_FRAMES = 1;

// Output retrieval switches.
// Keep these explicit so you can remove CPU transfers you do not need.
static constexpr bool RETRIEVE_LEFT_IMAGE = true;
static constexpr bool RETRIEVE_RIGHT_IMAGE = true;
static constexpr bool RETRIEVE_DEPTH_IMAGE_FOR_DISPLAY = true;  // VIEW::DEPTH, 8-bit display only
static constexpr bool RETRIEVE_DEPTH_MAP_F32 = true;            // MEASURE::DEPTH, numeric Z depth
static constexpr bool RETRIEVE_CONFIDENCE_MAP_F32 = true;       // MEASURE::CONFIDENCE
static constexpr bool RETRIEVE_POINT_CLOUD_XYZRGBA = true;      // MEASURE::XYZRGBA

// Optional diagnostic outputs. These increase CPU/GPU transfer and display cost.
static constexpr bool RETRIEVE_DISPARITY_F32 = false;           // MEASURE::DISPARITY
static constexpr bool RETRIEVE_NORMALS_F32 = false;             // MEASURE::NORMALS
static constexpr bool RETRIEVE_DEPTH_U16_MM = false;            // MEASURE::DEPTH_U16_MM

}  // namespace user_settings

// -----------------------------
// Small utilities
// -----------------------------

// Convert a sl::Mat stored in CPU memory to a cv::Mat header that shares the same memory.
// No image copy occurs here. The ZED sl::Mat must remain alive while the cv::Mat is used.
static cv::Mat slMatToCvMat(sl::Mat& input) {
    int cv_type = -1;

    switch (input.getDataType()) {
        case sl::MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
        case sl::MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
        case sl::MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
        case sl::MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
        case sl::MAT_TYPE::U8_C1:  cv_type = CV_8UC1;  break;
        case sl::MAT_TYPE::U8_C2:  cv_type = CV_8UC2;  break;
        case sl::MAT_TYPE::U8_C3:  cv_type = CV_8UC3;  break;
        case sl::MAT_TYPE::U8_C4:  cv_type = CV_8UC4;  break;
        case sl::MAT_TYPE::U16_C1: cv_type = CV_16UC1; break;
        default:
            throw std::runtime_error("Unsupported sl::Mat type for OpenCV conversion.");
    }

    return cv::Mat(
        static_cast<int>(input.getHeight()),
        static_cast<int>(input.getWidth()),
        cv_type,
        input.getPtr<sl::uchar1>(sl::MEM::CPU),
        input.getStepBytes(sl::MEM::CPU));
}

static void showScaled(const std::string& window_name, const cv::Mat& image, double scale) {
    if (image.empty()) {
        return;
    }

    const int display_width  = static_cast<int>(image.cols * scale);
    const int display_height = static_cast<int>(image.rows * scale);

    cv::Mat display_image;

    if (scale > 0.0 && std::abs(scale - 1.0) > 1e-6) {
        cv::resize(image, display_image, cv::Size(display_width, display_height), 0.0, 0.0, cv::INTER_AREA);
    } else {
        display_image = image;
    }

    // Force the OpenCV window to match the displayed image aspect ratio.
    cv::resizeWindow(window_name, display_image.cols, display_image.rows);
    cv::imshow(window_name, display_image);
}

static std::string metersText(float millimeters) {
    if (!std::isfinite(millimeters)) {
        return "nan";
    }

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) << (millimeters / 1000.0f) << " m";
    return oss.str();
}

class FpsMeter {
public:
    void tick() {
        ++frame_count_;
        const auto now = std::chrono::steady_clock::now();
        const double elapsed_sec = std::chrono::duration<double>(now - last_time_).count();

        if (elapsed_sec >= 0.5) {
            fps_ = static_cast<double>(frame_count_) / elapsed_sec;
            frame_count_ = 0;
            last_time_ = now;
        }
    }

    double fps() const { return fps_; }

private:
    std::chrono::steady_clock::time_point last_time_ = std::chrono::steady_clock::now();
    int frame_count_ = 0;
    double fps_ = 0.0;
};

static void drawOverlay(
    cv::Mat& image,
    double app_fps,
    float depth_z_mm,
    float confidence,
    const sl::float4& xyzrgba,
    float range_mm) {

    const int x = image.cols / 2;
    const int y = image.rows / 2;

    cv::drawMarker(image, cv::Point(x, y), cv::Scalar(0, 255, 255, 255), cv::MARKER_CROSS, 30, 2);

    std::vector<std::string> lines;
    {
        std::ostringstream oss;
        oss << "App FPS: " << std::fixed << std::setprecision(1) << app_fps;
        lines.push_back(oss.str());
    }
    lines.push_back("Center Z depth: " + metersText(depth_z_mm));
    lines.push_back("Center range:   " + metersText(range_mm));

    {
        std::ostringstream oss;
        oss << "Confidence:     ";
        if (std::isfinite(confidence)) {
            oss << std::fixed << std::setprecision(1) << confidence << " / 100 (lower is better)";
        } else {
            oss << "nan";
        }
        lines.push_back(oss.str());
    }

    {
        std::ostringstream oss;
        oss << "XYZ mm:         ";
        if (std::isfinite(xyzrgba.x) && std::isfinite(xyzrgba.y) && std::isfinite(xyzrgba.z)) {
            oss << std::fixed << std::setprecision(1)
                << xyzrgba.x << ", " << xyzrgba.y << ", " << xyzrgba.z;
        } else {
            oss << "nan, nan, nan";
        }
        lines.push_back(oss.str());
    }

    const int base_x = 20;
    int base_y = 35;
    const int line_height = 26;

    for (const std::string& line : lines) {
        cv::putText(image, line, cv::Point(base_x, base_y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(0, 0, 0, 255), 4, cv::LINE_AA);
        cv::putText(image, line, cv::Point(base_x, base_y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(255, 255, 255, 255), 1, cv::LINE_AA);
        base_y += line_height;
    }
}

static void createWindows() {
    if (user_settings::RETRIEVE_LEFT_IMAGE) {
        cv::namedWindow("ZED Left + Measurements", cv::WINDOW_NORMAL);
    }
    if (user_settings::RETRIEVE_RIGHT_IMAGE) {
        cv::namedWindow("ZED Right", cv::WINDOW_NORMAL);
    }
    if (user_settings::RETRIEVE_DEPTH_IMAGE_FOR_DISPLAY) {
        cv::namedWindow("ZED Depth Display Only", cv::WINDOW_NORMAL);
    }
    if (user_settings::RETRIEVE_CONFIDENCE_MAP_F32) {
        cv::namedWindow("ZED Confidence Display", cv::WINDOW_NORMAL);
    }
}

int main() {
    using namespace user_settings;

    std::cout << "Starting ZED stereo camera prototype...\n";
    std::cout << "Press ESC or Q in an OpenCV window to exit.\n\n";

    sl::Camera zed;

    // -----------------------------
    // Configure camera before open()
    // -----------------------------
    sl::InitParameters init_params;

    init_params.camera_resolution = CAMERA_RESOLUTION;
    init_params.camera_fps = CAMERA_FPS;
    init_params.depth_mode = DEPTH_MODE;
    init_params.coordinate_units = COORDINATE_UNITS;
    init_params.coordinate_system = COORDINATE_SYSTEM;
    init_params.depth_minimum_distance = DEPTH_MINIMUM_DISTANCE;
    init_params.depth_maximum_distance = DEPTH_MAXIMUM_DISTANCE;
    init_params.sdk_verbose = 1;

    // If the camera temporarily disconnects, this lets the SDK try to recover during grab().
    init_params.async_grab_camera_recovery = true;

    // 0 means no SDK-side compute FPS cap. The camera FPS setting above remains the capture target.
    init_params.grab_compute_capping_fps = 0;

    const sl::ERROR_CODE open_status = zed.open(init_params);
    if (open_status != sl::ERROR_CODE::SUCCESS) {
        std::cerr << "ERROR: zed.open() failed: " << open_status << "\n";
        return 1;
    }

    const sl::CameraInformation camera_info = zed.getCameraInformation();
    const sl::Resolution image_size = camera_info.camera_configuration.resolution;

    std::cout << "Camera opened.\n";
    std::cout << "  Serial number: " << camera_info.serial_number << "\n";
    std::cout << "  Resolution:    " << image_size.width << " x " << image_size.height << "\n";
    std::cout << "  Target FPS:    " << CAMERA_FPS << "\n";
    std::cout << "  Depth mode:    " << static_cast<int>(DEPTH_MODE) << "\n\n";

    // -----------------------------
    // Configure per-frame processing
    // -----------------------------
    sl::RuntimeParameters runtime_params;
    runtime_params.enable_depth = true;
    runtime_params.enable_fill_mode = ENABLE_DEPTH_FILL_MODE;
    runtime_params.confidence_threshold = CONFIDENCE_THRESHOLD;
    runtime_params.texture_confidence_threshold = TEXTURE_CONFIDENCE_THRESHOLD;
    runtime_params.measure3D_reference_frame = sl::REFERENCE_FRAME::CAMERA;

    // -----------------------------
    // Allocate ZED output buffers
    // -----------------------------
    // CPU memory is used because OpenCV display and getValue() are CPU operations.
    sl::Mat left_image(image_size, sl::MAT_TYPE::U8_C4, sl::MEM::CPU);
    sl::Mat right_image(image_size, sl::MAT_TYPE::U8_C4, sl::MEM::CPU);
    sl::Mat depth_image_display(image_size, sl::MAT_TYPE::U8_C4, sl::MEM::CPU);

    sl::Mat depth_map_f32(image_size, sl::MAT_TYPE::F32_C1, sl::MEM::CPU);
    sl::Mat confidence_map_f32(image_size, sl::MAT_TYPE::F32_C1, sl::MEM::CPU);
    sl::Mat point_cloud_xyzrgba(image_size, sl::MAT_TYPE::F32_C4, sl::MEM::CPU);
    sl::Mat disparity_f32(image_size, sl::MAT_TYPE::F32_C1, sl::MEM::CPU);
    sl::Mat normals_f32(image_size, sl::MAT_TYPE::F32_C4, sl::MEM::CPU);
    sl::Mat depth_u16_mm(image_size, sl::MAT_TYPE::U16_C1, sl::MEM::CPU);

    createWindows();

    FpsMeter fps_meter;
    uint64_t frame_index = 0;

    // -----------------------------
    // Main camera loop
    // -----------------------------
    while (true) {
        // grab() blocks until a new frame is available, then runs image/depth processing.
        const sl::ERROR_CODE grab_status = zed.grab(runtime_params);
        if (grab_status != sl::ERROR_CODE::SUCCESS) {
            // TIMEOUT / CAMERA_REBOOTING can happen during transient USB issues.
            // For a simple prototype, skip this cycle and keep trying.
            continue;
        }

        fps_meter.tick();
        ++frame_index;

        // retrieveImage() returns display/image views.
        if (RETRIEVE_LEFT_IMAGE) {
            zed.retrieveImage(left_image, sl::VIEW::LEFT, sl::MEM::CPU);
        }
        if (RETRIEVE_RIGHT_IMAGE) {
            zed.retrieveImage(right_image, sl::VIEW::RIGHT, sl::MEM::CPU);
        }
        if (RETRIEVE_DEPTH_IMAGE_FOR_DISPLAY) {
            // This is an 8-bit normalized visualization. Do not use it for measurement.
            zed.retrieveImage(depth_image_display, sl::VIEW::DEPTH, sl::MEM::CPU);
        }

        // retrieveMeasure() returns numeric measures for real computation.
        if (RETRIEVE_DEPTH_MAP_F32) {
            zed.retrieveMeasure(depth_map_f32, sl::MEASURE::DEPTH, sl::MEM::CPU);
        }
        if (RETRIEVE_CONFIDENCE_MAP_F32) {
            zed.retrieveMeasure(confidence_map_f32, sl::MEASURE::CONFIDENCE, sl::MEM::CPU);
        }
        if (RETRIEVE_POINT_CLOUD_XYZRGBA) {
            zed.retrieveMeasure(point_cloud_xyzrgba, sl::MEASURE::XYZRGBA, sl::MEM::CPU);
        }
        if (RETRIEVE_DISPARITY_F32) {
            zed.retrieveMeasure(disparity_f32, sl::MEASURE::DISPARITY, sl::MEM::CPU);
        }
        if (RETRIEVE_NORMALS_F32) {
            zed.retrieveMeasure(normals_f32, sl::MEASURE::NORMALS, sl::MEM::CPU);
        }
        if (RETRIEVE_DEPTH_U16_MM) {
            zed.retrieveMeasure(depth_u16_mm, sl::MEASURE::DEPTH_U16_MM, sl::MEM::CPU);
        }

        // Sample the center pixel as a simple measurement sanity check.
        const int center_x = static_cast<int>(image_size.width / 2);
        const int center_y = static_cast<int>(image_size.height / 2);

        float center_depth_z_mm = std::numeric_limits<float>::quiet_NaN();
        float center_confidence = std::numeric_limits<float>::quiet_NaN();
        sl::float4 center_xyzrgba = {
            std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::quiet_NaN()
        };
        float center_range_mm = std::numeric_limits<float>::quiet_NaN();

        if (RETRIEVE_DEPTH_MAP_F32) {
            depth_map_f32.getValue(center_x, center_y, &center_depth_z_mm);
        }
        if (RETRIEVE_CONFIDENCE_MAP_F32) {
            confidence_map_f32.getValue(center_x, center_y, &center_confidence);
        }
        if (RETRIEVE_POINT_CLOUD_XYZRGBA) {
            point_cloud_xyzrgba.getValue(center_x, center_y, &center_xyzrgba);
            if (std::isfinite(center_xyzrgba.x) &&
                std::isfinite(center_xyzrgba.y) &&
                std::isfinite(center_xyzrgba.z)) {
                center_range_mm = std::sqrt(
                    center_xyzrgba.x * center_xyzrgba.x +
                    center_xyzrgba.y * center_xyzrgba.y +
                    center_xyzrgba.z * center_xyzrgba.z);
            }
        }

        // Optional: show how to read other numeric outputs if enabled.
        if (RETRIEVE_DISPARITY_F32 && frame_index % 120 == 0) {
            float center_disparity = std::numeric_limits<float>::quiet_NaN();
            disparity_f32.getValue(center_x, center_y, &center_disparity);
            std::cout << "Center disparity: " << center_disparity << "\n";
        }

        if (RETRIEVE_NORMALS_F32 && frame_index % 120 == 0) {
            sl::float4 center_normal;
            normals_f32.getValue(center_x, center_y, &center_normal);
            std::cout << "Center normal XYZ: "
                      << center_normal.x << ", "
                      << center_normal.y << ", "
                      << center_normal.z << "\n";
        }

        if (RETRIEVE_DEPTH_U16_MM && frame_index % 120 == 0) {
            unsigned short center_depth_u16 = 0;
            depth_u16_mm.getValue(center_x, center_y, &center_depth_u16);
            std::cout << "Center depth U16 mm: " << center_depth_u16 << "\n";
        }

        // Display. Showing multiple full-res windows may be slower than camera capture.
        if ((frame_index % DISPLAY_EVERY_N_FRAMES) == 0) {
            if (RETRIEVE_LEFT_IMAGE) {
                cv::Mat left_cv = slMatToCvMat(left_image);
                drawOverlay(left_cv, fps_meter.fps(), center_depth_z_mm, center_confidence, center_xyzrgba, center_range_mm);
                showScaled("ZED Left + Measurements", left_cv, DISPLAY_SCALE);
            }

            if (RETRIEVE_RIGHT_IMAGE) {
                cv::Mat right_cv = slMatToCvMat(right_image);
                showScaled("ZED Right", right_cv, DISPLAY_SCALE);
            }

            if (RETRIEVE_DEPTH_IMAGE_FOR_DISPLAY) {
                cv::Mat depth_display_cv = slMatToCvMat(depth_image_display);
                showScaled("ZED Depth Display Only", depth_display_cv, DISPLAY_SCALE);
            }

            if (RETRIEVE_CONFIDENCE_MAP_F32) {
                cv::Mat confidence_32f = slMatToCvMat(confidence_map_f32);
                cv::Mat confidence_8u;
                confidence_32f.convertTo(confidence_8u, CV_8UC1, 255.0 / 100.0);
                showScaled("ZED Confidence Display", confidence_8u, DISPLAY_SCALE);
            }

            const int key = cv::waitKey(1);
            if (key == 27 || key == 'q' || key == 'Q') {
                break;
            }
        } else {
            // waitKey is still needed occasionally for OpenCV window event processing.
            const int key = cv::waitKey(1);
            if (key == 27 || key == 'q' || key == 'Q') {
                break;
            }
        }
    }

    std::cout << "Closing camera...\n";
    zed.close();
    cv::destroyAllWindows();

    return 0;
}
