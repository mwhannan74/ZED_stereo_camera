/**
 * @file zed_stereo_camera_main.cpp
 * @brief Demo executable for the ZED OpenCV bridge.
 *
 * The ZED SDK dependency is intentionally isolated in zed_bridge.
 * This file consumes camera output as cv::Mat values, displays selected images,
 * and overlays simple center-pixel measurements.
 */

#include "zed_opencv_camera.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace user_settings
{

    static constexpr zed_bridge::CameraResolution CAMERA_RESOLUTION = zed_bridge::CameraResolution::HD720;
    static constexpr int CAMERA_FPS = 60;

    //static constexpr zed_bridge::DepthMode DEPTH_MODE = zed_bridge::DepthMode::NeuralLight;
    static constexpr zed_bridge::DepthMode DEPTH_MODE = zed_bridge::DepthMode::NeuralPlus;

    static constexpr float DEPTH_MINIMUM_DISTANCE = 300.0f;
    static constexpr float DEPTH_MAXIMUM_DISTANCE = 12000.0f;

    static constexpr int CONFIDENCE_THRESHOLD = 95;
    static constexpr int TEXTURE_CONFIDENCE_THRESHOLD = 100;

    static constexpr bool ENABLE_DEPTH_FILL_MODE = false;

    // OpenCV display controls. Window display can become the bottleneck before
    // camera capture or depth computation does.
    static constexpr double DISPLAY_SCALE = 1.0;
    static constexpr int DISPLAY_EVERY_N_FRAMES = 1;

    static constexpr bool RETRIEVE_LEFT_IMAGE = true;
    static constexpr bool RETRIEVE_RIGHT_IMAGE = true;
    static constexpr bool RETRIEVE_DEPTH_IMAGE_FOR_DISPLAY = true;
    static constexpr bool RETRIEVE_DEPTH_MAP_F32 = true;
    static constexpr bool RETRIEVE_CONFIDENCE_MAP_F32 = true;
    static constexpr bool RETRIEVE_POINT_CLOUD_XYZRGBA = true;
    static constexpr bool RETRIEVE_DISPARITY_F32 = false;
    static constexpr bool RETRIEVE_NORMALS_F32 = false;
    static constexpr bool RETRIEVE_DEPTH_U16_MM = false;

} // namespace user_settings

/**
 * @brief Copies demo constants into the bridge configuration object.
 * @return Camera configuration used to open the bridge.
 */
static zed_bridge::ZedCameraConfig makeCameraConfig()
{
    zed_bridge::ZedCameraConfig config;
    config.resolution = user_settings::CAMERA_RESOLUTION;
    config.fps = user_settings::CAMERA_FPS;
    config.depth_mode = user_settings::DEPTH_MODE;
    config.depth_minimum_distance_mm = user_settings::DEPTH_MINIMUM_DISTANCE;
    config.depth_maximum_distance_mm = user_settings::DEPTH_MAXIMUM_DISTANCE;
    config.confidence_threshold = user_settings::CONFIDENCE_THRESHOLD;
    config.texture_confidence_threshold = user_settings::TEXTURE_CONFIDENCE_THRESHOLD;
    config.enable_depth_fill_mode = user_settings::ENABLE_DEPTH_FILL_MODE;
    config.retrieve_left_image = user_settings::RETRIEVE_LEFT_IMAGE;
    config.retrieve_right_image = user_settings::RETRIEVE_RIGHT_IMAGE;
    config.retrieve_depth_image_for_display = user_settings::RETRIEVE_DEPTH_IMAGE_FOR_DISPLAY;
    config.retrieve_depth_map_f32 = user_settings::RETRIEVE_DEPTH_MAP_F32;
    config.retrieve_confidence_map_f32 = user_settings::RETRIEVE_CONFIDENCE_MAP_F32;
    config.retrieve_point_cloud_xyzrgba = user_settings::RETRIEVE_POINT_CLOUD_XYZRGBA;
    config.retrieve_disparity_f32 = user_settings::RETRIEVE_DISPARITY_F32;
    config.retrieve_normals_f32 = user_settings::RETRIEVE_NORMALS_F32;
    config.retrieve_depth_u16_mm = user_settings::RETRIEVE_DEPTH_U16_MM;
    return config;
}

/**
 * @brief Displays an OpenCV image with optional scaling.
 * @param window_name Existing OpenCV window name.
 * @param image Image to display.
 * @param scale Display scale factor, where 1.0 keeps native size.
 */
static void showScaled(const std::string &window_name, const cv::Mat &image, double scale)
{
    if (image.empty())
    {
        return;
    }

    cv::Mat display_image;
    if (scale > 0.0 && std::abs(scale - 1.0) > 1e-6)
    {
        // INTER_AREA is a good default when shrinking images for display.
        const int display_width = static_cast<int>(image.cols * scale);
        const int display_height = static_cast<int>(image.rows * scale);
        cv::resize(image, display_image, cv::Size(display_width, display_height), 0.0, 0.0, cv::INTER_AREA);
    }
    else
    {
        display_image = image;
    }

    // resizeWindow() keeps WINDOW_NORMAL windows matched to the image aspect
    // ratio instead of leaving a stretched or square-looking window.
    cv::resizeWindow(window_name, display_image.cols, display_image.rows);
    cv::imshow(window_name, display_image);
}

/**
 * @brief Formats a millimeter value as meters for display text.
 * @param millimeters Distance in millimeters.
 * @return Text like "1.234 m" or "nan".
 */
static std::string metersText(float millimeters)
{
    if (!std::isfinite(millimeters))
    {
        return "nan";
    }

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) << (millimeters / 1000.0f) << " m";
    return oss.str();
}

/**
 * @brief Lightweight application-side FPS estimator.
 */
class FpsMeter
{
public:
    /**
     * @brief Registers one processed frame and updates the rolling FPS estimate.
     */
    void tick()
    {
        ++frame_count_;
        const auto now = std::chrono::steady_clock::now();
        const double elapsed_sec = std::chrono::duration<double>(now - last_time_).count();

        if (elapsed_sec >= 0.5)
        {
            fps_ = static_cast<double>(frame_count_) / elapsed_sec;
            frame_count_ = 0;
            last_time_ = now;
        }
    }

    /**
     * @brief Returns the latest FPS estimate.
     */
    double fps() const { return fps_; }

private:
    std::chrono::steady_clock::time_point last_time_ = std::chrono::steady_clock::now();
    int frame_count_ = 0;
    double fps_ = 0.0;
};

/**
 * @brief Center-pixel depth, confidence, and point-cloud values for display.
 */
struct CenterMeasurement
{
    float depth_z_mm = std::numeric_limits<float>::quiet_NaN();
    float confidence = std::numeric_limits<float>::quiet_NaN();
    cv::Vec4f xyzrgba = {
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN()};
    float range_mm = std::numeric_limits<float>::quiet_NaN();
};

/**
 * @brief Samples the center pixel from the enabled frame outputs.
 * @param frame Current frame from the bridge.
 * @return Center-pixel measurement values, with NaNs for unavailable outputs.
 */
static CenterMeasurement sampleCenterMeasurement(const zed_bridge::ZedFrame &frame)
{
    CenterMeasurement measurement;

    // Use the center pixel as a simple sanity check. Real applications usually
    // measure a region, tracked object, or selected pixel instead.
    const cv::Mat &reference_image = !frame.left_bgra.empty() ? frame.left_bgra : frame.depth_mm_32f;
    if (reference_image.empty())
    {
        return measurement;
    }

    const int center_x = reference_image.cols / 2;
    const int center_y = reference_image.rows / 2;

    if (!frame.depth_mm_32f.empty())
    {
        // OpenCV indexes matrices as row, column: at<T>(y, x).
        measurement.depth_z_mm = frame.depth_mm_32f.at<float>(center_y, center_x);
    }
    if (!frame.confidence_32f.empty())
    {
        measurement.confidence = frame.confidence_32f.at<float>(center_y, center_x);
    }
    if (!frame.point_cloud_xyzrgba_32f.empty())
    {
        // XYZRGBA is represented as a four-channel float matrix. X/Y/Z are in
        // millimeters because the bridge configures the ZED SDK to use mm.
        measurement.xyzrgba = frame.point_cloud_xyzrgba_32f.at<cv::Vec4f>(center_y, center_x);
        if (std::isfinite(measurement.xyzrgba[0]) &&
            std::isfinite(measurement.xyzrgba[1]) &&
            std::isfinite(measurement.xyzrgba[2]))
        {
            measurement.range_mm = std::sqrt(
                measurement.xyzrgba[0] * measurement.xyzrgba[0] +
                measurement.xyzrgba[1] * measurement.xyzrgba[1] +
                measurement.xyzrgba[2] * measurement.xyzrgba[2]);
        }
    }

    return measurement;
}

/**
 * @brief Draws measurement text and a center marker on an image.
 * @param image Image to annotate in place.
 * @param app_fps Current application FPS estimate.
 * @param measurement Center-pixel values to display.
 */
static void drawOverlay(cv::Mat &image, double app_fps, const CenterMeasurement &measurement)
{
    const int x = image.cols / 2;
    const int y = image.rows / 2;

    cv::drawMarker(image, cv::Point(x, y), cv::Scalar(0, 255, 255, 255), cv::MARKER_CROSS, 30, 2);

    std::vector<std::string> lines;
    {
        std::ostringstream oss;
        oss << "App FPS: " << std::fixed << std::setprecision(1) << app_fps;
        lines.push_back(oss.str());
    }
    lines.push_back("Center Z depth: " + metersText(measurement.depth_z_mm));
    lines.push_back("Center range:   " + metersText(measurement.range_mm));

    {
        std::ostringstream oss;
        oss << "Confidence:     ";
        if (std::isfinite(measurement.confidence))
        {
            oss << std::fixed << std::setprecision(1) << measurement.confidence << " / 100 (lower is better)";
        }
        else
        {
            oss << "nan";
        }
        lines.push_back(oss.str());
    }

    {
        std::ostringstream oss;
        oss << "XYZ mm:         ";
        if (std::isfinite(measurement.xyzrgba[0]) &&
            std::isfinite(measurement.xyzrgba[1]) &&
            std::isfinite(measurement.xyzrgba[2]))
        {
            oss << std::fixed << std::setprecision(1)
                << measurement.xyzrgba[0] << ", "
                << measurement.xyzrgba[1] << ", "
                << measurement.xyzrgba[2];
        }
        else
        {
            oss << "nan, nan, nan";
        }
        lines.push_back(oss.str());
    }

    const int base_x = 20;
    int base_y = 35;
    const int line_height = 26;

    for (const std::string &line : lines)
    {
        // Draw text twice: a thick dark outline followed by a thin light layer.
        // This keeps the overlay readable on bright and dark camera regions.
        cv::putText(image, line, cv::Point(base_x, base_y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(0, 0, 0, 255), 4, cv::LINE_AA);
        cv::putText(image, line, cv::Point(base_x, base_y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(255, 255, 255, 255), 1, cv::LINE_AA);
        base_y += line_height;
    }
}

/**
 * @brief Creates OpenCV windows for the enabled output views.
 * @param config Camera configuration containing retrieval flags.
 */
static void createWindows(const zed_bridge::ZedCameraConfig &config)
{
    if (config.retrieve_left_image)
    {
        cv::namedWindow("ZED Left + Measurements", cv::WINDOW_NORMAL);
    }
    if (config.retrieve_right_image)
    {
        cv::namedWindow("ZED Right", cv::WINDOW_NORMAL);
    }
    if (config.retrieve_depth_image_for_display)
    {
        cv::namedWindow("ZED Depth Display Only", cv::WINDOW_NORMAL);
    }
    if (config.retrieve_confidence_map_f32)
    {
        cv::namedWindow("ZED Confidence Display", cv::WINDOW_NORMAL);
    }
}

/**
 * @brief Prints optional diagnostic values at a low frequency.
 * @param frame Current frame from the bridge.
 * @param frame_index Number of successfully grabbed frames.
 */
static void printOptionalDiagnostics(const zed_bridge::ZedFrame &frame, uint64_t frame_index)
{
    if (!frame.disparity_32f.empty() && frame_index % 120 == 0)
    {
        const int center_x = frame.disparity_32f.cols / 2;
        const int center_y = frame.disparity_32f.rows / 2;
        std::cout << "Center disparity: " << frame.disparity_32f.at<float>(center_y, center_x) << "\n";
    }

    if (!frame.normals_xyzrgba_32f.empty() && frame_index % 120 == 0)
    {
        const int center_x = frame.normals_xyzrgba_32f.cols / 2;
        const int center_y = frame.normals_xyzrgba_32f.rows / 2;
        const cv::Vec4f center_normal = frame.normals_xyzrgba_32f.at<cv::Vec4f>(center_y, center_x);
        std::cout << "Center normal XYZ: "
                  << center_normal[0] << ", "
                  << center_normal[1] << ", "
                  << center_normal[2] << "\n";
    }

    if (!frame.depth_u16_mm.empty() && frame_index % 120 == 0)
    {
        const int center_x = frame.depth_u16_mm.cols / 2;
        const int center_y = frame.depth_u16_mm.rows / 2;
        std::cout << "Center depth U16 mm: " << frame.depth_u16_mm.at<unsigned short>(center_y, center_x) << "\n";
    }
}

/**
 * @brief Opens the camera, displays enabled views, and exits on ESC or Q.
 * @return 0 on clean shutdown, non-zero if the camera fails to open.
 */
int main()
{
    std::cout << "Starting ZED stereo camera prototype...\n";
    std::cout << "Press ESC or Q in an OpenCV window to exit.\n\n";

    const zed_bridge::ZedCameraConfig config = makeCameraConfig();

    // From this point on, the app works through the OpenCV-facing bridge API.
    // There are no ZED SDK types in this file.
    zed_bridge::ZedOpenCvCamera camera(config);

    std::string open_error;
    if (!camera.open(&open_error))
    {
        std::cerr << "ERROR: " << open_error << "\n";
        return 1;
    }

    const zed_bridge::ZedCameraInfo camera_info = camera.cameraInfo();
    std::cout << "Camera opened.\n";
    std::cout << "  Serial number: " << camera_info.serial_number << "\n";
    std::cout << "  Resolution:    " << camera_info.width << " x " << camera_info.height << "\n";
    std::cout << "  Target FPS:    " << camera_info.target_fps << "\n\n";

    createWindows(config);

    FpsMeter fps_meter;
    zed_bridge::ZedFrame frame;
    uint64_t frame_index = 0;

    while (true)
    {
        // grab() returns false for transient camera/USB states. For a simple
        // tutorial app, skip that cycle and continue trying.
        if (!camera.grab(frame))
        {
            continue;
        }

        fps_meter.tick();
        ++frame_index;

        const CenterMeasurement measurement = sampleCenterMeasurement(frame);
        printOptionalDiagnostics(frame, frame_index);

        if ((frame_index % user_settings::DISPLAY_EVERY_N_FRAMES) == 0)
        {
            if (!frame.left_bgra.empty())
            {
                // The frame matrices are bridge-owned views. Drawing here edits
                // the current display buffer and is overwritten on the next grab.
                drawOverlay(frame.left_bgra, fps_meter.fps(), measurement);
                showScaled("ZED Left + Measurements", frame.left_bgra, user_settings::DISPLAY_SCALE);
            }
            if (!frame.right_bgra.empty())
            {
                showScaled("ZED Right", frame.right_bgra, user_settings::DISPLAY_SCALE);
            }
            if (!frame.depth_display_bgra.empty())
            {
                showScaled("ZED Depth Display Only", frame.depth_display_bgra, user_settings::DISPLAY_SCALE);
            }
            if (!frame.confidence_32f.empty())
            {
                // Confidence is numeric float data. Convert it to 8-bit only for
                // display; keep the 32-bit matrix for measurement/debug logic.
                cv::Mat confidence_8u;
                frame.confidence_32f.convertTo(confidence_8u, CV_8UC1, 255.0 / 100.0);
                showScaled("ZED Confidence Display", confidence_8u, user_settings::DISPLAY_SCALE);
            }
        }

        const int key = cv::waitKey(1);
        if (key == 27 || key == 'q' || key == 'Q')
        {
            break;
        }
    }

    std::cout << "Closing camera...\n";
    camera.close();
    cv::destroyAllWindows();

    return 0;
}
