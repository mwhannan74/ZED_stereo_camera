#pragma once

#include <opencv2/core.hpp>

#include <memory>
#include <string>

namespace zed_bridge
{

enum class CameraResolution
{
    VGA,
    HD720,
    HD1080
};

enum class DepthMode
{
    NeuralLight,
    Neural,
    NeuralPlus
};

struct ZedCameraConfig
{
    CameraResolution resolution = CameraResolution::HD720;
    int fps = 60;
    DepthMode depth_mode = DepthMode::NeuralLight;
    float depth_minimum_distance_mm = 300.0f;
    float depth_maximum_distance_mm = 12000.0f;
    int confidence_threshold = 95;
    int texture_confidence_threshold = 100;
    bool enable_depth_fill_mode = false;

    bool retrieve_left_image = true;
    bool retrieve_right_image = true;
    bool retrieve_depth_image_for_display = true;
    bool retrieve_depth_map_f32 = true;
    bool retrieve_confidence_map_f32 = true;
    bool retrieve_point_cloud_xyzrgba = true;
    bool retrieve_disparity_f32 = false;
    bool retrieve_normals_f32 = false;
    bool retrieve_depth_u16_mm = false;
};

/**
 * Camera metadata reported after a successful open.
 */
struct ZedCameraInfo
{
    unsigned int serial_number = 0;
    int width = 0;
    int height = 0;
    int target_fps = 0;
};

/**
 * OpenCV views of one grabbed ZED frame.
 *
 * Matrices reference bridge-owned buffers and remain valid until the next
 * successful grab or camera close. Clone any matrix that must outlive that.
 */
struct ZedFrame
{
    cv::Mat left_bgra;
    cv::Mat right_bgra;
    cv::Mat depth_display_bgra;
    cv::Mat depth_mm_32f;
    cv::Mat confidence_32f;
    cv::Mat point_cloud_xyzrgba_32f;
    cv::Mat disparity_32f;
    cv::Mat normals_xyzrgba_32f;
    cv::Mat depth_u16_mm;
};

/**
 * Owns a ZED camera and exposes selected outputs as OpenCV matrices.
 *
 * The public API intentionally avoids ZED/CUDA types so consumers can depend
 * on OpenCV-facing data while the implementation owns SDK-specific resources.
 */
class ZedOpenCvCamera
{
public:
    explicit ZedOpenCvCamera(ZedCameraConfig config = {});
    ~ZedOpenCvCamera();

    ZedOpenCvCamera(const ZedOpenCvCamera &) = delete;
    ZedOpenCvCamera &operator=(const ZedOpenCvCamera &) = delete;

    ZedOpenCvCamera(ZedOpenCvCamera &&) noexcept;
    ZedOpenCvCamera &operator=(ZedOpenCvCamera &&) noexcept;

    /**
     * Opens the first available ZED camera with the configured settings.
     */
    bool open(std::string *error_message = nullptr);

    /**
     * Grabs one frame and updates enabled matrices in `frame`.
     */
    bool grab(ZedFrame &frame);

    /**
     * Closes the camera if it is open.
     */
    void close();

    const ZedCameraConfig &config() const;
    ZedCameraInfo cameraInfo() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace zed_bridge
