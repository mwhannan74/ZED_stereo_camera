#pragma once

#include <opencv2/core.hpp>

#include <memory>
#include <string>

namespace zed_bridge
{

    /**
     * @brief Camera input resolution options exposed by the bridge API.
     */
    enum class CameraResolution
    {
        VGA,
        HD720,
        HD1080
    };

    /**
     * @brief Depth processing modes exposed without requiring ZED SDK headers.
     */
    enum class DepthMode
    {
        NeuralLight,
        Neural,
        NeuralPlus
    };

    /**
     * @brief Startup and retrieval settings for ZedOpenCvCamera.
     *
     * Distances are expressed in millimeters. Retrieval flags control which frame
     * outputs are copied from the ZED SDK into OpenCV-visible buffers.
     */
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
     * @brief Camera metadata reported after a successful open.
     */
    struct ZedCameraInfo
    {
        unsigned int serial_number = 0;
        int width = 0;
        int height = 0;
        int target_fps = 0;
    };

    /**
     * @brief OpenCV views of one grabbed ZED frame.
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
     * @brief Owns a ZED camera and exposes selected outputs as OpenCV matrices.
     *
     * The public API intentionally avoids ZED/CUDA types so consumers can depend
     * on OpenCV-facing data while the implementation owns SDK-specific resources.
     */
    class ZedOpenCvCamera
    {
    public:
        /**
         * @brief Creates a camera bridge with the requested startup settings.
         */
        explicit ZedOpenCvCamera(ZedCameraConfig config = {});

        /**
         * @brief Releases implementation resources.
         */
        ~ZedOpenCvCamera();

        ZedOpenCvCamera(const ZedOpenCvCamera &) = delete;
        ZedOpenCvCamera &operator=(const ZedOpenCvCamera &) = delete;

        ZedOpenCvCamera(ZedOpenCvCamera &&) noexcept;
        ZedOpenCvCamera &operator=(ZedOpenCvCamera &&) noexcept;

        /**
         * @brief Opens the first available ZED camera with the configured settings.
         * @param error_message Optional destination for a human-readable open error.
         * @return True if the camera opened successfully.
         */
        bool open(std::string *error_message = nullptr);

        /**
         * @brief Grabs one frame and updates enabled matrices in `frame`.
         * @param frame Destination for OpenCV views of enabled outputs.
         * @return True when a frame was successfully retrieved.
         */
        bool grab(ZedFrame &frame);

        /**
         * @brief Closes the camera if it is open.
         */
        void close();

        /**
         * @brief Returns the immutable bridge configuration.
         */
        const ZedCameraConfig &config() const;

        /**
         * @brief Returns metadata captured after opening the camera.
         */
        ZedCameraInfo cameraInfo() const;

    private:
        class Impl;
        std::unique_ptr<Impl> impl_;
    };

} // namespace zed_bridge
