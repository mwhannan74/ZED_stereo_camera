#pragma once

#include <opencv2/core.hpp>

#include <memory>
#include <string>

namespace zed_bridge
{

    /**
     * @brief Camera input resolution options exposed by the bridge API.
     * 
     * Mode   | Resolution | FPS             | FOV
     * HD2K   | 4416x1242  | 15              | Wide
     * HD1080 | 3840x1080  | 30, 15          | Wide
     * HD720  | 2560x720   | 60, 30, 15      | Extra Wide
     * VGA    | 1344x376   | 100, 60, 30, 15 | Extra Wide
     */
    enum class CameraResolution
    {
        ///< Lower image/depth resolution with the highest practical camera rate.
        VGA,

        ///< Good first choice for real-time work; commonly used at 60 FPS.
        HD720,

        ///< Higher image detail with a lower practical frame rate.
        HD1080,

        ///< Highest level of detail but lowest frame rate.
        HD2K
    };

    /**
     * @brief Depth processing modes exposed without requiring ZED SDK headers.
     */
    enum class DepthMode
    {
        ///< Fastest neural depth mode; good for first real-time prototypes. 
        NeuralLight,

        ///< Balanced neural depth mode. 
        // 100 FPS @ VGA with RTX 3070 Ti and 84% utilization.
        // 60 FPS @ HD720 with RTX 3070 Ti and 70% utilization.
        // 30 FPS @ HD1080 with RTX 3070 Ti and 50% utilization.
        // 15 FPS @ HD4k with RTX 3070 Ti and 30% utilization.
        Neural,

        ///< Higher-quality neural depth mode with heavier GPU cost. 
        // Does not support VGA
        // 48 FPS @ HD720 with RTX 3070 Ti and 83% utilization.
        // 30 FPS @ HD1080 with RTX 3070 Ti and 70% utilization.
        // 15 FPS @ HD4k with RTX 3070 Ti and 42% utilization.
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
        ///< Camera image/depth resolution requested before opening the camera.
        CameraResolution resolution = CameraResolution::HD720;

        ///< Target camera FPS requested before opening the camera.
        int fps = 60;

        ///< SDK depth-processing quality/performance mode.
        DepthMode depth_mode = DepthMode::NeuralLight;

        ///< Minimum valid depth in millimeters; values <= 0 use SDK defaults.
        float depth_minimum_distance_mm = 300.0f;

        ///< Maximum valid depth in millimeters; values <= 0 use SDK defaults.
        float depth_maximum_distance_mm = 12000.0f;

        ///< Runtime confidence threshold in [1,100]; lower rejects more pixels.
        int confidence_threshold = 95;

        ///< Texture confidence threshold in [1,100] for low-texture filtering.
        int texture_confidence_threshold = 100;

        ///< Enables SDK hole filling; useful for display, risky for measurement.
        bool enable_depth_fill_mode = false;

        ///< Retrieves the left camera image as BGRA 8-bit OpenCV data.
        bool retrieve_left_image = true;

        ///< Retrieves the right camera image as BGRA 8-bit OpenCV data.
        bool retrieve_right_image = true;

        ///< Retrieves the SDK colorized depth view for display only.
        bool retrieve_depth_image_for_display = true;

        ///< Retrieves numeric 32-bit floating-point Z depth in millimeters.
        bool retrieve_depth_map_f32 = true;

        ///< Retrieves numeric 32-bit floating-point confidence values.
        bool retrieve_confidence_map_f32 = true;

        ///< Retrieves 32-bit floating-point XYZRGBA point-cloud values.
        bool retrieve_point_cloud_xyzrgba = true;

        ///< Retrieves optional 32-bit floating-point disparity values.
        bool retrieve_disparity_f32 = false;

        ///< Retrieves optional 32-bit floating-point normal vectors.
        bool retrieve_normals_f32 = false;

        ///< Retrieves optional unsigned 16-bit depth in millimeters.
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
