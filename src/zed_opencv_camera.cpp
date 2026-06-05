/**
 * @file zed_opencv_camera.cpp
 * @brief Implements the ZED SDK to OpenCV bridge.
 *
 * This file is the only place in the project that talks directly to the
 * Stereolabs ZED SDK. It owns SDK-specific objects such as sl::Camera and
 * sl::Mat, then exposes zero-copy cv::Mat views through the public API.
 */

#include "zed_opencv_camera.hpp"

#include <sl/Camera.hpp>

#include <stdexcept>
#include <utility>

namespace zed_bridge
{
    namespace
    {

        /**
         * @brief Converts a public bridge resolution to the matching ZED SDK enum.
         */
        sl::RESOLUTION toSlResolution(CameraResolution resolution)
        {
            switch (resolution)
            {
            case CameraResolution::VGA:
                return sl::RESOLUTION::VGA;
            case CameraResolution::HD1080:
                return sl::RESOLUTION::HD1080;
            case CameraResolution::HD720:
            default:
                return sl::RESOLUTION::HD720;
            }
        }

        /**
         * @brief Converts a public bridge depth mode to the matching ZED SDK enum.
         */
        sl::DEPTH_MODE toSlDepthMode(DepthMode depth_mode)
        {
            switch (depth_mode)
            {
            case DepthMode::Neural:
                return sl::DEPTH_MODE::NEURAL;
            case DepthMode::NeuralPlus:
                return sl::DEPTH_MODE::NEURAL_PLUS;
            case DepthMode::NeuralLight:
            default:
                return sl::DEPTH_MODE::NEURAL_LIGHT;
            }
        }

        /**
         * @brief Builds an OpenCV matrix header over a ZED SDK matrix.
         * @param input Source ZED matrix in CPU memory.
         * @return cv::Mat header referencing the same pixel buffer.
         * @throws std::runtime_error if the ZED matrix type is unsupported.
         *
         * This function does not copy pixels. The returned cv::Mat points at the same
         * CPU buffer owned by the input sl::Mat. This is efficient for real-time video,
         * but callers must clone the cv::Mat if they need to keep it after the next
         * camera grab.
         */
        cv::Mat slMatToCvMat(sl::Mat &input)
        {
            int cv_type = -1;

            switch (input.getDataType())
            {
            case sl::MAT_TYPE::F32_C1:
                cv_type = CV_32FC1;
                break;
            case sl::MAT_TYPE::F32_C2:
                cv_type = CV_32FC2;
                break;
            case sl::MAT_TYPE::F32_C3:
                cv_type = CV_32FC3;
                break;
            case sl::MAT_TYPE::F32_C4:
                cv_type = CV_32FC4;
                break;
            case sl::MAT_TYPE::U8_C1:
                cv_type = CV_8UC1;
                break;
            case sl::MAT_TYPE::U8_C2:
                cv_type = CV_8UC2;
                break;
            case sl::MAT_TYPE::U8_C3:
                cv_type = CV_8UC3;
                break;
            case sl::MAT_TYPE::U8_C4:
                cv_type = CV_8UC4;
                break;
            case sl::MAT_TYPE::U16_C1:
                cv_type = CV_16UC1;
                break;
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

    } // namespace

    /**
     * @brief Private implementation that keeps ZED SDK types out of the public header.
     */
    class ZedOpenCvCamera::Impl
    {
    public:
        /**
         * @brief Stores configuration until the camera is opened.
         */
        explicit Impl(ZedCameraConfig camera_config)
            : config(std::move(camera_config))
        {
        }

        /**
         * @brief Opens the camera and allocates reusable output buffers.
         */
        bool open(std::string *error_message)
        {
            // InitParameters are one-time startup settings: camera resolution,
            // target FPS, depth mode, units, coordinate convention, and SDK options.
            sl::InitParameters init_params;
            init_params.camera_resolution = toSlResolution(config.resolution);
            init_params.camera_fps = config.fps;
            init_params.depth_mode = toSlDepthMode(config.depth_mode);
            init_params.coordinate_units = sl::UNIT::MILLIMETER;
            init_params.coordinate_system = sl::COORDINATE_SYSTEM::IMAGE;
            init_params.depth_minimum_distance = config.depth_minimum_distance_mm;
            init_params.depth_maximum_distance = config.depth_maximum_distance_mm;
            init_params.sdk_verbose = 1;
            init_params.async_grab_camera_recovery = true;
            init_params.grab_compute_capping_fps = 0;

            // Opening the camera creates the SDK capture/depth pipeline. If another
            // process owns the camera, this call usually fails.
            const sl::ERROR_CODE open_status = camera.open(init_params);
            if (open_status != sl::ERROR_CODE::SUCCESS)
            {
                if (error_message)
                {
                    *error_message = "zed.open() failed: " + std::string(sl::toString(open_status).c_str());
                }
                return false;
            }

            const sl::CameraInformation zed_info = camera.getCameraInformation();
            const sl::Resolution resolution = zed_info.camera_configuration.resolution;

            // Store a simple OpenCV-facing metadata snapshot for the demo app.
            camera_info.serial_number = zed_info.serial_number;
            camera_info.width = static_cast<int>(resolution.width);
            camera_info.height = static_cast<int>(resolution.height);
            camera_info.target_fps = config.fps;

            // RuntimeParameters are applied each time grab() runs. They control
            // per-frame depth filtering and can be changed while the camera is open.
            runtime_params.enable_depth = true;
            runtime_params.enable_fill_mode = config.enable_depth_fill_mode;
            runtime_params.confidence_threshold = config.confidence_threshold;
            runtime_params.texture_confidence_threshold = config.texture_confidence_threshold;
            runtime_params.measure3D_reference_frame = sl::REFERENCE_FRAME::CAMERA;

            allocateBuffers(resolution);
            is_open = true;
            return true;
        }

        /**
         * @brief Retrieves one camera frame and populates enabled OpenCV views.
         */
        bool grab(ZedFrame &frame)
        {
            if (!is_open)
            {
                return false;
            }

            const sl::ERROR_CODE grab_status = camera.grab(runtime_params);
            if (grab_status != sl::ERROR_CODE::SUCCESS)
            {
                // TIMEOUT or transient recovery states can happen during USB/camera
                // interruptions. The demo simply skips this frame and keeps trying.
                return false;
            }

            // retrieveImage() returns image-style outputs for display or ordinary
            // image processing. retrieveMeasure() returns numeric outputs such as
            // metric depth, confidence, disparity, normals, and point clouds.
            if (config.retrieve_left_image)
            {
                camera.retrieveImage(left_image, sl::VIEW::LEFT, sl::MEM::CPU);
                frame.left_bgra = slMatToCvMat(left_image);
            }
            if (config.retrieve_right_image)
            {
                camera.retrieveImage(right_image, sl::VIEW::RIGHT, sl::MEM::CPU);
                frame.right_bgra = slMatToCvMat(right_image);
            }
            if (config.retrieve_depth_image_for_display)
            {
                camera.retrieveImage(depth_image_display, sl::VIEW::DEPTH, sl::MEM::CPU);
                frame.depth_display_bgra = slMatToCvMat(depth_image_display);
            }
            if (config.retrieve_depth_map_f32)
            {
                camera.retrieveMeasure(depth_map_f32, sl::MEASURE::DEPTH, sl::MEM::CPU);
                frame.depth_mm_32f = slMatToCvMat(depth_map_f32);
            }
            if (config.retrieve_confidence_map_f32)
            {
                camera.retrieveMeasure(confidence_map_f32, sl::MEASURE::CONFIDENCE, sl::MEM::CPU);
                frame.confidence_32f = slMatToCvMat(confidence_map_f32);
            }
            if (config.retrieve_point_cloud_xyzrgba)
            {
                camera.retrieveMeasure(point_cloud_xyzrgba, sl::MEASURE::XYZRGBA, sl::MEM::CPU);
                frame.point_cloud_xyzrgba_32f = slMatToCvMat(point_cloud_xyzrgba);
            }
            if (config.retrieve_disparity_f32)
            {
                camera.retrieveMeasure(disparity_f32, sl::MEASURE::DISPARITY, sl::MEM::CPU);
                frame.disparity_32f = slMatToCvMat(disparity_f32);
            }
            if (config.retrieve_normals_f32)
            {
                camera.retrieveMeasure(normals_f32, sl::MEASURE::NORMALS, sl::MEM::CPU);
                frame.normals_xyzrgba_32f = slMatToCvMat(normals_f32);
            }
            if (config.retrieve_depth_u16_mm)
            {
                camera.retrieveMeasure(depth_u16_mm, sl::MEASURE::DEPTH_U16_MM, sl::MEM::CPU);
                frame.depth_u16_mm = slMatToCvMat(depth_u16_mm);
            }

            return true;
        }

        /**
         * @brief Closes the ZED camera if it is currently open.
         */
        void close()
        {
            if (is_open)
            {
                camera.close();
                is_open = false;
            }
        }

        /**
         * @brief Allocates bridge-owned ZED buffers for the opened camera resolution.
         */
        void allocateBuffers(const sl::Resolution &resolution)
        {
            // Allocate once and reuse every frame. Reusing buffers avoids repeated
            // heap allocations in the live camera loop.
            left_image = sl::Mat(resolution, sl::MAT_TYPE::U8_C4, sl::MEM::CPU);
            right_image = sl::Mat(resolution, sl::MAT_TYPE::U8_C4, sl::MEM::CPU);
            depth_image_display = sl::Mat(resolution, sl::MAT_TYPE::U8_C4, sl::MEM::CPU);
            depth_map_f32 = sl::Mat(resolution, sl::MAT_TYPE::F32_C1, sl::MEM::CPU);
            confidence_map_f32 = sl::Mat(resolution, sl::MAT_TYPE::F32_C1, sl::MEM::CPU);
            point_cloud_xyzrgba = sl::Mat(resolution, sl::MAT_TYPE::F32_C4, sl::MEM::CPU);
            disparity_f32 = sl::Mat(resolution, sl::MAT_TYPE::F32_C1, sl::MEM::CPU);
            normals_f32 = sl::Mat(resolution, sl::MAT_TYPE::F32_C4, sl::MEM::CPU);
            depth_u16_mm = sl::Mat(resolution, sl::MAT_TYPE::U16_C1, sl::MEM::CPU);
        }

        ZedCameraConfig config;
        ZedCameraInfo camera_info;

        // These SDK objects stay private to the bridge. Consumers only see the
        // cv::Mat views exposed through ZedFrame.
        sl::Camera camera;
        sl::RuntimeParameters runtime_params;
        bool is_open = false;

        sl::Mat left_image;
        sl::Mat right_image;
        sl::Mat depth_image_display;
        sl::Mat depth_map_f32;
        sl::Mat confidence_map_f32;
        sl::Mat point_cloud_xyzrgba;
        sl::Mat disparity_f32;
        sl::Mat normals_f32;
        sl::Mat depth_u16_mm;
    };

    /**
     * @brief Constructs the public camera wrapper and its private implementation.
     */
    ZedOpenCvCamera::ZedOpenCvCamera(ZedCameraConfig config)
        : impl_(std::make_unique<Impl>(std::move(config)))
    {
    }

    ZedOpenCvCamera::~ZedOpenCvCamera() = default;

    ZedOpenCvCamera::ZedOpenCvCamera(ZedOpenCvCamera &&) noexcept = default;

    ZedOpenCvCamera &ZedOpenCvCamera::operator=(ZedOpenCvCamera &&) noexcept = default;

    /**
     * @brief Opens the ZED camera through the private implementation.
     */
    bool ZedOpenCvCamera::open(std::string *error_message)
    {
        return impl_->open(error_message);
    }

    /**
     * @brief Grabs one frame through the private implementation.
     */
    bool ZedOpenCvCamera::grab(ZedFrame &frame)
    {
        return impl_->grab(frame);
    }

    /**
     * @brief Closes the camera through the private implementation.
     */
    void ZedOpenCvCamera::close()
    {
        impl_->close();
    }

    /**
     * @brief Returns the immutable camera configuration.
     */
    const ZedCameraConfig &ZedOpenCvCamera::config() const
    {
        return impl_->config;
    }

    /**
     * @brief Returns metadata captured after a successful open.
     */
    ZedCameraInfo ZedOpenCvCamera::cameraInfo() const
    {
        return impl_->camera_info;
    }

} // namespace zed_bridge
