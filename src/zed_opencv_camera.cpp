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

#include <algorithm>
#include <cmath>
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
            case CameraResolution::HD2K:
                return sl::RESOLUTION::HD2K;
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
                //return sl::DEPTH_MODE::NEURAL_LIGHT; // new version of SDK (v5)
                return sl::DEPTH_MODE::PERFORMANCE; // old version of SDK (v4)
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

        /**
         * @brief Converts a ZED SDK float3 to an OpenCV fixed-size vector.
         */
        cv::Vec3f toCvVec3(const sl::float3 &value)
        {
            return cv::Vec3f{value.x, value.y, value.z};
        }

        /**
         * @brief Converts a ZED SDK orientation quaternion to bridge x/y/z/w order.
         */
        cv::Vec4f toCvQuaternion(const sl::Orientation &orientation)
        {
            return cv::Vec4f{
                static_cast<float>(orientation.ox),
                static_cast<float>(orientation.oy),
                static_cast<float>(orientation.oz),
                static_cast<float>(orientation.ow)};
        }

        /**
         * @brief Converts the ZED SDK magnetometer heading state to the bridge enum.
         */
        MagneticHeadingState toBridgeHeadingState(sl::SensorsData::MagnetometerData::HEADING_STATE state)
        {
            using SlHeadingState = sl::SensorsData::MagnetometerData::HEADING_STATE;

            switch (state)
            {
            case SlHeadingState::GOOD:
                return MagneticHeadingState::Good;
            case SlHeadingState::OK:
                return MagneticHeadingState::Ok;
            case SlHeadingState::NOT_GOOD:
                return MagneticHeadingState::NotGood;
            case SlHeadingState::NOT_CALIBRATED:
                return MagneticHeadingState::NotCalibrated;
            case SlHeadingState::MAG_NOT_AVAILABLE:
            default:
                return MagneticHeadingState::Unavailable;
            }
        }

        /**
         * @brief Normalizes an angle to [0, 360) degrees.
         */
        double normalize360Deg(double angle_deg)
        {
            double wrapped_deg = std::fmod(angle_deg, 360.0);
            if (wrapped_deg < 0.0)
            {
                wrapped_deg += 360.0;
            }
            return wrapped_deg;
        }

        /**
         * @brief Normalizes an angle to [-180, 180) degrees.
         */
        double normalize180Deg(double angle_deg)
        {
            double wrapped_deg = normalize360Deg(angle_deg + 180.0) - 180.0;
            if (wrapped_deg == -180.0)
            {
                return 180.0;
            }
            return wrapped_deg;
        }

        /**
         * @brief Returns true when a magnetometer sample should correct fused heading.
         */
        bool hasUsableMagneticHeading(const MagnetometerSample &magnetometer)
        {
            if (!magnetometer.available ||
                !std::isfinite(magnetometer.magnetic_heading_deg) ||
                !std::isfinite(magnetometer.magnetic_heading_accuracy) ||
                magnetometer.magnetic_heading_accuracy < 0.0f)
            {
                return false;
            }

            return magnetometer.heading_state == MagneticHeadingState::Good ||
                   magnetometer.heading_state == MagneticHeadingState::Ok;
        }

        /**
         * @brief Converts an SDK startup-relative quaternion to roll, pitch, and yaw.
         */
        OrientationAngles quaternionToOrientationAnglesDeg(const cv::Vec4f &quaternion)
        {
            static constexpr double radians_to_degrees = 180.0 / 3.14159265358979323846;

            const double x = quaternion[0];
            const double y = quaternion[1];
            const double z = quaternion[2];
            const double w = quaternion[3];
            const double norm = std::sqrt(x * x + y * y + z * z + w * w);
            if (norm <= 0.0 || !std::isfinite(norm))
            {
                return {};
            }

            const double qx = x / norm;
            const double qy = y / norm;
            const double qz = z / norm;
            const double qw = w / norm;

            const double sinr_cosp = 2.0 * (qw * qx + qy * qz);
            const double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
            const double roll_rad = std::atan2(sinr_cosp, cosr_cosp);

            const double sinp = 2.0 * (qw * qy - qz * qx);
            const double pitch_rad = std::asin(std::max(-1.0, std::min(1.0, sinp)));

            const double siny_cosp = 2.0 * (qw * qz + qx * qy);
            const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
            const double yaw_rad = std::atan2(siny_cosp, cosy_cosp);
            const double yaw_relative_deg = normalize180Deg(yaw_rad * radians_to_degrees);

            return OrientationAngles{
                roll_rad * radians_to_degrees,
                pitch_rad * radians_to_degrees,
                yaw_relative_deg};
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
            init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
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
            resetHeadingFusion();
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

            // Start from an empty frame so disabled outputs cannot leave stale
            // cv::Mat headers from a previous use of the same ZedFrame object.
            frame = ZedFrame{};

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

            retrieveSensors(frame);

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
                resetHeadingFusion();
            }
        }

        /**
         * @brief Clears state used by the complementary heading filter.
         */
        void resetHeadingFusion()
        {
            has_previous_imu_yaw = false;
            previous_imu_yaw_enu_deg = std::numeric_limits<double>::quiet_NaN();
            has_fused_heading = false;
            previous_fused_heading_deg = std::numeric_limits<double>::quiet_NaN();
            last_corrected_magnetometer_timestamp_ns = 0;
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

        /**
         * @brief Retrieves frame-synchronized sensor data into the public frame type.
         */
        void retrieveSensors(ZedFrame &frame)
        {
            if (camera.getSensorsData(sensors_data, sl::TIME_REFERENCE::IMAGE) != sl::ERROR_CODE::SUCCESS)
            {
                return;
            }

            retrieveImu(frame.imu);
            retrieveMagnetometer(frame.magnetometer);
            updateFusedHeading(frame.fused_heading, frame.imu, frame.magnetometer);
        }

        /**
         * @brief Copies IMU data from the latest SDK sensor sample.
         */
        void retrieveImu(ImuSample &imu)
        {
            const uint64_t timestamp_ns = sensors_data.imu.timestamp.getNanoseconds();
            if (timestamp_ns == 0)
            {
                return;
            }

            imu.available = true;
            imu.timestamp_ns = timestamp_ns;
            imu.linear_acceleration_mps2 = toCvVec3(sensors_data.imu.linear_acceleration);
            imu.angular_velocity_dps = toCvVec3(sensors_data.imu.angular_velocity);
            imu.orientation_xyzw = toCvQuaternion(sensors_data.imu.pose.getOrientation());
            imu.orientation_angles_deg = quaternionToOrientationAnglesDeg(imu.orientation_xyzw);
        }

        /**
         * @brief Copies magnetometer data from the latest SDK sensor sample.
         */
        void retrieveMagnetometer(MagnetometerSample &magnetometer)
        {
            const auto &sdk_magnetometer = sensors_data.magnetometer;
            const uint64_t timestamp_ns = sdk_magnetometer.timestamp.getNanoseconds();
            if (!sdk_magnetometer.is_available || timestamp_ns == 0)
            {
                return;
            }

            magnetometer.available = true;
            magnetometer.timestamp_ns = timestamp_ns;
            magnetometer.magnetic_heading_deg = sdk_magnetometer.magnetic_heading;
            magnetometer.magnetic_heading_accuracy = sdk_magnetometer.magnetic_heading_accuracy;
            magnetometer.heading_state = toBridgeHeadingState(sdk_magnetometer.magnetic_heading_state);
        }

        /**
         * @brief Fuses IMU yaw deltas with absolute magnetic heading corrections.
         */
        void updateFusedHeading(
            FusedHeadingSample &fused_heading,
            const ImuSample &imu,
            const MagnetometerSample &magnetometer)
        {
            if (!imu.available || !std::isfinite(imu.orientation_angles_deg.yaw_relative_deg))
            {
                return;
            }

            const double yaw_enu_now_deg = imu.orientation_angles_deg.yaw_relative_deg;
            if (!has_previous_imu_yaw)
            {
                previous_imu_yaw_enu_deg = yaw_enu_now_deg;
                has_previous_imu_yaw = true;
            }

            if (!has_fused_heading)
            {
                if (!hasUsableMagneticHeading(magnetometer))
                {
                    return;
                }

                previous_fused_heading_deg = normalize360Deg(magnetometer.magnetic_heading_deg);
                has_fused_heading = true;
                last_corrected_magnetometer_timestamp_ns = magnetometer.timestamp_ns;

                fused_heading.available = true;
                fused_heading.heading_deg = previous_fused_heading_deg;
                fused_heading.predicted_heading_deg = previous_fused_heading_deg;
                fused_heading.magnetic_correction_error_deg = 0.0;
                previous_imu_yaw_enu_deg = yaw_enu_now_deg;
                return;
            }

            const double delta_yaw_enu_deg = normalize180Deg(yaw_enu_now_deg - previous_imu_yaw_enu_deg);
            previous_imu_yaw_enu_deg = yaw_enu_now_deg;

            const double heading_predicted_deg = normalize360Deg(previous_fused_heading_deg - delta_yaw_enu_deg);
            double heading_fused_deg = heading_predicted_deg;
            double heading_error_deg = std::numeric_limits<double>::quiet_NaN();

            if (hasUsableMagneticHeading(magnetometer) &&
                magnetometer.timestamp_ns != last_corrected_magnetometer_timestamp_ns)
            {
                const double correction_gain = std::clamp(config.heading_fusion_gain, 0.0, 1.0);
                heading_error_deg = normalize180Deg(magnetometer.magnetic_heading_deg - heading_predicted_deg);
                heading_fused_deg = normalize360Deg(
                    heading_predicted_deg + correction_gain * heading_error_deg);
                last_corrected_magnetometer_timestamp_ns = magnetometer.timestamp_ns;
            }

            previous_fused_heading_deg = heading_fused_deg;

            fused_heading.available = true;
            fused_heading.heading_deg = heading_fused_deg;
            fused_heading.predicted_heading_deg = heading_predicted_deg;
            fused_heading.magnetic_correction_error_deg = heading_error_deg;
        }

        ZedCameraConfig config;
        ZedCameraInfo camera_info;

        // These SDK objects stay private to the bridge. Consumers only see the
        // cv::Mat views exposed through ZedFrame.
        sl::Camera camera;
        sl::RuntimeParameters runtime_params;
        sl::SensorsData sensors_data;
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

        bool has_previous_imu_yaw = false;
        double previous_imu_yaw_enu_deg = std::numeric_limits<double>::quiet_NaN();
        bool has_fused_heading = false;
        double previous_fused_heading_deg = std::numeric_limits<double>::quiet_NaN();
        uint64_t last_corrected_magnetometer_timestamp_ns = 0;
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
