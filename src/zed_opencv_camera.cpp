#include "zed_opencv_camera.hpp"

#include <sl/Camera.hpp>

#include <stdexcept>
#include <utility>

namespace zed_bridge
{
namespace
{

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

class ZedOpenCvCamera::Impl
{
public:
    explicit Impl(ZedCameraConfig camera_config)
        : config(std::move(camera_config))
    {
    }

    bool open(std::string *error_message)
    {
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

        camera_info.serial_number = zed_info.serial_number;
        camera_info.width = static_cast<int>(resolution.width);
        camera_info.height = static_cast<int>(resolution.height);
        camera_info.target_fps = config.fps;

        runtime_params.enable_depth = true;
        runtime_params.enable_fill_mode = config.enable_depth_fill_mode;
        runtime_params.confidence_threshold = config.confidence_threshold;
        runtime_params.texture_confidence_threshold = config.texture_confidence_threshold;
        runtime_params.measure3D_reference_frame = sl::REFERENCE_FRAME::CAMERA;

        allocateBuffers(resolution);
        is_open = true;
        return true;
    }

    bool grab(ZedFrame &frame)
    {
        if (!is_open)
        {
            return false;
        }

        const sl::ERROR_CODE grab_status = camera.grab(runtime_params);
        if (grab_status != sl::ERROR_CODE::SUCCESS)
        {
            return false;
        }

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

    void close()
    {
        if (is_open)
        {
            camera.close();
            is_open = false;
        }
    }

    void allocateBuffers(const sl::Resolution &resolution)
    {
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

ZedOpenCvCamera::ZedOpenCvCamera(ZedCameraConfig config)
    : impl_(std::make_unique<Impl>(std::move(config)))
{
}

ZedOpenCvCamera::~ZedOpenCvCamera() = default;

ZedOpenCvCamera::ZedOpenCvCamera(ZedOpenCvCamera &&) noexcept = default;

ZedOpenCvCamera &ZedOpenCvCamera::operator=(ZedOpenCvCamera &&) noexcept = default;

bool ZedOpenCvCamera::open(std::string *error_message)
{
    return impl_->open(error_message);
}

bool ZedOpenCvCamera::grab(ZedFrame &frame)
{
    return impl_->grab(frame);
}

void ZedOpenCvCamera::close()
{
    impl_->close();
}

const ZedCameraConfig &ZedOpenCvCamera::config() const
{
    return impl_->config;
}

ZedCameraInfo ZedOpenCvCamera::cameraInfo() const
{
    return impl_->camera_info;
}

} // namespace zed_bridge
