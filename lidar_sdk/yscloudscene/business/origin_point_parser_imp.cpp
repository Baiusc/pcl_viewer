#include "origin_point_parser_imp.h"
#include "pointcloud_parser.h"
#include "log/logging.h"
#include "tiny_hut/common_toolset.h" 
#include "config/config_service.h"
#include "mf150_calibration_runner.h"
#include "cs96_raw_save.h"
#include "cs96_gamma_raw.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <string>
#include <cmath>
#include <algorithm>

#ifdef YS_HAS_MF150_PICK
#include "component/pointcloud/picking/mf150_origin_pick_provider.h"
#endif

// ===== Beta临时: 编码器角度刻度表 (来自 saveFrameToFile2 使用的 g_encoder_angle) =====
static constexpr float kBetaEncoderAngle[] = {
    0,       1.506,   3.034,   4.477,   5.968,   7.522,   9.005,   10.511,
    12.019,  13.492,  15.026,  16.532,  18.038,  19.567,  21.092,  22.53,
    24.097,  25.566,  27.05,   28.546,  30.091,  31.591,  33.087,  34.588,
    36.107,  37.61,   39.12,   40.605,  42.108,  43.651,  45.183,  46.659,
    48.164,  49.654,  51.167,  52.64,   54.164,  55.675,  57.19,   58.728,
    60.237,  61.732,  63.259,  64.725,  66.226,  67.735,  69.231,  70.737,
    72.27,   73.743,  75.256,  76.835,  78.27,   79.78,   81.325,  82.856,
    84.341,  85.829,  87.366,  88.868,  90.459,  91.903,  93.385,  94.857,
    96.346,  97.854,  99.358,  100.921, 102.343, 103.902, 105.408, 106.904,
    108.456, 109.967, 111.459, 112.987, 114.503, 116.008, 117.543, 119.007,
    120.474, 121.977, 123.484, 124.999, 126.467, 128.024, 129.546, 130.969,
    132.448, 134.004, 135.509, 137.024, 138.58,  140.075, 141.562, 143.056,
    144.582, 146.032, 147.555, 149.039, 150.554, 152.013, 153.535, 155.018,
    156.528, 158.006, 159.497, 161.052, 162.552, 164.051, 165.569, 167.075,
    168.571, 170.089, 171.573, 173.061, 174.538, 176.012, 177.575, 179.043,
    180.533, 182.065, 183.512, 185.031, 186.517, 188.025, 189.467, 191.002,
    192.523, 194.008, 195.518, 197.019, 198.502, 199.977, 201.496, 202.96,
    204.504, 205.957, 207.464, 208.941, 210.439, 211.966, 213.402, 214.942,
    216.505, 217.917, 219.389, 220.883, 222.359, 223.889, 225.342, 226.864,
    228.363, 229.854, 231.38,  232.83,  234.331, 235.814, 237.325, 238.806,
    240.298, 241.793, 243.28,  244.785, 246.281, 247.79,  249.294, 250.763,
    252.211, 253.761, 255.222, 256.77,  258.213, 259.744, 261.176, 262.764,
    264.277, 265.718, 267.199, 268.727, 270.191, 271.664, 273.147, 274.695,
    276.112, 277.614, 279.108, 280.585, 282.137, 283.559, 285.041, 286.551,
    288.127, 289.574, 291.042, 292.552, 294.078, 295.623, 297.058, 298.638,
    300.048, 301.535, 302.991, 304.526, 305.984, 307.501, 308.975, 310.456,
    312.004, 313.457, 314.987, 316.464, 317.998, 319.498, 321.037, 322.518,
    323.985, 325.485, 327.038, 328.488, 329.977, 331.506, 333.016, 334.43,
    335.954, 337.465, 338.966, 340.488, 341.993, 343.497, 344.968, 346.456,
    347.948, 349.451, 350.96,  352.537, 354.01,  355.483, 356.984, 358.474,
    360,
};
static constexpr size_t kBetaEncoderAngleCnt = sizeof(kBetaEncoderAngle) / sizeof(kBetaEncoderAngle[0]);

// 多点 LUT 距离补偿：每像素一维分段线性插值(与算法侧 algo_calib_apply.c 的
// Interpolate1D 一致：越界钳位边界值，未标定像素返回 0)。表尺寸 152×360。
static float LookupMultiTableOffset(const SslDistCalibMulti& table, uint16_t row, uint16_t col, float raw_dist)
{
    if (row >= SSL_MAX_ROWS || col >= SSL_MAX_COLS || raw_dist <= 0.0f) {
        return 0.0f;
    }
    const SslPixelDistCurve& curve = table.curves[row][col];
    if (curve.num_pts == 0) {
        return 0.0f;
    }
    if (curve.num_pts == 1) {
        return curve.offsets[0];
    }
    if (raw_dist <= curve.raw_dists[0]) {
        return curve.offsets[0];
    }
    if (raw_dist >= curve.raw_dists[curve.num_pts - 1]) {
        return curve.offsets[curve.num_pts - 1];
    }
    for (uint8_t i = 0; i < curve.num_pts - 1; ++i) {
        if (raw_dist >= curve.raw_dists[i] && raw_dist <= curve.raw_dists[i + 1]) {
            float dx = curve.raw_dists[i + 1] - curve.raw_dists[i];
            if (dx < 1e-6f) {
                return curve.offsets[i];
            }
            float ratio = (raw_dist - curve.raw_dists[i]) / dx;
            return curve.offsets[i] + ratio * (curve.offsets[i + 1] - curve.offsets[i]);
        }
    }
    return 0.0f;
}

namespace ys_scene {
OriginPointParseImp::OriginPointParseImp(QObject *parent)
    : QObject(parent)
{
    // 多点标定 LUT 表经 Qt 队列跨线程传递，需注册元类型
    qRegisterMetaType<std::shared_ptr<SslDistCalibMulti>>("std::shared_ptr<SslDistCalibMulti>");

    auto& config = you_see::ConfigService::Instance();
    
    // 初始化ROI值
    origin_point_roi_.min_slot = config.Get<int>("cloud_roi.min_slot", 0);
    origin_point_roi_.max_slot = config.Get<int>("cloud_roi.max_slot", 799);
    origin_point_roi_.min_pixel = config.Get<int>("cloud_roi.min_pixel", 0);
    origin_point_roi_.max_pixel = config.Get<int>("cloud_roi.max_pixel", 95);
	
    // 初始化标准标定参数
    if (LoadStandardCalibrationParams()) {
		standard_calibration_params_.CalculateTheta();
    }
}

OriginPointParseImp::~OriginPointParseImp()
{
    std::cout << "OriginPointParse Destruct" << std::endl; 
}

void OriginPointParseImp::SetPointcloudParser(PointcloudParser* parser)
{
    pointcloud_parser_ = parser;
}

void OriginPointParseImp::SetAlgorithmRepairGroundEnable(int state)
{
	algorithm_repair_ground_ = state;
}

void OriginPointParseImp::OriginPoint2xyz(const OriginFramePtr& origin_frame)
{
    if (!calibration_params_.inited) {
        LOG_DEBUG("Calibration parameters not initialized");
        return;
    }

    CloudFramePtr cartesian_frame;

    // 根据是否启用修复算法选择处理流程
    if (algorithm_repair_ground_) {
        // 方式1: 使用地面修复算法
        // 步骤1: 将原始点云转换为PointMatrix
        auto start_conversion = std::chrono::steady_clock::now();
        auto matrix = std::make_shared<algo_lidar::PointMatrix>(OriginToPointMatrix(origin_frame, origin_point_roi_));
        cartesian_frame = PointMatrixToCartesian(*matrix, origin_frame->frame_number);
        auto conversion_time = std::chrono::steady_clock::now() - start_conversion;

        // 步骤2: 应用地面修复算法
        auto start_repair = std::chrono::steady_clock::now();
        algo_lidar::Ground_I::doRepair(*matrix);
        auto repair_time = std::chrono::steady_clock::now() - start_repair;

        // 步骤3: 将处理后的PointMatrix转换为笛卡尔点云
        auto start_cartesian = std::chrono::steady_clock::now();
        cartesian_frame = PointMatrixToCartesian(*matrix, origin_frame->frame_number);
        auto cartesian_time = std::chrono::steady_clock::now() - start_cartesian;

        LOG_INFO("Convert: {} seconds, Fix: {} seconds, Cartesian transformation: {} seconds",
            std::chrono::duration<double>(conversion_time).count(),
            std::chrono::duration<double>(repair_time).count(),
            std::chrono::duration<double>(cartesian_time).count());

        // 复制元数据
        cartesian_frame->slot_counts = origin_frame->slot_counts;
        cartesian_frame->received_slots = origin_frame->received_slots;
        cartesian_frame->ms_timestamp = origin_frame->ms_timestamp;
    }
    else {
        // 方式2: 原始处理流程(不使用修复算法)
        // 步骤1: 原始点云转极坐标
        auto spherical_frame = OriginToSpherical(origin_frame, origin_point_roi_);

        // 步骤2: 极坐标转笛卡尔坐标
        cartesian_frame = SphericalToCartesian(spherical_frame);
    }

    LOG_DEBUG("Converted frame {} with {} points to XYZ format",
        cartesian_frame->frame_number,
        cartesian_frame->data.size());

    // 将结果点云添加到PointcloudParser
    if (pointcloud_parser_) {
        pointcloud_parser_->AddCartesianFrame(cartesian_frame);
    }
}

void OriginPointParseImp::SetRoi(ROISelectType type, PointCloudROIA roi_select_data)
{
    switch (type) {
    case ROISelectType::kROIReset:
        origin_point_roi_.min_slot = 0;
        origin_point_roi_.max_slot = 799;
        origin_point_roi_.min_pixel = 0;
        origin_point_roi_.max_pixel = 96;
        break;
    case ROISelectType::kROISet:
        origin_point_roi_ = roi_select_data;
        break;
    default:
        break;
    }
}

void OriginPointParseImp::UpdateCalibrationParams(PsCalibrationParamsHoldT calibration_params)
{
	calibration_params_ = calibration_params;
    calibration_params_.CalculateTheta();
    LOG_INFO_STYLED(STYLE_GOLD, "Calibration parameters updated in processor thread");
}

bool OriginPointParseImp::LoadStandardCalibrationParams()
{
    auto& config = you_see::ConfigService::Instance();

    // 加载基本参数
    standard_calibration_params_.lines_vertical = config.Get<uint16_t>("parameter.lines_v", 96);
    standard_calibration_params_.lines_horizontal = config.Get<uint16_t>("parameter.lines_h", 800);
    standard_calibration_params_.omega_0.value.f = config.Get<float>("parameter.omega0", -60.0f);
    standard_calibration_params_.theta_min.value.f = config.Get<float>("parameter.theta_min", -10.0f);
    standard_calibration_params_.theta_max.value.f = config.Get<float>("parameter.theta_max", 10.0f);
    standard_calibration_params_.theta_tilt.value.f = config.Get<float>("parameter.theta_tilt", -2.5f);
    standard_calibration_params_.u = config.Get<uint16_t>("parameter.u", 47);
    standard_calibration_params_.sat_peak_max = config.Get<uint16_t>("parameter.sat_peak_max", 1612);
    standard_calibration_params_.sat_peak_min = config.Get<uint16_t>("parameter.sat_peak_min", 0);
    standard_calibration_params_.dist_max = config.Get<uint16_t>("parameter.dist_max", 200);
    standard_calibration_params_.fov_max.value.f = config.Get<float>("parameter.fov_max", 120.0f);
    standard_calibration_params_.k.value.f = config.Get<float>("parameter.k", 1.0f);

    // 加载side_angular_offset数组
    standard_calibration_params_.side_angular_offset.clear();
    standard_calibration_params_.side_angular_offset.reserve(4);
    for (int i = 0; i < 4; i++) {
        std::string key = "side_angular_offset.side" + std::to_string(i) + "_angular_offset";
        MotorValueT value;
        value.value = config.Get<uint16_t>(key, 0.0f);
        standard_calibration_params_.side_angular_offset.push_back(value);
    }
    standard_calibration_params_.side_angular_offset_cnt = standard_calibration_params_.side_angular_offset.size();

    // 加载standard_encoder数组
    standard_calibration_params_.angle_face.clear();
    standard_calibration_params_.angle_face.reserve(4);
    for (int i = 1; i <= 4; i++) {
        std::string key = "standard_encoder.angle_face_" + std::to_string(i);
        uint16_t value = config.Get<uint16_t>(key, 0);
        standard_calibration_params_.angle_face.push_back(value);
    }
    standard_calibration_params_.standard_encoder_cnt = standard_calibration_params_.angle_face.size();

    // 加载vcsel_index数组
    standard_calibration_params_.pixel_in_vcsel_id.clear();
    standard_calibration_params_.pixel_in_vcsel_id.reserve(6);
    for (int i = 0; i < 6; i++) {
        std::string key = "vcsel_index.pixel_in_vcsel_id" + std::to_string(i);
        uint16_t value = config.Get<uint16_t>(key, 0);
        standard_calibration_params_.pixel_in_vcsel_id.push_back(value);
    }
    standard_calibration_params_.pixel_in_vcsel_id_cnt = standard_calibration_params_.pixel_in_vcsel_id.size();

    // 加载light_intensity数组
    standard_calibration_params_.int_ld.clear();
    standard_calibration_params_.int_ld.reserve(6);
    for (int i = 0; i < 6; i++) {
        std::string key = "light_intensity.int_ld" + std::to_string(i);
        UFValueT value;
        value.value.f = config.Get<float>(key, 1.0f);
        standard_calibration_params_.int_ld.push_back(value);
    }
    standard_calibration_params_.int_ld_cnt = standard_calibration_params_.int_ld.size();

    // 加载horizontal_angular_offset数组 - 从0到95按顺序加载
    standard_calibration_params_.omega_delta.clear();
    standard_calibration_params_.omega_delta.reserve(96);
    for (int i = 0; i < 96; i++) {
        std::string key = "horizontal_angular_offset.omega_delta" + std::to_string(i);
        UFValueT value;
        value.value.f = config.Get<float>(key, -1.3f);
        standard_calibration_params_.omega_delta.push_back(value);
    }
    standard_calibration_params_.omega_delta_cnt = standard_calibration_params_.omega_delta.size();

    // 加载zero_distance_offset数组 - 从0到95按顺序加载
    standard_calibration_params_.d_delta.clear();
    standard_calibration_params_.d_delta.reserve(96);
    for (int i = 0; i < 96; i++) {
        std::string key = "zero_distance_offset.d" + std::to_string(i) + "_delta";
        UFValueT value;
        value.value.f = config.Get<float>(key, 0.0f);
        standard_calibration_params_.d_delta.push_back(value);
    }
    standard_calibration_params_.d_delta_cnt = standard_calibration_params_.d_delta.size();

    // 加载distance_factor数组 - 从0到95按顺序加载
    standard_calibration_params_.a.clear();
    standard_calibration_params_.a.reserve(96);
    for (int i = 0; i < 96; i++) {
        std::string key = "distance_factor.a" + std::to_string(i);
        UFValueT value;
        value.value.f = config.Get<float>(key, 1.0f);
        standard_calibration_params_.a.push_back(value);
    }
    standard_calibration_params_.a_cnt = standard_calibration_params_.a.size();

    // 计算theta数组
    standard_calibration_params_.CalculateTheta();

    // 设置其他初始化字段
    standard_calibration_params_.save_frame_index = 0;
    standard_calibration_params_.previous_frame_scale = 0;
    standard_calibration_params_.slot0_omega = 0.0f;
    standard_calibration_params_.inited = true;

    return true;
}

SphericalFramePtr OriginPointParseImp::OriginToSpherical(const OriginFramePtr& origin_frame, const PointCloudROIA& roi)
{
    int slot_length = 32 * ToInt(OriginPointParser::single_cartesian) + 32;
    int slot_s_pixels = ToInt(OriginPointParser::single_cartesian);
    int echos_number = ToInt(OriginPointParser::echos_number);

    // 预估点数以预分配内存
    size_t est_point_count = origin_frame->slot_counts.size() * slot_s_pixels;

    // 创建SphericalFramePtr，使用与原始帧相同的frame_number
    auto spherical_frame = std::make_shared<you_see::CloudFrame<you_see::SphericalPoint>>(
        origin_frame->frame_number, est_point_count);

    // 复制slot_counts信息以保持一致性
    spherical_frame->slot_counts = origin_frame->slot_counts;
    spherical_frame->received_slots = origin_frame->received_slots;

    for (int i = 0; i < origin_frame->data.size() / slot_length; ++i) {
        if (origin_frame->slot_counts[i] < origin_point_roi_.min_slot ||
            origin_frame->slot_counts[i] > origin_point_roi_.max_slot) {
            continue;
        }

        uint8_t* data = origin_frame->data.data() + i * slot_length;
        OriginSlotHead* head = reinterpret_cast<OriginSlotHead*>(data);
		uint32_t face_id = 0;
		// 获取当前slot的旋转镜子角度
        float mirror_angle = calibration_params_.CalculateHorizontalAngle(&face_id, head->RotateMirrorAngle, head->slot_number);
        for (int j = 0; j < slot_s_pixels; ++j) {
            if (j < origin_point_roi_.min_pixel || j > origin_point_roi_.max_pixel) {
                continue;
            }

            OriginPixel* pixel = reinterpret_cast<OriginPixel*>(head->data + j * 32);
            auto [max_time, max_peak] = calibration_params_.FindMaxTimeAndPeak(pixel, echos_number);
            for (int k = 0; k < echos_number; ++k) {

                // 放回波数据
            }
            // 距离计算
            // 创建并添加球坐标点
            you_see::SphericalPoint sp{};
            sp.radius.f = calibration_params_.CalculateDistance(j, max_time);
            sp.theta.f = calibration_params_.theta[j];
            sp.phi.f = calibration_params_.Map2RangeRadian(j, mirror_angle);
            sp.reflectivity = max_peak; // 使用peak作为反射率

            spherical_frame->data.push_back(sp);
            // 放极坐标数据
        }
    }

    // 释放多余容量
    spherical_frame->data.shrink_to_fit();
    return spherical_frame;
}

CloudFramePtr OriginPointParseImp::SphericalToCartesian(const SphericalFramePtr& spherical_frame)
{
    // 创建CartesianFrame，使用与球坐标帧相同的frame_number
    auto cartesian_frame = std::make_shared<you_see::CloudFrame<you_see::CartesianPoint>>(
        spherical_frame->frame_number, spherical_frame->data.size());

    // 复制元数据以保持一致性
    cartesian_frame->slot_counts = spherical_frame->slot_counts;
    cartesian_frame->received_slots = spherical_frame->received_slots;
    cartesian_frame->ms_timestamp = spherical_frame->ms_timestamp;

    for (const auto& sp : spherical_frame->data) {
        you_see::CartesianPoint cp{};

        // 球坐标转笛卡尔坐标
        cp.x.f = sp.radius.f * std::cos(sp.theta.f) * std::cos(sp.phi.f);
        cp.y.f = sp.radius.f * std::cos(sp.theta.f) * std::sin(sp.phi.f);
        cp.z.f = sp.radius.f * std::sin(sp.theta.f);
        cp.reflectivity = sp.reflectivity;

        cartesian_frame->data.push_back(cp);
    }

    return cartesian_frame;
}

algo_lidar::PointMatrix OriginPointParseImp::OriginToPointMatrix(const OriginFramePtr &origin_frame,
                                                                 const PointCloudROIA &roi)
{
    int slot_length = 32 * ToInt(OriginPointParser::single_cartesian) + 32;
    int slot_s_pixels = ToInt(OriginPointParser::single_cartesian);
    int echos_number = ToInt(OriginPointParser::echos_number);

    // 创建矩阵，大小为 pixel数 x slot数
    int max_pixel = slot_s_pixels; // 96
    // int max_slot = 800;           // 800个slot
    int max_slot = 600;           // 600个slot
    algo_lidar::PointMatrix matrix(max_pixel, std::vector<algo_lidar::PointData>(max_slot));

    // 添加统计计数器
    int outer_loop_count = 0;  // 外层循环计数
    int inner_loop_count = 0;  // 内层循环(实际处理的点)计数
    int valid_point_count = 0; // 有效点计数

    for (int i = 0; i < origin_frame->data.size() / slot_length; ++i) {
        int current_slot = origin_frame->slot_counts[i];
        if (current_slot < roi.min_slot || current_slot > roi.max_slot) {
            continue;
        }

        outer_loop_count++;  // 外层循环计数增加

        uint8_t* data = origin_frame->data.data() + i * slot_length;
        OriginSlotHead* head = reinterpret_cast<OriginSlotHead*>(data);
        uint32_t face_id = 0;
        float mirror_angle = standard_calibration_params_.CalculateHorizontalAngle(&face_id,
            head->RotateMirrorAngle,
            head->slot_number);

        for (int j = 0; j < slot_s_pixels; ++j) {
            if (j < roi.min_pixel || j > roi.max_pixel) {
                continue;
            }

            inner_loop_count++;  // 内层循环计数增加

            OriginPixel* pixel = reinterpret_cast<OriginPixel*>(head->data + j * 32);

            // 初始化点数据
            algo_lidar::PointData& point_data = matrix[j][current_slot];
            point_data.pixel_idx = j;
            point_data.slot_idx = current_slot;
            point_data.is_valid = false;

            // 设置角度信息
            point_data.theta_rad = standard_calibration_params_.theta[j];
            point_data.omega_rad = standard_calibration_params_.Map2RangeRadian(j, mirror_angle);
            point_data.theta = point_data.theta_rad * 180.0f / M_PI;
            point_data.slot_ang = point_data.omega_rad * 180.0f / M_PI;

            // 首先找出最强回波
            auto [max_time, max_peak] = standard_calibration_params_.FindMaxTimeAndPeak(pixel, echos_number);

            // 如果有有效回波，设置基本点信息
            if (max_time > 0 && max_peak >= standard_calibration_params_.sat_peak_min) {
                point_data.is_valid = true;
                valid_point_count++;  // 有效点计数增加

                point_data.peak = max_peak;
                point_data.r = standard_calibration_params_.CalculateDistance(j, max_time);
                point_data.reflect = max_peak / 1612.0f;
                point_data.intensity = (max_peak > 255) ? 255 : max_peak;

                // 计算笛卡尔坐标
                point_data.x = point_data.r * std::cos(point_data.theta_rad) * std::cos(point_data.omega_rad);
                point_data.y = point_data.r * std::cos(point_data.theta_rad) * std::sin(point_data.omega_rad);
                point_data.z = point_data.r * std::sin(point_data.theta_rad);
            }

            // 收集所有有效回波数据
            std::vector<algo_lidar::Pulse> pulses;
            for (int k = 0; k < echos_number; ++k) {
                uint32_t perface_time = ((uint32_t)0x00 << 24) |
                    ((uint32_t)pixel[k].perface_time[2] << 16) |
                    ((uint32_t)pixel[k].perface_time[1] << 8) |
                    pixel[k].perface_time[0];

                // 创建回波数据
                algo_lidar::Pulse pulse;
                pulse.tof = perface_time * 0.001;
                pulse.width = pixel[k].pulse_width * 0.001;
                pulse.peak = pixel[k].peak;
                pulses.push_back(pulse);
            }

            // 按峰值从大到小排序回波
            std::sort(pulses.begin(), pulses.end(),
                [](const algo_lidar::Pulse& a, const algo_lidar::Pulse& b) {
                    return a.peak > b.peak;
                });

            point_data.list_pulse = pulses;

            // 如果有回波数据，设置选择索引
            if (!pulses.empty()) {
                point_data.select_idx = 0; // 使用最强回波
            }
        }
    }

    // 打印统计信息
    /*LOG_INFO("Outer loop execution times:{}, Inner loop execution times:{}, Number of valid points:{}",
        outer_loop_count,
        inner_loop_count,
        valid_point_count);*/
    return matrix;
}

CloudFramePtr OriginPointParseImp::PointMatrixToCartesian(const algo_lidar::PointMatrix& matrix, size_t frame_number)
{
    // 估计有效点数量
    size_t valid_count = 0;
    for (const auto& row : matrix) {
        for (const auto& point : row) {
            if (point.is_valid) valid_count++;
        }
    }

    // 创建结果点云
    auto cartesian_frame = std::make_shared<you_see::CloudFrame<you_see::CartesianPoint>>(
        frame_number, valid_count);

    // 填充点云数据 - 矩阵现在是 pixel行 x slot列
    for (const auto& pixel_row : matrix) {
        for (const auto& point : pixel_row) {
            if (point.is_valid) {
                you_see::CartesianPoint cp{};

                // 复制笛卡尔坐标
                cp.x.f = point.x;
                cp.y.f = point.y;
                cp.z.f = point.z;
                cp.reflectivity = point.peak;

                cartesian_frame->data.push_back(cp);
            }
        }
    }

    return cartesian_frame;
}

void OriginPointParseImp::OriginPointProcessor(const OriginFramePtr& origin_frame,
    std::tuple<int, int, std::string, std::string> save_point_frames)
{
    // CS96/Beta格式统一走 CS96OriginPointProcessor 处理
    size_t data_size = origin_frame->data.size();

    if (data_size == CS96GammaRawParser::frame_data_size) {
        CS96OriginPointProcessor(origin_frame, save_point_frames);
        return;
    }
    if (data_size == CS96BetaRawParser::frame_data_size) {
        // CS96 800-slot 格式 (~1.24MB)
        CS96OriginPointProcessor(origin_frame, save_point_frames);
        return;
    }
    if (data_size == CS96Beta600RawParser::frame_data_size) {
        // Beta 600-slot 格式 (~1.86MB, Alpha兼容UDP但peak大端序)
        CS96OriginPointProcessor(origin_frame, save_point_frames);
        return;
    }

    // 以下是 CS96 Alpha 路径
    int side = std::get<1>(save_point_frames);
    if (side != 0) {
        if (origin_frame->frame_number % ToInt(PrismFaces::four_sided_mirror) != side - 1) {
            return;
        }
    }

    int alpha_slot_size = 32 * ToInt(OriginPointParser::single_cartesian) + 32; // 3104
    auto alpha_slot_count = origin_frame->data.size() / alpha_slot_size;

    std::string save_directory = std::get<3>(save_point_frames);
    std::string save_dir{};
    std::string default_directory{ "origin_cloud" };
    if (save_directory != "") {
        default_directory = toolset::JoinPaths(default_directory, save_directory);
    }
    if (!toolset::DirectoryInit(default_directory, save_dir)) {
        return;
    }

    std::string name = std::get<2>(save_point_frames) +
        std::to_string(std::get<0>(save_point_frames)) + ".txt";

    auto file_path = toolset::JoinPaths(save_dir, name);
    std::ofstream file(file_path);
    if (!file.is_open()) {
        LOG_ERROR("Failed to open file for writing.");
        return;
    }

    int pixels_per_slot = ToInt(OriginPointParser::single_cartesian);
    int echo_count = ToInt(OriginPointParser::echos_number);
    int column_width = 20;

    for (int i = 0; i < alpha_slot_count; ++i) {
        if (origin_frame->slot_counts[i] < origin_point_roi_.min_slot ||
            origin_frame->slot_counts[i] > origin_point_roi_.max_slot) {
            continue;
        }
        uint8_t* data = origin_frame->data.data() + i * alpha_slot_size;
        file << "start of slot [" << i << "]" << std::endl;

        OriginSlotHead* head = reinterpret_cast<OriginSlotHead*>(data);
        file << "rotate mirror angle: " << head->RotateMirrorAngle << std::endl;

        for (int j = 0; j < pixels_per_slot; ++j) {
            if (j < origin_point_roi_.min_pixel || j > origin_point_roi_.max_pixel) {
                continue;
            }
            OriginPixel* pixel = reinterpret_cast<OriginPixel*>(head->data + j * 32);
            file << "pixel_number: " << j << " ";
            for (int k = 0; k < echo_count; ++k) {
                uint32_t perface_time = ((uint32_t)0x00 << 24) |
                    ((uint32_t)pixel[k].perface_time[2] << 16) |
                    ((uint32_t)pixel[k].perface_time[1] << 8) |
                    pixel[k].perface_time[0];
                file << std::internal << std::setw(column_width) << perface_time
                    << std::internal << std::setw(column_width) << pixel[k].pulse_width
                    << std::internal << std::setw(column_width) << pixel[k].peak;
            }
            file << std::endl;
        }
        file << "end of slot [" << i << "]" << std::endl << std::endl;
    }
    if (!file) {
        LOG_ERROR("Failed to write data to file.");
        return;
    }

    file.close();
    LOG_DEBUG("CS96 Alpha data saved to file successfully.");
}

void OriginPointParseImp::Mf150OriginPointProcessor(const OriginFramePtr& origin_frame,
    std::tuple<int, int, std::string, std::string> save_point_frames)
{
    std::string save_directory = std::get<3>(save_point_frames);
    size_t frame_data_size = Mf150RawParser::frame_data_size;
    if (origin_frame->data.size() < frame_data_size) {
        return;
    }

    std::string save_dir{};
    std::string default_directory{"origin_cloud"};
    if (save_directory != "") {
        default_directory = toolset::JoinPaths(default_directory, save_directory);
    }
    if (!toolset::DirectoryInit(default_directory, save_dir)) {
        return;
    }

    std::string name = std::get<2>(save_point_frames) +
        std::to_string(std::get<0>(save_point_frames)) + ".txt";
    auto file_path = toolset::JoinPaths(save_dir, name);

    SaveMf150Frame(origin_frame, std::get<0>(save_point_frames), file_path,
        std::get<2>(save_point_frames));
    LOG_DEBUG("MF150 data saved to file successfully.");
}

void OriginPointParseImp::SaveMf150Frame(const OriginFramePtr& origin_frame,
    int frame_index,
    const std::string& file_path,
    const std::string& prefix)
{
    std::ofstream file(file_path);
    if (!file.is_open()) {
        LOG_ERROR("Failed to open file for writing.");
        return;
    }

    int angle_step = 100;
    auto ang_pos = prefix.find("v_ang");
    if (ang_pos != std::string::npos) {
        auto val_str = prefix.substr(ang_pos + 5);
        auto underscore_pos = val_str.find('_');
        if (underscore_pos != std::string::npos) {
            try {
                angle_step = std::stoi(val_str.substr(0, underscore_pos));
            } catch (...) {
                angle_step = 100;
            }
        }
    }

    auto grid = BuildMf150PixelGrid(origin_frame);

    file << "frame_id: " << origin_frame->frame_number << std::endl;
    file << "timestamp: " << origin_frame->ms_timestamp << std::endl;
    file << "cols: " << Mf150RawParser::cols << "  rows: " << Mf150RawParser::rows << std::endl;
    file << "angle_per_col: " << angle_step << std::endl;
    file << std::endl;

    int column_width = 18;

    for (size_t col = 0; col < Mf150RawParser::cols; ++col) {
        file << "start of col [" << col << "], angle: " << (col * angle_step) << std::endl;

        for (size_t row = 0; row < Mf150RawParser::rows; ++row) {
            const auto& cell = grid[row][col];
            file << "row: " << std::setw(4) << row;

            if (cell.valid) {
                for (int echo = 0; echo < static_cast<int>(Mf150RawParser::echoes); ++echo) {
                    file << " " << std::setw(column_width) << cell.tof[echo]
                        << " " << std::setw(column_width) << cell.pulse_width[echo]
                        << " " << std::setw(column_width) << cell.peak[echo];
                }
            } else {
                for (int echo = 0; echo < static_cast<int>(Mf150RawParser::echoes); ++echo) {
                    file << " " << std::setw(column_width) << 0
                        << " " << std::setw(column_width) << 0
                        << " " << std::setw(column_width) << 0;
                }
            }
            file << std::endl;
        }

        file << "end of col [" << col << "]" << std::endl;
        file << std::endl;
    }

    file.close();
}

void OriginPointParseImp::CS96OriginPointProcessor(const OriginFramePtr& origin_frame,
    std::tuple<int, int, std::string, std::string> save_point_frames)
{
    size_t data_size = origin_frame->data.size();

    std::string save_directory = std::get<3>(save_point_frames);
    std::string save_dir{};
    std::string default_directory{"origin_cloud"};
    if (save_directory != "") {
        default_directory = toolset::JoinPaths(default_directory, save_directory);
    }
    if (!toolset::DirectoryInit(default_directory, save_dir)) {
        return;
    }

    std::string name = std::get<2>(save_point_frames) +
        std::to_string(std::get<0>(save_point_frames)) + ".txt";
    auto file_path = toolset::JoinPaths(save_dir, name);

    if (data_size == CS96GammaRawParser::frame_data_size) {
        if (!SaveCS96GammaRawFrameSync(origin_frame->data.data(), data_size,
                origin_frame->frame_number, origin_frame->ms_timestamp, file_path)) {
            LOG_ERROR("Failed to save CS96 Gamma raw frame: {}", file_path);
        }
    }
    else if (data_size == CS96Beta600RawParser::frame_data_size) {
        // Beta 600-slot: 3回声/32B像素/OriginPixel布局/peak大端序/角度位拼接
        SaveCS96Slot600Frame(origin_frame, file_path);
    } else {
        // CS96 800-slot: 2回声/16B像素/无预留位
        SaveCS96Slot800Frame(origin_frame, std::get<0>(save_point_frames), file_path,
            std::get<2>(save_point_frames));
    }
}

void OriginPointParseImp::SaveCS96Slot600Frame(const OriginFramePtr& origin_frame,
    const std::string& file_path)
{
    constexpr auto slot_length = CS96Beta600RawParser::slot_data_size;  // 3104
    constexpr auto pixel_stride = CS96Beta600RawParser::pixel_stride;   // 32
    constexpr auto slot_counts = CS96Beta600RawParser::slot_counts;     // 600
    constexpr auto echoes = CS96Beta600RawParser::echoes;               // 3

    std::ofstream file(file_path);
    if (!file.is_open()) {
        LOG_ERROR("Failed to open file for CS96-600 writing.");
        return;
    }

    int column_width = 20;

    for (size_t i = 0; i < slot_counts; ++i) {
        uint8_t* data = origin_frame->data.data() + i * slot_length;
        file << "start of slot [" << i << "]" << std::endl;

        OriginSlotHead* head = reinterpret_cast<OriginSlotHead*>(data);

        // 角度位拼接 + 编码器计算 (同 saveFrameToFile2 逻辑)
        {
            uint16_t orig_sn  = head->slot_number;
            uint16_t orig_rma = head->RotateMirrorAngle;
            uint8_t scale = orig_rma >> 8;
            uint16_t real_angle = ((orig_rma & 0x00FF) << 4) | ((orig_sn & 0xF000) >> 12);
            float integer_scale = (scale < kBetaEncoderAngleCnt)
                ? kBetaEncoderAngle[scale] : 0.0f;
            float computed_angle = (integer_scale + 0.0009f * real_angle) / 360.0f * 61440.0f;
            file << "rotate mirror angle: " << static_cast<int>(computed_angle) << std::endl;
        }

        for (size_t j = 0; j < CS96Beta600RawParser::pixels_per_slot; ++j) {
            OriginPixel* pixel = reinterpret_cast<OriginPixel*>(head->data + j * pixel_stride);
            file << "pixel_number: " << j << " ";
            for (size_t k = 0; k < echoes; ++k) {
                uint32_t perface_time = ((uint32_t)0x00 << 24) |
                    ((uint32_t)pixel[k].perface_time[2] << 16) |
                    ((uint32_t)pixel[k].perface_time[1] << 8) |
                    pixel[k].perface_time[0];
                // peak大端序 → 字节交换
                uint16_t peak_swapped = ((pixel[k].peak & 0xFF) << 8) | ((pixel[k].peak >> 8) & 0xFF);
                file << std::internal << std::setw(column_width) << perface_time
                    << std::internal << std::setw(column_width) << pixel[k].pulse_width
                    << std::internal << std::setw(column_width) << peak_swapped;
            }
            file << std::endl;
        }
        file << "end of slot [" << i << "]" << std::endl << std::endl;
    }

    file.close();
    LOG_DEBUG("CS96-600 data saved to file successfully.");
}

void OriginPointParseImp::SaveCS96Slot800Frame(const OriginFramePtr& origin_frame,
    int frame_index,
    const std::string& file_path,
    const std::string& prefix)
{
    bool success = SaveCS96RawFrameSync(
        origin_frame->data.data(),
        origin_frame->data.size(),
        origin_frame->frame_number,
        origin_frame->ms_timestamp,
        file_path);

    if (!success) {
        LOG_ERROR("Failed to save CS96-800 frame");
    }
}

std::vector<std::vector<Mf150RawPixel>> OriginPointParseImp::BuildMf150PixelGrid(
    const OriginFramePtr& origin_frame) const
{
    std::vector<std::vector<Mf150RawPixel>> grid;
    ParseMf150RawFrame(origin_frame->data.data(), origin_frame->data.size(), grid);
    return grid;
}

you_see::CartesianPoint OriginPointParseImp::Mf150PixelToXyz(
    uint32_t tof, uint16_t row, uint16_t col) const
{
    constexpr double kLightSpeed = 299792458e-9;
    constexpr int kURow = 75;

    you_see::CartesianPoint cp{};

    float theta = static_cast<float>((row - kURow) / 152.0 * 50.0 * M_PI / 180.0);
    float omega = static_cast<float>((col / 360.0 * 120.0 - 60.0) * M_PI / 180.0);
    float r_m = static_cast<float>(tof / 1000.0 * kLightSpeed / 2.0);
    float cos_theta = cosf(theta);

    cp.x.f = r_m * cos_theta * cosf(omega);
    cp.y.f = r_m * cos_theta * sinf(omega);
    cp.z.f = r_m * sinf(theta);

    return cp;
}

void OriginPointParseImp::Mf150OriginPoint2xyz(const OriginFramePtr& origin_frame)
{
    if (origin_frame->data.size() < Mf150RawParser::frame_data_size) {
        return;
    }

    auto grid = BuildMf150PixelGrid(origin_frame);

    const bool use_multi = mf150_multi_calib_enabled_ && mf150_multi_table_ != nullptr;
    const bool use_single = !use_multi && mf150_calib_enabled_ && !mf150_calib_map_.empty();
    if (use_multi || use_single) {
        constexpr double kLightSpeed = 299792458e-9;
        constexpr int kURow = 75;
        int echo_idx = mf150_preview_echo_index_;

        auto cartesian_frame = std::make_shared<you_see::CloudFrame<you_see::CartesianPoint>>(
            origin_frame->frame_number, Mf150RawParser::rows * Mf150RawParser::cols);
        cartesian_frame->ms_timestamp = origin_frame->ms_timestamp;

        size_t calib_rows = mf150_calib_map_.size();
        float offset_last_valid = 0.0f;

        for (size_t row = 0; row < Mf150RawParser::rows; ++row) {
            for (size_t col = 0; col < Mf150RawParser::cols; ++col) {
                const auto& cell = grid[row][col];
                if (!cell.valid || cell.tof[echo_idx] == 0) {
                    continue;
                }

                uint32_t tof = cell.tof[echo_idx];
                if (tof == Mf150RawParser::kFpgaInvalidPixelValue) {
                    continue;
                }

                float theta = static_cast<float>((static_cast<int>(row) - kURow) / 152.0 * 50.0 * M_PI / 180.0);
                float omega = static_cast<float>((col / 360.0 * 120.0 - 60.0) * M_PI / 180.0);
                float r = static_cast<float>(tof / 1000.0 * kLightSpeed / 2.0);

                if (r <= 0.0f) {
                    continue;
                }

                // 距离标定应用(与算法侧 AlgoCalib_ApplyDistanceCorrection 一致)：
                // 多点模型(每像素距离 LUT 插值)优先，其次单点模型(恒定 base offset)
                if (use_multi) {
                    r += LookupMultiTableOffset(*mf150_multi_table_,
                        static_cast<uint16_t>(row), static_cast<uint16_t>(col), r);
                } else if (use_single) {
                    // 查标定偏移：零值用上一个有效偏移替代
                    if (row < calib_rows && col < mf150_calib_map_[row].size()) {
                        float offset = mf150_calib_map_[row][col];
                        if (offset == 0.0f) {
                            offset = offset_last_valid;
                        } else {
                            offset_last_valid = offset;
                        }
                        r += offset;
                    }
                }

                float cos_theta = cosf(theta);

                you_see::CartesianPoint cp{};
                cp.x.f = r * cos_theta * cosf(omega);
                cp.y.f = r * cos_theta * sinf(omega);
                cp.z.f = r * sinf(theta);
                cp.reflectivity = cell.peak[echo_idx];

                cartesian_frame->data.push_back(cp);
            }
        }

        if (pointcloud_parser_) {
            pointcloud_parser_->AddCartesianFrame(cartesian_frame);
        }

#ifdef YS_HAS_MF150_PICK
        {
            auto cp_to_flat = std::make_shared<std::vector<uint32_t>>();
            cp_to_flat->reserve(cartesian_frame->data.size());

            for (size_t row = 0; row < Mf150RawParser::rows; ++row) {
                for (size_t col = 0; col < Mf150RawParser::cols; ++col) {
                    const auto& cell = grid[row][col];
                    int sel = echo_idx >= 0 && echo_idx < 3 ? echo_idx : 0;
                    if (!cell.valid || cell.tof[sel] == 0 || cell.tof[sel] == Mf150RawParser::kFpgaInvalidPixelValue) {
                        continue;
                    }
                    cp_to_flat->push_back((static_cast<uint32_t>(row) << 16) | static_cast<uint32_t>(col));
                }
            }

            latest_mf150_grid_ = std::make_shared<std::vector<std::vector<Mf150RawPixel>>>(std::move(grid));
            auto cp_copy = std::make_shared<std::vector<you_see::CartesianPoint>>(cartesian_frame->data);
            auto provider = std::make_shared<Mf150OriginPickProvider>(
                latest_mf150_grid_, cp_copy, cp_to_flat, echo_idx);
            emit PickDataReady(provider);
        }
#endif
        return;
    }

    auto cartesian_frame = std::make_shared<you_see::CloudFrame<you_see::CartesianPoint>>(
        origin_frame->frame_number, Mf150RawParser::rows * Mf150RawParser::cols);

    cartesian_frame->ms_timestamp = origin_frame->ms_timestamp;

    auto cp_to_flat = std::make_shared<std::vector<uint32_t>>();
    cp_to_flat->reserve(Mf150RawParser::rows * Mf150RawParser::cols);

    for (size_t row = 0; row < Mf150RawParser::rows; ++row) {
        for (size_t col = 0; col < Mf150RawParser::cols; ++col) {
            const auto& cell = grid[row][col];
            if (!cell.valid || cell.tof[0] == 0) {
                continue;
            }

            auto cp = Mf150PixelToXyz(cell.tof[mf150_preview_echo_index_],
                static_cast<uint16_t>(row), static_cast<uint16_t>(col));

            cartesian_frame->data.push_back(cp);
            cp_to_flat->push_back((static_cast<uint32_t>(row) << 16) | static_cast<uint32_t>(col));
        }
    }

    if (pointcloud_parser_) {
        pointcloud_parser_->AddCartesianFrame(cartesian_frame);
    }

#ifdef YS_HAS_MF150_PICK
    {
            latest_mf150_grid_ = std::make_shared<std::vector<std::vector<Mf150RawPixel>>>(std::move(grid));
        auto cp_copy = std::make_shared<std::vector<you_see::CartesianPoint>>(cartesian_frame->data);
        auto provider = std::make_shared<Mf150OriginPickProvider>(
            latest_mf150_grid_, cp_copy, cp_to_flat, mf150_preview_echo_index_);
        emit PickDataReady(provider);
    }
#endif
}

void OriginPointParseImp::SetMf150PreviewConfig(bool enabled, int echo_index)
{
    mf150_preview_enabled_ = enabled;
    if (echo_index >= 0 && echo_index < 3) {
        mf150_preview_echo_index_ = echo_index;
    }
}

void OriginPointParseImp::SetMf150CalibMap(const std::vector<std::vector<float>>& calib_map)
{
    mf150_calib_map_ = calib_map;
    if (!mf150_calib_map_.empty()) {
        LOG_INFO_STYLED(STYLE_GOLD, "MF150 calibration map loaded: {}×{}",
            mf150_calib_map_.size(),
            mf150_calib_map_[0].size());
    }
}

void OriginPointParseImp::SetMf150CalibEnabled(bool enabled)
{
    mf150_calib_enabled_ = enabled;
}

void OriginPointParseImp::SetMf150MultiCalibTable(std::shared_ptr<SslDistCalibMulti> table)
{
    mf150_multi_table_ = std::move(table);
    if (mf150_multi_table_) {
        LOG_INFO_STYLED(STYLE_GOLD, "MF150 multi calibration LUT loaded");
    } else {
        mf150_multi_table_.reset();
    }
}

void OriginPointParseImp::SetMf150MultiCalibEnabled(bool enabled)
{
    mf150_multi_calib_enabled_ = enabled;
}

algo_lidar::PointMatrix OriginPointParseImp::Mf150PixelGridToPointMatrix(
    const std::vector<std::vector<Mf150RawPixel>>& grid,
    int echo_index) const
{
    constexpr double kLightSpeed = 299792458e-9;
    constexpr int kURow = 75;

    size_t rows = grid.size();
    size_t cols = rows > 0 ? grid[0].size() : 0;
    algo_lidar::PointMatrix matrix(rows, std::vector<algo_lidar::PointData>(cols));

    for (size_t row = 0; row < rows; ++row) {
        for (size_t col = 0; col < cols; ++col) {
            const auto& cell = grid[row][col];
            auto& point = matrix[row][col];

            point.pixel_idx = static_cast<uint16_t>(row);
            point.slot_idx = static_cast<uint16_t>(col);
            point.is_valid = false;

            float theta_deg = static_cast<float>((static_cast<int>(row) - kURow) / 152.0 * 50.0);
            float omega_deg = static_cast<float>(col / 360.0 * 120.0 - 60.0);
            point.theta_rad = theta_deg * static_cast<float>(M_PI) / 180.0f;
            point.omega_rad = omega_deg * static_cast<float>(M_PI) / 180.0f;
            point.theta = theta_deg;
            point.slot_ang = omega_deg;

            if (!cell.valid) {
                continue;
            }

            int sel = echo_index >= 0 && echo_index < 3 ? echo_index : 0;
            if (cell.tof[sel] == 0) {
                continue;
            }

            uint32_t tof = cell.tof[sel];
            uint16_t peak = cell.peak[sel];
            uint32_t pulse_width = cell.pulse_width[sel];

            if (tof == Mf150RawParser::kFpgaInvalidPixelValue) {
                continue;
            }

            point.r = static_cast<float>(tof / 1000.0 * kLightSpeed / 2.0);
            point.peak = peak;

            float intensity = std::min(255.0f,
                static_cast<float>(static_cast<double>(peak) * pulse_width / 702.0 / 5.0 * 256.0));
            point.intensity = static_cast<uint8_t>(intensity);
            point.reflect = intensity / 255.0f;

            point.x = point.r * std::cos(point.theta_rad) * std::cos(point.omega_rad);
            point.y = point.r * std::cos(point.theta_rad) * std::sin(point.omega_rad);
            point.z = point.r * std::sin(point.theta_rad);

            std::vector<algo_lidar::Pulse> pulses;
            for (int e = 0; e < 3; ++e) {
                if (cell.tof[e] != 0 && cell.tof[e] != Mf150RawParser::kFpgaInvalidPixelValue) {
                    pulses.emplace_back(
                        static_cast<float>(cell.tof[e]),
                        static_cast<float>(cell.pulse_width[e]),
                        static_cast<float>(cell.peak[e]));
                }
            }
            std::sort(pulses.begin(), pulses.end(),
                [](const algo_lidar::Pulse& a, const algo_lidar::Pulse& b) {
                    return a.peak > b.peak;
                });
            point.list_pulse = pulses;
            if (!pulses.empty()) {
                point.select_idx = 0;
            }

            point.is_valid = true;
        }
    }

    return matrix;
}

void OriginPointParseImp::UpdateCs96BetaCalibParams(std::shared_ptr<Cs96BetaCalibParams> params)
{
    cs96_beta_calib_params_ = std::make_shared<Cs96BetaCalibParams>(*params);
    cs96_beta_preview_enabled_ = (params != nullptr && params->IsInited());
    LOG_INFO("CS96 Beta calibration params updated (cloned), preview_enabled={}", cs96_beta_preview_enabled_);
}

void OriginPointParseImp::Cs96BetaOriginPoint2xyz(const OriginFramePtr& origin_frame)
{
    if (!cs96_beta_calib_params_ || !cs96_beta_calib_params_->IsInited()) {
        return;
    }

    size_t data_size = origin_frame->data.size();
    auto& params = *cs96_beta_calib_params_;
    size_t slot_length = 0;
    size_t pixel_stride = 0;
    size_t slot_counts = 0;
    int echoes = 2;
    bool use_alpha_slot_head = false;

    if (data_size == CS96BetaRawParser::frame_data_size) {
        slot_length = CS96BetaRawParser::slot_data_size;
        pixel_stride = CS96BetaRawParser::pixel_byte_size;
        slot_counts = CS96BetaRawParser::slot_counts;
        echoes = static_cast<int>(CS96BetaRawParser::echoes);
    } else if (data_size == CS96Beta600RawParser::frame_data_size) {
        slot_length = CS96Beta600RawParser::slot_data_size;
        pixel_stride = CS96Beta600RawParser::pixel_stride;
        slot_counts = CS96Beta600RawParser::slot_counts;
        echoes = static_cast<int>(CS96Beta600RawParser::echoes);
        use_alpha_slot_head = true;
    } else {
        return;
    }

    size_t est_point_count = slot_counts * Cs96BetaCalibParams::kLinesVertical;
    auto cartesian_frame = std::make_shared<you_see::CloudFrame<you_see::CartesianPoint>>(
        origin_frame->frame_number, est_point_count);
    cartesian_frame->slot_counts = origin_frame->slot_counts;
    cartesian_frame->received_slots = origin_frame->received_slots;
    cartesian_frame->ms_timestamp = origin_frame->ms_timestamp;

    for (size_t i = 0; i < slot_counts; ++i) {
        const uint8_t* slot_data = origin_frame->data.data() + i * slot_length;

        uint32_t mirror_angle = 0;
        uint16_t slot_number = 0;
        int frame_id = 0;
        const uint8_t* pixel_base = nullptr;

        if (use_alpha_slot_head) {
            // 600-slot (Alpha 兼容 32B 头): 无 fpga_frame_id，按 scale 区间判断所在面
            const OriginSlotHead* head = reinterpret_cast<const OriginSlotHead*>(slot_data);
            slot_number = head->slot_number;

            uint16_t orig_sn = head->slot_number;
            uint16_t orig_rma = head->RotateMirrorAngle;
            mirror_angle = (static_cast<uint32_t>(orig_rma) << 4)
                | ((orig_sn & 0xF000) >> 12);
            frame_id = Cs96BetaCalibParams::GetFaceIdByScale(mirror_angle);

            pixel_base = head->data;
        } else {
            // 800-slot (16B 头): slot_number 在 byte0-1，fpga_frame_id 在 byte14
            uint32_t w0 = static_cast<uint32_t>(slot_data[0])
                | (static_cast<uint32_t>(slot_data[1]) << 8)
                | (static_cast<uint32_t>(slot_data[2]) << 16)
                | (static_cast<uint32_t>(slot_data[3]) << 24);
            mirror_angle = (w0 >> 12) & 0xFFFFF;

            slot_number = (static_cast<uint16_t>(slot_data[0])
                | (static_cast<uint16_t>(slot_data[1]) << 8)) & 0x0FFF;
            frame_id = static_cast<int>(slot_data[14]) % 4;

            pixel_base = slot_data + CS96BetaRawParser::slot_header_size;
        }

        float horizontal_corner = params.CalculateHorizontalAngle(frame_id, mirror_angle, slot_number);
        if (std::isnan(horizontal_corner)) {
            // scale 越界，角度无法计算，跳过该 slot
            continue;
        }

        // 低频诊断日志，与下位机 [Diag] 日志逐值比对
        if (slot_number != 0) {
            static int diag_cnt = 0;
            if (++diag_cnt % 800 == 0) {
                int scale = static_cast<int>((mirror_angle >> 12) & 0xFF);
                int real_angle = static_cast<int>(mirror_angle & 0xFFF);
                float angle = params.encoder_angle_table_[scale]
                    + 0.0009f * static_cast<float>(real_angle);
                float start = (frame_id >= 0
                    && frame_id < static_cast<int>(params.starting_angle_.size()))
                    ? params.starting_angle_[frame_id] : 0.0f;
                LOG_DEBUG("[Diag] frame={} start={:.2f} angle={:.2f} hc={:.2f} "
                    "mirror={:#08x} slot={}",
                    frame_id, start, angle, horizontal_corner, mirror_angle, slot_number);
            }
        }

        for (size_t j = 0; j < static_cast<size_t>(Cs96BetaCalibParams::kLinesVertical); ++j) {
            int pixel_idx = static_cast<int>(Cs96BetaCalibParams::kLinesVertical - 1 - j);

            float theta = params.theta_[pixel_idx];
            float omega = params.Map2RangeRadian(pixel_idx, horizontal_corner);

            uint32_t max_time = 0;
            uint16_t max_peak = 0;

            if (!use_alpha_slot_head) {
                const uint8_t* pixel_data = pixel_base + j * pixel_stride;

                for (int e = 0; e < echoes; ++e) {
                    size_t echo_offset = e * (pixel_stride / echoes);
                    uint32_t ft = static_cast<uint32_t>(pixel_data[echo_offset])
                        | (static_cast<uint32_t>(pixel_data[echo_offset + 1]) << 8)
                        | (static_cast<uint32_t>(pixel_data[echo_offset + 2]) << 16);
                    uint16_t peak = static_cast<uint16_t>(pixel_data[echo_offset + 6])
                        | (static_cast<uint16_t>(pixel_data[echo_offset + 7]) << 8);

                    if (ft == 0 || ft == CS96BetaRawParser::kFpgaInvalidPixelValue) {
                        continue;
                    }
                    if (peak > max_peak) {
                        max_peak = peak;
                        max_time = ft;
                    }
                }
            } else {
                const OriginPixel* pixel = reinterpret_cast<const OriginPixel*>(
                    pixel_base + j * pixel_stride);
                for (int e = 0; e < echoes; ++e) {
                    uint32_t ft = ((uint32_t)pixel[e].perface_time[2] << 16)
                        | ((uint32_t)pixel[e].perface_time[1] << 8)
                        | pixel[e].perface_time[0];
                    uint16_t peak = ((pixel[e].peak & 0xFF) << 8) | ((pixel[e].peak >> 8) & 0xFF);
                    if (ft == 0 || ft == CS96BetaRawParser::kFpgaInvalidPixelValue) {
                        continue;
                    }
                    if (peak > max_peak) {
                        max_peak = peak;
                        max_time = ft;
                    }
                }
            }

            if (max_time == 0) {
                continue;
            }

            float r = params.CalculateDistance(pixel_idx, max_time);

            you_see::CartesianPoint cp{};
            cp.x.f = r * std::cos(theta) * std::cos(omega);
            cp.y.f = r * std::cos(theta) * std::sin(omega) * -1.0f;
            cp.z.f = r * std::sin(theta);
            cp.reflectivity = max_peak;

            cartesian_frame->data.push_back(cp);
        }
    }

    cartesian_frame->data.shrink_to_fit();
    LOG_DEBUG("CS96 Beta: Converted frame {} with {} points",
        cartesian_frame->frame_number, cartesian_frame->data.size());

    if (pointcloud_parser_) {
        pointcloud_parser_->AddCartesianFrame(cartesian_frame);
    }
}

OriginPointTask::OriginPointTask(const OriginFramePtr& frame,
    OriginPointParseImp* processor,
    std::tuple<int, int, std::string, std::string> save_point_frames,
    OriginPointOperation operation_type,
    bool is_mf150,
    bool is_cs96_beta):
    frame_(frame),
    processor_(processor),
    save_origin_point_frames_(save_point_frames),
    operation_type_(operation_type),
    is_mf150_(is_mf150),
    is_cs96_beta_(is_cs96_beta)
{
}

void OriginPointTask::run()
{
    if (is_mf150_) {
        switch (operation_type_) {
        case OriginPointOperation::KOriginPointSave:
            emit processor_->Mf150OriginPointProcessor(frame_, save_origin_point_frames_);
            break;
        case OriginPointOperation::KOriginPointTrans:
            emit processor_->Mf150OriginPoint2xyz(frame_);
            break;
        case OriginPointOperation::KOriginPointBothSaveTrans:
            emit processor_->Mf150OriginPointProcessor(frame_, save_origin_point_frames_);
            emit processor_->Mf150OriginPoint2xyz(frame_);
            break;
        default:
            break;
        }
        return;
    }

    if (is_cs96_beta_) {
        switch (operation_type_) {
        case OriginPointOperation::KOriginPointSave:
            emit processor_->OriginPointProcessor(frame_, save_origin_point_frames_);
            break;
        case OriginPointOperation::KOriginPointTrans:
            emit processor_->Cs96BetaOriginPoint2xyz(frame_);
            break;
        case OriginPointOperation::KOriginPointBothSaveTrans:
            emit processor_->OriginPointProcessor(frame_, save_origin_point_frames_);
            emit processor_->Cs96BetaOriginPoint2xyz(frame_);
            break;
        default:
            break;
        }
        return;
    }

    switch (operation_type_) {
    case OriginPointOperation::KOriginPointSave:
        emit processor_->OriginPointProcessor(frame_, save_origin_point_frames_);
        break;
	case OriginPointOperation::KOriginPointTrans:
        emit processor_->OriginPoint2xyz(frame_);
        break;
	case OriginPointOperation::KOriginPointBothSaveTrans:
		emit processor_->OriginPointProcessor(frame_, save_origin_point_frames_);
		emit processor_->OriginPoint2xyz(frame_);
		break;
    }
}

}// namespace ys_scene

