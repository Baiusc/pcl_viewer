#include "pointcloud_parser_imp.h"
#include "log/logging.h"
#include "tiny_hut/common_toolset.h" 
#include "config/config_service.h"
#include "component/ys_scene_const.h"
// #include "lidaralg.h"
#include <iostream>
#include <fstream>
#include <QThread>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <span>
#include <utility>

namespace ys_scene {

namespace
{

[[nodiscard]] PointcloudRenderFramePtr MakeRenderFrame(
    CloudPureFramePtr points,
    std::vector<Rgb> colors,
    int frame_id,
    std::uint64_t sequence)
{
    if (!points || colors.size() != points->size() || frame_id < 0) {
        LOG_ERROR("Invalid pointcloud render frame: frame={}", frame_id);
        return {};
    }

    auto frame = std::make_shared<PointcloudRenderFrame>();
    frame->points = std::move(points);
    frame->colors = std::move(colors);
    frame->frame_id = frame_id;
    frame->sequence = sequence;

    const std::span<const you_see::CartesianPoint> view{
        *frame->points
    };

    bool has_bounds = false;

    for (const auto &point : view) {
        const QVector3D position{
            point.x.f,
            point.y.f,
            point.z.f,
        };

        if (!std::isfinite(position.x())
            || !std::isfinite(position.y())
            || !std::isfinite(position.z()))
        {
            LOG_ERROR("Non-finite point in render frame: frame={}", frame_id);
            return {};
        }

        if (!has_bounds) {
            frame->bounds_min = position;
            frame->bounds_max = position;
            has_bounds = true;
            continue;
        }

        frame->bounds_min.setX(std::min(frame->bounds_min.x(), position.x()));
        frame->bounds_min.setY(std::min(frame->bounds_min.y(), position.y()));
        frame->bounds_min.setZ(std::min(frame->bounds_min.z(), position.z()));

        frame->bounds_max.setX(std::max(frame->bounds_max.x(), position.x()));
        frame->bounds_max.setY(std::max(frame->bounds_max.y(), position.y()));
        frame->bounds_max.setZ(std::max(frame->bounds_max.z(), position.z()));
    }

    if (has_bounds) {
        frame->bounds_diagonal = static_cast<double>((frame->bounds_max - frame->bounds_min).length());
    }

    return frame;
}

} // namespace

PointcloudParserImp::PointcloudParserImp(
    std::span<const Rgb, max_steps_> colors, QObject *parent)
: QObject(parent),
  pointbuild_(std::make_unique<PointcloudBuild>(colors))
{
}

PointcloudParserImp::~PointcloudParserImp()
{
}

PointcloudBuild PointcloudParserImp::CaptureBuild() const
{
    const std::scoped_lock lock(settings_mutex_);
    return *pointbuild_;
}

void PointcloudParserImp::SetColorScale(std::span<const Rgb, max_steps_> colors)
{
    const std::scoped_lock lock(settings_mutex_);
    pointbuild_->SetColorScale(colors);
}

void PointcloudParserImp::SetDistanThrehold(std::tuple<double, double> threhold)
{
    const std::scoped_lock lock(settings_mutex_);
    pointbuild_->SetDistanceThrehold(threhold);
}

void PointcloudParserImp::SetAlgoItems(AlgorithmItemEnable items)
{
    const std::scoped_lock lock(settings_mutex_);
    algo_items_ = items;
}

void PointcloudParserImp::SetMappingMethod(MappingMethod mm)
{
    const std::scoped_lock lock(settings_mutex_);
    pointbuild_->SetMappingMethod(mm);
}

void PointcloudParserImp::SetDistance(int value, PointDistance distance_type)
{
    const std::scoped_lock lock(settings_mutex_);
    pointbuild_->SetDistance(value, distance_type);
}

void PointcloudParserImp::SetDisColor(int value, DistanceColor color_type)
{
    const std::scoped_lock lock(settings_mutex_);
    pointbuild_->SetDisColor(value, color_type);
}

void PointcloudParserImp::SetROI(ROISelectType type, PointCloudROIA roi_select_data)
{
    const std::scoped_lock lock(settings_mutex_);
    pointbuild_->SetROI(type, roi_select_data);
}

void PointcloudParserImp::SetLinesShowMode(int mode)
{
    const std::scoped_lock lock(settings_mutex_);
    pointbuild_->SetLinesShowMode(mode);
}

void PointcloudParserImp::SetAllSidesDisplay(bool enable)
{
    const std::scoped_lock lock(settings_mutex_);
    pointbuild_->SetAllSidesDisplay(enable);
}

void PointcloudParserImp::PointcloudSave(CloudPureFramePtr& points,
    std::string save_directory,
    int frame_id)
{
    std::string save_dir{};
    std::string default_directory{ "point_cloud" };
    if (save_directory != "") {
        default_directory = toolset::JoinPaths("point_cloud", save_directory);
    }
    if (!toolset::DirectoryInit(default_directory, save_dir)) {
        return;
    }

    std::string name = "point_cloud_" +
        toolset::GetCurrentTimestampAsString() + "_" + std::to_string(frame_id) + "_" + ".pcd";
    auto file_path = toolset::JoinPaths(save_dir, name);
    std::ofstream ofs(file_path, std::ios::binary);
    if (!ofs.is_open()) {
        LOG_ERROR("Failed to open file for writing.");
        return;
    }

    // ! write pcd header
    ofs << "# .PCD v0.7 - Point Cloud Data file format" << std::endl;
    ofs << "VERSION 0.7" << std::endl;
    ofs << "FIELDS x y z intensity" << std::endl;
    ofs << "SIZE 4 4 4 4" << std::endl;
    ofs << "TYPE F F F F" << std::endl;
    ofs << "COUNT 1 1 1 1" << std::endl;
    ofs << "WIDTH " << points->size() << std::endl;
    ofs << "HEIGHT 1" << std::endl;
    ofs << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
    ofs << "POINTS " << points->size() << std::endl;
    ofs << "DATA ascii" << std::endl;

    // ! write point data
    /*for (int i = 0; i < 800; ++i) {
        for (int j = 0; j < 96; ++j) {
            ofs << data->at(j).x.f << " " <<  data->at(j).y.f << " " << data->at(j).z.f << std::endl;
        }
    }*/
    for (const auto& point : *points) {
        ofs << point.x.f << " "
            << point.y.f << " "
            << point.z.f << " "
            << static_cast<int>(point.reflectivity) << std::endl;
    }

    ofs.close();
}

void PointcloudParserImp::PointcloudProcessor(
    CloudPureFramePtr data,
    std::shared_ptr<std::vector<int>> slot_counts,
    std::tuple<int, std::string> save_point_frames,
    int frame_index,
    std::uint64_t sequence)
{
    auto build = CaptureBuild();
    auto frame = build.PointcloudGen(data, std::move(slot_counts), frame_index, sequence);

    if (!frame) {
        return;
    }

    emit RenderFrameReady(frame);

    if (std::get<0>(save_point_frames) > 0) {
        PointcloudSave(data, std::get<1>(save_point_frames), frame_index);
    }
}

void PointcloudParserImp::PointcloudProcessorFlat(
    CloudPureFramePtr data,
    std::tuple<int, std::string> save_point_frames,
    int frame_index,
    std::uint64_t sequence)
{
    auto build = CaptureBuild();
    auto frame = build.PointcloudGenFlat(data, frame_index, sequence);

    if (!frame) {
        return;
    }

    emit RenderFrameReady(frame);

    if (std::get<0>(save_point_frames) > 0) {
        PointcloudSave(data, std::get<1>(save_point_frames), frame_index);
    }
}

PointcloudTask::PointcloudTask(CloudPureFramePtr data,
        std::shared_ptr<std::vector<int>> slot_counts,
        PointcloudParserImp* processor,
        std::tuple<int, std::string> save_point_frames_,
        int frame_index) :
        data_(data),
        slot_counts_(slot_counts),
        processor_(processor),
        save_point_frames_(save_point_frames_),
        frame_index_(frame_index)
    {

    }

    PointcloudTask::~PointcloudTask()
    {
        //LOG_INFO("PointcloudTask::~PointcloudTask()");
    }

void PointcloudTask::run()
{
    processor_->PointcloudProcessor(data_, slot_counts_, save_point_frames_, frame_index_, sequence_);
}

PointcloudTaskFlat::PointcloudTaskFlat(CloudPureFramePtr data,
    PointcloudParserImp* processor,
    std::tuple<int, std::string> save_point_frames_,
    int frame_index) :
    data_(data),
    processor_(processor),
    save_point_frames_(save_point_frames_),
    frame_index_(frame_index)
{
}

PointcloudTaskFlat::~PointcloudTaskFlat()
{
}

void PointcloudTaskFlat::run()
{
    processor_->PointcloudProcessorFlat(data_, save_point_frames_, frame_index_, sequence_);
}

PointcloudBuild::PointcloudBuild(std::span<const Rgb, max_steps_> colors) :
        min_show_{0.0},
        max_show_{280.0},
        mapping_method_{MappingMethod::kXAxisMap},
        LinesShowMode_{you_see::PointcloudRecordType::total_lines},
        all_sides_display_{false},
        prism_faces_{PrismFaces::four_sided_mirror}
{
    SetColorScale(colors);
    auto& config = you_see::ConfigService::Instance();

    // 初始化ROI值
    point_roi_.min_slot = config.Get<int>("cloud_roi.min_slot", 0);
    point_roi_.max_slot = config.Get<int>("cloud_roi.max_slot", 799);
    point_roi_.min_pixel = config.Get<int>("cloud_roi.min_pixel", 0);
    // 默认取单 SLOT 最大像素索引：alpha/beta(96 像素)不变，CS96-gamma(120 像素)才不会被截掉 96~119
    point_roi_.max_pixel = config.Get<int>("cloud_roi.max_pixel", kPointRoiDefaultMaxPixel);

    // 映射距离值
    distance_pos_.end_pos = config.Get<int>("distance.end", 240);
    distance_pos_.far_pos = config.Get<int>("distance.far", 100);
    distance_pos_.close_pos = config.Get<int>("distance.close", 20);
    distance_pos_.start_pos = config.Get<int>("distance.start", 0);

    // 映射颜色值
    distance_color_.far_end_color = config.Get<int>("distance_color.far_end", 100);
    distance_color_.far_start_color = config.Get<int>("distance_color.far_begin", 50);
    distance_color_.middle_end_color = config.Get<int>("distance_color.middle_end", 50);
    distance_color_.middle_start_color = config.Get<int>("distance_color.middle_begin", 0);
    distance_color_.close_end_color = config.Get<int>("distance_color.close_end", 255);
    distance_color_.close_start_color = config.Get<int>("distance_color.close_begin", 50);
    int method_type = you_see::ConfigService::Instance().Get<int>("map_method", 0);
    auto mm = static_cast<MappingMethod>(method_type);
    switch (mm) {
    case MappingMethod::kXAxisMap:
        mapping_method_ = MappingMethod::kXAxisMap;
        break;
    case MappingMethod::kYAxisMap:
        mapping_method_ = MappingMethod::kYAxisMap;
        break;
    case MappingMethod::kZAxisMap:
        mapping_method_ = MappingMethod::kZAxisMap;
        break;
    case MappingMethod::kReflectivetyMap:
        mapping_method_ = MappingMethod::kReflectivetyMap;
        break;
    };
}

PointcloudRenderFramePtr PointcloudBuild::PointcloudGen(CloudPureFramePtr &points,
                                                        std::shared_ptr<std::vector<int>> slot_counts,
                                                        const int &frame_index, std::uint64_t sequence)
{
    if (!points || !slot_counts || slot_counts->empty() || points->size() % slot_counts->size() != 0) {
        LOG_ERROR("Invalid structured pointcloud: frame={}", frame_index);
        return {};
    }
    std::vector<Rgb> reflective_rgb;
    reflective_rgb.reserve(points->size());
    const size_t points_pixels = points->size() / slot_counts->size();
    float min_radius_sq = std::pow((float)min_show_, 2);
    float max_radius_sq = std::pow((float)max_show_, 2);
    size_t total_slots = slot_counts->size();

    const size_t input_points = points->size();
    size_t roi_rejected = 0;
    size_t invalid_points = 0;
    size_t distance_rejected = 0;
    size_t zero_points = 0;
    //LOG_INFO("Mapping Method: {0} (0=X-axis, 1=Y-axis, 2=Z-axis, 3=Reflectivity)", static_cast<int>(mapping_method_));
    //LOG_INFO("Distance configuration - start:{0}, close:{1}, far:{2}, end:{3}",
    //    distance_pos_.start_pos, distance_pos_.close_pos, distance_pos_.far_pos, distance_pos_.end_pos);
    //LOG_INFO("Color configuration - close_start:{0}, close_end:{1}, middle_start:{2}, middle_end:{3}, far_start:{4}, far_end:{5}",
    //         distance_color_.close_start_color, distance_color_.close_end_color,
    //         distance_color_.middle_start_color, distance_color_.middle_end_color,
    //         distance_color_.far_start_color, distance_color_.far_end_color);

    //// 统计X轴范围和颜色分布
    //float min_x = std::numeric_limits<float>::max();
    //float max_x = std::numeric_limits<float>::lowest();
    //std::map<int, int> color_distribution; // 颜色值 -> 点数量

    // 创建一个新的vector来存储过滤后的点
    auto filtered_points = std::make_shared<std::vector<CartesianPoint>>();
    filtered_points->reserve(points->size()); // 预分配空间以提高性能

    for (int i = 0; i < total_slots; i++) {
        if ((*slot_counts)[i] < point_roi_.min_slot || (*slot_counts)[i] > point_roi_.max_slot) {
            roi_rejected += points_pixels;
            continue;
        }
        // 计算当前行的起始索引
        size_t slot_start = i * points_pixels;

        // 遍历当前行的每个点
        for (size_t j = 0; j < points_pixels; j++) {
            if (j < point_roi_.min_pixel || j > point_roi_.max_pixel) {
                ++roi_rejected;
                continue;
            }

            //// 根据LinesShowMode_判断是否处理当前点
            //switch (LinesShowMode_) {
            //case PointcloudRecordType::odd_number_lines:
            //    if (!(j & 1)) { // j为偶数则跳过
            //        continue;
            //    }
            //    break;
            //case PointcloudRecordType::even_lines:
            //    if (j & 1) { // j为奇数则跳过
            //        continue;
            //    }
            //    break;
            //case PointcloudRecordType::total_lines:
            //    // 处理所有点，不需要特殊处理
            //    break;
            //}

            float color = 0;
            size_t point_idx = slot_start + j;
            const CartesianPoint& point = (*points)[point_idx];
            if (!std::isfinite(point.x.f) || !std::isfinite(point.y.f) || !std::isfinite(point.z.f)) {
                ++invalid_points;
                continue;
            }
            if (point.x.f == 0 && point.y.f == 0 && point.z.f == 0) {
                ++zero_points;
            }
            // 计算点到原点的平方距离
            float dist_sq = point.x.f * point.x.f + point.y.f * point.y.f;

            // 如果点在有效范围内，则保留
            if (dist_sq >= min_radius_sq && dist_sq <= max_radius_sq) {
                filtered_points->push_back(point);

                //// 统计X轴范围（仅在X轴映射时）
                //if (mapping_method_ == MappingMethod::kXAxisMap) {
                //    min_x = std::min(min_x, point.x.f);
                //    max_x = std::max(max_x, point.x.f);
                //}

                // 判断是否是多面同显
                if (all_sides_display_) {
                // 同显每一帧点云上纯色, 不可使用color_mapper
                    DrawPrismFacesSingleColor(color, frame_index);
                }
                else {
                // 非同显可按照，上色规则来上色, 可使用color_mapper
                    switch (mapping_method_) {
                    case MappingMethod::kXAxisMap:
                        color = MapDistance2Color(const_cast<float&>(point.x.f));
                        break;
                    case MappingMethod::kYAxisMap:
                        color = MapDistance2Color(const_cast<float&>(point.y.f));
                        break;
                    case MappingMethod::kZAxisMap:
                        color = MapDistance2Color(const_cast<float&>(point.z.f));
                        break;
                    case MappingMethod::kReflectivetyMap:
                        reflective_rgb.push_back(GetColor(Normalize((int)((255 - point.reflectivity))), 256));
                        break;
                    default:
                        reflective_rgb.push_back(GetColor(Normalize((int)((255 - point.reflectivity))), 256));
                        break;
                    }
                }

                //reflective_rgb.push_back(GetColor(Normalize((int)((255 - point.reflectivity))), 256));
                if (all_sides_display_ || mapping_method_ != MappingMethod::kReflectivetyMap) {
                    reflective_rgb.push_back(GetColor(Normalize((int)color), 256));
                    //// 统计颜色分布
                    //if (mapping_method_ == MappingMethod::kXAxisMap) {
                    //    int color_int = static_cast<int>(color);
                    //    color_distribution[color_int]++;
                    //}
                }
            }
            else {
                ++distance_rejected;
            }
        }
    
    }
    // 用过滤后的点替换原始points
    points = filtered_points;

    //// 输出X轴映射的统计信息
    //if (mapping_method_ == MappingMethod::kXAxisMap) {
    //    LOG_INFO("X-axis range: min={}, max={}, range={}", min_x, max_x, max_x - min_x);
    //    LOG_INFO("Color distribution statistics (color value: number of points):");
    //    for (const auto& pair : color_distribution) {
    //        LOG_INFO("  Color {0}: {1} points", pair.first, pair.second);
    //    }

    //    // 检查是否所有点都映射到同一颜色
    //    if (color_distribution.size() == 1) {
    //        LOG_WARN("Warning: All points are mapped to the same color value {0}!", color_distribution.begin()->first);
    //        LOG_WARN("Possible reasons: 1) The X coordinate range exceeds the distance configuration range 2) The distance configuration is unreasonable");
    //    }
    //}

    auto frame = MakeRenderFrame(points, std::move(reflective_rgb), frame_index, sequence);
    // Throttle diagnostics across task-local builders without sharing rendering state.
    static std::atomic_size_t diagnostic_frame_count{0};
    if (diagnostic_frame_count.fetch_add(1, std::memory_order_relaxed) % 100 == 0) {
        LOG_INFO("PointcloudGen: frame={} input={} output={} slots={} pixels_per_slot={} "
                 "roi_rejected={} invalid={} zero={} distance_rejected={} roi=[{}..{}, {}..{}] distance=[{}, {}]",
            frame_index, input_points, points->size(), total_slots, points_pixels,
            roi_rejected, invalid_points, zero_points, distance_rejected,
            point_roi_.min_slot, point_roi_.max_slot, point_roi_.min_pixel, point_roi_.max_pixel, min_show_, max_show_);
        if (frame && !points->empty()) {
            LOG_INFO("PointcloudGen bounds: frame={} min=({}, {}, {}) max=({}, {}, {})",
                frame_index, frame->bounds_min.x(), frame->bounds_min.y(), frame->bounds_min.z(),
                frame->bounds_max.x(), frame->bounds_max.y(), frame->bounds_max.z());
        }
    }
    return frame;
}

PointcloudRenderFramePtr PointcloudBuild::PointcloudGenFlat(CloudPureFramePtr &points, const int &frame_index, std::uint64_t sequence)
{
    if (!points) {
        return {};
    }

    std::vector<Rgb> reflective_rgb;
    reflective_rgb.reserve(points->size());
    float min_radius_sq = std::pow((float)min_show_, 2);
    float max_radius_sq = std::pow((float)max_show_, 2);

    LOG_INFO("=== Frame {} Point cloud coloring debugging information ===", frame_index);
    LOG_INFO("Total points: {}", points->size());

    auto filtered_points = std::make_shared<std::vector<CartesianPoint>>();
    filtered_points->reserve(points->size());

    for (size_t i = 0; i < points->size(); i++) {
        const CartesianPoint& point = (*points)[i];

        float dist_sq = point.x.f * point.x.f + point.y.f * point.y.f;

        if (dist_sq >= min_radius_sq && dist_sq <= max_radius_sq) {
            filtered_points->push_back(point);

            float color = 0;

            if (all_sides_display_) {
                DrawPrismFacesSingleColor(color, frame_index);
            }
            else {
                switch (mapping_method_) {
                case MappingMethod::kXAxisMap:
                    color = MapDistance2Color(const_cast<float&>(point.x.f));
                    break;
                case MappingMethod::kYAxisMap:
                    color = MapDistance2Color(const_cast<float&>(point.y.f));
                    break;
                case MappingMethod::kZAxisMap:
                    color = MapDistance2Color(const_cast<float&>(point.z.f));
                    break;
                case MappingMethod::kReflectivetyMap:
                    reflective_rgb.push_back(GetColor(Normalize((int)((255 - point.reflectivity))), 256));
                    break;
                default:
                    reflective_rgb.push_back(GetColor(Normalize((int)((255 - point.reflectivity))), 256));
                    break;
                }
            }

            if (all_sides_display_ || mapping_method_ != MappingMethod::kReflectivetyMap) {
                reflective_rgb.push_back(GetColor(Normalize((int)color), 256));
            }
        }
    }

    points = filtered_points;

    return MakeRenderFrame(points, std::move(reflective_rgb), frame_index, sequence);
}

void PointcloudBuild::SetColorScale(std::span<const Rgb, max_steps_> colors)
{
    std::ranges::copy(colors, color_scale_element_.begin());
}

void PointcloudBuild::SetDistanceThrehold(std::tuple<double, double> threhold)
{
    if (std::get<0>(threhold) < std::get<1>(threhold)) {
        min_show_ = std::get<0>(threhold);
        max_show_ = std::get<1>(threhold);
    }
}

void PointcloudBuild::SetMappingMethod(MappingMethod &mm)
{
    mapping_method_ = mm;
}

void PointcloudBuild::SetDistance(int& value, PointDistance& distance_type)
{
    switch(distance_type) {
    case PointDistance::kStartPos:
        distance_pos_.start_pos = value;
        break;
    case PointDistance::kClosePos:
        distance_pos_.close_pos = value;
        break;
    case PointDistance::kFarPos:
        distance_pos_.far_pos = value;
        break;
    case PointDistance::kEndPos:
        distance_pos_.end_pos = value;
        break;
    default:
        break;
    }
}

void PointcloudBuild::SetDisColor(int& value, DistanceColor& color_type)
{
    switch (color_type) {
    case DistanceColor::kCloseStartColor:
        distance_color_.close_start_color = value;
        break;
    case DistanceColor::kCloseEndColor:
        distance_color_.close_end_color = value;
        break;
    case DistanceColor::kMiddleStartColor:
        distance_color_.middle_start_color = value;
        break;
    case DistanceColor::kMiddleEndColor:
        distance_color_.middle_end_color = value;
        break;
    case DistanceColor::kFarStartColor:
        distance_color_.far_start_color = value;
        break;
    case DistanceColor::kFarEndColor:
        distance_color_.far_end_color = value;
        break;
    }
}

void PointcloudBuild::SetROI(ROISelectType type, PointCloudROIA roi_select_data)
{
    switch (type) {
    case ROISelectType::kROIReset:
        point_roi_.min_slot = 0;
        point_roi_.max_slot = 799;
        point_roi_.min_pixel = 0;
        point_roi_.max_pixel = kPointRoiDefaultMaxPixel;
        break;
    case ROISelectType::kROISet:    
        point_roi_ = roi_select_data;
        break;
    default:
        break;
    }
}

void PointcloudBuild::SetLinesShowMode(int mode)
{
    LinesShowMode_ = static_cast<you_see::PointcloudRecordType>(mode);
}

void PointcloudBuild::SetAllSidesDisplay(bool enable)
{
    all_sides_display_ = enable;
}

float PointcloudBuild::MapRange(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
    /*return (toLow - toHigh) / (fromHigh - fromLow) * (value - fromLow) + toHigh;*/
    return (toHigh - toLow) / (fromHigh - fromLow) * (value - fromLow) + toLow;
}

float PointcloudBuild::MapDistance2Color(float& distance)
{
    if (!(distance_pos_.start_pos < distance_pos_.close_pos &&
        distance_pos_.close_pos < distance_pos_.far_pos &&
        distance_pos_.far_pos < distance_pos_.end_pos)) {
        std::cerr << "Error: Distance positions are not in the correct order." << std::endl;
        return 0.0f;
    }

    float color;

    if (distance < distance_pos_.start_pos) {
        color = distance_color_.close_start_color;
    }
    else if (distance < distance_pos_.close_pos) {
        color = MapRange(distance, distance_pos_.start_pos, distance_pos_.close_pos,
            distance_color_.close_start_color, distance_color_.close_end_color);
    }
    else if (distance < distance_pos_.far_pos) {
        color = MapRange(distance, distance_pos_.close_pos, distance_pos_.far_pos,
            distance_color_.middle_start_color, distance_color_.middle_end_color);
    }
    else if (distance < distance_pos_.end_pos) {
        color = MapRange(distance, distance_pos_.far_pos, distance_pos_.end_pos,
            distance_color_.far_start_color, distance_color_.far_end_color);
    }
    else {
        color = distance_color_.far_end_color;
    }

    return std::clamp(color, 0.0f, 255.0f);
    return color;
}

void PointcloudBuild::DrawPrismFacesSingleColor(float& color, const int& frame_index)
{
    switch (prism_faces_) {
    case PrismFaces::four_sided_mirror:
        Draw4FacesSingleColor(color, frame_index);
        break;
    default:
        Draw4FacesSingleColor(color, frame_index);
        break;
    }
}

void PointcloudBuild::Draw4FacesSingleColor(float& color, const int& frame_index)
{
    PrismFourFaces prismfour = static_cast<PrismFourFaces>(frame_index % 4 + 1);
    switch (prismfour) {
    case PrismFourFaces::mirror_fisrt_face:
        color = 50.0f;
        break;
    case PrismFourFaces::mirror_second_face:
        color = 90.0f;
        break;
    case PrismFourFaces::mirror_third_face:
        color = 168.0f;
        break;
    case PrismFourFaces::mirror_fourth_face:
        color = 253.0f;
        break;
    }
}

ScalarType PointcloudBuild::Normalize(ScalarType val) const
{
    // All mapping modes produce a scalar in [0, 255], independent of other frames.
    return std::clamp(val / ScalarType{255.0f}, ScalarType{0.0f}, ScalarType{1.0f});
}

inline const Rgb PointcloudBuild::GetColor(double relativepos, unsigned steps)
{
    if (relativepos >= 0.0 && relativepos <= 1.0)
    {
        //quantized (16 bits) version --> much faster than floor!
        unsigned index = (static_cast<unsigned>((relativepos * steps) * 65535.0)) >> 16;
        return color_scale_element_[((index * (max_steps_ - 1)) / steps)];
    }
    return blackRGB;
}

}; // namespace ys_scene
