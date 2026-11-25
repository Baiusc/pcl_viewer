#include "mf150_calib_controller.h"
#include "mf150_calibration_runner.h"
#include "business/origin_point_parser.h"
#include "component/mf150_preview_config/mf150_preview_config.h"
#include "ys_lidar_validate_type.h"

#include <QApplication>
#include <lumosy/buddies/lumomo_ui/flow_pipeline/flow_pipeline.h>

#include <chrono>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace ys_scene {

Mf150CalibController::Mf150CalibController(OriginPointParser* parser,
                                           Mf150PreviewConfig* dialog,
                                           QObject* parent)
    : QObject(parent)
    , parser_(parser)
    , dialog_(dialog)
{
}

void Mf150CalibController::StartCalibration(float dist_real, const std::string& bin_path)
{
    auto last_frame = parser_->GetLastMf150Frame();
    if (!last_frame) {
        emit CalibrationFailed("No MF150 frame available");
        return;
    }

    int echo_index = dialog_->GetEchoIndex();

    std::vector<std::vector<Mf150RawPixel>> raw_grid;
    ParseMf150RawFrame(last_frame->data.data(), last_frame->data.size(), raw_grid);

    last_single_raw_sample_.dist_real = dist_real;
    last_single_raw_sample_.reflectivity = dialog_->GetSingleSyncSaveReflectivity();
    last_single_raw_sample_.raw_grid = raw_grid;

    if (dialog_->IsSingleSyncSaveEnabled()) {
        DoSyncSaveSingle(raw_grid, last_frame->frame_number, last_frame->ms_timestamp, dist_real);
    }

    auto calib_grid = ExtractMf150CalibGrid(raw_grid, echo_index);

    auto* pipeline = dialog_->GetFlowPipeline();

    std::vector<std::vector<float>> calib_map;
    bool success = RunMf150Calibration(calib_grid,
        echo_index,
        dist_real,
        bin_path,
        [pipeline](int step_idx, int total_steps,
                   const std::string& step_name, const std::string& msg) {
            if (step_idx > 0 && step_idx <= pipeline->NodeCount()) {
                bool is_error = (step_name == "Error");
                pipeline->SetStatus(step_idx - 1, is_error
                    ? lumosy::FlowStatus::KFailed
                    : lumosy::FlowStatus::KCompleted);
                if (step_idx < pipeline->NodeCount()) {
                    pipeline->SetStatus(step_idx,
                        is_error ? lumosy::FlowStatus::KNotStarted
                                 : lumosy::FlowStatus::KInProgress);
                }
                pipeline->repaint();
                QApplication::processEvents();
            }
        },
        calib_map);

    if (success && !calib_map.empty()) {
        emit CalibrationFinished(calib_map);
    } else {
        emit CalibrationFailed("Calibration failed");
    }
}

bool Mf150CalibController::CollectFrame(float dist_real)
{
    auto last_frame = parser_->GetLastMf150Frame();
    if (!last_frame) {
        return false;
    }

    int echo_index = dialog_->GetEchoIndex();

    std::vector<std::vector<Mf150RawPixel>> raw_grid;
    ParseMf150RawFrame(last_frame->data.data(), last_frame->data.size(), raw_grid);

    if (dialog_->IsMultiSyncSaveEnabled()) {
        DoSyncSaveMulti(raw_grid, last_frame->frame_number, last_frame->ms_timestamp, dist_real);
    }

    auto calib_grid = ExtractMf150CalibGrid(raw_grid, echo_index);

    collected_raw_samples_.push_back({dist_real, dialog_->GetMultiSyncSaveReflectivity(), raw_grid});
    collected_samples_.push_back({dist_real, std::move(calib_grid)});
    return true;
}

void Mf150CalibController::RemoveCollectedFrame(int index)
{
    if (index >= 0 && static_cast<size_t>(index) < collected_samples_.size()) {
        collected_samples_.erase(collected_samples_.begin() + index);
        collected_raw_samples_.erase(collected_raw_samples_.begin() + index);
    }
}

void Mf150CalibController::SetRoiRect(int row, int col, int width, int height)
{
    roi_row_ = row;
    roi_col_ = col;
    roi_width_ = width;
    roi_height_ = height;
}

void Mf150CalibController::GetRoiRect(int& row, int& col, int& width, int& height) const
{
    row = roi_row_;
    col = roi_col_;
    width = roi_width_;
    height = roi_height_;
}

void Mf150CalibController::ClearCollectedFrames()
{
    collected_samples_.clear();
    collected_raw_samples_.clear();
    multi_sync_save_dir_.clear();
    multi_sync_save_index_ = 0;
}

size_t Mf150CalibController::GetCollectedFrameCount() const
{
    return collected_samples_.size();
}

void Mf150CalibController::StartMultiCalibration(const std::string& bin_path)
{
    if (collected_samples_.empty()) {
        emit MultiCalibrationFailed("No collected frames available");
        return;
    }

    if (collected_samples_.size() < 2) {
        emit MultiCalibrationFailed("Need at least 2 distances for multi-point calibration");
        return;
    }

    int echo_index = dialog_->GetEchoIndex();
    auto* pipeline = dialog_->GetMultiFlowPipeline();

    // LUT 表落盘路径：由传入 bin_path 派生(与启动时加载路径一致)
    std::string lut_path = bin_path;
    if (!lut_path.empty()) {
        std::filesystem::path bp(lut_path);
        lut_path = (bp.parent_path() /
            (bp.stem().string() + "_multi_lut.bin")).string();
    } else {
        lut_path = "calibration/mf150/map_solid_multi_lut.bin";
    }

    auto table = std::make_shared<SslDistCalibMulti>();
    bool success = RunMf150MultiCalibration(collected_samples_,
        echo_index,
        lut_path,
        [pipeline](int step_idx, int total_steps,
                   const std::string& step_name, const std::string& msg) {
            if (pipeline) {
                if (step_idx > 0 && step_idx <= pipeline->NodeCount()) {
                    bool is_error = (step_name == "Error");
                    pipeline->SetStatus(step_idx - 1, is_error
                        ? lumosy::FlowStatus::KFailed
                        : lumosy::FlowStatus::KCompleted);
                    if (step_idx < pipeline->NodeCount()) {
                        pipeline->SetStatus(step_idx,
                            is_error ? lumosy::FlowStatus::KNotStarted
                                     : lumosy::FlowStatus::KInProgress);
                    }
                    pipeline->repaint();
                    QApplication::processEvents();
                }
            }
        },
        *table);

    if (success) {
        emit MultiCalibrationFinished(table);
    } else {
        emit MultiCalibrationFailed("Multi-point calibration failed");
    }
}

bool Mf150CalibController::DoSyncSaveSingle(const std::vector<std::vector<Mf150RawPixel>>& grid,
    size_t frame_number, uint64_t ms_timestamp, float dist_real)
{
    if (grid.empty()) {
        return false;
    }

    // 获取板子的反射率
    std::string refl = dialog_->GetSingleSyncSaveReflectivity();

    auto now = std::chrono::system_clock::now();
    auto time_val = std::chrono::system_clock::to_time_t(now);
    std::tm tm;
    localtime_s(&tm, &time_val);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;

    std::ostringstream ts_oss;
    ts_oss << std::put_time(&tm, "%Y-%m-%d_%H.%M.%S")
           << "." << std::setfill('0') << std::setw(3) << ms.count();
    std::string timestamp = ts_oss.str();

    std::ostringstream dist_oss;
    dist_oss << std::fixed << std::setprecision(3) << dist_real;
    std::string dist_str = dist_oss.str();

    std::string dir_name = "f" + refl + "_d" + dist_str + "__" + timestamp;
    std::string file_name = "f" + refl + "_d" + dist_str + "_i1.txt";
    std::string full_path = "calibration/mf150/" + dir_name + "/" + file_name;

    return SaveMf150RawFrameToFile(grid, frame_number, ms_timestamp, full_path);
}

bool Mf150CalibController::DoSyncSaveMulti(const std::vector<std::vector<Mf150RawPixel>>& grid,
    size_t frame_number, uint64_t ms_timestamp, float dist_real)
{
    if (grid.empty()) {
        return false;
    }

    if (multi_sync_save_dir_.empty()) {
        auto now = std::chrono::system_clock::now();
        auto time_val = std::chrono::system_clock::to_time_t(now);
        std::tm tm;
        localtime_s(&tm, &time_val);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;

        std::ostringstream ts_oss;
        ts_oss << std::put_time(&tm, "%Y-%m-%d_%H.%M.%S")
               << "." << std::setfill('0') << std::setw(3) << ms.count();
        std::string timestamp = ts_oss.str();

        std::string dir_name = "mf150_multi_calib_" + timestamp;
        multi_sync_save_dir_ = "calibration/mf150/" + dir_name + "/";
        multi_sync_save_index_ = 0;
    }

    multi_sync_save_index_++;

    std::string refl = dialog_->GetMultiSyncSaveReflectivity();
    std::ostringstream dist_oss;
    dist_oss << std::fixed << std::setprecision(3) << dist_real;
    std::string dist_str = dist_oss.str();

    std::string file_name = "f" + refl + "_d" + dist_str + "_i"
        + std::to_string(multi_sync_save_index_) + ".txt";
    std::string full_path = multi_sync_save_dir_ + file_name;

    return SaveMf150RawFrameToFile(grid, frame_number, ms_timestamp, full_path);
}

std::string Mf150CalibController::MakeTimestamp() const
{
    auto now = std::chrono::system_clock::now();
    auto time_val = std::chrono::system_clock::to_time_t(now);
    std::tm tm;
    localtime_s(&tm, &time_val);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;

    std::ostringstream ts_oss;
    ts_oss << std::put_time(&tm, "%Y-%m-%d_%H.%M.%S")
           << "." << std::setfill('0') << std::setw(3) << ms.count();
    return ts_oss.str();
}

bool Mf150CalibController::SaveRawSampleToFile(const RawSample& sample,
    const std::string& dir, int index) const
{
    if (sample.raw_grid.empty()) {
        return false;
    }

    std::ostringstream dist_oss;
    dist_oss << std::fixed << std::setprecision(3) << sample.dist_real;
    std::string dist_str = dist_oss.str();

    std::string file_name = "f" + sample.reflectivity + "_d" + dist_str + "_i"
        + std::to_string(index) + ".txt";
    std::string full_path = dir + file_name;

    return SaveMf150RawFrameToFile(sample.raw_grid, 0, 0, full_path);
}

void Mf150CalibController::SaveSingleRawData()
{
    if (last_single_raw_sample_.raw_grid.empty()) {
        return;
    }

    std::string dir_name = "mf150_single_calib_" + MakeTimestamp();
    std::string dir = "calibration/mf150/" + dir_name + "/";

    SaveRawSampleToFile(last_single_raw_sample_, dir, 1);
}

void Mf150CalibController::SaveMultiRawData()
{
    if (collected_raw_samples_.empty()) {
        return;
    }

    std::string dir_name = "mf150_multi_calib_" + MakeTimestamp();
    std::string dir = "calibration/mf150/" + dir_name + "/";

    for (size_t i = 0; i < collected_raw_samples_.size(); ++i) {
        SaveRawSampleToFile(collected_raw_samples_[i], dir, static_cast<int>(i + 1));
    }
}

} // namespace ys_scene
