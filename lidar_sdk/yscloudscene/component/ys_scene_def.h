#ifndef YS_SCENE_DEF_H_
#define YS_SCENE_DEF_H_

#include "thread_safe_queue.hpp"
#include "ys_lidar_type.h"
#include "ys_lidar_validate_type.h"
#include <variant>
#include <QMetaType>
#include <chrono>

namespace ys_scene {

enum class PlayOperateType : uint32_t {
	kSeekFrame,
	kFastForward,
	kFastRewind,
};

enum class PointDistance : uint32_t {
	kStartPos,
	kClosePos,
	kFarPos,
	kEndPos
};

enum class DistanceColor : uint32_t {
	kCloseStartColor,
	kCloseEndColor,
	kMiddleStartColor,
	kMiddleEndColor,
	kFarStartColor,
	kFarEndColor
};

enum class MappingMethod : uint32_t {
	kXAxisMap,
	kYAxisMap,
	kZAxisMap,
	kReflectivetyMap
};

enum class ROISelectType : uint32_t {
	kROIReset,
	kROISet,
	kROICancel
};

enum class OptionalSlotNumber : uint32_t {
    KSlot200 = 200,
    KSlot400 = 400,
    KSlot600 = 600,
    KSlot800 = 800,
    KSlot1000 = 1000,
};

using PointDistancePosA = struct PointDistancePos {
	float start_pos;
	float close_pos;
	float far_pos;
	float end_pos;
};

using DistanceColorValA = struct DistanceColorVal {
	float close_start_color;
	float close_end_color;
	float middle_start_color;
	float middle_end_color;
	float far_start_color;
	float far_end_color;
};

using YscRecordStatusA = struct YscRecordStatus {
	YscRecordStatus() : is_recording{ false },
		is_recording_paused{false}{}
	std::atomic<bool> is_recording;
	std::atomic<bool> is_recording_paused;
};

using YscPlayStatusA = struct YscPlayStatus {
	YscPlayStatus() : is_read_fun_exit{false},
		is_playing{false},
		is_playing_paused{ false },
		seek_requested{false},
		fast_forward_requested{false},
		fast_rewind_requested{false},
		seek_frame_number{0},
		fast_forward_step{0},
		fast_rewind_step{0}{}
	
	void Reset() {
		is_read_fun_exit.store(false);
		is_playing.store(false);
		is_playing_paused.store(false);
		seek_requested.store(false);
		fast_forward_requested.store(false);
		fast_rewind_requested.store(false);
		seek_frame_number = 0;
		fast_forward_step = 0;
		fast_rewind_step = 0;
	}

	std::atomic<bool> is_read_fun_exit;
	std::atomic<bool> is_playing;
	std::atomic<bool> is_playing_paused;
	std::atomic<bool> seek_requested;
	std::atomic<bool> fast_forward_requested;
	std::atomic<bool> fast_rewind_requested;
	uint64_t seek_frame_number;
	uint64_t fast_forward_step;
	uint64_t fast_rewind_step;
};

using YscStatusA = struct YscStatus {
	YscStatus() : record_status{},
		play_status{}{}
	YscRecordStatusA record_status;
	YscPlayStatusA	play_status;
};

using AlgorithmItemEnable = struct AlgorithmItem{
	AlgorithmItem() : de_tail_filter{ false } {};
	bool de_tail_filter;
};
Q_DECLARE_METATYPE(AlgorithmItemEnable)

using DistanceMapA = struct DistanceMap {
	DistanceMap(uint16_t dis, std::string name_t, uint16_t col, PointDistance dis_t) :
		distance{ dis }, name{ name_t }, color{ col }, dis_type{ dis_t }{}
	bool operator==(const DistanceMap& other) const {
		return distance == other.distance;
	}

	uint16_t distance;
	std::string name;
	uint16_t color;
	PointDistance dis_type;
};

using DisstanceColorMapA = struct DisstanceColorMap {
	DisstanceColorMap(uint16_t col_value, std::string name_t, uint16_t col, DistanceColor col_val_t) :
		color_value{ col_value }, name{ name_t }, color{ col }, color_value_type{ col_val_t }{}
	uint16_t color_value;
	std::string name;
	uint16_t color;
	DistanceColor color_value_type;
};

// 单 SLOT 最大垂直像素数：alpha/beta 为 96，CS96-gamma 为 120。
// 点云显示 ROI 的默认像素上界取其最大索引，对像素更少的机型等价于"不过滤"，不改变既有显示效果。
constexpr uint16_t kMaxCartesianPixelsPerSlot = 120;
constexpr uint16_t kPointRoiDefaultMaxPixel = kMaxCartesianPixelsPerSlot - 1;

using PointCloudROIA = struct PointCloudROI {
	PointCloudROI():
		min_slot{ 0 }, 
		max_slot{ 0 },
		min_pixel{ 0 },
		max_pixel{ 0 } {}

	uint16_t min_slot;
	uint16_t max_slot;
	uint16_t min_pixel;
	uint16_t max_pixel;
};
Q_DECLARE_METATYPE(PointCloudROIA)

using DisMapList = std::vector<DistanceMapA>;
using DisColorMapList = std::vector<DisstanceColorMapA>;
// point cloud
using CloudPureFramePtr = std::shared_ptr<std::vector<you_see::CartesianPoint>>;
using CloudFramePtr = std::shared_ptr<you_see::CloudFrame<you_see::CartesianPoint>>;
using CloudFrameVariant = std::variant<CloudFramePtr, you_see::CloudFrameRecordEnd, you_see::CloudFrameThreadExit>;
using PointCloudFrameQueue = ThreadSafeQueue<CloudFrameVariant>;
using PointCloudOptionalFrame = std::optional<CloudFrameVariant>;

// origin point cloud
using OriginFramePtr = std::shared_ptr<you_see::CloudFrame<uint8_t>>;
using SphericalFramePtr = std::shared_ptr<you_see::CloudFrame<you_see::SphericalPoint>>;

} // namespace ys_scene

#endif // YS_SCENE_DEF_H_