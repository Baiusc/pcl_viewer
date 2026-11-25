#ifndef YS_SCENE_POINTCLOUD_PARSER_H_
#define YS_SCENE_POINTCLOUD_PARSER_H_

#include "ys_scene_def.h"
#include "pointcloud_frame_policy.h"
#include "component/color_types.hpp"
#include <QObject>
#include <thread>
#include <mutex>
#include <queue>
#include <optional>
#include <condition_variable>
#include <QList>

namespace ys_scene {

class PointcloudParserImp;
class PointcloudFileManager;
class PointcloudParser : public QObject {
	Q_OBJECT
public:
	explicit PointcloudParser(QObject* parent = nullptr);
    ~PointcloudParser();
	void StartParse1();
	void StartParse2();
	void Stop();
	void InitColor();
	void PointcloudReceive(PointCloudNetPacket* pointcloud, uint16_t size);
	void PointcloudReceive2(const uint32_t handle, PointCloudNetPacket* pointcloud, uint16_t size, void* context);
	// From the point cloud converted from the original point cloud, directly add cartesian_frame to the processing queue
	void AddCartesianFrame(CloudFramePtr frame);
	static void Mf150CartesianFrameTrampoline(const uint32_t handle, void* frame_ptr, void* context);
	void PointcloudParse();
	void PointcloudParse2();
	void Mf150PointcloudParse();
	void SetSavePointCloud(int frames);
	void SetDisplaySides(QList<bool> sides, int counts);
	std::vector<PointcloudParserImp*> GetProcessors();
	PointcloudFileManager* GetPointcloudFileManager();
	static void PointcloudReceiveTrampoline(const uint32_t handle, PointCloudNetPacket* pointcloud, uint16_t size, void* context);

public:
	//! functions for ColorScaleElement
	void Insert(const ColorScaleElement& step, bool auto_update = true);
	void Remove(int index, bool auto_update = true);
	void Clear();
	void Update();
	void Sort();

public slots:
    void SetOptionalSlotNumber(int slot_number);

public slots:
	void StartLoadYscFile(const std::string& filename);
	void StartPlayback();
	void GotoStartPlayback(const std::string& filename, bool paused);
	void SetPlaybackPaused(bool paused);
	void StopPlayback();
	void SetPlaybackSpeed(double speed);
	void SeekPosition(int frame_number);
	void SeekToEnd();
	void SeekOperate(uint32_t operate_type, uint64_t operate_value);

public slots:
    void ResetManagementSession();
    void SetPaused(bool paused);

signals:
	void SliderUpdateSig(int frame);
	void FilePlaybackEnd();

private:
	void PlaybackThreadFunc();

private:
	PointCloudFrameQueue playing_frame_queue_;
	std::thread playback_thread_;
	bool is_playing_;
	bool is_playing_paused_;
	std::string current_read_file_;

private:
	void CheckSavePointCloud();
	bool IsNextFrame(size_t frame_number);
	void DealFrame(CloudFramePtr frame);
	void DealFlatFrame(CloudFramePtr frame);
	void ResetPlayback(bool paused);
	// 整帧期望点数 = 期望 SLOT 数 × 每 SLOT 点数
	size_t ExpectedFramePoints() const;
	// 按当前每 SLOT 点数重算 min_available_data_size_(超时帧接纳阈值)
	void UpdateFrameExpectation();

private:
	std::thread parse_thread_;
	std::mutex mutex_;
	std::condition_variable cv_;
	std::vector<you_see::CartesianPoint> points_;
	std::vector<PointcloudParserImp*> processors_;
	std::tuple<int, std::string> save_point_frames_;
	int frame_id_;
	bool data_ready_;
	bool stop_;
	float speed_;
	static constexpr int processor_size_{ 2 };
	static constexpr int cal_{ 2 };
	int balance_count_;
	std::optional<size_t> first_frame_;
	std::optional<size_t> second_frame_;
	std::priority_queue<CloudFramePtr, std::vector<CloudFramePtr>,
		CloudFrameComparator<you_see::CartesianPoint>> complete_frames_;
	std::unordered_map<size_t, CloudFramePtr> incomplete_frames_;
	std::atomic<bool> finished{ false };
	std::atomic<size_t> next_frame_to_process_{ 0 };
    std::atomic<bool> paused_{false};
	static constexpr std::chrono::milliseconds frame_timeout_{ 250 }; // 大于单帧传输(~100ms)，小于同帧号再次出现的间隔(~300ms)，防止跨轮次混帧
	//static constexpr int min_available_data_size_{ 65000 };
	// 添加的为200 * n数量的槽
    int optinal_slot_number_;
    int expected_received_times_;
    // 每 SLOT 点数：由 SDK 按 UDP 包长解析(1266→96、2514→192、1578→120)，随首个包自适应更新
    int points_per_slot_{ static_cast<int>(you_see::OriginPointParser::single_cartesian) };
	//static constexpr int min_available_data_size_{ 50000 /*/ 2*/ };
	int min_available_data_size_;
	std::atomic<size_t> last_blocked_queue_size_{ 0 };
	PointcloudFileManager* pointcloud_file_manager_;

	// MF150专用处理管线
	std::thread mf150_parse_thread_;
	std::mutex mf150_mutex_;
	std::condition_variable mf150_cv_;
	std::queue<CloudFramePtr> mf150_complete_frames_;
	std::atomic<bool> mf150_stop_{false};

private:
	Rgb rgbscale_[max_steps_];
	QList<ColorScaleElement> steps_;
	bool update_;
	QList<bool> display_sides_;
};
};// namespace ys_scene  PointcloudParser

#endif // YS_SCENE_POINTCLOUD_PARSER_H_