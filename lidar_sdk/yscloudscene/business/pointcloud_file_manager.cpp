#include "pointcloud_file_manager.h"
#include "log/logging.h"
#include "tiny_hut/common_toolset.h" 

namespace ys_scene {
PointcloudFileManager::PointcloudFileManager(QObject* parent)
	: QObject(parent),
	ysc_status_{}
{
	write_thread_ = std::thread(&PointcloudFileManager::WriteThreadFunc, this);
}

PointcloudFileManager::~PointcloudFileManager()
{
	StopRecording();
	ResetReading();
	point_record_queue_.push(CloudFrameThreadExit());
	write_thread_.join();
}

void PointcloudFileManager::ResetReading()
{
	StopPlayback();
	{
		std::lock_guard<std::mutex> lock(is_playing_mutex_);
		ysc_status_.play_status.is_read_fun_exit.store(true);
		is_playing_cond_.notify_one();
	}
	if (read_thread_.joinable()) {
		read_thread_.join();
	}
}

void PointcloudFileManager::ResetNotify()
{
	//std::lock_guard<std::mutex> lock(is_playing_mutex_);
	is_playing_cond_.notify_one();
}

CloudFramePtr PointcloudFileManager::ReadLastFrame()
{
	return ysc_file_->ReadLastFrame();
}

void PointcloudFileManager::PlayOperate(uint32_t operate_type, uint64_t operate_value)
{
	if (ysc_status_.play_status.is_playing && ysc_status_.play_status.is_playing_paused) {
		ysc_status_.play_status.is_playing_paused.store(false);
	}

	PlayOperateType operate = static_cast<PlayOperateType>(operate_type);
	switch (operate) {
	case PlayOperateType::kSeekFrame:
		ysc_status_.play_status.seek_requested.store(true);
		ysc_status_.play_status.seek_frame_number = operate_value;
		break;
	case PlayOperateType::kFastForward:
		ysc_status_.play_status.fast_forward_requested.store(true);
		ysc_status_.play_status.fast_forward_step = operate_value;
		break;
	case PlayOperateType::kFastRewind:
		ysc_status_.play_status.fast_rewind_requested.store(true);
		ysc_status_.play_status.fast_rewind_step = operate_value;
		break;
	default:
		break;
	}
}

void PointcloudFileManager::StartRecording()
{
	std::lock_guard<std::mutex> lock(ysc_file_mutex_);
	if (!ysc_status_.record_status.is_recording.load()) {
		std::string save_dir{};
		std::string default_directory{ "record_cloud" };
		if (!toolset::DirectoryInit(default_directory, save_dir)) {
			return;
		}
		std::string name = toolset::GetCurrentTimestampAsString() + ".ysc";
		auto filepath = toolset::JoinPaths(save_dir, name);
		ysc_recording_file_ = std::make_unique<YscFile>(filepath);
		ysc_status_.record_status.is_recording.store(true);
		ysc_status_.record_status.is_recording_paused.store(false);
		frame_count_ = 0;
	}
}

void PointcloudFileManager::SetPausing(bool pause)
{
	ysc_status_.record_status.is_recording_paused = pause;
}

void PointcloudFileManager::StopRecording()
{
	{
		std::lock_guard<std::mutex> lock(ysc_file_mutex_);
		ysc_status_.record_status.is_recording.store(false);
		ysc_status_.record_status.is_recording_paused.store(false);
	}
	point_record_queue_.push(CloudFrameRecordEnd());

	// 等待写入线程处理完结束信号
	std::unique_lock<std::mutex> lock(ysc_file_mutex_);
	stop_condition_.wait(lock, [this]() { return stop_signal_processed_; });
	stop_signal_processed_ = false; // 重置标志以备下次使用
}

void PointcloudFileManager::SetPointRecordType(int type)
{
	point_lines_type_ = static_cast<you_see::PointcloudRecordType>(type);
}

void PointcloudFileManager::StartLoadYscFile(const std::string& file_path, PointCloudFrameQueue& frame_queue)
{
	//is_playing_.store(true);
	read_thread_ = std::thread(&PointcloudFileManager::ReadThreadFunc, 
		this, file_path, std::ref(frame_queue));
}

void PointcloudFileManager::StartPlayback()
{
	ysc_status_.play_status.is_playing.store(true);
	ysc_status_.play_status.is_read_fun_exit.store(false);
	is_playing_cond_.notify_one();
}

void PointcloudFileManager::SetPlaybackPaused(bool paused)
{
	ysc_status_.play_status.is_playing_paused.store(paused);
}

void PointcloudFileManager::StopPlayback()
{
	ysc_status_.play_status.is_playing.store(false);
	ysc_status_.play_status.is_playing_paused.store(false);
}

void PointcloudFileManager::WriteFrame(CloudFramePtr points)
{
	if (ysc_status_.record_status.is_recording.load() && 
		!ysc_status_.record_status.is_recording_paused.load()) {
		// 将点云数据加入队列
		points->frame_number = frame_count_++;
		point_record_queue_.push(points);
	}
}

bool PointcloudFileManager::ReadFrame(CloudFramePtr points)
{
	return false;
}

void PointcloudFileManager::WriteThreadFunc()
{
	while (true) {
		//LOG_WARN("point_record_queue_ is empty: {0}", point_record_queue_.empty());
		PointCloudOptionalFrame optional_frame = point_record_queue_.pop();
		if (optional_frame.has_value()) {
			CloudFrameVariant data = optional_frame.value();
			if (std::holds_alternative<CloudFramePtr>(data)) {
				CloudFramePtr frame = std::get<CloudFramePtr>(data);
				// process odd number,even,total lines
				if (!frame) {
                    continue;
                }
				CloudFrameLineProcess(frame);
				// 保存点云数据到ysc文件
				std::lock_guard<std::mutex> lock(ysc_file_mutex_);
				if (!ysc_recording_file_) {
					LOG_INFO("ysc_recording_file_ is nullptr, continue");
					continue;
				}
				
				ysc_recording_file_->SaveFrame(std::make_shared<std::vector<CartesianPoint>>(frame->data), 
					frame->ms_timestamp, 
					frame->slot_counts.size());
			}
			else if (std::holds_alternative<CloudFrameRecordEnd>(data)) {
				// 接收到结束信号,保存结尾数据并退出循环
				//LOG_WARN("current record process is end ysc_recording_file_ address: {0}", fmt::ptr(ysc_recording_file_.get()));
				std::lock_guard<std::mutex> lock(ysc_file_mutex_);
				ysc_recording_file_.reset();
				stop_signal_processed_ = true;
				stop_condition_.notify_one();
			}else if (std::holds_alternative<you_see::CloudFrameThreadExit>(data)) {
                // 线程退出信号
                break;
            }
		}
		else {
			break;
		}
	}
}

void PointcloudFileManager::ReadThreadFunc(const std::string& file_path, PointCloudFrameQueue& frame_queue)
{
	ysc_file_ = std::make_unique<YscFile>();
	if (!ysc_file_->Open(file_path)) {
		// 处理文件打开失败的情况
		return;
	}
	emit SliderTime(ysc_file_->GetYscFileHeader());

	std::unique_lock<std::mutex> lock(is_playing_mutex_);
	is_playing_cond_.wait(lock, [this] 
		{ return ysc_status_.play_status.is_playing.load() || 
					ysc_status_.play_status.is_read_fun_exit.load(); });
	//is_playing_cond_.wait(lock);
	//lock.unlock();

	if (ysc_status_.play_status.is_read_fun_exit) {
		return;
	}

	while (ysc_status_.play_status.is_playing) {
		if (!ysc_status_.play_status.is_playing_paused) {
			if (ysc_status_.play_status.seek_requested) {
				// 处理跳转请求
				ysc_file_->SeekToFrame(ysc_status_.play_status.seek_frame_number);
				frame_queue.clear();
				ysc_status_.play_status.seek_requested.store(false);
			}
			else if (ysc_status_.play_status.fast_forward_requested) {
				// 处理快进请求
				//uint64_t current_frame = ysc_file_->GetCurrentFrameNumber();
				frame_queue.clear();
				uint64_t target_frame = /*current_frame + */ysc_status_.play_status.fast_forward_step;
				ysc_file_->SeekToFrame(target_frame);
				ysc_status_.play_status.fast_forward_requested.store(false);
			}
			else if (ysc_status_.play_status.fast_rewind_requested) {
				// 处理快退请求
				//uint64_t current_frame = ysc_file_->GetCurrentFrameNumber();
				/*uint64_t target_frame = (current_frame > ysc_status_.play_status.fast_rewind_step) ? 
					(current_frame - ysc_status_.play_status.fast_rewind_step) : 0;*/
				uint64_t target_frame = ysc_status_.play_status.fast_rewind_step;
				frame_queue.clear();
				ysc_file_->SeekToFrame(target_frame);
				ysc_status_.play_status.fast_rewind_requested.store(false);
			}
			CloudFramePtr frame = ysc_file_->ReadFrame();
			if (frame) {
				while (frame_queue.size() >= max_queue_size) {
					std::this_thread::sleep_for(std::chrono::milliseconds(10));
				}
				frame_queue.push(frame);
				/*LOG_WARN("after read the ysc file the frame_queue's size is: {0}, the total bytes is: {1}",
					frame_queue.size(), frame_queue.size() * frame->data.size() * sizeof(CartesianPoint));*/
			}
			else {
				frame_queue.push(CloudFrameRecordEnd());
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
				//break;
			}

		}
		else {
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}
	ysc_file_->Close();
}

void PointcloudFileManager::CloudFrameLineProcess(CloudFramePtr& frame)
{
	if (point_lines_type_ == PointcloudRecordType::total_lines) {
		return;
	}

	size_t columns = frame->slot_counts.size();
	if (columns == 0) {
		return; // 无 SLOT 信息(如 MF150 整帧)不做奇偶行抽取，避免除零
	}
	size_t total_points = frame->data.size();
	size_t raw = total_points / columns;

	// 每 SLOT 点数随机型自适应：alpha/beta 单回波=96，CS96-gamma 单回波=120
	if (raw != static_cast<size_t>(you_see::ToInt(you_see::OriginPointParser::single_cartesian)) &&
		raw != static_cast<size_t>(kMaxCartesianPixelsPerSlot)) {
		LOG_WARN("Warning: Calculated raw is \"{}\" instead of expected 96 (alpha/beta) or 120 (CS96-gamma)", raw);
	}

	// 预分配结果vector
	std::vector<CartesianPoint> filtered_data;
	filtered_data.reserve(total_points / 2);  // 预估大小

#pragma omp parallel for ordered schedule(static) if(columns > 4)
	for (size_t col = 0; col < columns; ++col) {
		std::vector<CartesianPoint> column_data;
		column_data.reserve(raw / 2);

		for (size_t row = 0; row < raw; ++row) {
			bool keep_row = (point_lines_type_ == PointcloudRecordType::odd_number_lines) ?
				(row % 2 == 1) : (row % 2 == 0);

			if (keep_row) {
				column_data.push_back(frame->data[row + col * raw]);
			}
		}

#pragma omp ordered
		{
			filtered_data.insert(filtered_data.end(),
				column_data.begin(),
				column_data.end());
		}
	}

	frame->data = std::move(filtered_data);
}

}// namespace ys_scene 

