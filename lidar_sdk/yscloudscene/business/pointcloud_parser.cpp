#include "pointcloud_parser.h"
#include "pointcloud_parser_imp.h"
#include "pointcloud_frame_policy.h"
#include "ys_lidar_validate_type.h"
#include "log/logging.h"
#include "tiny_hut/common_toolset.h" 
#include "config/config_service.h"
#include "pointcloud_file_manager.h"
#include "component/ccgeom.hpp"
#include "component/ys_scene_def.h"

#include <QThread>
#include <QThreadPool>
//#ifdef _WIN32
//#include<winsock2.h>
//#pragma comment(lib, "Ws2_32.lib")
//#endif // WIN32

namespace ys_scene {
PointcloudParser::PointcloudParser(QObject *parent)
    : QObject(parent)
    , is_playing_{false}
    , is_playing_paused_{false}
    , save_point_frames_{0, ""}
    , frame_id_{0}
    , data_ready_{false}
    , stop_{false}
    , speed_{1.0f}
    , balance_count_{0}
    , optinal_slot_number_{800}
    , min_available_data_size_{50000}
{
    expected_received_times_ = you_see::ConfigService::Instance().Get<int>("optional_slot_number", 800) - 1;
	display_sides_.reserve(4);
	display_sides_ << true << true << true << true;
	InitColor();
	for (int i = 0; i < processor_size_; i++) {
        processors_.push_back(
			new PointcloudParserImp(rgbscale_, this));
	}
	// StartParse1(); // parse pointcloud by slot number 799
	pointcloud_file_manager_ = new PointcloudFileManager(this); //pointcloud recording and playback
	StartParse2();  // parse pointcloud by frame
	mf150_parse_thread_ = std::thread(&PointcloudParser::Mf150PointcloudParse, this);
}

PointcloudParser::~PointcloudParser()
{
    if (!stop_){
        Stop();
	}
	
	/*for (auto processor : processors_) {
		delete processor;
	}*/
}

void PointcloudParser::StartParse1()
{
	parse_thread_ = std::thread(&PointcloudParser::PointcloudParse, this);
}

void PointcloudParser::StartParse2()
{
	parse_thread_ = std::thread(&PointcloudParser::PointcloudParse2, this);
}

void PointcloudParser::Stop()
{
	{
		std::unique_lock<std::mutex> lock(mutex_);
		stop_ = true;
	}
    ResetPlayback(true);
	cv_.notify_all();
	if (parse_thread_.joinable()) {
		parse_thread_.join();
	}
	mf150_stop_ = true;
	mf150_cv_.notify_one();
	if (mf150_parse_thread_.joinable()) {
		mf150_parse_thread_.join();
	}
    //delete pointcloud_file_manager_;
    QThreadPool::globalInstance()->waitForDone();
}

void PointcloudParser::InitColor()
{
    Q_ASSERT(QThread::currentThread() == thread());
	Insert(ColorScaleElement(0.0, Qt::blue), false);
	Insert(ColorScaleElement(1.0 / 3.0, Qt::green), false);
	Insert(ColorScaleElement(2.0 / 3.0, Qt::yellow), false);
	Insert(ColorScaleElement(1.0, Qt::red), false);
	Update();
}

void PointcloudParser::PointcloudReceive(PointCloudNetPacket* pointcloud, uint16_t size)
{
	std::lock_guard<std::mutex> lock(mutex_);
	int slot_num = pointcloud->current_slot;

	CartesianPoint* points = reinterpret_cast<CartesianPoint*>(pointcloud->points);
	points_.insert(points_.end(), points, points + size);
	auto oops = sizeof(CartesianPoint);
	
	//pointcloud->frame_index = htons(pointcloud->frame_index);
	//pointcloud->total_slots = htons(pointcloud->total_slots);
	//slot_num = htons(slot_num);

	if (slot_num == static_cast<uint16_t>(OriginPointParser::truncation_slot_number)) {
		data_ready_ = true;
		frame_id_ = pointcloud->frame_index;
		cv_.notify_one();
		//LOG_DEBUG("show a complete frame id:{0}", frame_id_);
	}
}

void PointcloudParser::PointcloudReceive2(const uint32_t handle, PointCloudNetPacket* pointcloud, uint16_t size, void* context)
{
    if (paused_.load(std::memory_order_relaxed)) {
        return;
    }

	std::lock_guard<std::mutex> lock(mutex_);
	/*if (pointcloud->frame_index != 0) {
		return;
	}*/

	// 每包点数由包长决定(1266→96、1578→120)：机型切换或首个包到达时同步整帧期望值，
	// 避免沿用 96 像素的旧期望导致 gamma 帧在收满 640 个 SLOT(640×120=76800) 时被提前判定成帧
	if (size > 0 && size != points_per_slot_) {
		points_per_slot_ = size;
		UpdateFrameExpectation();
		LOG_INFO("PointcloudReceive2: points_per_slot={0}, expected_frame_points={1}, min_available_data_size_={2}",
			points_per_slot_, ExpectedFramePoints(), min_available_data_size_);
	}

	if (!first_frame_) {
		first_frame_ = pointcloud->frame_index;
		LOG_INFO("PointcloudReceive2: INIT first_frame_={0}", *first_frame_);
	}
	else if (!second_frame_ && pointcloud->frame_index != *first_frame_) {
		second_frame_ = pointcloud->frame_index;
		next_frame_to_process_ = *second_frame_;
		complete_frames_ = std::priority_queue<CloudFramePtr, std::vector<CloudFramePtr>,
			CloudFrameComparator<CartesianPoint>>(*second_frame_);
		LOG_INFO("PointcloudReceive2: INIT first_frame_={0} second_frame_={1} next_frame_to_process_={2}",
			*first_frame_, *second_frame_, next_frame_to_process_.load());
	}

	CartesianPoint* points = reinterpret_cast<CartesianPoint*>(pointcloud->points);
    if (size == 0 || pointcloud->current_slot >= expected_received_times_ + 1) {
        return;
    }

	auto& frame = incomplete_frames_[pointcloud->frame_index];
	if (!frame) {
		// 按整帧期望点数预分配：gamma(800×120=96000 点 ≈ 1.25MB/帧)不会中途扩容
		frame = std::make_shared<CloudFrame<CartesianPoint>>(pointcloud->frame_index, ExpectedFramePoints());
	}

    // 重复 UDP 包不能增加点数或导致提前成帧，slot_counts 必须与 data 一一对应。
    if (!frame->received_slots.insert(pointcloud->current_slot).second) {
        return;
    }
	frame->data.insert(frame->data.end(), points, points + size);
	frame->slot_counts.push_back(pointcloud->current_slot);
	frame->last_update = std::chrono::steady_clock::now();
	frame->receive_times++;
	if (frame->receive_times == 1) {
		static size_t last_frame_seen = static_cast<size_t>(-1);
		LOG_INFO("PointcloudReceive2: RX frame={0} first slot={1} total_slots={2} size={3} (prev_frame={4})",
			pointcloud->frame_index, pointcloud->current_slot, pointcloud->total_slots, size, last_frame_seen);
		last_frame_seen = pointcloud->frame_index;
	}
	if (frame_policy::IsFrameComplete(frame->data.size(), frame->received_slots.size(),
            ExpectedFramePoints(), expected_received_times_ + 1)) {
		frame_id_ = pointcloud->frame_index;
        //LOG_INFO("current frame id: {0}", pointcloud->frame_index);
        std::string missing_slots;
        for (int s = 0; s < ToInt(OriginPointParser::slot_counts); ++s) {
            if (frame->received_slots.find(static_cast<uint16_t>(s)) == frame->received_slots.end()) {
                missing_slots += std::to_string(s) + " ";
            }
        }
        LOG_INFO("PointcloudReceive2: frame={0} COMPLETE size={1} times={2} slots={3} -> enqueued (missing=[{4}])",
            pointcloud->frame_index, frame->data.size(), frame->receive_times,
            frame->received_slots.size(), missing_slots.empty() ? std::string("none") : missing_slots);

		if (second_frame_) {
			complete_frames_.push(frame);
			cv_.notify_one();
			//LOG_INFO("notified by complete frame: {0}", pointcloud->frame_index);
		}
		incomplete_frames_.erase(pointcloud->frame_index);
	}/*else if (pointcloud->current_slot == 0) {
		cv_.notify_one();
		LOG_INFO("notified by frame: {0}'s slot 0", pointcloud->frame_index);
	}*/
}

void PointcloudParser::AddCartesianFrame(CloudFramePtr frame)
{
    if (paused_.load(std::memory_order_relaxed)) {
        return;
    }

    if (frame->slot_counts.empty()) {
        std::lock_guard<std::mutex> lock(mf150_mutex_);
        mf150_complete_frames_.push(frame);
        mf150_cv_.notify_one();
        return;
    }

	std::lock_guard<std::mutex> lock(mutex_);

	if (!second_frame_) {
		first_frame_ = frame->frame_number;
		second_frame_ = frame->frame_number;
		next_frame_to_process_ = frame->frame_number;
		complete_frames_ = std::priority_queue<CloudFramePtr, std::vector<CloudFramePtr>,
			CloudFrameComparator<CartesianPoint>>(frame->frame_number);
		LOG_DEBUG("AddCartesianFrame queue initialized with number: {}", *second_frame_);
	}

	complete_frames_.push(frame);
	cv_.notify_one();
}

void PointcloudParser::Mf150CartesianFrameTrampoline(const uint32_t handle, void *frame_ptr, void *context)
{
    auto *parser = reinterpret_cast<PointcloudParser *>(context);
    auto *frame = static_cast<std::shared_ptr<you_see::CloudFrame<you_see::CartesianPoint>> *>(frame_ptr);
    parser->AddCartesianFrame(*frame);
    (void)handle;
}

void PointcloudParser::PointcloudParse()
{
	while (true) {
		std::unique_lock<std::mutex> lock(mutex_);
		cv_.wait(lock, [this] { return data_ready_; });

		auto data_count = ToInt(OriginPointParser::slot_counts) * ToInt(OriginPointParser::single_cartesian);
		if (data_count > points_.size()) {
			data_count = points_.size();
		}
		CloudPureFramePtr shared_data = std::make_shared<std::vector<CartesianPoint>>();
		auto step_size = 0;
		auto offset_begin = step_size * ToInt(OriginPointParser::single_cartesian);
		auto offset_end = step_size * ToInt(OriginPointParser::single_cartesian);
		shared_data->assign(points_.begin() + offset_begin, points_.begin() + data_count - offset_end);
		auto temp = std::make_shared<std::vector<int>>();

		QThreadPool::globalInstance()->start(
			new PointcloudTask(shared_data, 
				temp,
				processors_[balance_count_ % cal_], 
				save_point_frames_,
				frame_id_));
		
		CheckSavePointCloud();
		
		points_.erase(points_.begin(), points_.begin() + data_count);
		data_ready_ = false;
		balance_count_++;

		if (stop_) {
			break;
		}
	}
}

void PointcloudParser::PointcloudParse2()
{
	while (true) {
		std::unique_lock<std::mutex> lock(mutex_);
		cv_.wait_for(lock, std::chrono::milliseconds(50), [this] { 
			return stop_ ||(!complete_frames_.empty() && second_frame_); 
		});

		if (stop_) {
            break;
        }

		auto now = std::chrono::steady_clock::now();

		// 检查并处理超时的不完整帧
		for (auto it = incomplete_frames_.begin(); it != incomplete_frames_.end();) {
			if (now - it->second->last_update >= frame_timeout_) {
				std::string missing_slots;
				for (int s = 0; s < ToInt(OriginPointParser::slot_counts); ++s) {
					if (it->second->received_slots.find(static_cast<uint16_t>(s)) == it->second->received_slots.end()) {
						missing_slots += std::to_string(s) + " ";
					}
				}
				if (it->second->data.size() >= min_available_data_size_ && 
					second_frame_) {
					complete_frames_.push(it->second);
					LOG_INFO("TIMEOUT: frame={0} size={1} slots={2} -> enqueued (min={3}, missing=[{4}])",
						it->first, it->second->data.size(), it->second->received_slots.size(),
						min_available_data_size_, missing_slots.empty() ? std::string("none") : missing_slots);
					/*for (uint16_t slot = 0; slot < ToInt(OriginPointParser::slot_counts); ++slot) {
						if (it->second->received_slots.find(slot) == it->second->received_slots.end()) {
							LOG_INFO("Frame {0} missing slot {1}", it->first, slot);
						}
					}*/
				}
				else {
					LOG_INFO("TIMEOUT: frame={0} size={1} slots={2} -> DISCARDED (min={3}, first_frame_={4}, missing=[{5}])",
						it->first, it->second->data.size(), it->second->received_slots.size(),
						min_available_data_size_,
						first_frame_ ? std::to_string(*first_frame_) : std::string("none"),
						missing_slots.empty() ? std::string("none") : missing_slots);
				}
				it = incomplete_frames_.erase(it);
			}
			else {
				++it;
			}
		}
		while (!complete_frames_.empty() &&
			(complete_frames_.top()->frame_number == next_frame_to_process_ ||
				complete_frames_.top()->data.size() >= min_available_data_size_)) {
			auto frame = complete_frames_.top();
			complete_frames_.pop();
			LOG_INFO("Parse2 POP: frame={0} size={1} next_expected={2} min={3}",
				frame->frame_number, frame->data.size(),
				next_frame_to_process_.load(), min_available_data_size_);
			lock.unlock();

			pointcloud_file_manager_->WriteFrame(frame);
			// 将帧数据分发给Qt线程池处理
			if (display_sides_[0] == true) {
				if (frame->frame_number % 4 == 0) {
					DealFrame(frame);
				}
			}

			if (display_sides_[1] == true) {
				if (frame->frame_number % 4 == 1) {
					DealFrame(frame);
				}
			}

			if (display_sides_[2] == true) {
				if (frame->frame_number % 4 == 2) {
					DealFrame(frame);
				}
			}

			if (display_sides_[3] == true) {
				if (frame->frame_number % 4 == 3) {
					DealFrame(frame);
				}
			}

            if (frame->frame_number == next_frame_to_process_) {
                // CS96四面镜帧号0123循环；MF150走独立队列不经过此处
                next_frame_to_process_ = (next_frame_to_process_ + 1) % 4;
            }

            lock.lock();
            if (stop_) {
                return;
            }
        }
        if (!complete_frames_.empty()) {
            auto top = complete_frames_.top();
            size_t qs = complete_frames_.size();
            size_t last_blocked_qs = last_blocked_queue_size_.exchange(qs);
            if (last_blocked_qs == 0 || qs - last_blocked_qs >= 20) {
                LOG_INFO("PointcloudParse2 blocked: queue_size={0} top_frame={1} next_expected={2} data_size={3} threshold={4}",
                    qs, top->frame_number,
                    next_frame_to_process_.load(), top->data.size(), min_available_data_size_);
            }
        } else {
            last_blocked_queue_size_.store(0);
        }
    }
}

void PointcloudParser::SetSavePointCloud(int frames)
{
	auto default_frames = ToInt(PointCloudParser::default_save_frame_count);
	// !if a save operation is in progress, return
	if (std::get<0>(save_point_frames_) != 0) {
		return;
	}

	if (frames > default_frames) {
		std::get<0>(save_point_frames_) = frames;
		std::get<1>(save_point_frames_) = toolset::GetCurrentTimestampAsString();
	}
	else {
		std::get<0>(save_point_frames_) = frames;
		std::get<1>(save_point_frames_) = "";
	}
}

void PointcloudParser::SetDisplaySides(QList<bool> sides, int counts)
{
	display_sides_.clear();
	display_sides_ = sides;
}

std::vector<PointcloudParserImp*> PointcloudParser::GetProcessors()
{
	return processors_;
}

PointcloudFileManager* PointcloudParser::GetPointcloudFileManager()
{
	return pointcloud_file_manager_;
}

void PointcloudParser::PointcloudReceiveTrampoline(const uint32_t handle, 
												PointCloudNetPacket* pointcloud, 
												uint16_t size, 
												void* context)
{
	auto* pointcloud_paser = reinterpret_cast<PointcloudParser*>(context);
	pointcloud_paser->PointcloudReceive2(handle, pointcloud, size, context);
}

void PointcloudParser::Insert(const ColorScaleElement& step, bool auto_update)
{
    Q_ASSERT(QThread::currentThread() == thread());
	steps_.push_back(step);

	update_ = false;

	if (auto_update && steps_.size() >= static_cast<int>(min_steps_))
	{
		Update();
	}
}

void PointcloudParser::Remove(int index, bool auto_update)
{
    Q_ASSERT(QThread::currentThread() == thread());
	steps_.removeAt(index);
	update_ = false;

	if (auto_update)
		Update();
}

void PointcloudParser::Clear()
{
    Q_ASSERT(QThread::currentThread() == thread());
	steps_.clear();
	update_ = false;
}

void PointcloudParser::Update()
{
    Q_ASSERT(QThread::currentThread() == thread());
	update_ = false;

	if (steps_.size() >= static_cast<int>(min_steps_)){
		Sort();
		const unsigned stepCount = static_cast<unsigned>(steps_.size());
		assert(stepCount >= 2);
		assert(steps_.front().GetRelativePos() == 0.0);
		assert(steps_.back().GetRelativePos() == 1.0);
		if (steps_.front().GetRelativePos() != 0.0 || steps_.back().GetRelativePos() != 1.0)
		{
			LOG_DEBUG("oops problem occured");
		}
		else
		{
			unsigned j = 0; //current interval
			for (unsigned i = 0; i < max_steps_; ++i)
			{
				const double relativePos = static_cast<double>(i) / (max_steps_ - 1);

				//forward to the right interval
				while (j + 2 < stepCount && steps_[j + 1].GetRelativePos() < relativePos)
					++j;

				// linear interpolation
				const CCVector3d colBefore(steps_[j].GetColor().redF(),
					steps_[j].GetColor().greenF(),
					steps_[j].GetColor().blueF());

				const CCVector3d colNext(steps_[j + 1].GetColor().redF(),
					steps_[j + 1].GetColor().greenF(),
					steps_[j + 1].GetColor().blueF());

				//interpolation coef
				const double alpha = (relativePos - steps_[j].GetRelativePos()) /
					(steps_[j + 1].GetRelativePos() - steps_[j].GetRelativePos());

				const CCVector3d interpCol = colBefore + (colNext - colBefore) * alpha;

				rgbscale_[i] = Rgb(static_cast<ColorCompType>(interpCol.x * MAX),
					static_cast<ColorCompType>(interpCol.y * MAX),
					static_cast<ColorCompType>(interpCol.z * MAX));
			}

			update_ = true;
		}
	}
	else
	{

	}

	if (!update_)
	{
		//I saw an invalid scale and I want it painted black ;)
		for (unsigned i = 0; i < max_steps_; ++i)
		{
			rgbscale_[i] = black;
		}
	}
    for (auto* processor : processors_) {
        processor->SetColorScale(rgbscale_);
    }
}

void PointcloudParser::Sort()
{
    Q_ASSERT(QThread::currentThread() == thread());
	std::sort(steps_.begin(), steps_.end(), ColorScaleElement::IsSmaller);
}

void PointcloudParser::StartPlayback()
{
	is_playing_ = true;
	playback_thread_ = std::thread(&PointcloudParser::PlaybackThreadFunc, this);
	pointcloud_file_manager_->StartPlayback();
}

void PointcloudParser::GotoStartPlayback(const std::string& filename, bool paused)
{
	ResetPlayback(paused);
	pointcloud_file_manager_->ResetReading();
	StartLoadYscFile(filename);
	StartPlayback();
}

void PointcloudParser::SetPlaybackPaused(bool paused)
{
	is_playing_paused_ = paused;
	pointcloud_file_manager_->SetPlaybackPaused(paused);
}

void PointcloudParser::StopPlayback()
{
	is_playing_ = false;
	is_playing_paused_ = false;
	pointcloud_file_manager_->StopPlayback();
}

void PointcloudParser::SetPlaybackSpeed(double speed)
{
	speed_ = speed;
}

void PointcloudParser::SeekPosition(int frame_number)
{
	if (is_playing_) {
		// 清空当前队列
		while (!playing_frame_queue_.empty()) {
			playing_frame_queue_.pop();
		}
		// 重新读取文件并填充队列
		//std::thread read_thread(&PointcloudFileManager::ReadThreadFunc, &pointcloud_file_manager_, current_read_file_, std::ref(playing_frame_queue_), frame_number);
		//read_thread.detach();
	}
}

void PointcloudParser::SeekToEnd()
{
	is_playing_ = false;
	is_playing_paused_ = false;
	if (parse_thread_.joinable()) {
		parse_thread_.join();
	}
	while (!playing_frame_queue_.empty()) {
		playing_frame_queue_.pop();
	}
	// 读取文件最后一帧并处理
	std::shared_ptr<CloudFrame<CartesianPoint>> last_frame = 
		pointcloud_file_manager_->ReadLastFrame();
	if (last_frame) {
		DealFrame(last_frame);
	}
}

void PointcloudParser::SeekOperate(uint32_t operate_type, uint64_t operate_value)
{
	if (is_playing_paused_) {
		is_playing_paused_ = false;
	}
	pointcloud_file_manager_->PlayOperate(operate_type, operate_value);
}

void PointcloudParser::PlaybackThreadFunc()
{
	while (is_playing_) {
		if (!is_playing_paused_) {
			std::optional<CloudFrameVariant> optional_frame = playing_frame_queue_.pop();
			if (optional_frame.has_value()) {
				CloudFrameVariant data = optional_frame.value();
				if (std::holds_alternative<CloudFramePtr>(data)) {
					CloudFramePtr frame = std::get<CloudFramePtr>(data);
					emit SliderUpdateSig(frame->frame_number);
					DealFrame(frame);
				}
				else if (std::holds_alternative<CloudFrameRecordEnd>(data)) {
					// 接收到结束信号,发送结束信号退出循环
					emit FilePlaybackEnd();
					is_playing_ = false;
				}
			}
			else {
				is_playing_ = false;
				// 处理队列为空的情况
			}
		}
		std::this_thread::sleep_for(std::chrono::milliseconds((int)(100 / speed_))); // 10fps
	}
}

void PointcloudParser::CheckSavePointCloud()
{
	if (std::get<0>(save_point_frames_) != 0) {
		std::get<0>(save_point_frames_) --;
		if (std::get<0>(save_point_frames_) == 0) {
			std::get<1>(save_point_frames_) = "";
		}
	}
}

bool ys_scene::PointcloudParser::IsNextFrame(size_t frame_number)
{
	int diff = static_cast<int>(frame_number) - static_cast<int>(next_frame_to_process_);
	if (std::abs(diff) > max_frame_id / 2) {
		return diff < 0;
	}
	return diff == 0;
}

void PointcloudParser::DealFrame(CloudFramePtr frame)
{
	if (frame->slot_counts.empty()) {
		DealFlatFrame(frame);
		return;
	}

	//LOG_INFO("show frame id is:{0}", frame->frame_number);
	QThreadPool::globalInstance()->start(
		new PointcloudTask(std::make_shared<std::vector<CartesianPoint>>(frame->data),
			std::make_shared<std::vector<int>>(frame->slot_counts),
			processors_[balance_count_ % cal_],
			save_point_frames_,
			frame->frame_number));
	balance_count_++;
	CheckSavePointCloud();
}

void PointcloudParser::DealFlatFrame(CloudFramePtr frame)
{
	QThreadPool::globalInstance()->start(
		new PointcloudTaskFlat(std::make_shared<std::vector<CartesianPoint>>(frame->data),
			processors_[balance_count_ % cal_],
			save_point_frames_,
			frame->frame_number));
	balance_count_++;
	CheckSavePointCloud();
}

void PointcloudParser::ResetPlayback(bool paused)
{
	is_playing_ = false;
	is_playing_paused_ = paused;
	if (playback_thread_.joinable()) {
		playback_thread_.join();
	}
	playing_frame_queue_.clear();
}

void PointcloudParser::StartLoadYscFile(const std::string& filename)
{
	pointcloud_file_manager_->StartLoadYscFile(filename, playing_frame_queue_);
}

void PointcloudParser::SetOptionalSlotNumber(int slot_number)
{
    std::lock_guard<std::mutex> lock(mutex_);
    OptionalSlotNumber optional_slot_number_switch = static_cast<OptionalSlotNumber>(slot_number);
    switch (optional_slot_number_switch) {
    case OptionalSlotNumber::KSlot200:
        optinal_slot_number_ = 200;
        expected_received_times_ = 199;
        UpdateFrameExpectation();
        break;
    case OptionalSlotNumber::KSlot400:
        optinal_slot_number_ = 400;
        expected_received_times_ = 399;
        UpdateFrameExpectation();
        break;
    case OptionalSlotNumber::KSlot600:
        optinal_slot_number_ = 600;
        expected_received_times_ = 599;
        UpdateFrameExpectation();
        break;
    case OptionalSlotNumber::KSlot800:
        optinal_slot_number_ = 800;
        expected_received_times_ = 799;
        UpdateFrameExpectation();
        break;
    case OptionalSlotNumber::KSlot1000:
        optinal_slot_number_ = 1000;
        expected_received_times_ = 999;
        UpdateFrameExpectation();
        break;
    default:
        break;
    }
}

size_t PointcloudParser::ExpectedFramePoints() const
{
    return frame_policy::ExpectedFramePoints(points_per_slot_, expected_received_times_ + 1);
}

void PointcloudParser::UpdateFrameExpectation()
{
    // 期望 SLOT 数取配置值(expected_received_times_ = 配置值 - 1)：alpha/beta(96 点/SLOT)下
    // 结果与既有 N×96×0.92 一致，gamma(120 点/SLOT)自动得到 800×120×0.92 = 88320
    min_available_data_size_ = frame_policy::MinAvailableDataSize(points_per_slot_, expected_received_times_ + 1);
}

void PointcloudParser::SetPaused(bool paused)
{
    paused_.store(paused, std::memory_order_relaxed);
}

void PointcloudParser::Mf150PointcloudParse()
{
    while (true) {
        std::unique_lock<std::mutex> lock(mf150_mutex_);
        mf150_cv_.wait(lock, [this] {
            return mf150_stop_.load() || !mf150_complete_frames_.empty();
        });

        if (mf150_stop_) {
            break;
        }

        while (!mf150_complete_frames_.empty()) {
            auto frame = mf150_complete_frames_.front();
            mf150_complete_frames_.pop();
            lock.unlock();

            pointcloud_file_manager_->WriteFrame(frame);
            DealFrame(frame);

            lock.lock();
        }
    }
}

}// namespace ys_scene

void ys_scene::PointcloudParser::ResetManagementSession()
{
    std::scoped_lock lock(mutex_, mf150_mutex_);
    first_frame_.reset();
    second_frame_.reset();
    next_frame_to_process_ = 0;
    incomplete_frames_.clear();
    complete_frames_ = decltype(complete_frames_){};
    mf150_complete_frames_ = {};
}
