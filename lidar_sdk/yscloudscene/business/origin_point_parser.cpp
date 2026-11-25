#include "origin_point_parser.h"
#include "origin_point_parser_imp.h"
#include "log/logging.h"
#include "tiny_hut/common_toolset.h" 

#include <QThreadPool>
#ifdef _WIN32
#include<winsock2.h>
#pragma comment(lib, "Ws2_32.lib")
#endif // WIN32

namespace ys_scene {

OriginPointParser::OriginPointParser(QObject* parent):
    QObject{ parent },
	data_ready_ { false },
	stop_{ false },
    save_origin_point_{ false },
    balance_count_{0}
{
    for (int i = 0; i < processor_size_; ++i) {
        processors_.push_back(new OriginPointParseImp(this));
    }
	Start();
}

OriginPointParser::~OriginPointParser()
{
    Stop();
    for (auto processor : processors_) {
        delete processor;
    }
}

void OriginPointParser::Start()
{
	//parse_thread_ = std::thread(&OriginPointParser::OriginPointParse, this);
	parse_thread_ = std::thread(&OriginPointParser::OriginPointParse2, this);
}

void OriginPointParser::Stop()
{
    {
        std::unique_lock<std::mutex> lock(mutex_);
        stop_ = true;
    }
    cv_.notify_all();
    parse_thread_.join();
}

void OriginPointParser::OriginPointReceive(void* src, uint32_t length)
{
    std::lock_guard<std::mutex> lock(mutex_);
    uint8_t* src_local = reinterpret_cast<uint8_t*>(src);
    auto slot_num = length / 3113;
    for (auto i = 0; i < slot_num; ++i) {
        OriginPointData* origin_point = reinterpret_cast<OriginPointData*>(src_local + i * 3113);
        data_.insert(data_.end(), origin_point->data, origin_point->data + 3104);
        if (origin_point->slot_number == 799) {
            data_ready_ = true;
            cv_.notify_one();
            LOG_DEBUG("a complete frame received, frame_id {}", htons(origin_point->frame_id));
        }
    }
}

void OriginPointParser::OriginPointReceive2(const uint32_t handle, void* src, uint32_t length, void* context)
{
    if (paused_.load(std::memory_order_relaxed)) return;

    if (!src || length == 0) {
        return;
    }
    // Gamma 原始 SLOT 独立组帧，不套用 Beta/Alpha 像素布局、帧超时或坐标转换。
    if (length == CS96GammaRawParser::udp_packet_size) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto frame = cs96_gamma_assembler_.Feed(handle, static_cast<const uint8_t*>(src), length);
        if (frame) {
            // 保存线程跟不上时保持队列有界。
            if (complete_gamma_frames_.size() >= processor_size_) {
                complete_gamma_frames_.pop();
                LOG_WARN("CS96 Gamma raw frame queue full, dropping oldest frame");
            }
            complete_gamma_frames_.push(std::move(frame));
            cv_.notify_one();
        }
        return;
    }
    // 自动检测数据格式: MF150原始点云(1040B/pkt) vs CS96-beta(1561B/slot) vs 转镜式(3113B/slot)
    if (length % Mf150RawParser::udp_packet_size == 0) {
        mf150_detected_ = true;
        cs96beta_detected_ = false;
        OriginPointReceiveMf150(handle, src, length, context);
        return;
    }
    mf150_detected_ = false;

    size_t slot_size;
    if (length % CS96BetaRawParser::udp_packet_size == 0) {
        cs96beta_detected_ = true;
        slot_size = CS96BetaRawParser::udp_packet_size;
    } else {
        cs96beta_detected_ = false;
        slot_size = 3113;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    uint8_t* src_local = reinterpret_cast<uint8_t*>(src);
    auto slot_count = length / slot_size;
    for (auto i = 0; i < slot_count; ++i) {
        OriginPointData* origin_point = reinterpret_cast<OriginPointData*>(src_local + i * slot_size);
        if (!first_origin_frame_) {
            first_origin_frame_ = origin_point->frame_id;
        }else if(!second_origin_frame_ && origin_point->frame_id != *first_origin_frame_){
            second_origin_frame_ = origin_point->frame_id;
            next_frame_to_process_ = *second_origin_frame_;
            complete_origin_frames_ = std::priority_queue<std::shared_ptr<CloudFrame<uint8_t>>,
                std::vector<std::shared_ptr<CloudFrame<uint8_t>>>,
                CloudFrameComparator<uint8_t>>(*second_origin_frame_);
        }

        auto& origin_frame = incomplete_origin_frames_[origin_point->frame_id];
        //auto frame_id_i = htons(origin_point->frame_id);
		auto frame_id_i = origin_point->frame_id;
        if (!origin_frame) {
            size_t frame_size = cs96beta_detected_ ? CS96BetaRawParser::frame_data_size : origincloud_frame_size;
            origin_frame = std::make_shared<CloudFrame<uint8_t>>(frame_id_i, frame_size);
        }

        size_t data_len = cs96beta_detected_ ? origin_point->data_length : 3104;
        origin_frame->data.insert(origin_frame->data.end(), origin_point->data, origin_point->data + data_len);
        origin_frame->slot_counts.push_back(origin_point->slot_number);
        origin_frame->receive_times ++;
        origin_frame->last_update = std::chrono::steady_clock::now();

        bool frame_complete;
        if (cs96beta_detected_) {
            frame_complete = origin_frame->receive_times >= CS96BetaRawParser::slot_counts;
        } else {
            frame_complete = origin_frame->data.size() == origincloud_frame_size;
        }

        if (frame_complete) {
            if (second_origin_frame_) {
                complete_origin_frames_.push(origin_frame);
                cv_.notify_one();
            }
            //LOG_DEBUG("a complete frame received, frame_id {0}", frame_id_i);
            incomplete_origin_frames_.erase(origin_point->frame_id);
        }
    }
}

void OriginPointParser::OriginPointReceiveMf150(const uint32_t handle, void* src, uint32_t length, void* context)
{
    if (paused_.load(std::memory_order_relaxed)) {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    uint8_t* src_local = reinterpret_cast<uint8_t*>(src);
    auto packet_count = length / Mf150RawParser::udp_packet_size;

    for (auto i = 0u; i < packet_count; ++i) {
        uint8_t* packet = src_local + i * Mf150RawParser::udp_packet_size;

        uint16_t frame_id = static_cast<uint16_t>(packet[1]) | (static_cast<uint16_t>(packet[2]) << 8);

        if (!first_origin_frame_) {
            first_origin_frame_ = frame_id;
        } else if (!second_origin_frame_ && frame_id != *first_origin_frame_) {
            second_origin_frame_ = frame_id;
            next_frame_to_process_ = *second_origin_frame_;
            complete_origin_frames_ = std::priority_queue<OriginFramePtr,
                std::vector<OriginFramePtr>,
                CloudFrameComparator<uint8_t>>(*second_origin_frame_);
        }

        auto& origin_frame = incomplete_origin_frames_[frame_id];
        if (!origin_frame) {
            origin_frame = std::make_shared<CloudFrame<uint8_t>>(frame_id, Mf150RawParser::frame_data_size);
        }

        origin_frame->data.insert(origin_frame->data.end(),
            packet + Mf150RawParser::packet_header_size,
            packet + Mf150RawParser::udp_packet_size);
        origin_frame->receive_times++;
        origin_frame->last_update = std::chrono::steady_clock::now();

        if (origin_frame->data.size() >= Mf150RawParser::frame_data_size) {
            if (second_origin_frame_) {
                complete_origin_frames_.push(origin_frame);
                cv_.notify_one();
            } else {
                LOG_INFO("MF150 frame #{0} complete but second_origin not set, DISCARDED", frame_id);
            }
            incomplete_origin_frames_.erase(frame_id);
        }
    }
}

void OriginPointParser::OriginPointParse()
{
    while (true) {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] { return data_ready_; });
        if (save_origin_point_) {
            std::shared_ptr < std::vector<uint8_t>> shared_data =
                std::make_shared<std::vector<uint8_t>>(std::move(data_));
            /*QThreadPool::globalInstance()->start(
                new OriginPointTask(shared_data, processors_[balance_count_ % cal_]));*/
            save_origin_point_ = false;
        }
        data_.clear();
        data_ready_ = false;
        balance_count_++;
        if (stop_) {
            break;
        }
    }
}

void OriginPointParser::OriginPointParse2()
{
    while (true) {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] { 
            return stop_ || !complete_gamma_frames_.empty() ||
                (!complete_origin_frames_.empty() && second_origin_frame_);
        });

        if (stop_) {
            break;
        }

        if (!complete_gamma_frames_.empty()) {
            auto frame = complete_gamma_frames_.front();
            complete_gamma_frames_.pop();
            lock.unlock();
            DealCs96GammaOriginFrame(std::move(frame));
            continue;
        }

        auto now = std::chrono::steady_clock::now();

        // 检查并处理超时的不完整帧
        for (auto it = incomplete_origin_frames_.begin(); it != incomplete_origin_frames_.end();) {
            if (now - it->second->last_update >= frame_timeout_) {
                if (it->second->data.size() >= origincloud_frame_size - 3104 * 8 &&
                    second_origin_frame_ && it->first != *first_origin_frame_) {
                    complete_origin_frames_.push(it->second);
                    //LOG_INFO("frame:{0} will be calc", htons(it->first));
                    LOG_INFO("frame:{0} will be calc", it->first);
                }
                it = incomplete_origin_frames_.erase(it);
            }
            else {
                ++it;
            }
        }
        while (!complete_origin_frames_.empty() &&
            (complete_origin_frames_.top()->frame_number == next_frame_to_process_ ||
                complete_origin_frames_.top()->data.size() >= min_available_data_size_)) {
            auto frame = complete_origin_frames_.top();
            complete_origin_frames_.pop();
            lock.unlock();

            if (frame->data.size() == Mf150RawParser::frame_data_size) {
                DealMf150OriginFrame(frame);
            } else {
                DealOriginFrame(frame);
            }

            if (frame->frame_number == next_frame_to_process_) {
                next_frame_to_process_ = (next_frame_to_process_ + 1) % max_frame_id;
            }
            lock.lock();
        }
        if (!complete_origin_frames_.empty()) {
            auto top = complete_origin_frames_.top();
            size_t qs = complete_origin_frames_.size();
            size_t last_qs = last_blocked_origin_qs_.exchange(qs);
            if (last_qs == 0 || qs - last_qs >= 20) {
                LOG_INFO("OriginPointParse2 blocked: queue_size={0} top_frame={1} next_expected={2} data_size={3}",
                    qs, top->frame_number,
                    next_frame_to_process_.load(), top->data.size());
            }
        } else {
            last_blocked_origin_qs_.store(0);
        }
        if (stop_) {
            return;
        }
    }
}

void OriginPointParser::SetSaveOriginPoint(std::tuple<int, int, std::string> save_origin)
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto default_frames = ToInt(PointCloudParser::default_save_frame_count);
    // !if a save operation is in progress, return
    if (std::get<0>(save_origin_frames_) != 0) {
        return;
    }

    //if (std::get<0>(save_origin_frames_) > default_frames) {
        std::get<0>(save_origin_frames_) = std::get<0>(save_origin);
        need_save_count_ = std::get<0>(save_origin);
        left_frame_saving_ = std::get<0>(save_origin);
        std::get<1>(save_origin_frames_) = std::get<1>(save_origin);
        std::get<2>(save_origin_frames_) = std::get<2>(save_origin);
        std::get<3>(save_origin_frames_) = std::get<2>(save_origin) + "_" +
            toolset::GetCurrentTimestampAsString();
    //}
}

void OriginPointParser::SetSourceNode(const LidarInfo& node)
{
    std::lock_guard<std::mutex> lock(mutex_);
    cs96_gamma_assembler_.Reset(node);
    complete_gamma_frames_ = {};
}

void OriginPointParser::ClearSourceNode()
{
    std::lock_guard<std::mutex> lock(mutex_);
    cs96_gamma_assembler_.Reset();
    complete_gamma_frames_ = {};
}

void OriginPointParser::QueryPsCalibrationParamsT(const uint32_t handle)
{
	QueryPsCalibrationParams(handle, &OriginPointParser::UpdatePsCalibrationParamsTrampoline, this);
}

std::vector<OriginPointParseImp*> OriginPointParser::GetProcessors()
{
    return processors_;
}

void OriginPointParser::OriginPointReceiveTrampoline(const uint32_t handle, void* src, uint32_t length, void* context)
{
	auto* self = static_cast<OriginPointParser*>(context);
	if (self) {
		self->OriginPointReceive2(handle, src, length, context);
	}
}

void OriginPointParser::SetPointcloudParser(PointcloudParser* parser)
{
    for (auto processor : processors_) {
        processor->SetPointcloudParser(parser);
    }
}

void OriginPointParser::SetAlgorithmRepairGroundEnable(int state)
{
    for (auto processor : processors_) {
        processor->SetAlgorithmRepairGroundEnable(state);
    }
}

void OriginPointParser::SetMf150PreviewConfig(bool enabled, int echo_index)
{
    mf150_preview_enabled_ = enabled;
    for (auto processor : processors_) {
        QMetaObject::invokeMethod(processor, "SetMf150PreviewConfig",
            Qt::QueuedConnection,
            Q_ARG(bool, enabled),
            Q_ARG(int, echo_index));
    }
}

void OriginPointParser::SetMf150CalibMap(const std::vector<std::vector<float>>& calib_map)
{
    for (auto processor : processors_) {
        QMetaObject::invokeMethod(processor, "SetMf150CalibMap",
            Qt::QueuedConnection,
            Q_ARG(std::vector<std::vector<float>>, calib_map));
    }
}

void OriginPointParser::SetMf150CalibEnabled(bool enabled)
{
    for (auto processor : processors_) {
        QMetaObject::invokeMethod(processor, "SetMf150CalibEnabled",
            Qt::QueuedConnection,
            Q_ARG(bool, enabled));
    }
}

void OriginPointParser::SetMf150MultiCalibTable(std::shared_ptr<SslDistCalibMulti> table)
{
    for (auto processor : processors_) {
        QMetaObject::invokeMethod(processor, "SetMf150MultiCalibTable",
            Qt::QueuedConnection,
            Q_ARG(std::shared_ptr<SslDistCalibMulti>, table));
    }
}

void OriginPointParser::SetMf150MultiCalibEnabled(bool enabled)
{
    for (auto processor : processors_) {
        QMetaObject::invokeMethod(processor, "SetMf150MultiCalibEnabled",
            Qt::QueuedConnection,
            Q_ARG(bool, enabled));
    }
}

void OriginPointParser::SetCs96BetaCalibParams(std::shared_ptr<Cs96BetaCalibParams> params)
{
    for (auto processor : processors_) {
        QMetaObject::invokeMethod(processor, "UpdateCs96BetaCalibParams",
            Qt::QueuedConnection,
            Q_ARG(std::shared_ptr<Cs96BetaCalibParams>, params));
    }
}

void OriginPointParser::CheckSaveOrigin()
{
    if (std::get<0>(save_origin_frames_) != 0) {
        std::get<0>(save_origin_frames_)--;
        if (std::get<0>(save_origin_frames_) == 0) {
            std::get<1>(save_origin_frames_) = 0;
            std::get<2>(save_origin_frames_) = "";
            std::get<3>(save_origin_frames_) = "";
        }
    }
}

void OriginPointParser::DealCs96GammaOriginFrame(OriginFramePtr origin_frame)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (std::get<0>(save_origin_frames_) <= 0) {
        return;
    }
    const int face = std::get<1>(save_origin_frames_);
    const int frame_face = origin_frame->data[14] % 4;
    if (face != 0 && face - 1 != frame_face) {
        return;
    }
    QThreadPool::globalInstance()->start(
        new OriginPointTask(origin_frame, processors_[balance_count_ % cal_],
            save_origin_frames_, OriginPointOperation::KOriginPointSave));
    ++balance_count_;
    CheckSaveOrigin();
}

void OriginPointParser::DealOriginFrame(OriginFramePtr origin_frame)
{
    OriginPointOperation operation_type = OriginPointOperation::KOriginPointTrans;

    if (std::get<0>(save_origin_frames_) > 0) {
        int face = std::get<1>(save_origin_frames_);
        bool should_save = false;

        if (face == 0) {
            should_save = true;
        }
        else {
            int frame_mod = origin_frame->frame_number % you_see::ToInt(you_see::PrismFaces::four_sided_mirror);
            should_save = (face - 1 == frame_mod);
        }

        if (should_save) {
            operation_type = OriginPointOperation::KOriginPointBothSaveTrans;
        }
    }

    bool is_cs96_beta = (origin_frame->data.size() == CS96BetaRawParser::frame_data_size ||
        origin_frame->data.size() == CS96Beta600RawParser::frame_data_size);

    QThreadPool::globalInstance()->start(
        new OriginPointTask(origin_frame,
            processors_[balance_count_ % cal_],
            save_origin_frames_,
            operation_type,
            false,
            is_cs96_beta));

    balance_count_++;
    CheckSaveOrigin();
}

void OriginPointParser::DealMf150OriginFrame(OriginFramePtr origin_frame)
{
    last_mf150_frame_ = origin_frame;

    bool saving = (std::get<0>(save_origin_frames_) > 0);

    if (!saving && !mf150_preview_enabled_) {
        LOG_INFO("MF150 frame #{0} DROPPED (preview disabled, not saving)", origin_frame->frame_number);
        return;
    }

    OriginPointOperation operation_type;
    if (saving && mf150_preview_enabled_) {
        operation_type = OriginPointOperation::KOriginPointBothSaveTrans;
    } else if (saving) {
        operation_type = OriginPointOperation::KOriginPointSave;
    } else {
        operation_type = OriginPointOperation::KOriginPointTrans;
    }

    QThreadPool::globalInstance()->start(
        new OriginPointTask(origin_frame,
            processors_[balance_count_ % cal_],
            save_origin_frames_,
            operation_type,
            true));

    balance_count_++;

    size_t n = mf150_dispatched_count_.fetch_add(1) + 1;
    if (n % 50 == 0) {
        LOG_INFO("MF150 dispatched {0} frames, latest=#{1}", n, origin_frame->frame_number);
    }

    if (saving) {
        CheckSaveOrigin();
    }
}

bool OriginPointParser::IsNextFrame(size_t frame_number)
{
    int diff = static_cast<int>(frame_number) - static_cast<int>(next_frame_to_process_);
    if (std::abs(diff) > max_frame_id / 2) {
        return diff < 0;
    }
    return diff == 0;
}

void OriginPointParser::UpdatePsCalibrationParamsTrampoline(const uint32_t handle, const PsCalibrationParamsQuery* ps_calibration_params, void* context)
{
	auto* self = static_cast<OriginPointParser*>(context);
	if (self) {
		self->UpdatePsCalibrationParams(handle, ps_calibration_params, context);
	}
}

void OriginPointParser::UpdatePsCalibrationParams(const uint32_t handle, const PsCalibrationParamsQuery* ps_calibration_params, void* context)
{
    if (!ps_calibration_params) {
        return;
    }

    static constexpr uint16_t kMaxArrayCount = 2000;

    // 先复制基本参数
    ps_calibration_params_.ret_code = ps_calibration_params->ret_code;
    ps_calibration_params_.lines_vertical = ps_calibration_params->lines_vertical;
    ps_calibration_params_.lines_horizontal = ps_calibration_params->lines_horizontal;
    ps_calibration_params_.omega_0 = ps_calibration_params->omega_0;
    ps_calibration_params_.theta_min = ps_calibration_params->theta_min;
    ps_calibration_params_.theta_max = ps_calibration_params->theta_max;
    ps_calibration_params_.theta_tilt = ps_calibration_params->theta_tilt;
    ps_calibration_params_.u = ps_calibration_params->u;
    ps_calibration_params_.sat_peak_max = ps_calibration_params->sat_peak_max;
    ps_calibration_params_.sat_peak_min = ps_calibration_params->sat_peak_min;
    ps_calibration_params_.dist_max = ps_calibration_params->dist_max;
    ps_calibration_params_.fov_max = ps_calibration_params->fov_max;
    ps_calibration_params_.k = ps_calibration_params->k;

    // 动态数组的大小（做合法性校验，防止 MF150 等异物格式导致越界）
    auto safe_cnt = [](uint16_t raw) -> uint16_t {
        return (raw > 0 && raw < kMaxArrayCount) ? raw : 0;
    };

    ps_calibration_params_.side_angular_offset_cnt = safe_cnt(ps_calibration_params->side_angular_offset_cnt);
    ps_calibration_params_.standard_encoder_cnt = safe_cnt(ps_calibration_params->standard_encoder_cnt);
    ps_calibration_params_.pixel_in_vcsel_id_cnt = safe_cnt(ps_calibration_params->pixel_in_vcsel_id_cnt);
    ps_calibration_params_.int_ld_cnt = safe_cnt(ps_calibration_params->int_ld_cnt);
    ps_calibration_params_.omega_delta_cnt = safe_cnt(ps_calibration_params->omega_delta_cnt);
    ps_calibration_params_.d_delta_cnt = safe_cnt(ps_calibration_params->d_delta_cnt);
    ps_calibration_params_.a_cnt = safe_cnt(ps_calibration_params->a_cnt);

    // 解析动态数组数据
    uint8_t* data_ptr = const_cast<uint8_t*>(ps_calibration_params->data);
    size_t offset = 0;

    // 解析 side_angular_offset 数组
    ps_calibration_params_.side_angular_offset.resize(ps_calibration_params_.side_angular_offset_cnt);
    if (ps_calibration_params_.side_angular_offset_cnt > 0) {
        memcpy(ps_calibration_params_.side_angular_offset.data(), data_ptr + offset,
            ps_calibration_params_.side_angular_offset_cnt * sizeof(uint16_t));
        offset += ps_calibration_params_.side_angular_offset_cnt * sizeof(uint16_t);
    }

    // 解析 angle_face 数组
    ps_calibration_params_.angle_face.resize(ps_calibration_params_.standard_encoder_cnt);
    if (ps_calibration_params_.standard_encoder_cnt > 0) {
        memcpy(ps_calibration_params_.angle_face.data(), data_ptr + offset,
            ps_calibration_params_.standard_encoder_cnt * sizeof(uint16_t));
        offset += ps_calibration_params_.standard_encoder_cnt * sizeof(uint16_t);
    }

    // 解析 pixel_in_vcsel_id 数组
    ps_calibration_params_.pixel_in_vcsel_id.resize(ps_calibration_params_.pixel_in_vcsel_id_cnt);
    if (ps_calibration_params_.pixel_in_vcsel_id_cnt > 0) {
        memcpy(ps_calibration_params_.pixel_in_vcsel_id.data(), data_ptr + offset,
            ps_calibration_params_.pixel_in_vcsel_id_cnt * sizeof(uint16_t));
        offset += ps_calibration_params_.pixel_in_vcsel_id_cnt * sizeof(uint16_t);
    }

    // 解析 int_ld 数组
    ps_calibration_params_.int_ld.resize(ps_calibration_params_.int_ld_cnt);
    if (ps_calibration_params_.int_ld_cnt > 0) {
        memcpy(ps_calibration_params_.int_ld.data(), data_ptr + offset,
            ps_calibration_params_.int_ld_cnt * sizeof(uint32_t));
        offset += ps_calibration_params_.int_ld_cnt * sizeof(uint32_t);
    }

    // 解析 omega_delta 数组
    ps_calibration_params_.omega_delta.resize(ps_calibration_params_.omega_delta_cnt);
    if (ps_calibration_params_.omega_delta_cnt > 0) {
        memcpy(ps_calibration_params_.omega_delta.data(), data_ptr + offset,
            ps_calibration_params_.omega_delta_cnt * sizeof(uint32_t));
        offset += ps_calibration_params_.omega_delta_cnt * sizeof(uint32_t);
    }

    // 解析 d_delta 数组
    ps_calibration_params_.d_delta.resize(ps_calibration_params_.d_delta_cnt);
    if (ps_calibration_params_.d_delta_cnt > 0) {
        memcpy(ps_calibration_params_.d_delta.data(), data_ptr + offset,
            ps_calibration_params_.d_delta_cnt * sizeof(uint32_t));
        offset += ps_calibration_params_.d_delta_cnt * sizeof(uint32_t);
    }

    // 解析 a 数组
    ps_calibration_params_.a.resize(ps_calibration_params_.a_cnt);
    if (ps_calibration_params_.a_cnt > 0) {
        memcpy(ps_calibration_params_.a.data(), data_ptr + offset,
            ps_calibration_params_.a_cnt * sizeof(uint32_t));
    }
    DistributeCalibrationParamsToProcessors();
}

void OriginPointParser::DistributeCalibrationParamsToProcessors()
{
    for (auto processor : processors_) {
        QMetaObject::invokeMethod(processor, "UpdateCalibrationParams",
            Qt::QueuedConnection,
            Q_ARG(PsCalibrationParamsHoldT, ps_calibration_params_));
    }
}

void OriginPointParser::SetPaused(bool paused)
{
    paused_.store(paused, std::memory_order_relaxed);
}

}// namespace ys_scene

void ys_scene::OriginPointParser::ResetManagementSession()
{
    std::lock_guard<std::mutex> lock(mutex_);
    first_origin_frame_.reset();
    second_origin_frame_.reset();
    next_frame_to_process_ = 0;
    incomplete_origin_frames_.clear();
    complete_origin_frames_ = decltype(complete_origin_frames_){};
    complete_gamma_frames_ = {};
    cs96_gamma_assembler_.Reset();
}
