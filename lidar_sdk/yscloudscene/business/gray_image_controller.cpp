#include "gray_image_controller.h"

#include <QDateTime>
#include <QDir>
#include <QMetaObject>

namespace ys_scene {
GrayImageController::GrayImageController(QObject *parent)
    : QObject(parent),
      record_thread_(&GrayImageController::RecordLoop, this)
{
}

GrayImageController::~GrayImageController()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true;
    }
    record_cv_.notify_all();
    record_thread_.join();
}

void GrayImageController::BeginSession(uint32_t device_ip, uint64_t session_id)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (device_ip_ == device_ip && session_id_ == session_id) {
            return;
        }
        device_ip_ = device_ip;
        session_id_ = session_id;
        pending_frame_.reset();
        record_remaining_ = 0;
        recording_overflow_ = false;
    }
    displayed_frame_.reset();
    paused_ = false;
    mapper_.Reset();
    emit ImageReadySig({});
    emit FrameChangedSig();
}

void GrayImageController::EndSession()
{
    BeginSession(0, 0);
}

void GrayImageController::OfferFrame(you_see::GrayEchoFramePtr frame)
{
    if (!frame || !frame->IsComplete()) {
        return;
    }
    bool notify = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stopping_ || session_id_ == 0 || frame->device_ip != device_ip_ || frame->session_id != session_id_) {
            return;
        }
        pending_frame_ = frame;
        if (record_remaining_ > 0) {
            if (record_jobs_.size() >= 32) {
                record_remaining_ = 0;
                recording_overflow_ = true;
            }
            else {
                const QString name =
                    QString("%1_%2_frame_%3.csv").arg(record_prefix_).arg(++record_index_).arg(frame->frame_id);
                record_jobs_.push_back({frame, QDir(record_directory_).filePath(name), options_});
                --record_remaining_;
                record_cv_.notify_one();
            }
        }
        if (!notification_pending_) {
            notification_pending_ = true;
            notify = true;
        }
    }
    if (notify) {
        QMetaObject::invokeMethod(this, [this]() { DrainLatestFrame(); }, Qt::QueuedConnection);
    }
}

void GrayImageController::DrainLatestFrame()
{
    you_see::GrayEchoFramePtr frame;
    bool overflow = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        frame = std::move(pending_frame_);
        notification_pending_ = false;
        overflow = recording_overflow_;
        recording_overflow_ = false;
    }
    if (overflow) {
        emit RecordingOverflowSig();
    }
    if (!frame || paused_) {
        return;
    }
    if (displayed_frame_) {
        const uint16_t distance = static_cast<uint16_t>(frame->frame_id - displayed_frame_->frame_id);
        if (distance == 0 || distance >= 0x8000) {
            return;
        }
    }
    displayed_frame_ = std::move(frame);
    Render();
}

void GrayImageController::SetOptions(const GrayDisplayOptions &options)
{
    if (options.black_level >= options.white_level ||
        options.white_level > you_see::GetGrayGeometry(you_see::GrayFormat::KBeta).max_intensity) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        options_ = options;
    }
    Render();
}

void GrayImageController::SetPaused(bool paused)
{
    paused_ = paused;
    if (paused) {
        std::lock_guard<std::mutex> lock(mutex_);
        pending_frame_.reset();
        record_remaining_ = 0;
    }
}

void GrayImageController::Render()
{
    if (!displayed_frame_) {
        return;
    }
    const auto intensity = ExtractGrayIntensity(*displayed_frame_, options_.echo);
    emit ImageReadySig(OrientGrayImage(mapper_.Map(intensity, options_), options_.orientation));
    emit FrameChangedSig();
}

bool GrayImageController::SaveRawFrame(const QString &path, QString *error) const
{
    if (!displayed_frame_) {
        if (error) {
            *error = tr("尚未收到完整灰度帧");
        }
        return false;
    }
    return SaveGrayFrameCsv(*displayed_frame_, path, options_, error);
}

bool GrayImageController::StartRecording(int count, const QString &directory)
{
    if (count <= 0 || directory.isEmpty() || !QDir().mkpath(directory)) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (session_id_ == 0 || record_remaining_ > 0 || !record_jobs_.empty()) {
        return false;
    }
    record_directory_ = directory;
    record_prefix_ = QString("gray_%1_session_%2_%3_batch_%4")
                         .arg(device_ip_)
                         .arg(session_id_)
                         .arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss_zzz"))
                         .arg(++record_batch_id_);
    record_remaining_ = count;
    record_index_ = 0;
    return true;
}

void GrayImageController::StopRecording()
{
    std::lock_guard<std::mutex> lock(mutex_);
    record_remaining_ = 0;
}

void GrayImageController::RecordLoop()
{
    for (;;) {
        RecordJob job;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            record_cv_.wait(lock, [this]() { return stopping_ || !record_jobs_.empty(); });
            if (stopping_ && record_jobs_.empty()) {
                return;
            }
            job = std::move(record_jobs_.front());
            record_jobs_.pop_front();
        }
        QString error;
        if (!SaveGrayFrameCsv(*job.frame, job.path, job.options, &error)) {
            std::lock_guard<std::mutex> lock(mutex_);
            error += tr("；连续保存已停止，未保存 %1 帧").arg(record_jobs_.size() + record_remaining_ + 1);
            record_remaining_ = 0;
            record_jobs_.clear();
        }
        emit RecordingResultSig(job.path, error);
    }
}

you_see::GrayEchoFramePtr GrayImageController::GetDisplayedFrame() const
{
    return displayed_frame_;
}

const GrayDisplayOptions &GrayImageController::GetOptions() const
{
    return options_;
}
} // namespace ys_scene
