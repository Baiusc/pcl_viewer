#ifndef YS_SCENE_GRAY_IMAGE_CONTROLLER_H_
#define YS_SCENE_GRAY_IMAGE_CONTROLLER_H_

#include "gray_tone_mapper.h"

#include <QObject>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>

namespace ys_scene {
class GrayImageController : public QObject
{
    Q_OBJECT
  public:
    explicit GrayImageController(QObject *parent = nullptr);
    ~GrayImageController() override;
    void BeginSession(uint32_t device_ip, uint64_t session_id);
    void EndSession();
    // SDK worker 调用：显示只保留最新帧，连续记录使用独立有界队列。
    void OfferFrame(you_see::GrayEchoFramePtr frame);
    void SetOptions(const GrayDisplayOptions &options);
    void SetPaused(bool paused);
    bool SaveRawFrame(const QString &path, QString *error = nullptr) const;
    bool StartRecording(int count, const QString &directory);
    void StopRecording();
    you_see::GrayEchoFramePtr GetDisplayedFrame() const;
    const GrayDisplayOptions &GetOptions() const;

  signals:
    void ImageReadySig(const QImage &image);
    void FrameChangedSig();
    void RecordingResultSig(const QString &path, const QString &error);
    void RecordingOverflowSig();
  private:
    struct RecordJob
    {
        you_see::GrayEchoFramePtr frame;
        QString path;
        GrayDisplayOptions options;
    };

    void DrainLatestFrame();
    void Render();
    void RecordLoop();

    mutable std::mutex mutex_;
    uint32_t device_ip_{0};
    uint64_t session_id_{0};
    you_see::GrayEchoFramePtr pending_frame_;
    bool notification_pending_{false};
    bool recording_overflow_{false};
    int record_remaining_{0};
    uint64_t record_index_{0};
    uint64_t record_batch_id_{0};
    QString record_directory_;
    QString record_prefix_;
    std::deque<RecordJob> record_jobs_;
    std::condition_variable record_cv_;
    bool stopping_{false};
    std::thread record_thread_;
    bool paused_{false};
    you_see::GrayEchoFramePtr displayed_frame_;
    GrayDisplayOptions options_;
    GrayToneMapper mapper_;
};
} // namespace ys_scene

#endif // YS_SCENE_GRAY_IMAGE_CONTROLLER_H_
