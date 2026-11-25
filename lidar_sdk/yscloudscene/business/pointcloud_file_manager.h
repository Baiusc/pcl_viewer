#ifndef YS_SCENE_POINTCLOUD_FILE_MANAGER_H_
#define YS_SCENE_POINTCLOUD_FILE_MANAGER_H_

#include "ysc_file.h"
#include "ys_scene_def.h"
#include "ys_lidar_validate_type.h"
#include <QObject>
#include <thread>

namespace ys_scene {

class PointcloudFileManager : public QObject
{
	Q_OBJECT

public:
	PointcloudFileManager(QObject* parent = nullptr);
	~PointcloudFileManager();
    void ResetReading();
    void ResetNotify();
    CloudFramePtr ReadLastFrame();
    void PlayOperate(uint32_t operate_type, uint64_t operate_value);
   
public slots:
    void StartRecording();
    void SetPausing(bool pause);
    void StopRecording();
    void SetPointRecordType(int type);

    void StartLoadYscFile(const std::string& file_path, PointCloudFrameQueue& frame_queue);
    void StartPlayback();
    void SetPlaybackPaused(bool paused);
    void StopPlayback();

    void WriteFrame(CloudFramePtr points);
    bool ReadFrame(CloudFramePtr points);

signals:
    void SliderTime(ReplayFileHeader header);

private:
    void WriteThreadFunc();
    void ReadThreadFunc(const std::string& file_path, PointCloudFrameQueue& frame_queue);
    void CloudFrameLineProcess(CloudFramePtr& frame);

private:
    std::mutex ysc_file_mutex_;
    std::unique_ptr<YscFile> ysc_recording_file_;
    YscStatusA ysc_status_;
    PointCloudFrameQueue point_record_queue_;
    std::thread write_thread_;
    size_t frame_count_{ 0 };
    std::unique_ptr<YscFile> ysc_file_;
    
    // read deal
    std::thread read_thread_;
    std::mutex is_playing_mutex_;
    std::condition_variable is_playing_cond_;

    // 新增同步机制
    std::condition_variable stop_condition_;
    bool stop_signal_processed_ = false;

    // slot numbers
    you_see::PointcloudRecordType point_lines_type_;
};
} // namespace ys_scene

#endif // YS_SCENE_POINTCLOUD_FILE_MANAGER_H_