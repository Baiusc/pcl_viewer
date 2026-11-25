#ifndef YS_SCENE_PLAYBACKCONTROL_H_
#define YS_SCENE_PLAYBACKCONTROL_H_

#include "component/custom_slider.h"
#include "ys_scene_def.h"
#include "ysc_file.h"
#include "ys_lidar_validate_type.h"
#include <QWidget>
#include <QToolButton>
#include <QLabel>
#include <QHBoxLayout>
#include <QComboBox>
#include <QListWidget>
#include <QLabel>
#include <QGroupBox>
#include <QTimer>

namespace ys_scene {

class PlaybackControl : public QWidget
{
    Q_OBJECT

public:
    explicit PlaybackControl(QWidget* parent = nullptr);
    void GetCurrentYScHeader();

public:
    void SetRecordInterval(int ms);
    void SetLinesType(int type);
    void StartTimedRecording();
    void StopTimedRecording();

signals:
    void RecordingStarted();
    void RecordingPaused(bool pause);
    void RecordingStopped();
    void PlaybackStarted();
    void PlaybackPaused(bool pause);
    void PlaybackStopped();
    void GotoPlaybackStarted(const std::string& filename, bool paused);
    void RecordClicked(bool checked);
    void LoadYscFileHeader(const std::string& filename);
    void PlayClicked();
    void PauseClicked();
    void SeekPositionChanged(qint64 position);
    void SeekOperationSig(uint32_t operate_type, uint64_t operate_value);
    void SpeedChanged(double speed);
    void Resized();
    void PointcloudAidIntervalShow();

public slots:
    void SetDuration(qint64 duration);
    void SetPosition(int position);
    void OnSetSliderTime(ReplayFileHeader header);
    void PlayNextFile();
    
private slots:
    void OnPlayClicked();
    void OnSliderValueChanged();
    void OnSpeedChanged(int index);
    void OnRecordClicked();
    void OnRecordStopButtonClicked();
    void OnGotoEndClicked();
    void OnGotoStartClicked();
    void OnFastRewindClicked();
    void OnFastForwardClicked();
    void OnSpeedChanged(double speed);
    void OnFileSelected(QListWidgetItem* item);
    void ShowFileList();
    void UpdateLayout();

    void OnInitTimeout();
    void OnRecordTimeout();

private:
    void CreateControls();
    void CreateLayout();
    void CreateButton(QToolButton** button, const QString icon_path);
    void UpdateTimeLabel(qint64 position);
    void LoadYscFiles();
    void PlayShot(const std::string & filename);

    void StartNextRecording();

protected:
    void resizeEvent(QResizeEvent* event) override;

private:
    QToolButton* record_setting_button_;
    QToolButton* record_button_;
    QToolButton* record_stop_button_;
    QToolButton* goto_start_button_;
    QToolButton* fast_rewind_button_;
    QToolButton* play_button_;
    QToolButton* fast_forward_button_;
    QToolButton* goto_end_button_;
    QComboBox* speed_combo_;
    QLabel* time_label_;
    CustomSlider* slider_;
    qint64 duration_;
    //QListWidget* file_list_;
    QStringList file_list_;
    std::string file_directory_;
    QLabel *current_file_label_;
    QToolButton* file_list_button_;
    YscStatusA ysc_control_status_;
    bool switch_play_file_;
    bool auto_loop_playback_;
    QHBoxLayout* buttonLayout_;

    // Interval Recording
    QTimer* init_timer_;
    QTimer* interval_record_timer_;
    bool is_interval_recording_;
    int interval_;
    PointcloudRecordType record_line_mode_;
};

} //namespace ys_scene 


#endif // YS_SCENE_PLAYBACKCONTROL_H_