#include "playback_control.h"
#include "tiny_hut/common_toolset.h"
//#include "log/logging.h"
#include "config/config_service.h"
#include "toast/toast_tips.h"
#include "styleditem/center_editem_delegate.h"
#include <QTime>
#include <QTimer>
#include <QDir>
#include <QFileDialog>
#include <QLineEdit>

namespace ys_scene {
    PlaybackControl::PlaybackControl(QWidget* parent) :
    QWidget{ parent },
    file_directory_{},
    duration_{ 90000 },
    switch_play_file_{ false },
    auto_loop_playback_{ true }
{
    auto& config = you_see::ConfigService::Instance();
    is_interval_recording_ = config.Get<bool>("interval_record.enable_interval_recording", false);
    interval_ = config.Get<int>("interval_record.interval_time", 5000);
    record_line_mode_ = static_cast<PointcloudRecordType>(
        config.Get<int>("interval_record.line_mode", 3));
    ysc_control_status_.play_status.is_playing_paused.store(true);
    CreateControls();
    CreateLayout();

    // 初始化定时器
    init_timer_ = new QTimer(this);
    init_timer_->setSingleShot(true);
    connect(init_timer_, &QTimer::timeout, this, &PlaybackControl::OnInitTimeout);

    interval_record_timer_ = new QTimer(this);
    interval_record_timer_->setSingleShot(true);
    connect(interval_record_timer_, &QTimer::timeout, this, &PlaybackControl::OnRecordTimeout);
    if (is_interval_recording_) {
        init_timer_->start(8000);
    }
}

void PlaybackControl::SetDuration(qint64 duration)
{
    duration_ = duration;
    UpdateTimeLabel(0);
}

void PlaybackControl::SetPosition(int position)
{
    //slider_->setValue(position);
    if (switch_play_file_) {
        if (position != 0) {
            return;
        }
        else {
            switch_play_file_ = false;
        }
    }
    
    if (ysc_control_status_.play_status.fast_rewind_requested) {
        if (position != ysc_control_status_.play_status.fast_rewind_step) {
            return;
        }
        else {
            ysc_control_status_.play_status.fast_rewind_requested.store(false);
        }
    }

    if (ysc_control_status_.play_status.fast_forward_requested) {
        if (position != ysc_control_status_.play_status.fast_forward_step) {
            return;
        }
        else {
            ysc_control_status_.play_status.fast_forward_requested.store(false);
        }
    }
    
    if (ysc_control_status_.play_status.seek_requested) {
        if (position != ysc_control_status_.play_status.seek_frame_number) {
            return;
        }
        else {
            ysc_control_status_.play_status.seek_requested.store(false);
        }
    }
    slider_->SetProgressPosition(position);
}

void PlaybackControl::OnPlayClicked()
{
    if (!ysc_control_status_.play_status.is_playing) {
        ysc_control_status_.play_status.is_playing.store(true);
        std::string name = you_see::toolset::JoinPaths(file_directory_, 
            current_file_label_->text().toStdString());
        emit PlaybackStarted();
        //emit PlaybackStarted(name);
        play_button_->setIcon(QIcon(":/image/icons/pause.svg"));
    }
    else {
        ysc_control_status_.play_status.is_playing_paused.
            store(!ysc_control_status_.play_status.is_playing_paused);
        play_button_->setIcon(QIcon(ysc_control_status_.play_status.is_playing_paused ? 
            ":/image/icons/play.svg" :
            ":/image/icons/pause.svg"));
        emit PlaybackPaused(ysc_control_status_.play_status.is_playing_paused);
    }
}

void PlaybackControl::OnSliderValueChanged()
{
    auto value = slider_->RealValue();
    int frame_number = static_cast<int>(value / 100);
    emit SeekOperationSig(ToInt(PlayOperateType::kSeekFrame), frame_number);
    UpdateTimeLabel(value);
    ysc_control_status_.play_status.seek_requested.store(true);
    ysc_control_status_.play_status.seek_frame_number = frame_number;
    play_button_->setIcon(QIcon(":/image/icons/pause.svg"));
    ysc_control_status_.play_status.is_playing_paused.store(false);
}

void PlaybackControl::OnSpeedChanged(int index)
{
    double speeds[] = { 0.5, 1.0, 1.5, 2.0 };
    emit SpeedChanged(speeds[index]);
}

void PlaybackControl::OnRecordClicked()
{
    if (!ysc_control_status_.record_status.is_recording) {
        ysc_control_status_.record_status.is_recording.store(true);
        emit RecordingStarted();
        record_button_->setIcon(QIcon(":/image/icons/record_pause.svg"));
    }
    else {
        ysc_control_status_.record_status.is_recording_paused.
            store(!ysc_control_status_.record_status.is_recording_paused);
        record_button_->setIcon(QIcon(ysc_control_status_.record_status.is_recording_paused ? 
            ":/image/icons/record_continue.svg" :
            ":/image/icons/record_pause.svg"));
        emit RecordingPaused(ysc_control_status_.record_status.is_recording_paused);
    }
}

void PlaybackControl::OnRecordStopButtonClicked()
{
    ysc_control_status_.record_status.is_recording.store(false);
    ysc_control_status_.record_status.is_recording_paused.store(true);
    emit RecordingStopped();
    record_button_->setIcon(QIcon(":/image/icons/record.svg"));
}

void PlaybackControl::OnGotoEndClicked()
{
    // 获取当前文件在播放列表中的索引
    int currentIndex = file_list_.indexOf(current_file_label_->text());
    if (currentIndex < file_list_.size() - 1) {
        // 切换到下一个文件
        current_file_label_->setText(file_list_[currentIndex + 1]);
        std::string name = current_file_label_->text().toStdString();
        /*GotoPlaybackStarted(name);
        SetPosition(0);*/
        play_button_->setIcon(QIcon(":/image/icons/pause.svg"));
        ysc_control_status_.play_status.is_playing_paused.store(false);
        PlayShot(name);
    }
}

void PlaybackControl::OnGotoStartClicked()
{
    // 获取当前文件在播放列表中的索引
    int currentIndex = file_list_.indexOf(current_file_label_->text());
    if (currentIndex > 0) {
        // 切换到上一个文件
        current_file_label_->setText(file_list_[currentIndex - 1]);
        std::string name = current_file_label_->text().toStdString();
        /*GotoPlaybackStarted(name);
        SetPosition(0);*/
        play_button_->setIcon(QIcon(":/image/icons/pause.svg"));
        ysc_control_status_.play_status.is_playing_paused.store(false);
        PlayShot(name);
    }
    else {
        QString title(tr("warning! Already the first file"));
        QString text(tr("Please select another file to play"));
        //ToastTips::Instance().ShowTip(ToastPreset::WARNING_DARK, title, text);
    }
}

void PlaybackControl::OnFastRewindClicked()
{
    int step = duration_ / 10 /100; // 步长为文件总时长的十分之一
    int seekvalue = std::max(1, slider_->RealValue() / 100 - step);
    int new_position = slider_->RealValue() - step * 100;
    SetPosition(std::max(1, new_position / 100));
    SeekOperationSig(ToInt(PlayOperateType::kFastRewind), seekvalue);
    ysc_control_status_.play_status.fast_rewind_requested.store(true);
    ysc_control_status_.play_status.fast_rewind_step = seekvalue;
    play_button_->setIcon(QIcon(":/image/icons/pause.svg"));
    ysc_control_status_.play_status.is_playing_paused.store(false);
}

void PlaybackControl::OnFastForwardClicked()
{
    int step = duration_ / 10 /100; // 步长为文件总时长的十分之一
    int seekvalue = std::min((int)((duration_ - 100) / 100), slider_->RealValue() / 100 + step);
    int new_position = slider_->RealValue() + step * 100;
    SetPosition(std::min((int)(duration_ / 100), new_position / 100));
    SeekOperationSig(ToInt(PlayOperateType::kFastForward), seekvalue);
    ysc_control_status_.play_status.fast_forward_requested.store(true);
    ysc_control_status_.play_status.fast_forward_step = seekvalue;
    play_button_->setIcon(QIcon(":/image/icons/pause.svg"));
    ysc_control_status_.play_status.is_playing_paused.store(false);
}

void PlaybackControl::OnSpeedChanged(double speed)
{
    emit SpeedChanged(speed);
}

void PlaybackControl::OnFileSelected(QListWidgetItem* item)
{
    if (item) {
        emit PlaybackStarted();
    }
}

void PlaybackControl::OnSetSliderTime(ReplayFileHeader header)
{
    auto totalmicrosec = header.frame_count * 100;
    slider_->SetRealRange(0, totalmicrosec);
    duration_ = totalmicrosec;
    UpdateTimeLabel(0);
}

void PlaybackControl::ShowFileList()
{
    LoadYscFiles();
    QDialog fileList_dialog(this);
    fileList_dialog.setWindowTitle("ysc_files");

    QListWidget* list_widget = new QListWidget(&fileList_dialog);
    list_widget->addItems(file_list_);

    QVBoxLayout* layout = new QVBoxLayout(&fileList_dialog);
    layout->addWidget(list_widget);
    QHBoxLayout* buttonWrapLayout = qobject_cast<QHBoxLayout*>(
        qobject_cast<QVBoxLayout*>(this->layout())->itemAt(0)->layout());
    if (!buttonWrapLayout) return;

    QWidget* rightWidget = file_list_button_; // 使用黄色按钮作为参考
    QPoint pos = rightWidget->mapToGlobal(QPoint(0, 0));

    // 调整文件列表框的位置
    int x = pos.x() - 2.5 * fileList_dialog.width();
    int y = pos.y() - 8.3 * fileList_dialog.height();

    fileList_dialog.move(x, y);

    connect(list_widget, &QListWidget::itemClicked, this, [this, &fileList_dialog](QListWidgetItem* item) {
        current_file_label_->setText(item->text());
        //emit PlaybackStopped();
        play_button_->setIcon(QIcon(":/image/icons/pause.svg"));
        ysc_control_status_.play_status.is_playing_paused.store(false);
        auto name = item->text().toStdString();
        switch_play_file_ = true;
        PlayShot(name);
        fileList_dialog.accept();
        });

    fileList_dialog.exec();
}

void PlaybackControl::UpdateLayout()
{
    //QHBoxLayout* buttonWrapLayout = qobject_cast<QHBoxLayout*>(
    //    qobject_cast<QVBoxLayout*>(this->layout())->itemAt(0)->layout());
    //if (!buttonWrapLayout) return;

    //// 移除之前的弹性空间
    //while (buttonWrapLayout->count() > 3) {
    //    buttonWrapLayout->removeItem(buttonWrapLayout->itemAt(0));
    //}

    //// 计算左侧弹性空间的大小
    //int totalWidth = this->width();
    //int centerWidth = buttonLayout_->sizeHint().width();
    //int rightWidth = buttonWrapLayout->itemAt(buttonWrapLayout->count() - 1)->widget()->sizeHint().width();
    //int leftSpacing = (totalWidth - centerWidth) / 2 - rightWidth;

    //// 在左侧插入调整后的弹性空间
    //buttonWrapLayout->insertStretch(0, leftSpacing);

    //buttonWrapLayout->update();
    //buttonWrapLayout->activate();
}

void PlaybackControl::OnInitTimeout()
{
    if (is_interval_recording_) {
        StartNextRecording();
    }
}

void PlaybackControl::OnRecordTimeout()
{
    if (is_interval_recording_) {
        record_stop_button_->click();
        StartNextRecording();
    }
}

void PlaybackControl::CreateControls()
{
    CreateButton(&record_setting_button_, ":/image/icons/record_settings.svg");
    connect(record_setting_button_, &QToolButton::clicked, [this]() { 
        emit PointcloudAidIntervalShow(); });

    CreateButton(&record_button_, ":/image/icons/record.svg");
    connect(record_button_, &QToolButton::clicked, this, &PlaybackControl::OnRecordClicked);

    CreateButton(&record_stop_button_, ":/image/icons/record_stop.svg");
    connect(record_stop_button_, &QToolButton::clicked, this, &PlaybackControl::OnRecordStopButtonClicked);

    CreateButton(&goto_start_button_, ":/image/icons/gotostart.svg");
    connect(goto_start_button_, &QToolButton::clicked, [this](){ emit SeekPositionChanged(0); });

    CreateButton(&fast_rewind_button_, ":/image/icons/fastrewind.svg");
    fast_rewind_button_->setStyleSheet("QToolButton {border:none;}");

    CreateButton(&play_button_, ":/image/icons/play.svg");
    connect(play_button_, &QToolButton::clicked, this, &PlaybackControl::OnPlayClicked);

    CreateButton(&fast_forward_button_, ":/image/icons/fastforward.svg");

    CreateButton(&goto_end_button_, ":/image/icons/gotoend.svg");
    connect(goto_end_button_, &QToolButton::clicked, [this]() { emit SeekPositionChanged(duration_); });

    current_file_label_ = new QLabel(this);
    current_file_label_->setStyleSheet("color: #1ECABC; background: transparent;");

    CreateButton(&file_list_button_, ":/image/icons/list.svg");
    connect(file_list_button_, &QToolButton::clicked, this, &PlaybackControl::ShowFileList);

    // 创建 speed_combo_
    speed_combo_ = new QComboBox(this);
    speed_combo_->addItem("0.5x", 0.5);
    speed_combo_->addItem("1x", 1.0);
    speed_combo_->addItem("2x", 2.0);
    speed_combo_->addItem("4x", 4.0);
    speed_combo_->setFixedSize(64, 32);  // 设置固定大小
    speed_combo_->setEditable(false); // 设置为不可编辑，保持下拉功能

    // 创建居中委托
    CenteredItemDelegate* centeredDelegate = new CenteredItemDelegate(speed_combo_);
    speed_combo_->setItemDelegate(centeredDelegate);

    // 为下拉视图也设置居中
    QListView* listView = new QListView();
    listView->setItemDelegate(centeredDelegate);
    speed_combo_->setView(listView);

    // 设置字体
    QFont comboFont("Arial", 16); // 字体 Arial，字号 16
    speed_combo_->setFont(comboFont);

    // 设置样式
    speed_combo_->setStyleSheet(R"(
QComboBox {
    border: none;
    background-color: transparent;  /* 与父控件背景颜色一致 */
    color: #059669;
}
QComboBox::drop-down {
    width: 0px;
    border: 10px;
}
QComboBox QAbstractItemView {
    background-color: transparent; /* 下拉列表背景颜色与父控件一致 */
    color: #059669;
    selection-background-color: transparent;
}
QComboBox::item {
    background-color: transparent; /* 每个下拉项的背景颜色 */
    color: #059669;
}
QComboBox::item:selected {
    background-color: transparent; /* 下拉项选中时的背景颜色 */
    color: #059669;
}
)");

    connect(speed_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int index) {
        double speed = speed_combo_->itemData(index).toDouble();
        OnSpeedChanged(speed);
        });

    connect(speed_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int index) {
        double speed = speed_combo_->itemData(index).toDouble();
        OnSpeedChanged(speed);
        });

    time_label_ = new QLabel(this);
    time_label_->setText("00:00:00.000 / 00:00:00.000");
    time_label_->setAlignment(Qt::AlignCenter);
    time_label_->setStyleSheet("color: #1ECABC;");
    UpdateTimeLabel(0);

    slider_ = new CustomSlider(this);
    slider_->setRange(0, 90000); // 90 seconds 
    slider_->setPageStep(5000); // 5 seconds
    slider_->setFixedHeight(50);
    connect(slider_, &QSlider::sliderPressed, this, &PlaybackControl::OnSliderValueChanged);
    connect(goto_start_button_, &QToolButton::clicked, this, &PlaybackControl::OnGotoStartClicked);
    connect(goto_end_button_, &QToolButton::clicked, this, &PlaybackControl::OnGotoEndClicked);
    connect(fast_rewind_button_, &QToolButton::clicked, this, &PlaybackControl::OnFastRewindClicked);
    connect(fast_forward_button_, &QToolButton::clicked, this, &PlaybackControl::OnFastForwardClicked);
    LoadYscFiles();
}

void PlaybackControl::CreateLayout()
{
    // 创建主布局
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->setSpacing(5);
    mainLayout->setContentsMargins(5, 5, 5, 5);

    QHBoxLayout* buttonWrapLayout = new QHBoxLayout();
    buttonWrapLayout->setSpacing(0);
    buttonWrapLayout->setContentsMargins(0, 0, 0, 0);

    // 创建中间的按钮布局
    QWidget* centerWidget = new QWidget();
    buttonLayout_ = new QHBoxLayout(centerWidget);
    buttonLayout_->setSpacing(5);
    buttonLayout_->setContentsMargins(0, 0, 0, 0);
    buttonLayout_->addWidget(record_setting_button_);
    buttonLayout_->addWidget(record_button_);
    buttonLayout_->addWidget(record_stop_button_);
    buttonLayout_->addSpacing(25);
    buttonLayout_->addWidget(goto_start_button_);
    buttonLayout_->addWidget(fast_rewind_button_);
    buttonLayout_->addWidget(play_button_);
    buttonLayout_->addWidget(fast_forward_button_);
    buttonLayout_->addWidget(goto_end_button_);
    buttonLayout_->addWidget(speed_combo_);

    // 创建右侧控件布局
    QWidget* rightWidget = new QWidget();
    QHBoxLayout* rightLayout = new QHBoxLayout(rightWidget);
    rightLayout->setSpacing(5);
    rightLayout->setContentsMargins(0, 0, 0, 0);
    rightLayout->addWidget(current_file_label_);
    rightLayout->addWidget(file_list_button_);

    // 设置大小策略
    centerWidget->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
    rightWidget->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);

    // 将部件添加到按钮包装布局
    buttonWrapLayout->addStretch(1);  // 左侧弹性空间
    buttonWrapLayout->addWidget(centerWidget, 0, Qt::AlignHCenter);
    buttonWrapLayout->addStretch(1);  // 右侧弹性空间
    buttonWrapLayout->addWidget(rightWidget);

    // 创建滑块布局
    QHBoxLayout* sliderLayout = new QHBoxLayout();
    sliderLayout->addWidget(slider_);
    sliderLayout->addWidget(time_label_);

    // 将布局添加到主布局
    mainLayout->addLayout(buttonWrapLayout);
    mainLayout->addLayout(sliderLayout);

    // 设置主窗口的大小策略
    this->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

    // 确保布局更新
    this->setLayout(mainLayout);

}

void PlaybackControl::CreateButton(QToolButton** button, const QString icon_path)
{
    *button = new QToolButton(this);
    (*button)->setIcon(QIcon(icon_path));
    (*button)->setCheckable(true);
    (*button)->setIconSize(QSize(30, 30));
    (*button)->setStyleSheet("QToolButton {border:none;}");
}

void PlaybackControl::UpdateTimeLabel(qint64 position)
{
    QTime currentTime(0, 0, 0, 0);
    currentTime = currentTime.addMSecs(position);

    QTime totalTime(0, 0, 0, 0);
    totalTime = totalTime.addMSecs(duration_);

    time_label_->setText(QString("%1/%2")
        .arg(currentTime.toString("HH:mm:ss.zzz"))
        .arg(totalTime.toString("HH:mm:ss.zzz")));
}

void PlaybackControl::LoadYscFiles()
{
    std::string save_dir{};
    std::string default_directory{ "record_cloud" };
    if (!you_see::toolset::DirectoryInit(default_directory, save_dir)) {
        return;
    }
    file_directory_ = save_dir;
    QDir dir(QString::fromStdString(save_dir));  // 当前目录,你可以修改为特定的路径
    QStringList nameFilters;
    nameFilters << "*.ysc";
    QFileInfoList fileInfoList = dir.entryInfoList(nameFilters, QDir::Files, QDir::Name);
    if (fileInfoList.empty()) {
        return;
    }
    current_file_label_->setText(fileInfoList.begin()->fileName());
    
    if (!file_list_.empty()) {
        file_list_.clear();
    }
    for (const QFileInfo& fileInfo : fileInfoList) {
        file_list_.append(fileInfo.fileName());
    }

    /*if (file_list_->count() > 0) {
        file_list_->setCurrentRow(0);
    }*/
}

void PlaybackControl::PlayShot(const std::string& filename)
{
    ysc_control_status_.play_status.Reset();
    ysc_control_status_.play_status.is_playing.store(true);
    //is_playing_paused_ = false;
    slider_->Reset();
    std::string name = you_see::toolset::JoinPaths(file_directory_,
        filename);
    //emit LoadYscFileHeader(name);
    emit GotoPlaybackStarted(name, ysc_control_status_.play_status.is_playing_paused);
}

void PlaybackControl::StartNextRecording()
{
    if (!is_interval_recording_) return;
    record_button_->click();
    interval_record_timer_->start(interval_);
    //LOG_INFO("started recording for {}ms", interval_);
}

void PlaybackControl::PlayNextFile()
{
    // 获取当前文件在播放列表中的索引
    int currentIndex = file_list_.indexOf(current_file_label_->text());
    if (currentIndex < file_list_.size() - 1) {
        // 切换到下一个文件
        current_file_label_->setText(file_list_[currentIndex + 1]);
    }
    else {
        // 如果已经是最后一个文件,则从头开始循环播放
        current_file_label_->setText(file_list_[0]);
    }
    std::string name = you_see::toolset::JoinPaths(file_directory_,
        current_file_label_->text().toStdString());
    //PlayShot(name);
    GotoPlaybackStarted(name, ysc_control_status_.play_status.is_playing_paused);
    SetPosition(0);
}

void PlaybackControl::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);

    QHBoxLayout* buttonWrapLayout = qobject_cast<QHBoxLayout*>(
        qobject_cast<QVBoxLayout*>(this->layout())->itemAt(0)->layout());
    if (!buttonWrapLayout) return;

    int totalWidth = this->width();
    int centerWidth = buttonLayout_->sizeHint().width();
    int rightWidth = buttonWrapLayout->itemAt(buttonWrapLayout->count() - 1)->widget()->sizeHint().width();

    int leftSpacing = (totalWidth - centerWidth) / 2;
    int rightSpacing = leftSpacing - rightWidth;

    buttonWrapLayout->setStretch(0, leftSpacing);
    buttonWrapLayout->setStretch(2, rightSpacing);
}

void PlaybackControl::GetCurrentYScHeader()
{
    std::string name = you_see::toolset::JoinPaths(file_directory_,
        current_file_label_->text().toStdString());
    emit LoadYscFileHeader(name);
}

void PlaybackControl::SetRecordInterval(int ms)
{
    interval_ = ms;
}

void PlaybackControl::SetLinesType(int type)
{
    record_line_mode_ = static_cast<PointcloudRecordType>(type);
}

void PlaybackControl::StartTimedRecording()
{
    if (!is_interval_recording_) {
        is_interval_recording_ = true;
        // 等待8秒后开始第一次录制
        init_timer_->start(8000);
        //LOG_INFO("Waiting 8s before first recording...");
    }
}

void PlaybackControl::StopTimedRecording()
{
    is_interval_recording_ = false;
    init_timer_->stop();
    interval_record_timer_->stop();
    record_stop_button_->click();  // 停止当前录制
    //LOG_INFO("Timed recording stopped");
}

}; // namespace ys_scene 