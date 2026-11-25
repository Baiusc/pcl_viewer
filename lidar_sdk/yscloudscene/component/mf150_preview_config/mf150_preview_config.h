#ifndef MF150_PREVIEW_CONFIG_H
#define MF150_PREVIEW_CONFIG_H

#include <QDialog>
#include <functional>
#include <string>

class QCheckBox;
class QComboBox;
class QGroupBox;
class QRadioButton;
class QDoubleSpinBox;
class QPushButton;
class QLabel;
class QTableWidget;
class QVBoxLayout;

namespace lumosy {
class FlowPipeline;
}

namespace ys_scene {

class Mf150PreviewConfig : public QDialog {
    Q_OBJECT

public:
    using CalibrationCallback = std::function<void(float dist_real, const std::string& bin_path)>;
    using CollectFrameCallback = std::function<bool(float dist_real)>;
    using DeleteFrameCallback = std::function<void(int index)>;
    using MultiCalibCallback = std::function<void(const std::string& bin_path)>;
    using SaveRawDataCallback = std::function<void()>;
    using RoiPickModeCallback = std::function<void(bool active)>;

    explicit Mf150PreviewConfig(QWidget* parent = nullptr);
    ~Mf150PreviewConfig() override = default;

    void SetPreviewEnabled(bool enabled);
    void SetEchoIndex(int index);

    bool IsPreviewEnabled() const;
    int GetEchoIndex() const;

    void SetCalibrationCallback(CalibrationCallback cb);
    void SetCalibrationEnabled(bool enabled);
    bool IsCalibrationEnabled() const;
    void SetCalibMapLoaded(bool loaded);

    lumosy::FlowPipeline* GetFlowPipeline() const;

    void SetCollectFrameCallback(CollectFrameCallback cb);
    void SetDeleteFrameCallback(DeleteFrameCallback cb);
    void SetMultiCalibCallback(MultiCalibCallback cb);
    void SetSaveSingleRawDataCallback(SaveRawDataCallback cb);
    void SetSaveMultiRawDataCallback(SaveRawDataCallback cb);
    void SetRoiPickModeCallback(RoiPickModeCallback cb);
    void SetRoiInfo(int row, int col, int width, int height);
    void SetRoiPickModeActive(bool active);
    void SetMultiCalibEnabled(bool enabled);
    bool IsMultiCalibEnabled() const;
    void SetMultiCalibMapLoaded(bool loaded);
    // 多点标定(新版每像素 LUT 模型)概况：有效像素数与最大距离点数
    void SetMultiCalibSummary(int valid_pixels, int max_dist_pts);
    lumosy::FlowPipeline* GetMultiFlowPipeline() const;

    bool IsSingleSyncSaveEnabled() const;
    bool IsMultiSyncSaveEnabled() const;
    std::string GetSingleSyncSaveReflectivity() const;
    std::string GetMultiSyncSaveReflectivity() const;

signals:
    void EchoIndexChanged(int index);
    void PreviewEnabledChanged(bool enabled);
    void SingleCalibEnabledChanged(bool enabled);
    void MultiCalibEnabledChanged(bool enabled);
    void RoiVisibilityChanged(bool visible);

private slots:
    void OnStartCalibration();
    void OnCollectFrame();
    void OnDeleteSelectedFrame();
    void OnStartMultiCalibration();
    void OnSaveSingleRawData();
    void OnSaveMultiRawData();
    void OnToggleRoiPickMode();
    void OnEchoSelectionChanged();

private:
    void SetupSinglePointUi(QVBoxLayout* layout);
    void SetupMultiPointUi(QVBoxLayout* layout);

    QCheckBox* cb_enable_preview_{ nullptr };
    QRadioButton* rb_echo0_{ nullptr };
    QRadioButton* rb_echo1_{ nullptr };
    QRadioButton* rb_echo2_{ nullptr };

    QDoubleSpinBox* sb_dist_real_{ nullptr };
    QPushButton* btn_start_calib_{ nullptr };
    QLabel* lbl_calib_status_{ nullptr };
    lumosy::FlowPipeline* flow_pipeline_{ nullptr };
    QCheckBox* cb_enable_calib_{ nullptr };

    QDoubleSpinBox* sb_multi_dist_real_{ nullptr };
    QPushButton* btn_collect_frame_{ nullptr };
    QTableWidget* table_collected_{ nullptr };
    QPushButton* btn_delete_selected_{ nullptr };
    lumosy::FlowPipeline* multi_flow_pipeline_{ nullptr };
    QLabel* lbl_multi_calib_status_{ nullptr };
    QPushButton* btn_start_multi_calib_{ nullptr };
    QCheckBox* cb_enable_multi_calib_{ nullptr };
    // 新版多点标定(LUT 模型)概况显示
    QLabel* lbl_multi_summary_{ nullptr };

    QCheckBox* cb_sync_save_single_{ nullptr };
    QComboBox* cb_refl_single_{ nullptr };
    QPushButton* btn_save_single_raw_{ nullptr };
    QCheckBox* cb_sync_save_multi_{ nullptr };
    QComboBox* cb_refl_multi_{ nullptr };
    QPushButton* btn_save_multi_raw_{ nullptr };
    QPushButton* btn_toggle_roi_{ nullptr };
    QCheckBox* cb_show_roi_{ nullptr };
    QLabel* lbl_roi_info_{ nullptr };

    CalibrationCallback calib_callback_;
    CollectFrameCallback collect_frame_callback_;
    DeleteFrameCallback delete_frame_callback_;
    MultiCalibCallback multi_calib_callback_;
    SaveRawDataCallback save_single_raw_data_callback_;
    SaveRawDataCallback save_multi_raw_data_callback_;
    RoiPickModeCallback roi_pick_mode_callback_;
};

} // namespace ys_scene

#endif // MF150_PREVIEW_CONFIG_H
