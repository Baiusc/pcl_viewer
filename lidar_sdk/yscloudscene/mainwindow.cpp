#include <QVBoxLayout>
#include <QUrl>
#include <QTimer>
#include "component/device_management/mode_management_widget.h"
#include "component/device_management/network_dialog.h"
#include "component/device_management/sn_dialog.h"
#include "mainwindow.h" // * the first included header file
#include "flash_dialog.h"
//#include "./ui_mainwindow.h"
#include <QVBoxLayout>

#ifdef _WIN32   // * these headerfile must be here
#include <winsock2.h>
#endif // WIN32

#include "component/raw_display_widget.h" // * the second included header files must be components's part
        // * if not ,the confliction may be occur between QcustomPlot and Qt
#include "component/image_widget.h"
#include "component/tools/ip_utils.h"
#include "lumomo_ui/base/floating_toolbar.h"
#include "lumomo_ui/firmware_version/firmware_version_panel.h"

#include "motor_widget.h"


#include "business/app_manager.h"

#include "tiny_hut/common_toolset.h" 
#include "toolset/file_handler.h"
#include "you_see/business/node_founder.h"// * the third included header files must be processor part.
#include "you_see/business/node_supervisor.h"
#include "you_see/business/raw_histogram_processor.h"// *if not may cause winsock2's functions redefinetion
#include "you_see/business/gray_image_processor.h"
#include "business/raw_histogram_parser.h"
#include "business/raw_histogram_parse_imp.h"
#include "business/firmware_version/firmware_version_codec.h"


#include "you_see/business/origin_point_processor.h"
#include "you_see/business/fl6532_range_processor.h"
#include "business/origin_point_parser.h"
#include "business/origin_point_parser_imp.h"
#include "business/fl6532_point_parser.h"
#include "business/fl6532_range_parser.h"

#include "ys_lidar_user.h"
#include "you_see/business/point_processor.h"
#include "business/pointcloud_parser.h"
#include "business/pointcloud_parser_imp.h"
#include "business/pointcloud_file_manager.h"
#include "business/matrix_lidar/matrix_lidar_adapter.h"

#include "you_see/business/instruction_processor.h"
#include "config/config_service.h"
#include "ys_lidar_validate_type.h"
#include "ys_lidar_validate_user.h"
//#include "component/mqtt_client/robust_mqtt_client.h"
#include "component/pointcloud/pointcloud_display.h"
#include "component/pointcloud/playback_control.h"
#include "component/pointcloud/picking/single_pick_overlay.h"
#include "component/pointcloud/picking/single_pick_list.h"
#include "component/pointcloud/picking/roi_pick_engine.h"
#include "component/pointcloud/picking/pick_info_panel.h"
#include "component/continuoussaveorigin.h"
#include "component/continuousdealpoint.h"
#include "component/colormaptype_distancethreshold.h"
#include "component/colormapper/colormapper.h"
#include "component/roi_select.h"
#include "component/interval_record/record_interval.h"
#include "component/node_manager/node_manager.h"
#include "component/node_detail_panel/node_detail_panel.h"
#include "component/status_panel/status_panel.h"
#include "component/matrix_lidar_options/matrix_lidar_options.h"
#include "component/mf150_preview_config/mf150_preview_config.h"
#include "component/cs96_beta_preview_config/cs96_beta_preview_config.h"
#include "business/mf150_calib_controller.h"
#include "business/mf150_calibration_runner.h"
#include "business/cs96_beta_calib_params.h"
#include "business/cs96_beta_param_loader.h"
#include "business/lidar_properties_config/lidar_model_config.h"
#include "component/network_settings/network_settings_widget.h"
//#include "scene/resource_initializer.h"

#include <functional>
#include <filesystem>
#include <QSvgRenderer>
#include <QPainter>
#include <QPixmap>
#include <fstream>
#include <iostream>
#include <QThreadPool>
#include <QMenu>
#include <QMenuBar>
#include <QFile>
#include <QFileInfo>
#include <QCoreApplication>
#include <QDebug>
#include <QResizeEvent>

namespace ys_scene {
// 多点 LUT 表概况统计(供 MF150 预览对话框显示)，实现在本文件尾部
void ComputeMultiTableSummary(const SslDistCalibMulti& table, int& out_valid_pixels, int& out_max_dist_pts);
}

MainWindow::MainWindow(QWidget *parent)
    : lumosy::WinNativeWindow(parent)
    , toolbar_(nullptr)
    , save_origin_point_action_(nullptr)
    , roi_select_action_(nullptr)
    , status_panel_action_(nullptr)
    , matrix_lidar_options_action_(nullptr)
    , pointcloud_view_(nullptr)
    , raw_histogram_view_(nullptr)
    , motor_view_(nullptr)
    , show_playback_action_(nullptr)
    , distance_colormap_action_(nullptr)
    , english_us_action_(nullptr)
    , chinese_simpled_action_(nullptr)
    , ui2(new ys_scene::RawDisplayWidget(parent))
    , colormap_dialog_(nullptr)
    , matrix_lidar_adapter_(nullptr)
    , property_tree_widget_(nullptr)
{

    lumosy::CommandDeck::Initialize();
    //ResourceInitializer::initialize();
    resize(1536, 960);
    // setStyleSheet(...) // Removed global stylesheet setting if it conflicts, or keep it if it's for other things. 
    // The original code set QMainWindow::separator style. WinNativeWindow might not use it but let's keep it for now.
    setStyleSheet(R"(
        QMainWindow::separator {
            background: rgb(0, 0, 0);
            width: 0px;
            height: 0px;
        }
    )");
    InitLogger();
    int logger_level = 2;
    SetLoggerLevel(logger_level);
    // configuration files
    ConfigService::Instance().Initialize();
#ifdef _WIN32
    WORD sockVersion = MAKEWORD(2, 2);  // 使用 Winsock 2.2
    WSADATA wsdata;
    int wsaResult = WSAStartup(sockVersion, &wsdata);
    if (wsaResult != 0) {
        LOG_ERROR("WSAStartup failed with error: {}", wsaResult);
    }

    // 验证请求的版本是否可用
    if (LOBYTE(wsdata.wVersion) != 2 || HIBYTE(wsdata.wVersion) != 2) {
        LOG_ERROR("Could not find a usable version of Winsock.dll");
        WSACleanup();
    }
    LOG_INFO("WSAStartup initialized successfully");
#endif // _WIN32

    // Setup Layout for WinNativeWindow/CommandDeck
    QWidget* central_container = new QWidget(this);
    setCentralWidget(central_container);

    QVBoxLayout* main_layout = new QVBoxLayout(central_container);
    main_layout->setContentsMargins(0, 0, 0, 0);
    main_layout->setSpacing(0);

    command_deck_ = new lumosy::CommandDeck(this);
    main_layout->addWidget(command_deck_);
    this->SetTitleBarWidget(command_deck_);
    
    connect(command_deck_, &lumosy::CommandDeck::RequestMinimize, this, &QMainWindow::showMinimized);
    connect(command_deck_, &lumosy::CommandDeck::RequestMaximize, [this]() {
        if (this->isMaximized()) showNormal();
        else showMaximized();
        });
    connect(command_deck_, &lumosy::CommandDeck::RequestClose, this, &QMainWindow::close);

    // Initialize menubar_ and toolbar_ from command_deck_
    menubar_ = command_deck_->GetMenuBar();
    toolbar_ = command_deck_->GetToolBar();
    toolbar_->setIconSize(QSize(32, 32));

    QMenu* menuview_ = nullptr;
    CreatUIElements(menuview_, tr("View"), "translationKey");

    CreatUIElements(pointcloud_view_, tr("point cloud"), "translationKey", menuview_);
    CreatUIElements(raw_histogram_view_, tr("raw histogram"), "translationKey", menuview_);
    CreatUIElements(motor_view_, tr("motor"), "translationKey", menuview_);
    
    // toolbar_ = this->addToolBar(tr("Main ToolBar")); // Removed, using CommandDeck's toolbar
    CreatUIElements(save_origin_point_action_, tr("save_origin"), "translationKey");
    // save_point_cloud_action_ 已移动到 PointcloudDisplay 的 FloatingToolbar 中
    CreatUIElements(roi_select_action_, tr("roi_select"), "translationKey");
    CreatUIElements(status_panel_action_, tr("status_panel"), "translationKey");
    CreatUIElements(matrix_lidar_options_action_, tr("matrix_lidar_options"), "translationKey");
    CreatUIElements(mf150_preview_action_, tr("mf150_preview"), "translationKey");
    CreatUIElements(cs96_beta_preview_action_, tr("cs96_beta_preview"), "translationKey");
    CreatUIElements(firmware_version_action_, tr("firmware_version"), "translationKey");
    toolbar_->addAction(save_origin_point_action_);
    toolbar_->addAction(roi_select_action_);
    toolbar_->addAction(status_panel_action_);
    toolbar_->addAction(matrix_lidar_options_action_);
    toolbar_->addAction(mf150_preview_action_);
    toolbar_->addAction(cs96_beta_preview_action_);
    toolbar_->addAction(firmware_version_action_);
    auto* network_action = toolbar_->addAction(tr("设备 IP 发现与配置"));
    connect(network_action, &QAction::triggered, this, [this]() {
        ys_scene::NetworkDialog dialog(this);
        dialog.exec();
    });
    auto* sn_action = toolbar_->addAction(tr("设备 SN 管理"));
    connect(sn_action, &QAction::triggered, this, [this]() {
        ys_scene::SnDialog dialog(this);
        dialog.exec();
    });
    auto* firmware_flash_action = toolbar_->addAction(tr("固件烧录"));
    connect(firmware_flash_action, &QAction::triggered, this, [this]() {
        ti_dfu::FlashDialog dialog(this);
        dialog.exec();
    });

    connect(pointcloud_view_, &QAction::triggered, [&]() {
        stacked_widget_->setCurrentIndex(0);
        });
    connect(raw_histogram_view_, &QAction::triggered, [&]() {
        stacked_widget_->setCurrentIndex(1);
        });
    connect(motor_view_, &QAction::triggered, [&]() {
        stacked_widget_->setCurrentIndex(2);
        });

    QThreadPool::globalInstance()->setMaxThreadCount(4);
    Initialization();

    // * raw histogram deal
    raw_histogram_parser_ = new ys_scene::RawHistogramParser(this);
    you_see::GrayImageProcessorGetInstance().SetFrameCallback(
        [controller = ui2->GetGrayController()](you_see::GrayEchoFramePtr frame) {
            controller->OfferFrame(std::move(frame));
        });

    qRegisterMetaType<QVector<double>>("QVector<double>");
    qRegisterMetaType<you_see::DeviceType>();
    // 跨线程队列连接传递 pick 数据源（MF150 原始点解析线程 -> 主线程），
    // 必须注册元类型，否则 Qt 会丢弃 PickDataReady 信号导致选点失效
    qRegisterMetaType<std::shared_ptr<ys_scene::PickDataProvider>>(
        "std::shared_ptr<ys_scene::PickDataProvider>");

    for (auto processor : raw_histogram_parser_->GetProcessors()) {
        connect(processor, &RawHistogramParseImp::grayvec_sig2,
            ui2->GetImageWidget(), &ys_scene::ImageWidget::UpdataImage2, Qt::QueuedConnection);
        connect(processor, &RawHistogramParseImp::grayvec_full_frame_sig,
            ui2->GetImageWidget(), &ys_scene::ImageWidget::UpdateFullFrame, Qt::QueuedConnection);
        connect(processor, &RawHistogramParseImp::mf150_tracked_bins_sig,
            ui2, &ys_scene::RawDisplayWidget::OnMf150TrackedBins, Qt::QueuedConnection);
    }

    // 像素分配变化时同步更新所有处理器的追踪列表
    connect(ui2, &ys_scene::RawDisplayWidget::Mf150PixelAssignedSig,
        [this](const std::vector<std::pair<std::pair<int,int>, int>>& tracked) {
            for (auto processor : raw_histogram_parser_->GetProcessors()) {
                processor->SetMf150TrackedPixels(tracked);
            }
        });

    // 播放/暂停时新选像素：请求重新发送最近一帧，立即刷新直方图显示
    connect(ui2, &ys_scene::RawDisplayWidget::RequestLatestFrameSig,
        raw_histogram_parser_, &RawHistogramParser::SendData);

    // 峰值范围滑块：灰度计算使用的 bins 区间变化时透传解析器
    connect(ui2, &ys_scene::RawDisplayWidget::BinRangeChangedSig,
        raw_histogram_parser_, &RawHistogramParser::SetBinRange);

    // 设备类型变化（SDK 回调线程）时同步 UI：适配峰值范围滑块 bins 总数
    connect(raw_histogram_parser_, &RawHistogramParser::DeviceTypeChangedSig,
        ui2, &ys_scene::RawDisplayWidget::OnDeviceTypeChanged);

    // ui2 在初始化列表中构造，早于 ConfigService::Initialize()，此处重新加载 MF150 配置
    ui2->LoadMf150PlotConfig();

    // 从配置文件恢复追踪列表，初始化处理器
    {
        auto tracked = ui2->GetMf150TrackedList();
        if (!tracked.empty()) {
            for (auto processor : raw_histogram_parser_->GetProcessors()) {
                processor->SetMf150TrackedPixels(tracked);
            }
        }
    }

    connect(ui2->GetImageWidget(), &ImageWidget::PauseSig, 
        raw_histogram_parser_, &RawHistogramParser::SetPause);
    connect(ui2->GetImageWidget(), &ImageWidget::GetFrameSig,
        raw_histogram_parser_, &RawHistogramParser::SendData);
    connect(ui2->GetImageWidget(), &ImageWidget::ContinuousSaveSig,
        raw_histogram_parser_, &RawHistogramParser::SetSaveRawHistogram);
    connect(raw_histogram_parser_, &RawHistogramParser::SendFrameSig, 
        ui2, &RawDisplayWidget::GetFrame);
   
    // * origin data deal
    origin_point_parser_ = new ys_scene::OriginPointParser(this);
    SetOriginPointCallback(&ys_scene::OriginPointParser::OriginPointReceiveTrampoline, origin_point_parser_);

    // mf150_fl6532 RANGE 模式(0x2E 原始像素 / LZ4 压缩帧)：组帧在 SDK
    // Fl6532RangeProcessor(零拷贝 IOContext 队列 + 组帧 worker)内完成；
    // 业务层完成 LZ4 解压/逆转置后喂算法，输出有组织点云到点云渲染层
    fl6532_range_parser_ = new ys_scene::Fl6532RangeParser(this);
    you_see::Fl6532RangeProcessorGetInstance().SetFrameCallback(
        [this](you_see::Fl6532RangeFramePtr frame) {
            fl6532_range_parser_->OnRangeFrame(std::move(frame));
        });
    connect(fl6532_range_parser_, &ys_scene::Fl6532RangeParser::RangePointCloudReadySig,
        this, [this](ys_scene::CloudFramePtr frame) {
            if (pointcloud_parser_ && frame) {
                pointcloud_parser_->AddCartesianFrame(std::move(frame));
            }
        }, Qt::QueuedConnection);

    connect(save_origin_point_action_, &QAction::triggered, [&]() {
        ContinuousSaveOrigin dialog(this);
        connect(&dialog, &ContinuousSaveOrigin::AlgorithmRepairGroundEnable, [this](bool checked) {
            origin_point_parser_->SetAlgorithmRepairGroundEnable(checked);
            });
        if (dialog.exec() == QDialog::Accepted) {
            origin_point_parser_->SetSaveOriginPoint(dialog.GetSaveParams());
        }
        });

    connect(mf150_preview_action_, &QAction::triggered, [this]() {
        auto* dialog = new Mf150PreviewConfig(this);
        auto* controller = new Mf150CalibController(origin_point_parser_, dialog);
        dialog->setAttribute(Qt::WA_DeleteOnClose);
        controller->setParent(dialog);

        dialog->SetPreviewEnabled(mf150_preview_enabled_);
        dialog->SetEchoIndex(mf150_preview_echo_index_);
        dialog->SetCalibrationEnabled(mf150_calib_enabled_);
        dialog->SetCalibMapLoaded(!mf150_calib_map_.empty());
        dialog->SetMultiCalibEnabled(mf150_multi_calib_enabled_);
        const bool multi_loaded = mf150_multi_table_ != nullptr;
        dialog->SetMultiCalibMapLoaded(multi_loaded);
        if (multi_loaded) {
            int valid_pixels = 0;
            int max_dist_pts = 0;
            ys_scene::ComputeMultiTableSummary(*mf150_multi_table_, valid_pixels, max_dist_pts);
            dialog->SetMultiCalibSummary(valid_pixels, max_dist_pts);
        }

        if (!roi_picker_) {
            roi_picker_ = new RoiPickEngine(this);
        }

        dialog->SetRoiPickModeCallback([this](bool active) {
            if (active) {
                previous_pick_engine_ = pointcloud_display_->GetPickEngine();
                pointcloud_display_->SetPickEngine(roi_picker_);
            } else {
                pointcloud_display_->SetPickEngine(previous_pick_engine_);
                previous_pick_engine_ = nullptr;
            }
        });

        connect(roi_picker_, &RoiPickEngine::RoiDefined,
            controller, [this, controller, dialog](int row, int col, int w, int h) {
                controller->SetRoiRect(row, col, w, h);
                dialog->SetRoiInfo(row, col, w, h);

                std::vector<float> roi_vertices;
                roi_picker_->BuildRoiVertices(roi_vertices);
                pointcloud_display_->UpdateRoiOverlay(roi_vertices);
            });

        connect(roi_picker_, &RoiPickEngine::RoiCleared,
            controller, [this, dialog]() {
                pointcloud_display_->UpdateRoiOverlay({});
                dialog->SetRoiInfo(0, 0, 0, 0);
            });

        connect(roi_picker_, &RoiPickEngine::RoiStateChanged,
            this, [this]() {
                std::vector<float> roi_vertices;
                roi_picker_->BuildRoiVertices(roi_vertices);
                pointcloud_display_->UpdateRoiOverlay(roi_vertices);
            });

        connect(dialog, &Mf150PreviewConfig::RoiVisibilityChanged,
            pointcloud_display_, &PointcloudDisplay::SetRoiVisible);

        dialog->SetCalibrationCallback([controller](float dist_real, const std::string& bin_path) {
            controller->StartCalibration(dist_real, bin_path);
        });

        dialog->SetCollectFrameCallback([controller](float dist_real) -> bool {
            return controller->CollectFrame(dist_real);
        });

        dialog->SetDeleteFrameCallback([controller](int index) {
            controller->RemoveCollectedFrame(index);
        });

        dialog->SetMultiCalibCallback([controller](const std::string& bin_path) {
            controller->StartMultiCalibration(bin_path);
        });

        dialog->SetSaveSingleRawDataCallback([controller]() {
            controller->SaveSingleRawData();
        });

        dialog->SetSaveMultiRawDataCallback([controller]() {
            controller->SaveMultiRawData();
        });

        connect(controller, &Mf150CalibController::CalibrationFinished,
            dialog, [this, dialog](const std::vector<std::vector<float>>& calib_map) {
                mf150_calib_map_ = calib_map;
                mf150_calib_enabled_ = true;
                origin_point_parser_->SetMf150CalibMap(mf150_calib_map_);
                origin_point_parser_->SetMf150CalibEnabled(true);
                dialog->SetCalibMapLoaded(true);
                dialog->SetCalibrationEnabled(true);
            });

        connect(controller, &Mf150CalibController::MultiCalibrationFinished,
            dialog, [this, dialog](std::shared_ptr<SslDistCalibMulti> table) {
                if (!table) {
                    LOG_WARN("Multi-point calibration finished with empty LUT table");
                    return;
                }
                mf150_multi_table_ = std::move(table);
                mf150_multi_calib_enabled_ = true;
                origin_point_parser_->SetMf150MultiCalibTable(mf150_multi_table_);
                origin_point_parser_->SetMf150MultiCalibEnabled(true);
                dialog->SetMultiCalibMapLoaded(true);
                dialog->SetMultiCalibEnabled(true);
                int valid_pixels = 0;
                int max_dist_pts = 0;
                ys_scene::ComputeMultiTableSummary(*mf150_multi_table_, valid_pixels, max_dist_pts);
                dialog->SetMultiCalibSummary(valid_pixels, max_dist_pts);
            });

        connect(controller, &Mf150CalibController::CalibrationFailed,
            dialog, [this](const std::string& error_msg) {
                LOG_WARN("Calibration failed: {}", error_msg);
            });

        connect(controller, &Mf150CalibController::MultiCalibrationFailed,
            dialog, [this](const std::string& error_msg) {
                LOG_WARN("Multi-point calibration failed: {}", error_msg);
            });

        connect(dialog, &Mf150PreviewConfig::EchoIndexChanged,
            this, [this, dialog](int index) {
                origin_point_parser_->SetMf150PreviewConfig(
                    dialog->IsPreviewEnabled(), index);
            });

        connect(dialog, &Mf150PreviewConfig::PreviewEnabledChanged,
            this, [this, dialog](bool enabled) {
                origin_point_parser_->SetMf150PreviewConfig(
                    enabled, dialog->GetEchoIndex());
            });

        connect(dialog, &Mf150PreviewConfig::SingleCalibEnabledChanged,
            this, [this](bool enabled) {
                mf150_calib_enabled_ = enabled;
                origin_point_parser_->SetMf150CalibEnabled(enabled);
                if (enabled && !mf150_calib_map_.empty()) {
                    origin_point_parser_->SetMf150CalibMap(mf150_calib_map_);
                }
            });

        connect(dialog, &Mf150PreviewConfig::MultiCalibEnabledChanged,
            this, [this](bool enabled) {
                mf150_multi_calib_enabled_ = enabled;
                origin_point_parser_->SetMf150MultiCalibEnabled(enabled);
                if (enabled && mf150_multi_table_) {
                    origin_point_parser_->SetMf150MultiCalibTable(mf150_multi_table_);
                }
            });

        bool orig_preview_enabled = mf150_preview_enabled_;
        int orig_echo_index = mf150_preview_echo_index_;

        connect(dialog, &QDialog::finished, this,
            [this, dialog, orig_preview_enabled, orig_echo_index](int result) {
                if (result == QDialog::Accepted) {
                    mf150_preview_enabled_ = dialog->IsPreviewEnabled();
                    mf150_preview_echo_index_ = dialog->GetEchoIndex();
                } else {
                    origin_point_parser_->SetMf150PreviewConfig(
                        orig_preview_enabled, orig_echo_index);
                }
            });

        dialog->show();
    });

    connect(cs96_beta_preview_action_, &QAction::triggered, [this]() {
        auto* dialog = new Cs96BetaPreviewConfig(this);
        dialog->setAttribute(Qt::WA_DeleteOnClose);

        dialog->SetLoadConfigCallback([this](const std::string& ini_path,
            const std::string& csv_path) -> bool {
            auto params = std::make_shared<Cs96BetaCalibParams>();
            if (!Cs96BetaParamLoader::LoadFromIni(*params, ini_path)) {
                LOG_ERROR("CS96 Beta: Failed to load calibration ini: {}", ini_path);
                return false;
            }
            if (!Cs96BetaParamLoader::LoadEncoderAngle(*params, csv_path)) {
                LOG_ERROR("CS96 Beta: Failed to load encoder angle csv: {}", csv_path);
                return false;
            }
            origin_point_parser_->SetCs96BetaCalibParams(params);
            LOG_INFO("CS96 Beta: Calibration loaded, theta[0]={}, theta[95]={}",
                params->theta_[0], params->theta_[95]);
            return true;
        });

        connect(dialog, &Cs96BetaPreviewConfig::PreviewEnabledChanged,
            this, [this, dialog](bool enabled) {
                if (enabled) {
                    LOG_INFO("CS96 Beta preview enabled");
                } else {
                    LOG_INFO("CS96 Beta preview disabled");
                }
            });

        dialog->show();
    });

    // 启动时尝试加载已有标定表（优先新路径 calibration/mf150/，兼容旧路径 calibration/）
    {
        std::string default_calib_path = "calibration/mf150/map_solid.bin";
        if (!std::filesystem::exists(default_calib_path)) {
            default_calib_path = "calibration/map_solid.bin";
        }
        ys_scene::LoadCalibMapFromFile(default_calib_path, mf150_calib_map_);
        if (!mf150_calib_map_.empty()) {
            origin_point_parser_->SetMf150CalibMap(mf150_calib_map_);
            LOG_INFO("Loaded calibration map: {}×{}",
                mf150_calib_map_.size(), mf150_calib_map_[0].size());
        }

        // 多点标定(新版每像素 LUT 模型)表加载；旧版 coeffs/linear 模型文件已随算法升级废弃
        {
            std::string multi_lut_path = "calibration/mf150/map_solid_multi_lut.bin";
            if (!std::filesystem::exists(multi_lut_path)) {
                multi_lut_path = "calibration/map_solid_multi_lut.bin";
            }
            auto table = std::make_shared<SslDistCalibMulti>();
            if (ys_scene::LoadMultiCalibTableFromFile(multi_lut_path, *table)) {
                mf150_multi_table_ = std::move(table);
                origin_point_parser_->SetMf150MultiCalibTable(mf150_multi_table_);
                LOG_INFO("Loaded multi calibration LUT table: {}", multi_lut_path);
            }
        }
    }

    motor_widget_ = new ys_scene::MotorWidget(this);

    // ! pointcloud display
    pointcloud_display_ = new ys_scene::PointcloudDisplay(this);
    // ! pointcloud replay
    

    QMenu* setupview_{ nullptr };
    QMenu* language_menu_{ nullptr };
    CreatUIElements(setupview_, tr("Setup"), "translationKey");
    CreatUIElements(language_menu_, tr("Language"), "translationKey", setupview_);
    setupview_->addMenu(language_menu_);
    CreatUIElements(english_us_action_, tr("English US"), "translationKey");
    CreatUIElements(chinese_simpled_action_, tr("Chinese Simpled"), "translationKey");
    english_us_action_->setCheckable(true);
    chinese_simpled_action_->setCheckable(true);
    language_menu_->addAction(english_us_action_);
    language_menu_->addAction(chinese_simpled_action_);
    connect(english_us_action_, &QAction::triggered, this, &MainWindow::SwitchToEnglish);
    connect(chinese_simpled_action_, &QAction::triggered, this, &MainWindow::SwitchToChinese);
    stacked_widget_ = new QStackedWidget(this);
    stacked_widget_->addWidget(pointcloud_display_);
    stacked_widget_->addWidget(ui2);
    stacked_widget_->addWidget(motor_widget_);
    // setCentralWidget(stacked_widget_); // Removed, adding to main_layout instead
    main_layout->addWidget(stacked_widget_);

    this->setWindowTitle(tr("YSCloudScene"));
    this->setProperty("translationKey", "YSCloudScene");
    setWindowIcon(QIcon(":/image/icons/sun.svg"));
    SwitchLanguage("zh_CN");
    
    CreateContinuousDealPoint();
    CreateNodeManager();
    SetupFloatingToolbar();

    CreateToolActions();
    CreateViewActions();
    CreatePickActions();
    CreatePlaybackControl();
    CreateRecordInterval();
    CreateNodeStatusPanel();
    CreateMatrixLidarOptions();
    CreatePropertyTree();
    CreateFirmwareVersionPanel();
    
    //StartDefaultApps();

    //CreateMqttClient();
}

MainWindow::~MainWindow()
{
    // 停止并 join Gray worker 后才释放控制器，避免已取出的回调访问悬空 QObject。
    you_see::GrayImageProcessorGetInstance().SetFrameCallback(nullptr);
    you_see::GrayImageProcessorGetInstance().StopListening();
    // 先解除 SDK 完整帧回调(其指向 fl6532_range_parser_，属本窗口子对象)
    you_see::Fl6532RangeProcessorGetInstance().SetFrameCallback(nullptr);

    if (matrix_lidar_adapter_){
        delete matrix_lidar_adapter_;
        matrix_lidar_adapter_ = nullptr;
    }

    if (raw_histogram_parser_){
        raw_histogram_parser_->Stop();
    }

    if (pointcloud_parser_){
        pointcloud_parser_->Stop();
    }
#ifdef _WIN32
    WSACleanup();
#endif

	Cleanup();
    delete ui2;
}

void MainWindow::PoincloudRenderInit()
{
    // !pointcloud deal
    qRegisterMetaType<ys_scene::PointcloudRenderFramePtr>();
    pointcloud_parser_ = new ys_scene::PointcloudParser(this);
    // !Receive pointcloud by slot number
    SetPointCloudCallback(&ys_scene::PointcloudParser::PointcloudReceiveTrampoline, pointcloud_parser_);
    // !Receive MF150 point cloud by cartesian frame
    SetMf150CartesianFrameCallback(&ys_scene::PointcloudParser::Mf150CartesianFrameTrampoline, pointcloud_parser_);

    // !Receive mf150_fl6532 point cloud by raw packet
    fl6532_point_parser_ = new ys_scene::Fl6532PointParser(this);
    SetFl6532PacketCallback(&ys_scene::Fl6532PointParser::ReceivePacketTrampoline, fl6532_point_parser_);
    fl6532_point_parser_->SetPointcloudParser(pointcloud_parser_);

    // Set the relationship between the original point cloud and the real point cloud
    origin_point_parser_->SetPointcloudParser(pointcloud_parser_);
    for (auto processor : pointcloud_parser_->GetProcessors()) {
        // Merge frames before the GUI queue; the callback owns no widget pointer.
        connect(processor, &ys_scene::PointcloudParserImp::RenderFrameReady, processor,
            [mailbox = pointcloud_display_->GetRenderMailbox()](ys_scene::PointcloudRenderFramePtr frame) {
                mailbox->Publish(std::move(frame));
            }, Qt::DirectConnection);
    }

    auto origin_processors = origin_point_parser_->GetProcessors();
    for (auto processor : origin_processors) {
        connect(processor, &ys_scene::OriginPointParseImp::PickDataReady,
            pointcloud_display_, &ys_scene::PointcloudDisplay::SetPickData);
    }
    // pointcloud recording and playback signals and slots connect
    assert(pointcloud_parser_ != nullptr);
    auto cloud_record_playback = pointcloud_parser_->GetPointcloudFileManager();
    // record component
    connect(pointcloud_replay_, &PlaybackControl::RecordingStarted,
        cloud_record_playback, &PointcloudFileManager::StartRecording);
    connect(pointcloud_replay_, &PlaybackControl::RecordingPaused,
        cloud_record_playback, &PointcloudFileManager::SetPausing);
    connect(pointcloud_replay_, &PlaybackControl::RecordingStopped,
        cloud_record_playback, &PointcloudFileManager::StopRecording);
    connect(record_interval_, &RecordInterval::LinesType,
        cloud_record_playback, &PointcloudFileManager::SetPointRecordType);
    // playback component
    connect(pointcloud_replay_, &PlaybackControl::LoadYscFileHeader,
        pointcloud_parser_, &PointcloudParser::StartLoadYscFile);
    connect(pointcloud_replay_, &PlaybackControl::PlaybackStarted,
        pointcloud_parser_, &PointcloudParser::StartPlayback);
    connect(pointcloud_replay_, &PlaybackControl::PlaybackPaused,
        pointcloud_parser_, &PointcloudParser::SetPlaybackPaused);
    connect(pointcloud_replay_, &PlaybackControl::PlaybackStopped,
        pointcloud_parser_, &PointcloudParser::StopPlayback);
    connect(pointcloud_replay_, &PlaybackControl::GotoPlaybackStarted,
        pointcloud_parser_, &PointcloudParser::GotoStartPlayback);
    connect(cloud_record_playback, &PointcloudFileManager::SliderTime,
        pointcloud_replay_, &PlaybackControl::OnSetSliderTime);
    connect(pointcloud_parser_, &PointcloudParser::SliderUpdateSig,
        pointcloud_replay_, &PlaybackControl::SetPosition);
    connect(pointcloud_parser_, &PointcloudParser::FilePlaybackEnd,
        pointcloud_replay_, &PlaybackControl::PlayNextFile);
    connect(pointcloud_replay_, &PlaybackControl::SpeedChanged,
        pointcloud_parser_, &PointcloudParser::SetPlaybackSpeed);
    connect(pointcloud_replay_, &PlaybackControl::SeekOperationSig,
        pointcloud_parser_, &PointcloudParser::SeekOperate, Qt::QueuedConnection);
    pointcloud_replay_->GetCurrentYScHeader();

    // Open roi select dialog
    roi_select_dialog_ = new ROISelect(this);
    connect(roi_select_dialog_, &ys_scene::ROISelect::OptionalSlotNumberSig, pointcloud_parser_,
            &PointcloudParser::SetOptionalSlotNumber);
    connect(pointcloud_display_, &PointcloudDisplay::PickFreezeChanged,
        pointcloud_parser_, &PointcloudParser::SetPaused);
    connect(pointcloud_display_, &PointcloudDisplay::PickFreezeChanged,
        origin_point_parser_, &ys_scene::OriginPointParser::SetPaused);
    connect(roi_select_action_, &QAction::triggered, this, [this]() {
        roi_select_dialog_->show();
        });

    connect(roi_select_dialog_, &ys_scene::ROISelect::RawHistogramDataProportion, raw_histogram_parser_,
            &RawHistogramParser::SetMinFrameCompleteness);

    for (auto processor : raw_histogram_parser_->GetProcessors()) {
        connect(roi_select_dialog_, &ys_scene::ROISelect::RawHistogramOffsetSig, processor,
                &RawHistogramParseImp::SetRawMaxOffset);
    }

    for (auto processor : pointcloud_parser_->GetProcessors()) {
        connect(roi_select_dialog_, &ys_scene::ROISelect::ROISelectTypeSig,
            processor, &PointcloudParserImp::SetROI);
        connect(continuous_deal_point_, &ContinuousDealPoint::AllFacesDisplay,
            processor, &PointcloudParserImp::SetAllSidesDisplay);
    }

    for (auto processor : origin_point_parser_->GetProcessors()) {
        connect(roi_select_dialog_, &ys_scene::ROISelect::ROISelectTypeSig,
            processor, &OriginPointParseImp::SetRoi);
    }

}
void MainWindow::SavePointCloud()
{
    pointcloud_parser_->SetSavePointCloud(ToInt(PointCloudParser::default_save_frame_count));
}

void MainWindow::CreateContinuousDealPoint()
{
    continuous_deal_point_ = new ys_scene::ContinuousDealPoint(this);
    connect(continuous_deal_point_, &ContinuousDealPoint::DisplaySide, [&](QList<bool> side_status, int counts) {

        pointcloud_parser_->SetDisplaySides(side_status, counts);
        });
    connect(continuous_deal_point_, &ContinuousDealPoint::PointcloudSave, this, &MainWindow::SavePointCloudFrames);
    connect(continuous_deal_point_, &ContinuousDealPoint::AllFacesDisplay, pointcloud_display_,
        &ys_scene::PointcloudDisplay::SetShowAllMirrors);
}

void MainWindow::CreatePlaybackControl()
{
    pointcloud_replay_ = new ys_scene::PlaybackControl(this);
    playback_dock_ = new QDockWidget("Playback Control", this);
    playback_dock_->setWidget(pointcloud_replay_);
    playback_dock_->setAllowedAreas(Qt::BottomDockWidgetArea);
    playback_dock_->setFeatures(QDockWidget::NoDockWidgetFeatures);  // 禁用所有特性，包括标题栏
    playback_dock_->setTitleBarWidget(new QWidget());  // 使用空白 widget 替换标题栏
    QColor backgroundColor(4, 4, 4);  // 0.05 * 255 约等于 12
    QPalette pal;//定义一个QPalette
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    pal.setColor(QPalette::Window, backgroundColor);
#else
    pal.setColor(QPalette::Background, backgroundColor);
#endif
    playback_dock_->setAutoFillBackground(true);//加这句，下面设置颜色时，可以设置标题和控件主体的颜色，否则只能设置标题栏的颜色，
    playback_dock_->setPalette(pal);
    addDockWidget(Qt::BottomDockWidgetArea, playback_dock_);
    // 默认隐藏
    playback_dock_->hide();
}

void MainWindow::CreateToolActions()
{
    QMenu* viewMenu = nullptr;
    CreatUIElements(viewMenu, tr("Tools"), "translationKey");
    CreatUIElements(show_playback_action_, tr("Show Playback Control"), "translationKey", viewMenu);
    CreatUIElements(distance_colormap_action_, tr("Show Distance Colormap"), "translationKey", viewMenu);
    show_playback_action_->setCheckable(true);
    connect(show_playback_action_, &QAction::triggered, this, [this](bool checked) {
        playback_dock_->setVisible(checked);
        show_playback_action_->setText(checked ? tr("Hide Playback Control"): tr("Show Playback Control"));
        });

    distance_colormap_action_->setCheckable(true);
    connect(distance_colormap_action_, &QAction::triggered, this, [this](bool checked) {
        distance_colormap_action_->setText(checked ? tr("Hide Distance Colormap") : tr("Show Distance Colormap"));
        if (colormap_dialog_) {
            colormap_dialog_->show();
            colormap_dialog_->raise();
            colormap_dialog_->activateWindow();
            return;
        }

        // 创建新的对话框
        colormap_dialog_ = new ColorMapTypeDistanceThreshold(this);
        colormap_dialog_->setAttribute(Qt::WA_DeleteOnClose);

        for (auto processor : pointcloud_parser_->GetProcessors()) {
            connect(colormap_dialog_, &ColorMapTypeDistanceThreshold::DistanceShowThrehold,
                processor, &PointcloudParserImp::SetDistanThrehold);
            connect(colormap_dialog_, &ColorMapTypeDistanceThreshold::AlgorithmItemEnableSig,
                processor, &PointcloudParserImp::SetAlgoItems);
            connect(colormap_dialog_->GetColorMapper(), &ColorMapper::DistanceSliderReleased,
                processor, &PointcloudParserImp::SetDistance);
            connect(colormap_dialog_->GetColorMapper(), &ColorMapper::ColorSliderReleased,
                processor, &PointcloudParserImp::SetDisColor);
            connect(colormap_dialog_, &ColorMapTypeDistanceThreshold::ColorMapType,
                processor, &PointcloudParserImp::SetMappingMethod);
            connect(colormap_dialog_, &ColorMapTypeDistanceThreshold::LinesShowMode,
                processor, &PointcloudParserImp::SetLinesShowMode);
        }

        // 连接finished信号以处理对话框关闭
        connect(colormap_dialog_, &QDialog::finished, this, [this](int result) {
            if (result == QDialog::Accepted) {
                // m_colorMapDialog->GetDistanceThrehold();
                int i = 0;
            }
            colormap_dialog_ = nullptr;  // 对话框关闭后重置指针
            });

        // 显示非模态对话框
        colormap_dialog_->show();
    });
}

QIcon MainWindow::CreateSvgIcon(const QString& iconPath, int size)
{
    QSvgRenderer renderer(iconPath);
    if (!renderer.isValid()) {
        return QIcon(iconPath);
    }
    QPixmap pixmap(size, size);
    pixmap.fill(Qt::transparent);
    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing, true);
    renderer.render(&painter);
    painter.end();
    return QIcon(pixmap);
}

void MainWindow::CreateViewActions()
{
    // 视角切换按钮（参照 CloudCompare 的视角导航）
    auto addViewAction = [this](QAction*& action, const QString& iconPath, const QString& tooltip,
                                ys_scene::PointcloudDisplay::ViewOrientation orientation) {
        QIcon icon = CreateSvgIcon(iconPath, 32);
        action = new QAction(icon, tooltip, this);
        action->setToolTip(tooltip);
        toolbar_->addAction(action);
        connect(action, &QAction::triggered, this, [this, orientation]() {
            if (pointcloud_display_) {
                pointcloud_display_->SetView(orientation);
            }
        });
    };

    toolbar_->addSeparator();
    addViewAction(view_top_action_, ":/image/icons/pointcloud/icon_pointcloud_top.svg", tr("Top View"),
        ys_scene::PointcloudDisplay::ViewOrientation::Top);
    addViewAction(view_bottom_action_, ":/image/icons/pointcloud/icon_pointcloud_bottom.svg", tr("Bottom View"),
        ys_scene::PointcloudDisplay::ViewOrientation::Bottom);
    addViewAction(view_front_action_, ":/image/icons/pointcloud/icon_pointcloud_front.svg", tr("Front View"),
        ys_scene::PointcloudDisplay::ViewOrientation::Front);
    addViewAction(view_back_action_, ":/image/icons/pointcloud/icon_pointcloud_back.svg", tr("Back View"),
        ys_scene::PointcloudDisplay::ViewOrientation::Back);
    addViewAction(view_left_action_, ":/image/icons/pointcloud/icon_pointcloud_left.svg", tr("Left View"),
        ys_scene::PointcloudDisplay::ViewOrientation::Left);
    addViewAction(view_right_action_, ":/image/icons/pointcloud/icon_pointcloud_right.svg", tr("Right View"),
        ys_scene::PointcloudDisplay::ViewOrientation::Right);
    addViewAction(view_iso_front_action_, ":/image/icons/pointcloud/icon_iso_pointcloud_front.svg", tr("Iso Front View"),
        ys_scene::PointcloudDisplay::ViewOrientation::IsoView1);
    addViewAction(view_iso_back_action_, ":/image/icons/pointcloud/icon_iso_pointcloud_back.svg", tr("Iso Back View"),
        ys_scene::PointcloudDisplay::ViewOrientation::IsoView2);
}

void MainWindow::SwitchLanguage(const QString& language)
{
    qDebug() << "Switching language to:" << language;

    // 移除当前的翻译器
    QCoreApplication::removeTranslator(&translator_);

    // 加载新的翻译文件
    QString filename = ":/Internationalization/YSCloudScene_" + language + ".qm";
    qDebug() << "Attempting to load translation file:" << filename;

    if (translator_.load(filename)) {
        qDebug() << "Translation file loaded successfully";
        QCoreApplication::installTranslator(&translator_);
    }
    else {
        qDebug() << "Failed to load translation file";
    }

    RetranslateUi();

    // 更新菜单项的选中状态
    if (language == "en_US") {
        english_us_action_->setChecked(true);
        chinese_simpled_action_->setChecked(false);
    }
    else if (language == "zh_CN") {
        english_us_action_->setChecked(false);
        chinese_simpled_action_->setChecked(true);
    }
}

void MainWindow::RetranslateUi()
{

    QString title_key = this->property("translationKey").toString();
    if (!title_key.isEmpty()) {
        QString translatedTooltip = tr(title_key.toUtf8().constData());
        setWindowTitle(translatedTooltip);
    }

    // 更新菜单栏
    // Use command_deck_->GetMenuBar() instead of menuBar()
    if (command_deck_ && command_deck_->GetMenuBar()) {
        for (QAction* action : command_deck_->GetMenuBar()->actions()) {
            if (action->menu()) {
                UpdateMenuText(action->menu());
            }
        }
    }

    // 更新工具栏
    if (toolbar_ == nullptr) {
        return;
    }

    for (QAction* action : toolbar_->actions()) {
        QString key = action->property("translationKey").toString();
        if (!key.isEmpty()) {
            QString translatedText = tr(key.toUtf8().constData());
            action->setText(translatedText);

            QString tooltipKey = action->property("tooltipKey").toString();
            if (!tooltipKey.isEmpty()) {
                QString translatedTooltip = tr(tooltipKey.toUtf8().constData());
                action->setToolTip(translatedTooltip);
            }
        }
    }
}


void MainWindow::UpdateMenuText(QMenu* menu)
{
    QString originalText = menu->property("translationKey").toString();
    if (originalText.isEmpty()) {
        originalText = menu->title();
    }

    QString translatedText = tr(originalText.toUtf8().constData());
    menu->setTitle(translatedText);

    qDebug() << "Menu: Original text:" << originalText << "Translated text:" << translatedText;

    // 处理菜单中的所有动作
    for (QAction* action : menu->actions()) {
        if (action->menu()) {
            // 如果动作有子菜单，递归调用
            UpdateMenuText(action->menu());
        }
        else {
            // 处理普通动作
            QString actionOriginalText = action->property("translationKey").toString();
            if (actionOriginalText.isEmpty()) {
                actionOriginalText = action->text();
            }
            QString actionTranslatedText = tr(actionOriginalText.toUtf8().constData());
            action->setText(actionTranslatedText);
        }
    }
}

void MainWindow::CreateRecordInterval()
{
    record_interval_ = new RecordInterval(this);
    //record_interval_->setAttribute(Qt::WA_DeleteOnClose);
    connect(pointcloud_replay_, &PlaybackControl::PointcloudAidIntervalShow, this, [this]() {
        record_interval_->show();
        });
    connect(record_interval_, &RecordInterval::LinesType, this, [this](int type) {
        pointcloud_replay_->SetLinesType(type);
        });
    connect(record_interval_, &RecordInterval::SetIntervalEnable, this, [this](bool enable) {
        if (enable) {
            pointcloud_replay_->StartTimedRecording();
        }
        else {
            pointcloud_replay_->StopTimedRecording();
        }
        });
    connect(record_interval_, &RecordInterval::SetIntervalValue, this, [this](int interval) {
        pointcloud_replay_->SetRecordInterval(interval);
        });
}

void MainWindow::CreateNodeManager()
{    
    // 创建 NodeManager（已经是 FloatingPanel 的派生类）
    node_manager_ = new ys_scene::NodeManager(this);
    SetAddNodeListCallback(&ys_scene::NodeManager::NodeManagerTrampoline, node_manager_);

    // 连接信号槽
    connect(node_manager_, &NodeManager::LidarConnectionChanged,
    [this](const uint32_t& ip, bool enabled) {
        if (enabled) {
            if (const auto* node = you_see::NodeSupervisorGetInstance().GetNode(ip)) {
                origin_point_parser_->SetSourceNode(*node);
            }
            ConnectNode(ip);
            // motor_widget_->SetMotorEncoderValueCallback();

            if (property_tree_widget_) {
                property_tree_widget_->SetTargetDevice(ip);
                // property_tree_widget_->show();
                // property_tree_widget_->raise();
            }

            if (float_save_point_cloud_action_) {
                float_save_point_cloud_action_->setIcon(QIcon(":/image/icons/pointcloud_save_available.svg"));
                float_save_point_cloud_action_->setEnabled(true);
            }
            if (float_config_tree_action_) {
                float_config_tree_action_->setIcon(QIcon(":/image/icons/configure_tree_available.svg"));
                float_config_tree_action_->setEnabled(true);
            }

        }
        else {
            DisconnectNode(ip);
            origin_point_parser_->ClearSourceNode();
            ui2->EndGraySession();
            raw_histogram_parser_->SetPause(false);
            if (property_tree_widget_ && !property_tree_widget_->property("managementSession").toBool()) {
                property_tree_widget_->hide();
            }

            // 断开的是当前固件版本查询目标时清空面板，避免残留上一台设备的信息
            if (firmware_version_panel_ && ip == firmware_device_ip_) {
                firmware_device_ip_ = 0;
                firmware_version_panel_->SetTargetDevice(0);
                firmware_version_panel_->Clear();
            }

            if (float_save_point_cloud_action_) {
                float_save_point_cloud_action_->setIcon(QIcon(":/image/icons/pointcloud_save_unavailable.svg"));
                float_save_point_cloud_action_->setEnabled(false);
            }
            if (float_config_tree_action_) {
                float_config_tree_action_->setIcon(QIcon(":/image/icons/configure_tree_unavailable.svg"));
                float_config_tree_action_->setEnabled(false);
            }

        }
    });

    connect(node_manager_, &NodeManager::QueryPsCalibrationParamsSig, [this](const uint32_t handle) {
		origin_point_parser_->QueryPsCalibrationParamsT(handle);
        motor_widget_->SetMotorEncoderValueCallback();
        });

    // 节点连接成功 -> 按型号加载属性配置（属性面板显隐仍由工具栏「Device Attributes」控制）
    //               并自动查询一次系统固件版本(0x0302)；板端在握手完成前不回包，
    //               因此查询只能挂在此处，不能在启动时无条件发
    connect(node_manager_, &NodeManager::LidarConnected, this,
            [this](const you_see::LidarInfo& node) {
        if (node.node_work_mode == you_see::NodeWorkMode::KGray) {
            ui2->BeginGraySession(node.ip, you_see::GrayImageProcessorGetInstance().GetSessionId(),
                                  you_see::GrayFormatForNode(node));
            stacked_widget_->setCurrentIndex(1);
            if (float_save_point_cloud_action_) {
                float_save_point_cloud_action_->setEnabled(false);
                float_save_point_cloud_action_->setIcon(QIcon(":/image/icons/pointcloud_save_unavailable.svg"));
            }
        }
        else {
            ui2->EndGraySession();
        }
        LoadLidarModelProperties(node);
        QueryFirmwareVersion(node.ip);
    });

    // 添加对Matrix Lidar连接的处理
    matrix_lidar_adapter_ = new ys_scene::MatrixLidarAdapter();
    connect(node_manager_, &NodeManager::MatrixLidarConnectionChanged,
    [this](const QString &ip, int port, bool enabled) {
        if (enabled) {
            // 连接Matrix Lidar
            bool success = matrix_lidar_adapter_->Connect(ip.toStdString(), port);
            node_manager_->OnMatrixLidarConnected(ip, success);
            if (success){
                matrix_lidar_adapter_->SetPointcloudCallback(
                    std::bind(&ys_scene::PointcloudParser::AddCartesianFrame, 
                        pointcloud_parser_,
                        std::placeholders::_1));
                        
                if (float_save_point_cloud_action_) {
                    float_save_point_cloud_action_->setIcon(QIcon(":/image/icons/pointcloud_save_available.svg"));
                    float_save_point_cloud_action_->setEnabled(true);
                }
                if (float_config_tree_action_) {
                    float_config_tree_action_->setIcon(QIcon(":/image/icons/configure_tree_available.svg"));
                    float_config_tree_action_->setEnabled(true);
                }
                
                if (property_tree_widget_) {
                    // property_tree_widget_->show();
                    // property_tree_widget_->raise();
                }

            }
        }
        else {
            // 断开Matrix Lidar
            matrix_lidar_adapter_->DisConnect();
            node_manager_->OnMatrixLidarConnected(ip, false);

            if (float_save_point_cloud_action_) {
                float_save_point_cloud_action_->setIcon(QIcon(":/image/icons/pointcloud_save_unavailable.svg"));
                float_save_point_cloud_action_->setEnabled(false);
            }
            if (float_config_tree_action_) {
                float_config_tree_action_->setIcon(QIcon(":/image/icons/configure_tree_unavailable.svg"));
                float_config_tree_action_->setEnabled(false);
            }

            if (property_tree_widget_) property_tree_widget_->hide();
        }
    });

    // 创建 NodeDetailPanel (作为顶层窗口或同级窗口)
    node_detail_panel_ = new ys_scene::NodeDetailPanel(this);

    // 让 DetailPanel 主动附着到 Manager (单向依赖，符合您的设计)
    // 这样 DetailPanel 会自动处理跟随、显示、隐藏和数据更新
    node_detail_panel_->AttachToManager(node_manager_);


    // 显示 NodeManager
    node_manager_->show();
    
    // 初始化面板高度
    UpdateNodeManagerPanelHeights();

}

void MainWindow::CreateNodeStatusPanel()
{
    status_panel_ = new ys_scene::StatusPanel(this);
    //connect(node_manager_, &NodeManager::StatusPanelQueryStartSig, status_panel_, &StatusPanel::NodeStatusQueryStart);
    connect(status_panel_, &StatusPanel::LidarWorkModeChanged, this, &MainWindow::StausSet);
	connect(status_panel_, &StatusPanel::LidarConnectionStatusChanged, node_manager_, &NodeManager::DisconnectNodeSlot);

    status_panel_->hide();

    connect(status_panel_action_, &QAction::triggered, this, [this]() {
        if (status_panel_) {
            status_panel_->show();
            status_panel_->raise();
            status_panel_->activateWindow();
            return;
        }
    });
}

void MainWindow::CreateMatrixLidarOptions()
{
    matrix_lidar_options_ = new ys_scene::MatrixLidarOptions(this);

    // 连接信号，处理添加Matrix Lidar节点的请求
    connect(matrix_lidar_options_, &ys_scene::MatrixLidarOptions::AddMatrixLidarNodeRequested, node_manager_,
            &ys_scene::NodeManager::AddMatrixLidarNode);

    matrix_lidar_options_->hide();

    connect(matrix_lidar_options_action_, &QAction::triggered, this, [this]() {
        if (matrix_lidar_options_) {
            matrix_lidar_options_->show();
            matrix_lidar_options_->raise();
            matrix_lidar_options_->activateWindow();
            return;
        }
    });
}

void MainWindow::CreatePropertyTree()
{
    // Use 'this' as parent to embed it as a top-layer child widget (like NodeManager)
    property_tree_widget_ = new lumosy::PropertyTreeWidget(this); 
    auto* management = new ys_scene::ModeManagementWidget(property_tree_widget_);
    property_tree_widget_->contentLayout()->addWidget(management);
    connect(property_tree_widget_, &lumosy::PropertyTreeWidget::TargetDeviceChanged, management,
        [management](uint32_t ip) { management->SetAddress(lumosy::IPUtils::Uint32ToIPString(ip)); });
    connect(node_manager_, &NodeManager::NodeClicked, management,
        [this, management](const you_see::LidarInfo& node, const QString&) {
            management->SetAddress(lumosy::IPUtils::Uint32ToIPString(node.ip));
            property_tree_widget_->show();
        });
    // Retain the selected HTTP target while the SDK session is being replaced.
    auto reconnect_ip = std::make_shared<uint32_t>(0);
    auto reconnect_mode = std::make_shared<int>(-1);
    auto* reconnect_timer = new QTimer(management);
    reconnect_timer->setInterval(250);
    connect(management->Client(), &ys_scene::ManagementClient::AddressChanged, this,
        [this, reconnect_ip, reconnect_mode, reconnect_timer] {
            *reconnect_ip = 0;
            *reconnect_mode = -1;
            reconnect_timer->stop();
            origin_point_parser_->SetPaused(false);
            pointcloud_parser_->SetPaused(false);
            property_tree_widget_->setProperty("managementSession", false);
        });
    connect(management->Client(), &ys_scene::ManagementClient::Switching, this,
        [this, management, reconnect_ip, reconnect_mode] {
            const QUrl url(management->Client()->Address());
            const uint32_t ip = lumosy::IPUtils::IPStringToUint32(url.host());
            const auto* node = you_see::NodeSupervisorGetInstance().GetNode(ip);
            if (!node || !node->is_connected) return;
            *reconnect_ip = ip;
            *reconnect_mode = -1;
            property_tree_widget_->setProperty("managementSession", true);
            origin_point_parser_->SetPaused(true);
            pointcloud_parser_->SetPaused(true);
            DisconnectNode(ip);
        });
    connect(management->Client(), &ys_scene::ManagementClient::Changed, this,
        [management, reconnect_ip] {
            if (*reconnect_ip || !management->Client()->IsFresh()) return;
            const auto& snapshot = management->Client()->Snapshot();
            const auto ip = lumosy::IPUtils::IPStringToUint32(QUrl(management->Client()->Address()).host());
            const auto* node = you_see::NodeSupervisorGetInstance().GetNode(ip);
            if (node && node->is_connected && (snapshot.phase != "COMMITTED"
                || snapshot.actual != static_cast<int>(node->node_work_mode)))
                emit management->Client()->Switching();
        });
    connect(management->Client(), &ys_scene::ManagementClient::Stable, this,
        [reconnect_ip, reconnect_mode, reconnect_timer](int mode) {
            if (*reconnect_ip && (mode == 1 || mode == 2)) {
                *reconnect_mode = mode;
                reconnect_timer->start();
            }
        });
    connect(reconnect_timer, &QTimer::timeout, this,
        [this, reconnect_ip, reconnect_mode, reconnect_timer] {
            const auto* node = you_see::NodeSupervisorGetInstance().GetNode(*reconnect_ip);
            // Wait for fresh discovery of the confirmed mode and completed receiver teardown.
            if (!node || node->node_work_mode != *reconnect_mode || node->is_connected
                || !node->instruction_processor_stoped || !node->data_processor_stoped) return;
            origin_point_parser_->ResetManagementSession();
            pointcloud_parser_->ResetManagementSession();
            origin_point_parser_->SetSourceNode(*node);
            origin_point_parser_->SetPaused(false);
            pointcloud_parser_->SetPaused(false);
            ConnectNode(*reconnect_ip);
            *reconnect_ip = 0;
            reconnect_timer->stop();
            property_tree_widget_->setProperty("managementSession", false);
        });
    property_tree_widget_->setWindowTitle(tr("Device Attributes"));
    property_tree_widget_->setTitle(tr("Device Attributes"));

    // Positioning: Right side, mirror NodeManager
    // NodeManager is at (20, kPanelTopMargin).
    // So this should be at (width - 500 - 20, kPanelTopMargin).
    int width = 360;
    int startX = this->width() - width - 20;
    property_tree_widget_->setGeometry(startX, kPanelTopMargin, width, 700);
    
    // Make sure it is on top of other siblings
    property_tree_widget_->raise(); // Display on top layer
    property_tree_widget_->hide(); // Hidden by default, toggled by toolbar

    //property_tree_widget_->RegisterCustomWidget("NetworkSettings", [this](QWidget* parent) {
    //    auto* w = new ys_scene::NetworkSettingsWidget(parent);
    //    connect(property_tree_widget_, &lumosy::PropertyTreeWidget::TargetDeviceChanged, 
    //            w, &ys_scene::NetworkSettingsWidget::SetDeviceHandle);
    //    // Initialize with current if available
    //    w->SetDeviceHandle(property_tree_widget_->GetCurrentDeviceHandle());
    //    return w;
    //});

    // 属性树配置按雷达型号动态加载：
    // 连接成功后由 LoadLidarModelProperties() 从 exe 同级目录
    // lidar_properties_config/<model_key>/ 加载对应 JSON（此处不再预加载任何配置）

    // 属性值变更：按通用 Key-Value 属性接口下发到当前设备
    connect(property_tree_widget_, &lumosy::PropertyTreeWidget::PropertyChanged,
            this, [this](const QString& property_id, const QVariant& value) {
        const uint32_t handle = property_tree_widget_->GetCurrentDeviceHandle();
        if (handle == 0) {
            qWarning() << "[PropertyTree] no target device, drop property change:"
                       << property_id << "=" << value;
            return;
        }

        PropertyValueT property_value{};
        QByteArray string_holder;
        switch (value.userType()) {
        case QMetaType::Bool:
            property_value.type = kPropertyValueBool;
            property_value.data.bool_value = value.toBool() ? 1 : 0;
            break;
        case QMetaType::Double:
        case QMetaType::Float:
            property_value.type = kPropertyValueFloat;
            property_value.data.float_value = value.toDouble();
            break;
        case QMetaType::Int:
        case QMetaType::UInt:
        case QMetaType::LongLong:
        case QMetaType::ULongLong:
            property_value.type = kPropertyValueInt;
            property_value.data.int_value = value.toLongLong();
            break;
        default:
            string_holder = value.toString().toUtf8();
            property_value.type = kPropertyValueString;
            property_value.data.string_value = string_holder.constData();
            break;
        }

        const QByteArray property_key = property_id.toUtf8();
        if (!SetDeviceProperty(handle, property_key.constData(), &property_value,
                               &MainWindow::PropertySetCallbackTrampoline, this)) {
            qWarning() << "[PropertyTree] property rejected locally:" << property_id << "=" << value;
        }
    });

    // 属性读取：按通用 Key-Value 属性接口向当前设备发起 Get
    connect(property_tree_widget_, &lumosy::PropertyTreeWidget::PropertyReadRequested,
            this, [this](const QString& property_id) {
        const uint32_t handle = property_tree_widget_->GetCurrentDeviceHandle();
        if (handle == 0) {
            qWarning() << "[PropertyTree] no target device, drop property read:" << property_id;
            return;
        }

        const QByteArray property_key = property_id.toUtf8();
        if (!GetDeviceProperty(handle, property_key.constData(),
                               &MainWindow::PropertyGetCallbackTrampoline, this)) {
            qWarning() << "[PropertyTree] property read rejected locally:" << property_id;
        }
    });
}

void MainWindow::CreateFirmwareVersionPanel()
{
    firmware_version_panel_ = new lumosy::FirmwareVersionPanel(this);
    firmware_version_panel_->setTitle(tr("Firmware Version"));
    firmware_version_panel_->setMainWindow(this);
    firmware_version_panel_->setPreferredWidth(420);

    // 定位：右上方属性树（Device Attributes）左侧，避免两面板完全重叠
    const int width = 420;
    const int gap = 12;
    const int x = qMax(20, this->width() - width * 2 - 20 - gap);
    firmware_version_panel_->setGeometry(x, kPanelTopMargin, width, 560);
    firmware_version_panel_->raise();
    firmware_version_panel_->hide();   // 默认隐藏，由工具栏 action 控制显隐

    // 「刷新」：向当前目标设备重新发起查询
    connect(firmware_version_panel_, &lumosy::FirmwareVersionPanel::RefreshRequested, this, [this]() {
        QueryFirmwareVersion(firmware_version_panel_->GetCurrentDeviceHandle());
    });

    // 工具栏开关键：与浮动工具栏的「Device Attributes」一致，点击切换显隐
    connect(firmware_version_action_, &QAction::triggered, this, [this]() {
        if (!firmware_version_panel_) {
            return;
        }
        if (firmware_version_panel_->isVisible()) {
            firmware_version_panel_->hide();
        }
        else {
            firmware_version_panel_->show();
            firmware_version_panel_->raise();
            firmware_version_panel_->activateWindow();
        }
    });
}

void MainWindow::QueryFirmwareVersion(uint32_t device_ip)
{
    if (!firmware_version_panel_) {
        return;
    }

    if (device_ip == 0) {
        firmware_version_panel_->SetError(tr("No device connected"));
        return;
    }

    firmware_device_ip_ = device_ip;
    firmware_version_panel_->SetTargetDevice(device_ip);
    firmware_version_panel_->SetTargetDeviceText(lumosy::IPUtils::Uint32ToIPString(device_ip));
    firmware_version_panel_->SetLoading(true);

    QuerySysVersion(device_ip, &MainWindow::FirmwareVersionCallbackTrampoline, this);
}

void MainWindow::FirmwareVersionCallbackTrampoline(const uint32_t handle,
                                                   const you_see::FirmwareVersionInfoT* info,
                                                   void* context)
{
    auto* self = static_cast<MainWindow*>(context);
    if (self == nullptr || info == nullptr) {
        return;
    }

    // 本回调在 SDK 网络线程执行：载荷指针仅在回调期内有效，
    // 必须先按值拷贝(200 字节 POD)再切回主线程，绝不把指针带出作用域
    const you_see::FirmwareVersionInfoT info_copy = *info;
    const uint32_t ack_handle = handle;

    QMetaObject::invokeMethod(self, [self, info_copy, ack_handle]() {
        if (ack_handle != self->firmware_device_ip_) {
            qWarning() << "[FirmwareVersion] ack from other device:" << ack_handle
                       << "expected:" << self->firmware_device_ip_;
        }

        if (!self->firmware_version_panel_) {
            return;
        }

        // C 结构体 -> Qt 结构的转换统一放在主线程完成
        const lumosy::FirmwareVersionInfo version_info = ys_scene::ToFirmwareVersionInfo(info_copy);
        self->firmware_version_panel_->SetVersionInfo(version_info);
    }, Qt::QueuedConnection);
}

namespace {

QVariant FormatDevicePropertyValue(const QString& key, const PropertyValueT& value)
{
    if (value.type == kPropertyValueBool)
        return QVariant(value.data.bool_value != 0);

    if (key == QLatin1String(YS_PROP_KEY_FPGA_VERSION_DATE)) {
        const QString hex = QString::asprintf("%08X", static_cast<uint32_t>(value.data.int_value));
        return hex.left(4) + QLatin1Char('-') + hex.mid(4, 2) + QLatin1Char('-') + hex.mid(6, 2);
    }

    if (key == QLatin1String(YS_PROP_KEY_FPGA_VERSION_TIME)) {
        const QString hex = QString::asprintf("%08X", static_cast<uint32_t>(value.data.int_value));
        return hex.mid(2, 2) + QLatin1Char(':') + hex.mid(4, 2) + QLatin1Char(':') + hex.mid(6, 2);
    }

    return QVariant(static_cast<qlonglong>(value.data.int_value));
}

} // namespace

void MainWindow::PropertySetCallbackTrampoline(const uint32_t handle, const PropertySetAckT* ack, void* context)
{
    auto* self = static_cast<MainWindow*>(context);
    if (!self || !ack)
        return;

    // ack 在 SDK 事件循环线程回调，按值拷贝后切回主线程
    const uint32_t ack_handle = handle;
    const uint8_t ret_code = ack->ret_code;
    const uint8_t command_set = ack->command_set;
    const uint8_t command_id = ack->command_id;
    const QString key = ack->key ? QString::fromUtf8(ack->key) : QString();

    QMetaObject::invokeMethod(self, [ack_handle, ret_code, command_set, command_id, key]() {
        qDebug() << "[PropertyTree][SetDeviceProperty] key:" << key
                 << "ret_code:" << ret_code
                 << "cmd_set:" << command_set
                 << "cmd_id:" << command_id
                 << "handle:" << ack_handle;
    }, Qt::QueuedConnection);
}

void MainWindow::PropertyGetCallbackTrampoline(const uint32_t handle, const PropertyGetAckT* ack, void* context)
{
    auto* self = static_cast<MainWindow*>(context);
    if (!self || !ack)
        return;

    // ack 在 SDK 事件循环线程回调，按值拷贝后切回主线程
    const uint32_t ack_handle = handle;
    const uint8_t ret_code = ack->ret_code;
    const uint8_t command_set = ack->command_set;
    const uint8_t command_id = ack->command_id;
    const QString key = ack->key ? QString::fromUtf8(ack->key) : QString();
    const QVariant value = FormatDevicePropertyValue(key, ack->value);

    QMetaObject::invokeMethod(self, [self, ack_handle, ret_code, command_set, command_id, key, value]() {
        if (ret_code == 0 && value.isValid()) {
            if (self->property_tree_widget_)
                self->property_tree_widget_->UpdatePropertyValue(key, value);
            qDebug() << "[PropertyTree][GetDeviceProperty] key:" << key
                     << "value:" << value
                     << "cmd_set:" << command_set
                     << "cmd_id:" << command_id
                     << "handle:" << ack_handle;
        } else {
            qWarning() << "[PropertyTree][GetDeviceProperty] failed, key:" << key
                       << "ret_code:" << ret_code
                       << "cmd_set:" << command_set
                       << "cmd_id:" << command_id
                       << "handle:" << ack_handle;
        }
    }, Qt::QueuedConnection);
}

void MainWindow::LoadLidarModelProperties(const you_see::LidarInfo& node)
{
    if (!property_tree_widget_) {
        return;
    }

    const QString model_key = ys_scene::LidarModelConfigKey(node);
    if (model_key.isEmpty()) {
        qWarning() << "Unknown lidar model, skip property tree loading. ip:" << node.ip;
        return;
    }

    // 确保属性树绑定当前设备（handle 相同则不会重复触发 TargetDeviceChanged）
    property_tree_widget_->SetTargetDevice(node.ip);

    const QString config_dir = ys_scene::LidarModelConfigDir(model_key);
    if (!QFileInfo(config_dir).isDir()) {
        qWarning() << "Lidar property config dir not found:" << config_dir;
        return;
    }

    // 加载该型号目录下全部 *.json（支持一个型号拆分多个配置文件）
    property_tree_widget_->LoadConfigFromDir(config_dir);
    qDebug() << "Loaded lidar property tree from:" << config_dir << "for model:" << model_key;
}

//void MainWindow::StartDefaultApps()
//{
//    app_manager_ = new ys_scene::AppManager(this);
//    QString current_dir = QDir::currentPath();
//    QString mosquitto_path = current_dir + QDir::separator() + "mosquitto";
//    QString mosquitto_name = "mosquitto.exe";
//
//    QString ys_scene_aid = "YsCloudAid.exe";
//    app_manager_->StartApp(mosquitto_path, mosquitto_name);
//    app_manager_->StartApp(current_dir, ys_scene_aid);
//
//}

//void MainWindow::CreateMqttClient()
//{
//    robust_mqtt_client_ = std::make_shared<you_see::RobustMqttClient>("mqtt://localhost:1883", "ys_scene_client");
//    robust_mqtt_client_->ConnectMo();
//    robust_mqtt_client_->Subscribe("ys_scene/ys_scene_aid");
//    connect(pointcloud_replay_, &PlaybackControl::PointcloudAidIntervalShow, this, &MainWindow::PointSaveIntervalShow);
//}
//
//void MainWindow::PointSaveIntervalShow()
//{
//    json message = {
//        {"command_set", ToInt(DInstSet::dinst_app_sys)},
//        {"command_id", ToInt(DInstAppSys::pointcloud_aid_interval_show)}
//    };
//    robust_mqtt_client_->Publish("ys_scene/ys_scene_aid", message);
//}

void MainWindow::SavePointCloudFrames(const QString& value)
{
    pointcloud_parser_->SetSavePointCloud(value.toInt());
}

void MainWindow::SwitchToEnglish()
{
    SwitchLanguage("en_US");
}

void MainWindow::SwitchToChinese()
{
    SwitchLanguage("zh_CN");
}

void MainWindow::StausSet(const uint32_t& handle, const uint8_t& mode, bool enabled)
{
	LidarState state = static_cast<LidarState>(mode);
	QString mode_str = "Unknown";
    switch (state) {
    case LidarState::kLidarFree:
        mode_str = tr("Free");
        break;
    case LidarState::kLidarScan:
		mode_str = tr("Scan");
        break;
	case LidarState::kLidarFault:
		mode_str = tr("Fault");
		break;
	case LidarState::kLidarDebug:
		mode_str = tr("Debug");
		break;
	case LidarState::kLidarUnknown:
		mode_str = tr("Unknown");
		break;
	default:
		break;
    }
}

void MainWindow::resizeEvent(QResizeEvent* event)
{
    lumosy::WinNativeWindow::resizeEvent(event);
    UpdateNodeManagerPanelHeights();
    UpdateFloatingToolbarPosition();
}


void MainWindow::UpdateNodeManagerPanelHeights()
{
    if (!node_manager_) {
        return;
    }
    
    // 计算可用高度
    // 面板从 y=kPanelTopMargin 开始，底部留 20px 边距
    // command_deck_ 高度约 65px，所以面板起始 y=kPanelTopMargin 是合理的
    int available_height = height() - kPanelTopMargin - 20;  // 减去顶部偏移和底部边距
    
    // 确保最小可用高度
    if (available_height < 400) {
        available_height = 400;
    }
    
    // 更新 NodeManager 位置（Y 与 FloatingToolbar 竖直对齐）
    node_manager_->move(20, kPanelTopMargin);

    // 通知 NodeManager 更新高度，NodeManager 会通过信号通知 NodeDetailPanel
    node_manager_->UpdateAvailableHeight(available_height);
}

void MainWindow::SetupFloatingToolbar()
{
    floating_toolbar_ = new lumosy::FloatingToolbar(this);
    floating_toolbar_->setOrientation(Qt::Horizontal);

    // 添加点云保存按钮
    float_save_point_cloud_action_ = floating_toolbar_->addAction(
        QIcon(":/image/icons/pointcloud_save_unavailable.svg"),
        tr("Save point cloud")
    );
    float_save_point_cloud_action_->setEnabled(false);
    
    if (continuous_deal_point_) {
        connect(float_save_point_cloud_action_, &QAction::triggered, continuous_deal_point_, &ys_scene::ContinuousDealPoint::ShowModal);
    }

    // 添加配置树按钮
    float_config_tree_action_ = floating_toolbar_->addAction(
        QIcon(":/image/icons/configure_tree_unavailable.svg"),
        tr("Device Attributes")
    );
    float_config_tree_action_->setEnabled(false);

    connect(float_config_tree_action_, &QAction::triggered, this, [this]() {
        if (property_tree_widget_) {
            if (property_tree_widget_->isVisible()) {
                property_tree_widget_->hide();
            }
            else {
                property_tree_widget_->show();
                property_tree_widget_->raise();
            }
        }
        });
    
    // 默认显示工具栏，并确保在顶层
    floating_toolbar_->show();
    floating_toolbar_->raise();
    UpdateFloatingToolbarPosition();
}

void MainWindow::UpdateFloatingToolbarPosition()
{
    if (!floating_toolbar_) return;

    // 定位到正中心上方，Y 与 NodeManager 竖直对齐
    int x = (width() - floating_toolbar_->width()) / 2;
    int y = kPanelTopMargin;
    floating_toolbar_->move(x, y);
}

void MainWindow::CreatePickActions()
{
    // 与视角切换按钮之间加分隔符，区分功能区
    toolbar_->addSeparator();

    single_pick_overlay_action_ = new QAction(
        CreateSvgIcon(":/image/icons/pointcloud/icon_pointcloud_pick.svg", 32), tr("pointcloud standard"), this);
    single_pick_overlay_action_->setCheckable(true);
    toolbar_->addAction(single_pick_overlay_action_);

    single_pick_list_action_ = new QAction(
        CreateSvgIcon(":/image/icons/pointcloud/icon_pointcloud_picklist.svg", 32), tr("select list"), this);
    single_pick_list_action_->setCheckable(true);
    toolbar_->addAction(single_pick_list_action_);

    overlay_picker_ = new SinglePickOverlay(this);
    list_picker_    = new SinglePickList(this);
    pick_info_panel_ = new PickInfoPanel(this);
    pick_info_panel_->setDraggable(true);
    pick_info_panel_->hide();
    pick_info_panel_->setMinimumWidth(520);

    connect(single_pick_overlay_action_, &QAction::toggled,
        this, &MainWindow::onOverlayPickToggled);
    connect(single_pick_list_action_, &QAction::toggled,
        this, &MainWindow::onListPickToggled);

    connect(list_picker_, &SinglePickList::PointPicked,
        pick_info_panel_, &PickInfoPanel::AppendPickedPoint);
    connect(list_picker_, &SinglePickList::PointRemoved,
        pick_info_panel_, &PickInfoPanel::RemoveRowAt);

    connect(pick_info_panel_, &PickInfoPanel::SelectionCleared,
        list_picker_, &SinglePickList::ClearSelection);
    connect(pick_info_panel_, &PickInfoPanel::SelectionCleared,
        pointcloud_display_, [this]() { pointcloud_display_->update(); });

    connect(list_picker_, &SinglePickList::SelectionCleared,
        pick_info_panel_, &PickInfoPanel::ClearAll);

    connect(overlay_picker_, &SinglePickOverlay::PointPicked,
        pointcloud_display_, [this]() { pointcloud_display_->update(); });
    connect(overlay_picker_, &SinglePickOverlay::SelectionCleared,
        pointcloud_display_, [this]() { pointcloud_display_->update(); });

    connect(pointcloud_display_, &PointcloudDisplay::PickModeExited,
        this, [this]() {
            single_pick_overlay_action_->setChecked(false);
            single_pick_list_action_->setChecked(false);
            pick_info_panel_->hide();
        });
}

void MainWindow::onOverlayPickToggled(bool checked)
{
    if (!checked) {
        overlay_picker_->ClearSelection();
        pointcloud_display_->SetPickEngine(nullptr);
        return;
    }

    single_pick_list_action_->setChecked(false);
    pointcloud_display_->SetPickEngine(overlay_picker_);
}

void MainWindow::onListPickToggled(bool checked)
{
    if (!checked) {
        list_picker_->ClearSelection();
        pick_info_panel_->ClearAll();
        pointcloud_display_->SetPickEngine(nullptr);
        pick_info_panel_->hide();
        return;
    }

    single_pick_overlay_action_->setChecked(false);
    pointcloud_display_->SetPickEngine(list_picker_);

    int panel_w = 500;
    int panel_h = qMin(height() - 100, 600);
    int panel_x = width() - panel_w - 20;
    int panel_y = kPanelTopMargin;
    pick_info_panel_->setGeometry(panel_x, panel_y, panel_w, panel_h);
    pick_info_panel_->show();
}


namespace ys_scene {

void ComputeMultiTableSummary(const SslDistCalibMulti& table, int& out_valid_pixels, int& out_max_dist_pts)
{
    out_valid_pixels = 0;
    out_max_dist_pts = 0;
    for (int row = 0; row < SSL_MAX_ROWS; ++row) {
        for (int col = 0; col < SSL_MAX_COLS; ++col) {
            const auto& curve = table.curves[row][col];
            if (curve.num_pts >= 2) {
                ++out_valid_pixels;
                out_max_dist_pts = (std::max)(out_max_dist_pts, static_cast<int>(curve.num_pts));
            }
        }
    }
}

} // namespace ys_scene

