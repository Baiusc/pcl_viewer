#pragma once
#include "ys_lidar_validate_type.h"
#include <qcustomplot/qcustomplot.h>
#include <QWidget>
#include <QVector>
#include <QMutex>
#include <QMenu>

QT_BEGIN_NAMESPACE
namespace Ui { class motor_widget; };
QT_END_NAMESPACE

namespace ys_scene {
class MotorWidget : public QWidget
{
	Q_OBJECT

public:
	MotorWidget(QWidget* parent = nullptr);
	~MotorWidget();
	void SetMotorEncoderValueCallback();

signals:
	void MotorSetSig(uint8_t type, uint32_t value);
	void saveOperationFinished(bool success, QString message);

private slots:
	static void MotorParamsTrampoline(const uint32_t handle, const MotorParamsQuery* device_status, void* context);
	static void MotorEncoderValueTrampoline(const uint32_t handle, const MotorEncoderReportT* encoder_valus, void* context);
	void MotorQueryHandle(const uint32_t handle, const MotorParamsQuery* device_status, void* context);
	void MotorEncoderValueHandle(const uint32_t handle, const MotorEncoderReportT* encoder_values, void* context);
	void UpdatePlot(); // 更新码盘刻度图表
	void UpdateSpeedPlot(); // 更新速度图表
	void onSaveOperationFinished(bool success, QString message);

private:
	void PlotRightKeyEnhancement();
	void SpeedPlotRightKeyEnhancement();
	void SaveSelectedData();
	void SaveSelectedSpeedData();
	double CalculateSpeed(double current_value, double prev_value, double time_diff_us);

private:
	Ui::motor_widget* ui;
	QVector<double> motor_time_data_; // 存储时间数据（微秒）
	QVector<double> motor_code_disk_scale_data_; // 存储码盘刻度数据(现在用于存储speed_rpm)
	QVector<double> motor_speed_data_; // 存储速度数据（RPM）
	QVector<double> motor_angle_data_; // 存储累积角度数据

	uint32_t last_disk_scale_ = 0;
	int64_t disk_cycle_count_ = 0;

	bool paused_ = false; // 是否暂停更新
	int max_data_points_ = 125000; // 最大数据点数
	QMutex data_mutex_; // 保护数据的互斥锁
	bool user_zoomed_code_disk_{ false }; // 码盘刻度图用户缩放标志
	bool user_zoomed_speed_{ false }; // 速度图用户缩放标志

	double refresh_interval_ = 0.2; // 默认刷新间隔(秒)
	int accumulated_samples_ = 0;   // 用于追踪累积的样本数量

	// 码盘刻度图的选区相关变量
	bool selecting_code_disk_data_ = false;
	QCPItemRect* code_disk_selection_rect_ = nullptr;
	QPoint code_disk_selection_start_;
	QMenu* code_disk_context_menu_ = nullptr;

	// 速度图的选区相关变量
	bool selecting_speed_data_ = false;
	QCPItemRect* speed_selection_rect_ = nullptr;
	QPoint speed_selection_start_;
	QMenu* speed_context_menu_ = nullptr;


	// 时间相关
	double sample_interval_us_ = 200.0; // 采样间隔（微秒）
	QDateTime start_time_; // 开始时间
	double last_value_ = 0.0; // 上一个码盘值，用于计算速度
	bool has_last_value_ = false; // 是否有上一个码盘值

	bool first_call_ = true; // 标记 MotorEncoderValueHandle 是否是第一次调用
	double base_time_us_ = 0.0; // 基准时间（微秒）
	int64_t point_count_ = 0; // 点计数

	QCPRange last_speed_x_range_;     // 速度图上一次的X轴范围
	QCPRange last_speed_y_range_;     // 速度图上一次的Y轴范围
	bool first_speed_plot_ = true;    // 速度图是否是首次绘制
};
}// namespace ys_scene