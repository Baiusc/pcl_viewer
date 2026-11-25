#include "motor_widget.h"
#include "ui_motor_widget.h"
#include "log/logging.h"
#include "ys_lidar_validate_user.h"
#include "tiny_hut/common_toolset.h"
#include "ys_lidar_const.h"


#include <winsock2.h>
#include <QtConcurrent/QtConcurrent>
#include <QFuture>
#include <QFutureWatcher>
#include <QMessagebox>
#include <QFile>
#include <QTextStream>
#include <QDateTime>
#include <QOpenGLWidget>
#include <limits>
#include <cmath>

namespace ys_scene {

	using namespace you_see;

MotorWidget::MotorWidget(QWidget* parent)
	: QWidget(parent)
	, ui(new Ui::motor_widget())
{
	ui->setupUi(this);
	ui->ds_motor_stop_angle->setRange(0.0, 360.0);
	ui->ds_motor_stop_angle->setSingleStep(0.01);
	ui->ds_motor_stop_angle->setValue(89.10);

	QDoubleSpinBox* spin_boxes[] = {
		ui->ds_face1_horizon_zero_offset,
		ui->ds_face2_horizon_zero_offset,
		ui->ds_face3_horizon_zero_offset,
		ui->ds_face4_horizon_zero_offset
	};

	// 批量设置所有QDoubleSpinBox的属性
	for (auto spin_box : spin_boxes)
	{
		// 设置范围为0~5度
		spin_box->setRange(0.0, 65535.0);
		spin_box->setDecimals(0);
		spin_box->setSingleStep(1.0);
		// 可选：设置后缀为度符号
		//spin_box->setSuffix("°");
	}

	QDoubleSpinBox* spin_boxes1[] = {
		ui->dsb_pid_p,
		ui->dsb_pid_i,
		ui->dsb_pid_d
	};

	for (auto spin_box : spin_boxes1)
	{
		// 设置范围为0~5度
		spin_box->setRange(0.0, 50.0);
		spin_box->setDecimals(0);
		spin_box->setSingleStep(1.0);
		// 可选：设置后缀为度符号
		//spin_box->setSuffix("°");
	}

	connect(ui->pb_mirror_horizental_zero_offset_query, &QPushButton::clicked, [&]() {
		QueryMotorParams(0, MotorAQueryType::KHorizentalZeroOffset, &MotorWidget::MotorParamsTrampoline, this);
		});
	connect(ui->pb_pid_query, &QPushButton::clicked, [&]() {
		QueryMotorParams(0, MotorAQueryType::KPIDClosedLoopParams, &MotorWidget::MotorParamsTrampoline, this);
		});
	// 创建按钮和对应面值的映射
	QVector<QPair<QPushButton*, QPair<QDoubleSpinBox*, int>>> button_face_map = {
		{ui->pb_face1_horizental_zero_offset_set, {ui->ds_face1_horizon_zero_offset, 0}},
		{ui->pb_face2_horizental_zero_offset_set, {ui->ds_face2_horizon_zero_offset, 1}},
		{ui->pb_face3_horizental_zero_offset_set, {ui->ds_face3_horizon_zero_offset, 2}},
		{ui->pb_face4_horizental_zero_offset_set, {ui->ds_face4_horizon_zero_offset, 3}}
	};

	// 为每个按钮设置连接
	for (const auto& item : button_face_map) {
		QPushButton* button = item.first;
		QDoubleSpinBox* spin_box = item.second.first;
		int mirrorFace = item.second.second;

		connect(button, &QPushButton::clicked, [this, spin_box, mirrorFace]() {
			MotorValueT motor_value;
			motor_value.value = static_cast<uint16_t>(spin_box->value());

			MotorParamsSetT motor_params;
			motor_params.type = 0x06;
			motor_params.mirror_face = mirrorFace;
			motor_params.value.value = motor_value.value;

			SetMotorHorizentalZeroOffset(0, &motor_params, &MotorWidget::MotorParamsTrampoline, this);
			});
	}

	connect(ui->pb_pid_modify, &QPushButton::clicked, [this]() {
		MotorPidSetT motor_pid;
		motor_pid.value[0] = static_cast<uint32_t>(true);
		motor_pid.value[1] = static_cast<uint32_t>(ui->dsb_pid_p->value());
		motor_pid.value[2] = static_cast<uint32_t>(ui->dsb_pid_i->value());
		motor_pid.value[3] = static_cast<uint32_t>(ui->dsb_pid_d->value());
		SetMotorPID(0, &motor_pid, &MotorWidget::MotorParamsTrampoline, this);

		// 添加清空数据的代码，但保持缩放比例和位置
		QMutexLocker locker(&data_mutex_);

		// 保存当前缩放和位置
		QCPRange speed_x_range = ui->motor_speed->xAxis->range();
		QCPRange speed_y_range = ui->motor_speed->yAxis->range();
		QCPRange code_disk_x_range = ui->motor_code_disk_scale->xAxis->range();
		QCPRange code_disk_y_range = ui->motor_code_disk_scale->yAxis->range();

			// 清空数据缓存
			motor_time_data_.clear();
			motor_code_disk_scale_data_.clear();
			motor_speed_data_.clear();
			motor_angle_data_.clear();
		motor_angle_data_.clear();

			// 重置计数器和基础状态变量，但保持缩放标志不变
			point_count_ = 0;
			accumulated_samples_ = 0;
			has_last_value_ = false;
			last_disk_scale_ = 0;
			disk_cycle_count_ = 0;		has_last_value_ = false;
		last_disk_scale_ = 0;
		disk_cycle_count_ = 0;

		// 标记下一次数据处理为第一次调用，重新计算基准时间
		first_call_ = true;

		// 清空图表数据
		ui->motor_code_disk_scale->graph(0)->data()->clear();
		ui->motor_speed->graph(0)->data()->clear();

		// 恢复缩放和位置
		ui->motor_speed->xAxis->setRange(speed_x_range);
		ui->motor_speed->yAxis->setRange(speed_y_range);
		ui->motor_code_disk_scale->xAxis->setRange(code_disk_x_range);
		ui->motor_code_disk_scale->yAxis->setRange(code_disk_y_range);

		// 重绘图表
		ui->motor_code_disk_scale->replot(QCustomPlot::rpQueuedReplot);
		ui->motor_speed->replot(QCustomPlot::rpQueuedReplot);

		});

	// 初始化码盘刻度图
	ui->motor_code_disk_scale->addGraph();
	ui->motor_code_disk_scale->graph(0)->setPen(QPen(Qt::blue));
	ui->motor_code_disk_scale->SetRawLabelText(tr("motor code disk scale value"));
	ui->motor_code_disk_scale->xAxis->setLabel(tr("point count"));
	ui->motor_code_disk_scale->yAxis->setLabel(tr("code disk scale value"));
	ui->motor_code_disk_scale->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
	ui->motor_code_disk_scale->setOpenGl(true);
	bool isOpenGLEnabled = ui->motor_code_disk_scale->openGl();
	qDebug() << "Motor Code Disk Scale OpenGL enabled:" << isOpenGLEnabled;
	// 初始化速度图
	ui->motor_speed->addGraph();
	ui->motor_speed->graph(0)->setPen(QPen(Qt::red));
	ui->motor_speed->SetRawLabelText(tr("motor speed"));
	ui->motor_speed->xAxis->setLabel(tr("time(seconds)"));
	ui->motor_speed->yAxis->setLabel(tr("speed(RPM)"));
	ui->motor_speed->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
	ui->motor_speed->setOpenGl(true);
	bool is_OpenGLEnabled = ui->motor_speed->openGl();
	qDebug() << "Motor Speed OpenGL enabled:" << is_OpenGLEnabled;

	// 设置刷新率下拉框
	ui->cb_refresh_interval->addItem("0.1s (10Hz)", 0.1);
	ui->cb_refresh_interval->addItem("0.2s (5Hz)", 0.2);
	ui->cb_refresh_interval->addItem("0.5s (2Hz)", 0.5);
	ui->cb_refresh_interval->addItem("1.0s (1Hz)", 1.0);
	ui->cb_refresh_interval->addItem("2.0s (0.5Hz)", 2.0);
	ui->cb_refresh_interval->addItem("5.0s (0.2Hz)", 5.0);

	// 设置默认值为0.2s
	ui->cb_refresh_interval->setCurrentIndex(1);

	connect(ui->cb_refresh_interval, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int index) {
		refresh_interval_ = ui->cb_refresh_interval->itemData(index).toDouble();
		// 重置累积样本计数
		accumulated_samples_ = 0;
		});

	// Initialize speed calc range
	ui->sb_speed_calc_range->setRange(0, 100000);
	ui->sb_speed_calc_range->setValue(400); // Default to ~1 second window
	ui->sb_speed_calc_range->setToolTip(tr("Number of latest data points to include in statistics "));

	ui->pb_pause_continue->setText(tr("pause"));
	connect(ui->pb_pause_continue, &QPushButton::clicked, [this]() {
		paused_ = !paused_;
		ui->pb_pause_continue->setText(paused_ ? tr("continue") : tr("pause"));

		// 如果从暂停恢复到继续，重置累积样本计数
		if (!paused_) {
			// 添加清空数据的代码，但保持缩放比例和位置
			QMutexLocker locker(&data_mutex_);

			// 保存当前缩放和位置
			QCPRange speed_x_range = ui->motor_speed->xAxis->range();
			QCPRange speed_y_range = ui->motor_speed->yAxis->range();
			QCPRange code_disk_x_range = ui->motor_code_disk_scale->xAxis->range();
			QCPRange code_disk_y_range = ui->motor_code_disk_scale->yAxis->range();

			// 清空数据缓存
			motor_time_data_.clear();
			motor_code_disk_scale_data_.clear();
			motor_speed_data_.clear();

			// 重置计数器和基础状态变量，但保持缩放标志不变
			point_count_ = 0;
			accumulated_samples_ = 0;
			has_last_value_ = false;

			// 标记下一次数据处理为第一次调用，重新计算基准时间
			first_call_ = true;

			// 清空图表数据
			ui->motor_code_disk_scale->graph(0)->data()->clear();
			ui->motor_speed->graph(0)->data()->clear();

			// 恢复缩放和位置
			ui->motor_speed->xAxis->setRange(speed_x_range);
			ui->motor_speed->yAxis->setRange(speed_y_range);
			ui->motor_code_disk_scale->xAxis->setRange(code_disk_x_range);
			ui->motor_code_disk_scale->yAxis->setRange(code_disk_y_range);

			// 重绘图表
			ui->motor_code_disk_scale->replot(QCustomPlot::rpQueuedReplot);
			ui->motor_speed->replot(QCustomPlot::rpQueuedReplot);
		}
	});

	ui->le_motor_file_prefix->setText("motor_data");
	ui->le_motor_file_prefix->setPlaceholderText(tr("Enter the file name prefix"));
	ui->le_motor_file_prefix->setToolTip(tr("The date and time will be automatically added when saving"));
	PlotRightKeyEnhancement();
	SpeedPlotRightKeyEnhancement();
	// 初始化数据容器
	motor_time_data_.reserve(max_data_points_);
	motor_code_disk_scale_data_.reserve(max_data_points_);
	motor_speed_data_.reserve(max_data_points_);
	motor_angle_data_.reserve(max_data_points_);

	// 记录开始时间
	start_time_ = QDateTime::currentDateTime();
	//connect(this, &MotorWidget::saveOperationFinished, this, &MotorWidget::onSaveOperationFinished, Qt::QueuedConnection); 
	last_speed_x_range_ = QCPRange(0, 1);
	last_speed_y_range_ = QCPRange(0, 1);
}

MotorWidget::~MotorWidget()
{
	delete ui;
}

void MotorWidget::SetMotorEncoderValueCallback()
{
	SetMotorEncoderCallback(0, &MotorWidget::MotorEncoderValueTrampoline, this);
}

void MotorWidget::MotorParamsTrampoline(const uint32_t handle, const MotorParamsQuery* device_status, void* context)
{
	auto* self = static_cast<MotorWidget*>(context);
	if (self) {
		self->MotorQueryHandle(handle, device_status, context);
	}
}

void MotorWidget::MotorEncoderValueTrampoline(const uint32_t handle, const MotorEncoderReportT* encoder_valus, void* context)
{
	auto* self = static_cast<MotorWidget*>(context);
	if (self) {
		self->MotorEncoderValueHandle(handle, encoder_valus, context);
	}
}

void MotorWidget::MotorQueryHandle(const uint32_t handle, const MotorParamsQuery* device_status, void* context)
{
	MotorAQueryType type = static_cast<MotorAQueryType>(device_status->type);
	switch (type)
	{
	case MotorAQueryType::KHorizentalZeroOffset:
	{
		int sides = static_cast<int>(PrismFaces::four_sided_mirror);
		std::array<MotorValueT, static_cast<int>(PrismFaces::four_sided_mirror)> motor_value;
		uint16_t* value_array = reinterpret_cast<uint16_t*>(const_cast<uint8_t*>(device_status->value));
		for (size_t i = 0; i < sides; i++) {
			motor_value[i].value = value_array[i]; // drop ntohl
		}

		QMetaObject::invokeMethod(this, [this, motor_value]() {
			QDoubleSpinBox* spin_boxes[] = {
				ui->ds_face1_horizon_zero_offset,
				ui->ds_face2_horizon_zero_offset,
				ui->ds_face3_horizon_zero_offset,
				ui->ds_face4_horizon_zero_offset
			};

			for (size_t i = 0; i < motor_value.size(); i++) {
				spin_boxes[i]->setValue(motor_value[i].value);
			}
		});
		break;
	}
	case MotorAQueryType::KPIDClosedLoopParams:
	{
		uint32_t pid_params[pid_params_cnt];
		uint32_t* value_array = reinterpret_cast<uint32_t*>(const_cast<uint8_t*>(device_status->value));
		for (size_t i = 0; i < pid_params_cnt; i++) {
			pid_params[i] = value_array[i]; // drop ntohl
		}

		QMetaObject::invokeMethod(this, [this, pid_params]() {
			QDoubleSpinBox* spin_boxes[] = {
				ui->dsb_pid_p,
				ui->dsb_pid_i,
				ui->dsb_pid_d
			};
			for (size_t i = 1; i < pid_params_cnt; i++) {
				spin_boxes[i-1]->setValue(static_cast<double>(pid_params[i]));
			}
		});
		break;
	}
	default:
		break;
	}
}

void MotorWidget::MotorEncoderValueHandle(const uint32_t handle, const MotorEncoderReportT* encoder_values, void* context)
{
	// 如果暂停了，不处理数据
	if (paused_)
		return;

	// 使用锁保护数据
	{
		QMutexLocker locker(&data_mutex_);

		// 获取encoder_data中的所有MotorCodeSpeedT值
		MotorCodeSpeedT* encoder_data = reinterpret_cast<MotorCodeSpeedT*>(const_cast<uint8_t*>(encoder_values->encoder_data));
		uint16_t data_count = encoder_values->encoder_cnt;

		// 只在第一次调用或从暂停恢复后的第一次调用时计算基准时间
		if (first_call_) {
			base_time_us_ = (QDateTime::currentDateTime().toMSecsSinceEpoch() - start_time_.toMSecsSinceEpoch()) * 1000.0;
			first_call_ = false;
		}

		// 添加码盘刻度数据及对应的时间
		for (uint16_t i = 0; i < data_count; i++) {
			point_count_++;
			
			uint32_t current_scale = encoder_data[i].code_disk_scale;
			double speed = encoder_data[i].speed_rpm.value.f;

			// 处理码盘值累积
			if (point_count_ > 1) {
				int32_t diff = static_cast<int32_t>(current_scale) - static_cast<int32_t>(last_disk_scale_);
				if (diff < -120) { // 正向旋转过零点 (e.g. 239 -> 0)
					disk_cycle_count_++;
				}
				else if (diff > 120) { // 反向旋转过零点 (e.g. 0 -> 239)
					disk_cycle_count_--;
				}
			}
			last_disk_scale_ = current_scale;
			
			// 计算累积角度作为X轴
			double accum_angle = disk_cycle_count_ * 240.0 + current_scale;

			// 码盘刻度图: X=累积角度, Y=速度
			motor_angle_data_.append(accum_angle);
			motor_code_disk_scale_data_.append(speed);

			// 速度图: X=时间, Y=速度
			double time_in_seconds = (base_time_us_ + (point_count_ - 1) * sample_interval_us_) / 1e6; // 转换为秒
			motor_time_data_.append(time_in_seconds);
			motor_speed_data_.append(speed);

			// 如果超过最大数据点数，移除最早的数据
			if (motor_time_data_.size() > max_data_points_) {
				motor_time_data_.removeFirst();
				motor_code_disk_scale_data_.removeFirst();
				motor_speed_data_.removeFirst();
				motor_angle_data_.removeFirst();
			}
            /*
			// 码盘刻度图使用点计数作为X轴
			point_count_++;
            double ft8215_s_value = static_cast<double>(encoder_data[i]) / 100;
			motor_code_disk_scale_data_.append(ft8215_s_value);

			// 速度图使用时间作为X轴，基于基准时间递增
			// 每个数据点时间 = 基准时间 + 累计点数 * 采样间隔
			double time_in_seconds = (base_time_us_ + (point_count_ - 1) * sample_interval_us_) / 1e6; // 转换为秒
			motor_time_data_.append(time_in_seconds);

			// 计算速度 - 每间隔3个点计算一次 (第1点和第5点)
			double speed = 0.0;

			// 确保有足够的历史数据来计算速度
			if (motor_code_disk_scale_data_.size() >= 5) {
				int current_idx = motor_code_disk_scale_data_.size() - 1;  // 当前点索引
				int reference_idx = current_idx - 4;  // 间隔3个点的参考点索引

				double current_value = motor_code_disk_scale_data_[current_idx];
				double reference_value = motor_code_disk_scale_data_[reference_idx];

				// 如果当前值和参考值相等，使用上一个有效速度值
				if (current_value == reference_value && !motor_speed_data_.isEmpty()) {
					speed = motor_speed_data_.last();
				}
				else {
					// 计算两点间的时间差（微秒）
					double time_diff_us = (time_in_seconds - motor_time_data_[reference_idx]) * 1e6;

					// 使用修改后的计算速度函数
					speed = CalculateSpeed(current_value, reference_value, time_diff_us);
				}
			}
			else if (!motor_speed_data_.isEmpty()) {
				// 如果没有足够的历史数据但有之前的速度值，使用上一个速度值
				speed = motor_speed_data_.last();
			}

			// 添加速度值到数据集
			motor_speed_data_.append(speed);

			// 如果超过最大数据点数，移除最早的数据
			if (motor_time_data_.size() > max_data_points_) {
				motor_time_data_.removeFirst();
				motor_code_disk_scale_data_.removeFirst();
				motor_speed_data_.removeFirst();
			}
            */
		}

		// 累积样本数量
		accumulated_samples_ += data_count;
	}

	double samples_per_refresh = 25 * refresh_interval_ * 400;

	// 当累积的样本数量超过每次刷新所需的样本数量时更新图表
	if (accumulated_samples_ >= samples_per_refresh) {
		accumulated_samples_ = 0; // 重置计数器

		// 使用QMetaObject::invokeMethod确保在UI线程中更新图表
		QMetaObject::invokeMethod(this, "UpdatePlot", Qt::QueuedConnection);
		QMetaObject::invokeMethod(this, "UpdateSpeedPlot", Qt::QueuedConnection);
	}
}

void MotorWidget::UpdatePlot()
{
	QMutexLocker locker(&data_mutex_);

	// 更新码盘刻度图表数据 - 使用累积角度作为X轴
	ui->motor_code_disk_scale->graph(0)->setData(motor_angle_data_, motor_code_disk_scale_data_);

	// 保存当前轴范围
	static bool user_zoomed = false;
	static QCPRange last_x_range = ui->motor_code_disk_scale->xAxis->range();
	static QCPRange last_y_range = ui->motor_code_disk_scale->yAxis->range();

	// 检测用户是否手动调整了视图
	if (ui->motor_code_disk_scale->xAxis->range() != last_x_range ||
		ui->motor_code_disk_scale->yAxis->range() != last_y_range) {
		user_zoomed_code_disk_ = true;
	}

	// 只有在用户没有手动调整或明确要求自动缩放时才自动调整轴范围
	if (!user_zoomed_code_disk_) {
		// 自动调整X轴范围以显示最新数据
		// 假设每圈240个单位，显示最近的1200个单位(5圈)
		double key_range = 1200.0; 
		double key_min = motor_angle_data_.isEmpty() ? 0 : motor_angle_data_.last() - key_range;
		double key_max = motor_angle_data_.isEmpty() ? key_range : motor_angle_data_.last();

		ui->motor_code_disk_scale->xAxis->setRange(key_min, key_max);

		// 调整Y轴范围
		double value_min = *std::min_element(motor_code_disk_scale_data_.constBegin(), motor_code_disk_scale_data_.constEnd());
		double value_max = *std::max_element(motor_code_disk_scale_data_.constBegin(), motor_code_disk_scale_data_.constEnd());
		double value_margin = (value_max - value_min) * 0.1;

		ui->motor_code_disk_scale->yAxis->setRange(value_min - value_margin, value_max + value_margin);
	}

	// Calculate statistics (Minimum and Variance)
	int calc_range = ui->sb_speed_calc_range->value();
	if (calc_range > 0 && !motor_code_disk_scale_data_.isEmpty()) {
		double min_val = std::numeric_limits<double>::max();
		double sum = 0.0;
		double sum_sq = 0.0;
		int count = 0;

		int data_size = motor_code_disk_scale_data_.size();
		int start_idx = std::max(0, data_size - calc_range);

		for (int i = start_idx; i < data_size; ++i) {
			double val = motor_code_disk_scale_data_[i];
			// if (val > 200.0) continue;

			if (val < min_val) min_val = val;
			sum += val;
			sum_sq += val * val;
			count++;
		}

		if (count > 0) {
			double mean = sum / count;
			double variance = (sum_sq / count) - (mean * mean);
			ui->lb_speed_minimum->setText(QString::number(min_val, 'f', 2));
			ui->lb_speed_variance->setText(QString::number(variance, 'f', 4));
		}
		else {
			ui->lb_speed_minimum->setText("-");
			ui->lb_speed_variance->setText("-");
		}
	}

	// 保存当前轴范围用于下次比较
	last_x_range = ui->motor_code_disk_scale->xAxis->range();
	last_y_range = ui->motor_code_disk_scale->yAxis->range();

	// 重绘图表
	//ui->motor_code_disk_scale->replot(QCustomPlot::rpQueuedReplot);
	ui->motor_code_disk_scale->replot();

}

void MotorWidget::UpdateSpeedPlot()
{
	QMutexLocker locker(&data_mutex_);

	// 更新速度图表数据
	ui->motor_speed->graph(0)->setData(motor_time_data_, motor_speed_data_);

	// 检测用户是否手动调整了视图
	if (ui->motor_speed->xAxis->range() != last_speed_x_range_ ||
		ui->motor_speed->yAxis->range() != last_speed_y_range_) {
		user_zoomed_speed_ = true;
	}

	// 当有新数据时需要调整X轴范围，但保持用户设置的缩放比例
	if (!motor_time_data_.isEmpty()) {
		double latest_time = motor_time_data_.last();

		if (first_speed_plot_) {
			// 首次绘图时，设置一个初始范围
			double key_range = 5.0; // 显示5秒数据
			ui->motor_speed->xAxis->setRange(latest_time - key_range, latest_time);
			first_speed_plot_ = false;
		}
		else if (user_zoomed_speed_) {
			// 用户已缩放，保持缩放比例，但将最新数据移至视图中心
			QCPRange current_x_range = ui->motor_speed->xAxis->range();
			double range_size = current_x_range.size();

			// 计算新范围，使最新点位于视图中心
			double new_upper = latest_time + range_size / 2;
			double new_lower = new_upper - range_size;

			// 应用新范围
			ui->motor_speed->xAxis->setRange(new_lower, new_upper);
		}
		else {
			// 用户未缩放，使用默认行为
			double key_range = 5.0; // 显示5秒数据
			double key_min = qMax(latest_time - key_range, motor_time_data_.first());
			ui->motor_speed->xAxis->setRange(key_min, latest_time);

			// 调整Y轴范围 (仅在用户未缩放时)
			if (!motor_speed_data_.isEmpty()) {
				// 只考虑当前可见区域内的数据点来调整Y轴
				QVector<double> visible_speeds;
				for (int i = 0; i < motor_time_data_.size(); ++i) {
					if (motor_time_data_[i] >= key_min && motor_time_data_[i] <= latest_time) {
						visible_speeds.append(motor_speed_data_[i]);
					}
				}

				if (!visible_speeds.isEmpty()) {
					double value_min = *std::min_element(visible_speeds.constBegin(), visible_speeds.constEnd());
					double value_max = *std::max_element(visible_speeds.constBegin(), visible_speeds.constEnd());

					// 确保Y轴有合理的范围
					if (value_max > value_min) {
						double value_margin = (value_max - value_min) * 0.1;
						ui->motor_speed->yAxis->setRange(value_min - value_margin, value_max + value_margin);
					}
					else {
						// 如果数据范围太小，设置一个默认范围
						ui->motor_speed->yAxis->setRange(-100, 100);
					}
				}
			}
		}
	}

	// 保存当前轴范围用于下次比较
	last_speed_x_range_ = ui->motor_speed->xAxis->range();
	last_speed_y_range_ = ui->motor_speed->yAxis->range();

	// 重绘图表
	ui->motor_speed->replot(QCustomPlot::rpQueuedReplot);
}

void MotorWidget::onSaveOperationFinished(bool success, QString message)
{
	if (success) {
		QMessageBox::information(this, tr("Save Successfully"),
			QString(tr("Data has been successfully saved to:\n%1")).arg(QDir::toNativeSeparators(message)));
	}
	else {
		QMessageBox::critical(this, tr("Save failed"), message);
	}
}

void MotorWidget::PlotRightKeyEnhancement()
{
	// 设置码盘刻度图的选区
	code_disk_selection_rect_ = new QCPItemRect(ui->motor_code_disk_scale);
	code_disk_selection_rect_->setVisible(false);
	code_disk_selection_rect_->setPen(QPen(QColor(255, 200, 50, 200)));
	code_disk_selection_rect_->setBrush(QBrush(QColor(255, 200, 50, 50)));

	// 码盘刻度图右键菜单
	ui->motor_code_disk_scale->setContextMenuPolicy(Qt::CustomContextMenu);
	code_disk_context_menu_ = new QMenu(this);
	code_disk_context_menu_->addAction(tr("Save selected data"), this, &MotorWidget::SaveSelectedData);
	code_disk_context_menu_->addAction(tr("Uncheck"), [this]() {
		code_disk_selection_rect_->setVisible(false);
		selecting_code_disk_data_ = false;
		ui->motor_code_disk_scale->replot();
		});

	// 鼠标事件
	connect(ui->motor_code_disk_scale, &QCustomPlot::mousePress, [this](QMouseEvent* event) {
		if (event->button() == Qt::LeftButton && paused_) {
			code_disk_selection_start_ = event->pos();
			double x = ui->motor_code_disk_scale->xAxis->pixelToCoord(code_disk_selection_start_.x());

			code_disk_selection_rect_->topLeft->setCoords(x, ui->motor_code_disk_scale->yAxis->range().upper);
			code_disk_selection_rect_->bottomRight->setCoords(x, ui->motor_code_disk_scale->yAxis->range().lower);
			code_disk_selection_rect_->setVisible(true);
			selecting_code_disk_data_ = true;
		}
		});

	connect(ui->motor_code_disk_scale, &QCustomPlot::mouseMove, [this](QMouseEvent* event) {
		// 处理选区
		if (selecting_code_disk_data_) {
			double x = ui->motor_code_disk_scale->xAxis->pixelToCoord(event->pos().x());
			code_disk_selection_rect_->bottomRight->setCoords(x, ui->motor_code_disk_scale->yAxis->range().lower);
			code_disk_selection_rect_->topLeft->setCoords(ui->motor_code_disk_scale->xAxis->pixelToCoord(code_disk_selection_start_.x()),
				ui->motor_code_disk_scale->yAxis->range().upper);
			ui->motor_code_disk_scale->replot();
		}
		});

	connect(ui->motor_code_disk_scale, &QCustomPlot::mouseRelease, [this](QMouseEvent* event) {
		if (event->button() == Qt::LeftButton && selecting_code_disk_data_) {
			selecting_code_disk_data_ = false;
			ui->motor_code_disk_scale->replot();
		}
		});

	connect(ui->motor_code_disk_scale, &QCustomPlot::customContextMenuRequested, [this](const QPoint& pos) {
		if (code_disk_selection_rect_->visible()) {
			code_disk_context_menu_->popup(ui->motor_code_disk_scale->mapToGlobal(pos));
		}
		});
}

void MotorWidget::SpeedPlotRightKeyEnhancement()
{
	// 设置速度图的选区
	speed_selection_rect_ = new QCPItemRect(ui->motor_speed);
	speed_selection_rect_->setVisible(false);
	speed_selection_rect_->setPen(QPen(QColor(255, 200, 50, 200)));
	speed_selection_rect_->setBrush(QBrush(QColor(255, 200, 50, 50)));

	// 速度图右键菜单
	ui->motor_speed->setContextMenuPolicy(Qt::CustomContextMenu);
	speed_context_menu_ = new QMenu(this);
	speed_context_menu_->addAction(tr("Save selected speed data"), this, &MotorWidget::SaveSelectedSpeedData);
	speed_context_menu_->addAction(tr("Uncheck"), [this]() {
		speed_selection_rect_->setVisible(false);
		selecting_speed_data_ = false;
		ui->motor_speed->replot();
		});

	// 鼠标事件
	connect(ui->motor_speed, &QCustomPlot::mousePress, [this](QMouseEvent* event) {
		if (event->button() == Qt::LeftButton && paused_) {
			speed_selection_start_ = event->pos();
			double x = ui->motor_speed->xAxis->pixelToCoord(speed_selection_start_.x());

			speed_selection_rect_->topLeft->setCoords(x, ui->motor_speed->yAxis->range().upper);
			speed_selection_rect_->bottomRight->setCoords(x, ui->motor_speed->yAxis->range().lower);
			speed_selection_rect_->setVisible(true);
			selecting_speed_data_ = true;
		}
		});

	connect(ui->motor_speed, &QCustomPlot::mouseMove, [this](QMouseEvent* event) {
		// 处理选区
		if (selecting_speed_data_) {
			double x = ui->motor_speed->xAxis->pixelToCoord(event->pos().x());
			speed_selection_rect_->bottomRight->setCoords(x, ui->motor_speed->yAxis->range().lower);
			speed_selection_rect_->topLeft->setCoords(ui->motor_speed->xAxis->pixelToCoord(speed_selection_start_.x()),
				ui->motor_speed->yAxis->range().upper);
			ui->motor_speed->replot();
		}
		});

	connect(ui->motor_speed, &QCustomPlot::mouseRelease, [this](QMouseEvent* event) {
		if (event->button() == Qt::LeftButton && selecting_speed_data_) {
			selecting_speed_data_ = false;
			ui->motor_speed->replot();
		}
		});

	connect(ui->motor_speed, &QCustomPlot::customContextMenuRequested, [this](const QPoint& pos) {
		if (speed_selection_rect_->visible()) {
			speed_context_menu_->popup(ui->motor_speed->mapToGlobal(pos));
		}
		});
}

void MotorWidget::SaveSelectedData()
{
	if (!code_disk_selection_rect_->visible())
		return;

	QCPRange x_range(code_disk_selection_rect_->topLeft->key(), code_disk_selection_rect_->bottomRight->key());
	if (x_range.lower > x_range.upper)
		std::swap(x_range.lower, x_range.upper);

	// 获取用户设置的文件名前缀
	QString prefix = ui->le_motor_file_prefix->text().trimmed();
	if (prefix.isEmpty()) {
		prefix = "motor_data"; // 如果为空，使用默认前缀
	}

	std::string save_dir{};

	if (!toolset::DirectoryInit("motor_data", save_dir)) {
		return;
	}
	std::string name = "motor_data" + toolset::GetCurrentTimestampAsString() + "_disk_scale_" + ".txt";
	std::string file_path = toolset::JoinPaths(save_dir, name);

	// 转换为QString并保存，确保生命周期足够长
	QString qfile_path = QString::fromStdString(file_path);

	// 复制需要的数据，避免在后台线程中使用互斥锁
	QVector<double> selected_time_data;
	QVector<double> selected_scale_data;
	QVector<double> selected_indices;

	{
		QMutexLocker locker(&data_mutex_);

		// 创建点计数索引数组
		QVector<double> indices;
		indices.reserve(motor_code_disk_scale_data_.size());
		for (int i = 0; i < motor_code_disk_scale_data_.size(); ++i) {
			double index = point_count_ - motor_code_disk_scale_data_.size() + i + 1;
			indices.append(index);
		}

		// 只复制选中范围内的数据
		for (int i = 0; i < indices.size(); ++i) {
			if (indices[i] >= x_range.lower && indices[i] <= x_range.upper) {
				selected_indices.append(indices[i]);
				selected_scale_data.append(motor_code_disk_scale_data_[i]);
				selected_time_data.append(motor_time_data_[i]);  // 同时保存对应的时间
			}
		}
	}

	// 如果没有选中数据
	if (selected_scale_data.isEmpty()) {
		QMessageBox::warning(this, tr("No data"), tr("There are no data points in the selected range"));
		return;
	}

	// 创建进度对话框
	QProgressDialog* progress = new QProgressDialog(tr("Saving data..."), tr("Cancel"), 0, selected_scale_data.size(), this);
	progress->setWindowModality(Qt::WindowModal);
	progress->setAutoClose(true);
	progress->setAutoReset(true);
	progress->setMinimumDuration(0); // 立即显示
	progress->show();

	// 使用共享的布尔值来跟踪取消状态
	QSharedPointer<bool> cancelled(new bool(false));

	// 当用户取消时，设置标志
	connect(progress, &QProgressDialog::canceled, [cancelled]() {
		*cancelled = true;
		});

	// 启动异步任务
	QFuture<void> future = QtConcurrent::run([=]() {
		QFile file(qfile_path);
		if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
			// 使用信号通知保存失败
			emit saveOperationFinished(false, tr("Unable to open file for writing"));
			return;
		}

		QTextStream out(&file);
		out << tr("Point index, Time (seconds), Code disk scale value, Angle (°)\n"); // 表头

		const int total = selected_scale_data.size();

		for (int i = 0; i < total; ++i) {
			// 检查是否取消
			if (*cancelled) {
				file.close();
				file.remove(); // 删除未完成的文件
				return;
			}

			// 写入数据
			double index = selected_indices[i];
			double time_val = selected_time_data[i];
			double scale_val = selected_scale_data[i];
			double angle = (scale_val / 65535.0) * 360.0; // 转换为角度

			out << QString::number(index, 'f', 0) << ","
				<< QString::number(time_val, 'f', 3) << ","  // 毫秒精度
				<< scale_val << ","
				<< QString::number(angle, 'f', 2) << "\n";

			// 更新进度
			QMetaObject::invokeMethod(progress, "setValue", Qt::QueuedConnection,
				Q_ARG(int, i + 1));
		}

		file.close();

		// 通知保存成功
		if (!*cancelled) {
			emit saveOperationFinished(true, qfile_path);
		}
		});

	// 当异步任务完成时，自动删除进度对话框
	QFutureWatcher<void>* watcher = new QFutureWatcher<void>(this);
	connect(watcher, &QFutureWatcher<void>::finished, [=]() {
		progress->deleteLater();
		watcher->deleteLater();
		});
	watcher->setFuture(future);
}

void MotorWidget::SaveSelectedSpeedData()
{
	if (!speed_selection_rect_->visible())
		return;

	QCPRange x_range(speed_selection_rect_->topLeft->key(), speed_selection_rect_->bottomRight->key());
	if (x_range.lower > x_range.upper)
		std::swap(x_range.lower, x_range.upper);

	// 获取用户设置的文件名前缀
	QString prefix = ui->le_motor_file_prefix->text().trimmed();
	if (prefix.isEmpty()) {
		prefix = "motor_data"; // 如果为空，使用默认前缀
	}

	std::string save_dir{};

	if (!toolset::DirectoryInit("motor_data", save_dir)) {
		return;
	}
	std::string name = "motor_data" + toolset::GetCurrentTimestampAsString() + "_speed_" + ".txt";
	std::string file_path = toolset::JoinPaths(save_dir, name);

	// 转换为QString并保存，确保生命周期足够长
	QString qfile_path = QString::fromStdString(file_path);

	// 复制需要的数据，避免在后台线程中使用互斥锁
	QVector<double> selected_time_data;
	QVector<double> selected_speed_data;

	{
		QMutexLocker locker(&data_mutex_);

		// 只复制选中范围内的数据
		for (int i = 0; i < motor_time_data_.size(); ++i) {
			if (motor_time_data_[i] >= x_range.lower && motor_time_data_[i] <= x_range.upper) {
				selected_time_data.append(motor_time_data_[i]);
				selected_speed_data.append(motor_speed_data_[i]);
			}
		}
	}

	// 如果没有选中数据
	if (selected_time_data.isEmpty()) {
		QMessageBox::warning(this, tr("No data"), tr("There are no data points in the selected range"));
		return;
	}

	// 创建进度对话框
	QProgressDialog* progress = new QProgressDialog(tr("Saving data..."), tr("Cancel"), 0, selected_time_data.size(), this);
	progress->setWindowModality(Qt::WindowModal);
	progress->setAutoClose(true);
	progress->setAutoReset(true);
	progress->setMinimumDuration(0); // 立即显示
	progress->show();

	// 使用共享的布尔值来跟踪取消状态
	QSharedPointer<bool> cancelled(new bool(false));

	// 当用户取消时，设置标志
	connect(progress, &QProgressDialog::canceled, [cancelled]() {
		*cancelled = true;
		});

	// 启动异步任务
	QFuture<void> future = QtConcurrent::run([=]() {
		QFile file(qfile_path);
		if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
			// 使用信号通知保存失败
			emit saveOperationFinished(false, tr("Unable to open file for writing"));
			return;
		}

		QTextStream out(&file);
		out << tr("Time (seconds), Speed ​​(RPM)\n"); // 表头

		const int total = selected_time_data.size();

		for (int i = 0; i < total; ++i) {
			// 检查是否取消
			if (*cancelled) {
				file.close();
				file.remove(); // 删除未完成的文件
				return;
			}

			// 写入数据
			double time_val = selected_time_data[i];
			double speed_val = selected_speed_data[i];

			out << QString::number(time_val, 'f', 6) << ","
				<< QString::number(speed_val, 'f', 2) << "\n";

			// 更新进度
			QMetaObject::invokeMethod(progress, "setValue", Qt::QueuedConnection,
				Q_ARG(int, i + 1));
		}

		file.close();

		// 通知保存成功
		if (!*cancelled) {
			emit saveOperationFinished(true, qfile_path);
		}
		});

	// 当异步任务完成时，自动删除进度对话框
	QFutureWatcher<void>* watcher = new QFutureWatcher<void>(this);
	connect(watcher, &QFutureWatcher<void>::finished, [=]() {
		progress->deleteLater();
		watcher->deleteLater();
		});
	watcher->setFuture(future);
}

double MotorWidget::CalculateSpeed(double current_value, double prev_value, double time_diff_us)
{
	// 处理数值环绕的情况
	double delta = current_value - prev_value;
	if (delta < -32768) { // 如果差值很大（负方向），可能是环绕
		delta += 65536;
	}
	else if (delta > 32768) { // 如果差值很大（正方向），可能是环绕
		delta -= 65536;
	}

	// 将码盘差值转换为弧度差值
	double rad_per_tick = 2.0 * M_PI / 65536.0;
	double rad_delta = delta * rad_per_tick;

	// 计算角速度（弧度/秒）
	double angular_velocity = rad_delta / (time_diff_us * 1e-6);

	// 转换为RPM
	double rpm = angular_velocity * 60.0 / (2.0 * M_PI);
	if (rpm > 1000.0) {
		LOG_ERROR("the rpm is:{}, the current value is:{}, the pre value is:{}", rpm, current_value, prev_value);
	}

	/*if (rpm < 130 || (rpm > 160 && rpm < 180)) {
		LOG_ERROR("the rpm is:{}, the current value is:{}, the pre value is:{}", rpm, current_value, prev_value);
	}
	*/
	return rpm;
}

} // namespace you_see