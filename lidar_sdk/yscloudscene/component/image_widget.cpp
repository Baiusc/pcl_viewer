#include <QResizeEvent>
#include <QShowEvent>
#include "image_widget.h"
#include "continuoussaveraw.h"
#include <utility>
#include <QTimer>
#include <algorithm>
#include <cstdint>
#include <cmath>
namespace ys_scene {

ImageWidget::ImageWidget(QWidget* parent, int cols, int rows) :
	QWidget(parent),
	cols_(cols),
	rows_(rows),
	user_has_interacted_(false),
	scale_factor_(1.0),
	image_(cols_, rows_, QImage::Format_Grayscale8),
	image_pos_(0, 0),
	dragging_(false)
{
	setMouseTracking(true);
	context_menu_ = new QMenu(this);

	// 直方图选择：每个菜单项携带自己的 plot 下标，点选后直接请求显示
	for (int i = 0; i < custom_plots; i++) {
		QAction* action = context_menu_->addAction(QString("custom plot %1").arg(i));
        plot_actions_.push_back(action);
		connect(action, &QAction::triggered, this, [this, i]() {
			emit PlotShowSig(i, std::make_pair(hovered_cell_.x(), hovered_cell_.y()));
		});
	}

	// 暂停/恢复：checkable 表达二态，文字随状态切换
	toggle_action_ = context_menu_->addAction(QString("hold analysis"));
	toggle_action_->setCheckable(true);
	connect(toggle_action_, &QAction::toggled, this, [this](bool paused) {
		toggle_action_->setText(paused ? QString("play") : QString("hold analysis"));
		save_frame_action_->setEnabled(paused);
		continuous_save_action_->setEnabled(!paused);
		emit PauseSig(paused);
        if (paused && !gray_mode_) {
			QTimer::singleShot(150, this, [this]() {
				emit GetFrameSig();
			});
		}
	});

	save_frame_action_ = context_menu_->addAction(QString("save frame"));
	save_frame_action_->setData(static_cast<int>(ImageCommand::kSaveFrame));
	connect(save_frame_action_, &QAction::triggered, this, &ImageWidget::HandleAction);

	continuous_save_action_ = context_menu_->addAction(QString("continuous save raws"));
	continuous_save_action_->setData(static_cast<int>(ImageCommand::kContinuousSaveRaws));
	connect(continuous_save_action_, &QAction::triggered, this, &ImageWidget::HandleAction);

    save_image_action_ = context_menu_->addAction(tr("保存灰度图 PNG"));
    save_image_action_->setVisible(false);
    connect(save_image_action_, &QAction::triggered, this, &ImageWidget::SaveImageSig);
    save_frame_action_->setEnabled(false);

	image_.fill(100);
    InitializeImageDisplay();
}

ImageWidget::~ImageWidget()
{

}

void ImageWidget::SetGrayMode(bool enabled)
{
    if (gray_mode_ == enabled) {
        return;
    }
    gray_mode_ = enabled;
    point_data_.clear();
    ema_initialized_ = false;
    hovered_cell_ = QPoint(-1, -1);
    toggle_action_->setChecked(false);
    for (auto* action : plot_actions_) {
        action->setVisible(!enabled);
    }
    save_image_action_->setVisible(enabled);
    save_frame_action_->setText(enabled ? tr("保存双回波原始帧 CSV") : QString("save frame"));
    continuous_save_action_->setText(enabled ? tr("连续保存双回波帧") : QString("continuous save raws"));
    image_.fill(0);
    update();
}

QImage ImageWidget::GetImage() const
{
    return image_;
}

void ImageWidget::SetDisplayImage(const QImage& image)
{
    if (!gray_mode_) {
        return;
    }
    if (image.isNull()) {
        image_.fill(0);
    }
    else {
        SetSize(image.width(), image.height());
        image_ = image;
    }
    update();
}

void ImageWidget::SetSize(int cols, int rows)
{
	if (cols_ == cols && rows_ == rows) {
		return;
	}
	cols_ = cols;
	rows_ = rows;
    hovered_cell_ = QPoint(-1, -1);
    point_data_.clear();
    ema_initialized_ = false;
	image_ = QImage(cols_, rows_, QImage::Format_Grayscale8);
	image_.fill(100);
	user_has_interacted_ = false;
	InitializeImageDisplay();
}

void ImageWidget::UpdataImage(const std::vector<std::vector<int>>& gray_matrix)
{
    if (gray_mode_ || gray_matrix.empty()) {
        return;
    }
	if (static_cast<int>(gray_matrix.size()) != cols_ ||
		static_cast<int>(gray_matrix[0].size()) != rows_) {
		return;
	}

	for (int x = 0; x < cols_; ++x) {
		for (int y = 0; y < rows_; ++y) {
			int gray_value = gray_matrix[x][y];
			image_.setPixel(x, y, qRgb(gray_value, gray_value, gray_value));
		}
	}
	update();
}

void ImageWidget::UpdataImage2(const int& slot, const std::vector<uint16_t>& gray_vec)
{
    if (gray_mode_) {
        return;
    }
	if (static_cast<int>(gray_vec.size()) != rows_) {
		return;
	}

	// 按 slot 累积，收齐整帧后再统一归一化
	point_data_[slot] = gray_vec;
	if (static_cast<int>(point_data_.size()) < cols_) {
		return;
	}

	// 1. 求全帧 min/max
	uint16_t frame_min = UINT16_MAX;
	uint16_t frame_max = 0;
	for (const auto& item : point_data_) {
		for (uint16_t v : item.second) {
			if (v < frame_min) frame_min = v;
			if (v > frame_max) frame_max = v;
		}
	}
	if (frame_min == UINT16_MAX) frame_min = 0;

	// 2. EMA 平滑 min/max，减少帧间闪烁
	if (!ema_initialized_) {
		ema_min_ = static_cast<float>(frame_min);
		ema_max_ = static_cast<float>(frame_max);
		ema_initialized_ = true;
	}
	else {
		ema_min_ = ema_alpha_ * static_cast<float>(frame_min) + (1.0f - ema_alpha_) * ema_min_;
		ema_max_ = ema_alpha_ * static_cast<float>(frame_max) + (1.0f - ema_alpha_) * ema_max_;
	}

	// 3. 用平滑后的范围归一化写入 image_
	float range = std::max(1.0f, ema_max_ - ema_min_);
	uchar* raw_data = image_.bits();
	int bytes_per_line = image_.bytesPerLine();
	int img_width = image_.width();
	int img_height = image_.height();
	for (const auto& item : point_data_) {
		int x = item.first;
		if (x < 0 || x >= img_width) continue;
		const auto& vec = item.second;
		int y_limit = std::min(static_cast<int>(vec.size()), img_height);
		for (int y = 0; y < y_limit; ++y) {
			float normalized = (static_cast<float>(vec[y]) - ema_min_) / range * 255.0f;
			normalized = std::max(0.0f, std::min(255.0f, normalized));
			raw_data[y * bytes_per_line + x] = static_cast<uchar>(normalized);
		}
	}

	point_data_.clear();
	update();
}

void ImageWidget::UpdateFullFrame(int cols, int rows, const std::vector<uint16_t>& gray_vec)
{
    if (gray_mode_) {
        return;
    }
    SetSize(cols, rows);
    if (static_cast<int>(gray_vec.size()) != cols_ * rows_) {
        return;
    }

    // 整帧刷新与按 slot 累积路径互斥，清掉累积缓存避免切换模式后脏数据
    point_data_.clear();

    // 1. 求全帧 min/max
	uint16_t frame_min = UINT16_MAX;
	uint16_t frame_max = 0;
	for (uint16_t v : gray_vec) {
		if (v < frame_min) frame_min = v;
		if (v > frame_max) frame_max = v;
	}
	if (frame_min == UINT16_MAX) frame_min = 0;

	// 2. EMA 平滑 min/max，减少帧间闪烁
	if (!ema_initialized_) {
		ema_min_ = static_cast<float>(frame_min);
		ema_max_ = static_cast<float>(frame_max);
		ema_initialized_ = true;
	}
	else {
		ema_min_ = ema_alpha_ * static_cast<float>(frame_min) + (1.0f - ema_alpha_) * ema_min_;
		ema_max_ = ema_alpha_ * static_cast<float>(frame_max) + (1.0f - ema_alpha_) * ema_max_;
	}

	// 3. 用平滑后的范围归一化写入 image_
	float range = std::max(1.0f, ema_max_ - ema_min_);
	uchar* raw_data = image_.bits();
	int bytes_per_line = image_.bytesPerLine();
	for (int row = 0; row < rows_; ++row) {
		for (int col = 0; col < cols_; ++col) {
			float normalized = (static_cast<float>(gray_vec[row * cols_ + col]) - ema_min_) / range * 255.0f;
			normalized = std::max(0.0f, std::min(255.0f, normalized));
			raw_data[row * bytes_per_line + col] = static_cast<uchar>(normalized);
		}
	}
	update();
}

void ImageWidget::DrawHoveredCell(QPainter& painter)
{
	if (hovered_cell_.x() >= 0 && hovered_cell_.y() >= 0)
	{
		// 计算每个格子的大小（图像坐标系）
		double cellWidth  = static_cast<double>(image_.width())  / static_cast<double>(cols_);
		double cellHeight = static_cast<double>(image_.height()) / static_cast<double>(rows_);

		// 计算悬停格子在图像坐标系中的位置
		QRectF cellRectInImage(hovered_cell_.x() * cellWidth,
			hovered_cell_.y() * cellHeight,
			cellWidth,
			cellHeight);

		// 将格子矩形从图像坐标系映射到窗口坐标系
		QRectF cellRectInWidget(
			cellRectInImage.left() * scale_factor_ + image_pos_.x(),
			cellRectInImage.top() * scale_factor_ + image_pos_.y(),
			cellRectInImage.width() * scale_factor_,
			cellRectInImage.height() * scale_factor_);

		// 设置填充颜色为半透明红色
		painter.setPen(Qt::NoPen);
		painter.setBrush(QColor(255, 0, 0));

		// 绘制填充矩形
		painter.drawRect(cellRectInWidget);

		// 绘制文字
		painter.setPen(QColor("#1cec6e"));
		QFont textFont = painter.font();
		textFont.setPointSize(25);  // 字体大小可根据需要调整
		textFont.setFamily("Arial");
		textFont.setBold(true);
		painter.setFont(textFont);

		// 计算文字位置（在窗口坐标系中）
		QPointF textPos = cellRectInWidget.topLeft() + QPointF(5, 15);

		// 绘制文字
		painter.drawText(textPos, QString("%1, %2").arg(hovered_cell_.x()).arg(hovered_cell_.y()));
	}
}

QPoint ImageWidget::MapToImage(const QPointF& widget_pos, QPointF& draw_grid) const
{
	QPointF image_posf = (widget_pos - image_pos_) / scale_factor_;
	draw_grid = image_posf;
	return QPoint(static_cast<int>(std::floor(image_posf.x())),
				static_cast<int>(std::floor(image_posf.y())));
}

QPoint ImageWidget::CalculateGridPosition(const QPoint& image_pos) const
{
	if (image_pos.x() < 0 || 
		image_pos.y() < 0 || 
		image_pos.x() >= image_.width() || 
		image_pos.y() >= image_.height()) {
		return QPoint(-1, -1);
	}
	int gridx = image_pos.x() / (image_.width()  / cols_);
	int gridy = image_pos.y() / (image_.height() / rows_);
	return QPoint(gridx, gridy);
}

void ImageWidget::InitializeImageDisplay()
{
    // No-op: auto-fit is now handled in paintEvent when !user_has_interacted_
    update();
}

QSize ImageWidget::sizeHint() const
{
    return QSize(400, 400);
}

QSize ImageWidget::minimumSizeHint() const
{
    return QSize(100, 100);
}

void ImageWidget::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
    update();
}

void ImageWidget::showEvent(QShowEvent* event)
{
    QWidget::showEvent(event);
    update();
}

void ImageWidget::paintEvent(QPaintEvent* event)
{
	QPainter painter(this);
	painter.setRenderHint(QPainter::Antialiasing);

	painter.fillRect(rect(), QColor(25, 25, 25));

	// Auto-fit: recalculate scale and position every frame until user interacts
	if (!user_has_interacted_ && width() > 0 && height() > 0 &&
	    image_.width() > 0 && image_.height() > 0) {
		double sx = static_cast<double>(width()) / image_.width();
		double sy = static_cast<double>(height()) / image_.height();
		scale_factor_ = std::min(sx, sy);
		// Center the image in the widget
		double scaled_w = image_.width() * scale_factor_;
		double scaled_h = image_.height() * scale_factor_;
		image_pos_.setX((width() - scaled_w) / 2.0);
		image_pos_.setY((height() - scaled_h) / 2.0);
	}

	// 计算图像在窗口中的绘制区域（考虑缩放和拖动）
	QSizeF scaledImageSize = image_.size() * scale_factor_;
	QRectF imageRect(image_pos_, scaledImageSize);

	// 绘制缩放和平移后的图像
	painter.drawImage(imageRect, image_);

	// 绘制悬停的格子
	DrawHoveredCell(painter);
}

void ImageWidget::wheelEvent(QWheelEvent* event)
{
	// 获取鼠标在窗口坐标系中的位置
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
	QPointF mousePos = event->position();
#else
	QPointF mousePos = event->pos();
#endif

	// 计算缩放前，鼠标位置在图像坐标系中的位置
	QPointF imagePosBeforeScale = (mousePos - image_pos_) / scale_factor_;

	// 根据滚轮方向调整缩放因子，调整缩放步长为1.05
	double scaleFactor = (event->angleDelta().y() > 0) ? 1.05 : 1.0 / 1.05;
	scale_factor_ *= scaleFactor;

	// 限制缩放因子范围（调整最大值为50.0）
	scale_factor_ = qBound(0.1, scale_factor_, 50.0);

	// 计算缩放后，图像在窗口坐标系中的新偏移量，以实现以鼠标位置为中心的缩放
	image_pos_ = mousePos - imagePosBeforeScale * scale_factor_;

	update();
	user_has_interacted_ = true;
}

void ImageWidget::mouseMoveEvent(QMouseEvent* event)
{
	if (dragging_) {
		QPointF delta = event->localPos() - last_point_pos_;
		image_pos_ += delta;
		last_point_pos_ = event->localPos();
		update();
		user_has_interacted_ = true;
	}
	else {
		// 将鼠标位置从窗口坐标系映射到图像坐标系
		QPointF imagePos = (event->localPos() - image_pos_) / scale_factor_;

		// 检查鼠标是否在图像范围内
		if (imagePos.x() >= 0 && imagePos.x() < image_.width() &&
			imagePos.y() >= 0 && imagePos.y() < image_.height()) {

			// 计算每个格子的大小
			double cellWidth  = static_cast<double>(image_.width())  / static_cast<double>(cols_);
			double cellHeight = static_cast<double>(image_.height()) / static_cast<double>(rows_);

			// 计算悬停的格子索引
			int gridX = static_cast<int>(imagePos.x() / cellWidth);
			int gridY = static_cast<int>(imagePos.y() / cellHeight);

			// 更新悬停格子
			hovered_cell_ = QPoint(gridX, gridY);
            emit PixelHoveredSig(gridX, gridY);
			update();
		}
		else {
			// 如果鼠标不在图像范围内，取消悬停格子
			hovered_cell_ = QPoint(-1, -1);
            emit PixelHoveredSig(-1, -1);
			update();
		}
	}
}

void ImageWidget::mousePressEvent(QMouseEvent* event)
{
	if (event->button() == Qt::LeftButton) {
		last_point_pos_ = event->localPos();
		dragging_ = true;
	}
}

void ImageWidget::mouseReleaseEvent(QMouseEvent* event)
{
	if (event->button() == Qt::LeftButton) {
		dragging_ = false;
	}
}

void ImageWidget::contextMenuEvent(QContextMenuEvent* event)
{
	/*QPointF scaledPos = event->pos() / scale_factor_;
	int cellWidth = ToInt(RawParser::slot_counts) / ToInt(RawParser::slot_counts);
	int cellHeight = ToInt(RawParser::slot_s_pixels) / ToInt(RawParser::slot_s_pixels);

	int col = scaledPos.x() / cellWidth;
	int row = scaledPos.y() / cellHeight;*/

	if (hovered_cell_.y() >= 0 && hovered_cell_.y() < rows_ &&
		hovered_cell_.x() >= 0 && hovered_cell_.x() < cols_)
	{
		context_menu_->exec(event->globalPos());
	}
}

void ImageWidget::HandleAction()
{
	QAction* action = qobject_cast<QAction*>(sender());
	if (!action) {
		return;
	}

	switch (static_cast<ImageCommand>(action->data().toInt())) {
	case ImageCommand::kSaveFrame:
		emit SaveFrameSig();
		break;
	case ImageCommand::kContinuousSaveRaws: {
        if (gray_mode_) {
            // Gray 的帧数/目录选择由灰度面板处理，无直方图采集参数。
            emit ContinuousSaveGraySig();
            break;
        }
		ContinuousSaveRaw dialog(this);
		if (dialog.exec() == QDialog::Accepted) {
			emit ContinuousSaveSig(dialog.GetSaveParams());
		}
		break;
	}
	}
}

}// namespace ys_scene
