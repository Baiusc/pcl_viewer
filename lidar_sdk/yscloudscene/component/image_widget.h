#ifndef YS_SCENE_IMAGE_WIDGET_H_
#define YS_SCENE_IMAGE_WIDGET_H_

#include "ys_lidar_validate_type.h"
#include <QWidget>
#include <QImage>
#include <QPainter>
#include <QWheelEvent>
#include <QMenu>
#include <QAction>
#include <unordered_map>
#include <vector>
#include <cstdint>

namespace ys_scene {
using namespace you_see;

class ImageWidget : public QWidget {
    Q_OBJECT
public:
    ImageWidget(QWidget* parent = nullptr, int cols = RawParser::kRawColsCs96,
              int rows = RawParser::kRawRowsCs96);
    ~ImageWidget();
    void SetGrayMode(bool enabled);
    QImage GetImage() const;

public slots:
    void UpdataImage(const std::vector<std::vector<int>>& gray_matrix);
    void UpdataImage2(const int& slot, const std::vector<uint16_t>& gray_vec);
    void UpdateFullFrame(int cols, int rows, const std::vector<uint16_t>& gray_vec);
    void SetSize(int cols, int rows);
    void SetDisplayImage(const QImage& image);

private slots:
    void HandleAction();

signals:
    void PlotShowSig(int plot_index, std::pair<int, int> location);
    void SaveFrameSig();
    void PauseSig(bool pause);
    void GetFrameSig();
    void ContinuousSaveSig(std::tuple<int, std::string> save_info);
    void ContinuousSaveGraySig();
    void SaveImageSig();
    void PixelHoveredSig(int x, int y);

private:
    // 菜单命令 ID：通过 QAction::setData 携带，槽内按 ID 分派（避免字符串匹配）
    enum class ImageCommand : int {
        kSaveFrame = 0,
        kContinuousSaveRaws,
    };

    void DrawHoveredCell(QPainter& painter);
    QPoint MapToImage(const QPointF& widget_pos, QPointF& draw_grid) const;
    QPoint CalculateGridPosition(const QPoint& image_pos) const;
    void InitializeImageDisplay();

  protected:
    void paintEvent(QPaintEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void contextMenuEvent(QContextMenuEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;
    QSize sizeHint() const override;
    QSize minimumSizeHint() const override;
    void showEvent(QShowEvent* event) override;

private:
    int cols_;
    int rows_;
    bool user_has_interacted_;
    double scale_factor_;
    QImage image_;
    QPoint hovered_cell_;
    QPointF hovered_draw_pos_;
    QPointF image_pos_;
    QPointF last_point_pos_;
    bool dragging_;
    QMenu* context_menu_{nullptr};
    // 暂停/恢复（checkable 表达二态）、保存单帧、连续保存菜单项
    QAction* toggle_action_{nullptr};
    QAction* save_frame_action_{nullptr};
    QAction* continuous_save_action_{nullptr};
    static constexpr int custom_plots{6};
    bool gray_mode_{false};
    std::vector<QAction*> plot_actions_;
    QAction* save_image_action_{nullptr};

    // 归一化：按 slot 累积整帧数据后再统一归一化（beta/alpha 走此路径）
    std::unordered_map<int, std::vector<uint16_t>> point_data_;
    // EMA 平滑归一化范围(min/max)，减少帧间闪烁
    float ema_min_{0.0f};
    float ema_max_{1.0f};
    bool ema_initialized_{false};
    static constexpr float ema_alpha_{0.15f}; // EMA 平滑系数，越小越平滑
};

}// namespace ys_scene
#endif // YS_SCENE_IMAGE_WIDGET_H_
