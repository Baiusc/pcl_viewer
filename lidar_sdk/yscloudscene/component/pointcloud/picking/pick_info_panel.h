#pragma once

#include "picked_point_info.h"
#include <lumomo_ui/base/floating_panel.h>
#include <QTableWidget>
#include <QTextEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QList>
#include <QMouseEvent>

namespace ys_scene {

class PickInfoPanel : public lumosy::FloatingPanel
{
    Q_OBJECT

public:
    explicit PickInfoPanel(QWidget* parent = nullptr);

public slots:
    void AppendPickedPoint(const PickedPointInfo& info);
    void ClearAll();
    void RemoveRowAt(int index);

protected:
    void keyPressEvent(QKeyEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;

signals:
    void SelectionCleared();

private:
    void SetupUi();
    void UpdateDetailView(int row);
    void RemoveSelectedRows();
    void ExportToCsv();

    QTableWidget* table_{nullptr};
    QTextEdit*    detail_view_{nullptr};
    QPushButton*  clear_btn_{nullptr};
    QPushButton*  export_btn_{nullptr};

    QList<PickedPointInfo> points_;
    bool clearing_{false};
    QPoint drag_start_{};
    bool  dragging_{false};
};

} // namespace ys_scene
