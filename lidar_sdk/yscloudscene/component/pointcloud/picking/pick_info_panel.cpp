#include "pick_info_panel.h"
#include <QHeaderView>
#include <QKeyEvent>
#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <QMessageBox>
#include <QHBoxLayout>
#include <algorithm>

namespace ys_scene {

PickInfoPanel::PickInfoPanel(QWidget* parent)
    : lumosy::FloatingPanel(parent)
{
    setWindowTitle(tr("点选列表"));
    setWindowFlags(Qt::Tool | Qt::FramelessWindowHint);
    setTitleBarVisible(false);
    setDraggable(false);
    setMinimumSize(680, 350);
    SetupUi();
}

void PickInfoPanel::SetupUi()
{
    auto* outer = contentLayout();
    outer->setContentsMargins(8, 8, 8, 8);

    auto* btn_row = new QHBoxLayout();

    clear_btn_ = new QPushButton(tr("清除"));
    export_btn_ = new QPushButton(tr("导出CSV"));

    btn_row->addWidget(clear_btn_);
    btn_row->addWidget(export_btn_);
    btn_row->addStretch();

    outer->addLayout(btn_row);

    table_ = new QTableWidget(0, 11, this);
    table_->setHorizontalHeaderLabels({
        tr("#"), tr("Row"), tr("Col"),
        tr("X"), tr("Y"), tr("Z"),
        tr("ρ(m)"), tr("θ°"), tr("ω°"), tr("峰值"), tr("反射率")
    });
    table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_->setSelectionMode(QAbstractItemView::SingleSelection);
    table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table_->horizontalHeader()->setStretchLastSection(true);
    table_->verticalHeader()->setVisible(false);
    table_->setColumnWidth(0, 36);
    table_->setColumnWidth(1, 46);
    table_->setColumnWidth(2, 46);
    table_->setColumnWidth(3, 64);
    table_->setColumnWidth(4, 64);
    table_->setColumnWidth(5, 64);
    table_->setColumnWidth(6, 58);
    table_->setColumnWidth(7, 52);
    table_->setColumnWidth(8, 52);
    table_->setColumnWidth(9, 52);
    table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Interactive);

    outer->addWidget(table_);

    detail_view_ = new QTextEdit(this);
    detail_view_->setReadOnly(true);
    detail_view_->setMaximumHeight(150);
    detail_view_->setPlaceholderText(tr("选中行以查看完整回波详情"));
    outer->addWidget(detail_view_);

    connect(clear_btn_, &QPushButton::clicked, this, &PickInfoPanel::ClearAll);
    connect(export_btn_, &QPushButton::clicked, this, &PickInfoPanel::ExportToCsv);
    connect(table_, &QTableWidget::currentCellChanged, this,
        [this](int row, int, int, int) {
            if (row >= 0) UpdateDetailView(row);
        });
}

void PickInfoPanel::mousePressEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton) {
        drag_start_ = event->globalPos();
        dragging_ = true;
    }

    FloatingPanel::mousePressEvent(event);
}

void PickInfoPanel::mouseMoveEvent(QMouseEvent* event)
{
    if (dragging_ && (event->buttons() & Qt::LeftButton)) {
        QPoint delta = event->globalPos() - drag_start_;
        move(pos() + delta);
        drag_start_ = event->globalPos();
        return;
    }

    FloatingPanel::mouseMoveEvent(event);
}

void PickInfoPanel::AppendPickedPoint(const PickedPointInfo& info)
{
    points_.append(info);

    int row = table_->rowCount();
    table_->insertRow(row);

    auto add = [&](int col, const QString& text) {
        auto* item = new QTableWidgetItem(text);
        item->setTextAlignment(Qt::AlignCenter);
        table_->setItem(row, col, item);
    };

    add(0, QString::number(row + 1));
    add(1, QString::number(info.row));
    add(2, QString::number(info.col));
    add(3, QString::number(info.x, 'f', 2));
    add(4, QString::number(info.y, 'f', 2));
    add(5, QString::number(info.z, 'f', 2));
    add(6, QString::number(info.rho, 'f', 2));
    add(7, QString::number(info.theta_deg, 'f', 2));
    add(8, QString::number(info.omega_deg, 'f', 2));
    add(9, QString::number(info.echoes[0].peak));
    add(10, QString::number(info.reflectivity));

    table_->scrollToBottom();
}

void PickInfoPanel::UpdateDetailView(int row)
{
    if (row < 0 || row >= points_.size()) {
        detail_view_->clear();
        return;
    }

    const auto& info = points_[row];
    QString html;
    html += QString("<b>笛卡尔坐标:</b> X=%1 Y=%2 Z=%3<br>")
        .arg(info.x, 0, 'f', 3).arg(info.y, 0, 'f', 3).arg(info.z, 0, 'f', 3);
    html += QString("<b>θ=</b>%1° <b>ω=</b>%2° <b>ρ=</b>%3m<br>")
        .arg(info.theta_deg, 0, 'f', 2).arg(info.omega_deg, 0, 'f', 2).arg(info.rho, 0, 'f', 2);
    html += "<b>回波:</b><br>";

    for (int e = 0; e < 3; ++e) {
        const auto& echo = info.echoes[e];
        if (!echo.valid) {
            html += QString("  Echo%1: 无效<br>").arg(e);
            continue;
        }
        html += QString("  Echo%1(峰值=%2): leading=%3 trailing=%4 width=%5<br>")
            .arg(e).arg(echo.peak)
            .arg(echo.leading_tof).arg(echo.trailing_tof).arg(echo.pulse_width);
    }

    detail_view_->setHtml(html);
}

void PickInfoPanel::RemoveSelectedRows()
{
    auto selected = table_->selectionModel()->selectedRows();
    if (selected.isEmpty()) {
        return;
    }

    QList<int> rows;
    for (const auto& idx : selected) {
        rows.append(idx.row());
    }
    std::sort(rows.begin(), rows.end(), std::greater<int>());

    for (int row : rows) {
        points_.removeAt(row);
        table_->removeRow(row);
    }

    if (points_.isEmpty()) {
        detail_view_->clear();
        emit SelectionCleared();
    }
}

void PickInfoPanel::RemoveRowAt(int index)
{
    if (index < 0 || index >= points_.size()) {
        return;
    }

    points_.removeAt(index);
    table_->removeRow(index);

    for (int i = 0; i < table_->rowCount(); ++i) {
        table_->item(i, 0)->setText(QString::number(i + 1));
    }

    if (points_.isEmpty()) {
        detail_view_->clear();
        emit SelectionCleared();
    }
}

void PickInfoPanel::ClearAll()
{
    if (clearing_) {
        return;
    }

    clearing_ = true;
    points_.clear();
    table_->setRowCount(0);
    detail_view_->clear();
    emit SelectionCleared();
    clearing_ = false;
}

void PickInfoPanel::ExportToCsv()
{
    if (points_.isEmpty()) {
        return;
    }

    QString path = QFileDialog::getSaveFileName(this, tr("导出CSV"), QString(), "CSV (*.csv)");
    if (path.isEmpty()) {
        return;
    }

    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::warning(this, tr("错误"), tr("无法打开文件"));
        return;
    }

    QTextStream out(&file);
    out << "Index,Row,Col,X,Y,Z,Rho,Theta,Omega,Reflectivity";
    for (int e = 0; e < 3; ++e) {
        out << QString(",Echo%1_Peak,Echo%1_Leading,Echo%1_Trailing,Echo%1_Width").arg(e);
    }
    out << "\n";

    for (int i = 0; i < points_.size(); ++i) {
        const auto& p = points_[i];
        out << (i + 1) << ","
            << p.row << "," << p.col << ","
            << p.x << "," << p.y << "," << p.z << ","
            << p.rho << "," << p.theta_deg << "," << p.omega_deg << ","
            << p.reflectivity;
        for (int e = 0; e < 3; ++e) {
            out << "," << p.echoes[e].peak
                << "," << p.echoes[e].leading_tof
                << "," << p.echoes[e].trailing_tof
                << "," << p.echoes[e].pulse_width;
        }
        out << "\n";
    }

    file.close();
}

void PickInfoPanel::keyPressEvent(QKeyEvent* event)
{
    if (event->key() == Qt::Key_Delete) {
        RemoveSelectedRows();
        return;
    }

    FloatingPanel::keyPressEvent(event);
}

} // namespace ys_scene
