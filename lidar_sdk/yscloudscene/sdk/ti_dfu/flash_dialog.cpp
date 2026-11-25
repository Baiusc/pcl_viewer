#include "flash_dialog.h"

#include <QCloseEvent>
#include <QComboBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QTextCursor>
#include <QVBoxLayout>

namespace ti_dfu {

FlashDialog::FlashDialog(QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle(Flasher::tr("TI 固件烧录"));
    resize(760, 500);
    auto* layout = new QVBoxLayout(this);
    auto* settings = new QWidget(this);
    auto* form = new QFormLayout(settings);
    auto add_path = [this, form](const QString& title, bool directory, const QString& filter) {
        auto* row = new QWidget(this);
        auto* row_layout = new QHBoxLayout(row);
        row_layout->setContentsMargins(0, 0, 0, 0);
        auto* edit = new QLineEdit(row);
        auto* browse = new QPushButton(Flasher::tr("浏览…"), row);
        row_layout->addWidget(edit);
        row_layout->addWidget(browse);
        form->addRow(title, row);
        connect(browse, &QPushButton::clicked, this, [this, edit, title, directory, filter]() {
            const QString path = directory ? QFileDialog::getExistingDirectory(this, title, edit->text())
                                           : QFileDialog::getOpenFileName(this, title, edit->text(), filter);
            if (!path.isEmpty()) {
                edit->setText(path);
            }
        });
        return edit;
    };
    auto* workflow = new QComboBox(settings);
    workflow->addItem(Flasher::tr("Flash Writer（工厂镜像 → flash 接口）"), int(Workflow::KFlashWriter));
    workflow->addItem(Flasher::tr("专用两阶段更新（update.img → tispl.bin 接口）"), int(Workflow::KDirectUpdate));
    form->addRow(Flasher::tr("烧录流程"), workflow);
    auto* tool = add_path(Flasher::tr("dfu-util"), false, Flasher::tr("可执行文件 (*.exe);;所有文件 (*)"));
    auto* boot = add_path(Flasher::tr("boot 文件目录"), true, {});
    auto* image = add_path(Flasher::tr("固件镜像"), false, Flasher::tr("固件镜像 (*.img *.bin);;所有文件 (*)"));
    layout->addWidget(settings);

    auto* status = new QLabel(Flasher::tr("请让目标设备进入 USB DFU 模式，并确认镜像与设备型号匹配。"), this);
    status->setWordWrap(true);
    layout->addWidget(status);
    auto* progress = new QProgressBar(this);
    layout->addWidget(progress);
    auto* log = new QPlainTextEdit(this);
    log->setReadOnly(true);
    log->setMaximumBlockCount(2000);
    layout->addWidget(log, 1);
    auto* start = new QPushButton(Flasher::tr("开始烧录"), this);
    layout->addWidget(start, 0, Qt::AlignRight);

    connect(start, &QPushButton::clicked, this, [this, settings, status, progress, log, start, tool, boot, image, workflow]() {
        if (tool->text().trimmed().isEmpty() || boot->text().trimmed().isEmpty() || image->text().trimmed().isEmpty()) {
            status->setText(Flasher::tr("请选择 dfu-util、boot 文件目录和固件镜像。"));
            return;
        }
        const auto selected_workflow = static_cast<Workflow>(workflow->currentData().toInt());
        const QString target = selected_workflow == Workflow::KFlashWriter
            ? QStringLiteral("flash") : QStringLiteral("tispl.bin");
        if (QMessageBox::question(this, windowTitle(),
                                  Flasher::tr("将 %1 发送到设备的 %2 接口。请确认镜像与所选流程匹配。")
                                      .arg(QFileInfo(image->text()).fileName(), target),
                                  QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes) {
            return;
        }
        log->clear();
        progress->reset();
        const Options options{tool->text().trimmed(), boot->text().trimmed(), image->text().trimmed(),
                              selected_workflow};
        if (flasher_.Start(options)) {
            settings->setEnabled(false);
            start->setEnabled(false);
        }
    });
    connect(&flasher_, &Flasher::StageChanged, this, [status](const QString& name, int index, int total) {
        status->setText(Flasher::tr("第 %1 / %2 阶段：%3").arg(index).arg(total).arg(name));
    });
    connect(&flasher_, &Flasher::ProgressChanged, this, [progress](int percent) {
        progress->setRange(0, percent < 0 ? 0 : 100);
        if (percent >= 0) {
            progress->setValue(percent);
        }
    });
    connect(&flasher_, &Flasher::LogReceived, this, [log](QString output) {
        output.replace(QStringLiteral("\r\n"), QStringLiteral("\n"));
        output.replace('\r', '\n');
        log->moveCursor(QTextCursor::End);
        log->insertPlainText(output);
        log->ensureCursorVisible();
    });
    connect(&flasher_, &Flasher::Completed, this,
            [settings, status, progress, start](bool success, const QString& detail) {
        settings->setEnabled(true);
        start->setEnabled(true);
        if (progress->maximum() == 0) {
            progress->setRange(0, 100);
            progress->reset();
        }
        status->setText(success ? detail : Flasher::tr("烧录失败：%1").arg(detail));
    });
}

void FlashDialog::reject()
{
    if (!flasher_.IsRunning()) {
        QDialog::reject();
    }
}

void FlashDialog::closeEvent(QCloseEvent* event)
{
    if (flasher_.IsRunning()) {
        event->ignore();
        return;
    }
    QDialog::closeEvent(event);
}

} // namespace ti_dfu
