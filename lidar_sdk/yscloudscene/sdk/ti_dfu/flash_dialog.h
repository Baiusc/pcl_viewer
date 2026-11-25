#pragma once

#include <QDialog>

#include "flasher.h"

class QCloseEvent;

namespace ti_dfu {

class FlashDialog : public QDialog
{
public:
    explicit FlashDialog(QWidget* parent = nullptr);
    void reject() override;

protected:
    void closeEvent(QCloseEvent* event) override;

private:
    Flasher flasher_;
};

} // namespace ti_dfu
