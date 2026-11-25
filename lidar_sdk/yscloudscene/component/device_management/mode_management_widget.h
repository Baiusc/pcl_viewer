#pragma once
#include "business/device_management/management_client.h"
#include <QWidget>
class QLineEdit;
class QComboBox;
class QPushButton;
class QLabel;

namespace ys_scene {
class ModeManagementWidget : public QWidget
{
    Q_OBJECT
  public:
    explicit ModeManagementWidget(QWidget *parent = nullptr);
    void SetAddress(const QString &address);

    ManagementClient *Client()
    {
        return &client_;
    }
  protected:
    void showEvent(QShowEvent *event) override;
    void hideEvent(QHideEvent *event) override;
  private:
    void Update();
    ManagementClient client_;
    QLineEdit *address_;
    QComboBox *modes_;
    QPushButton *apply_;
    QLabel *state_;
    QLabel *result_;
};
} // namespace ys_scene
