#ifndef IPADDRESS_WIDGET_H
#define IPADDRESS_WIDGET_H

#include <QLabel>
#include <QMenu>
#include <QSpinBox>
#include <QTimer>
#include <QWidget>

namespace ys_scene {

class IPAddressWidget : public QWidget
{
    Q_OBJECT
  public:
    enum class IPMode { NormalIP, SubnetMask };

    explicit IPAddressWidget(QWidget *parent = nullptr, bool editable = true, IPMode mode = IPMode::NormalIP);
    ~IPAddressWidget();
    QString GetIP() const;
    void SetIP(const QString &ip);
    bool IsValid() const;
    QString GetInvalidMessage() const;

    void SetMode(IPMode mode);
    IPMode GetMode() const;

  signals:
    void IpChanged(const QString &ip);
  protected:
    bool eventFilter(QObject *obj, QEvent *event) override;

  private slots:
    void ShowSubnetMenu(const QPoint &pos);
    void ValidateWithDelay();
  private:
    QVector<QSpinBox *> octets_; // 存放4个数字框
    QVector<QLabel *> dots_;     // 3个点点
    bool editable_;
    QWidget *tooltip_;        // 提示框
    IPMode mode_;             // IP模式
    bool inputCompleted_;     // 标记用户是否完成输入
    QTimer *validationTimer_; // 延迟验证计时器

    void SetupUI();
    void ValidateAndEmit();
    void CreateTooltip();
    void ShowTooltip(const QString &message);
    void HideTooltip();
    void setPlaceholderText(const QString &text);

    QString ValidateNormalIP(int first, int second, int third, int fourth) const;
    QString ValidateSubnetMask(int first, int second, int third, int fourth) const;
};

} // namespace ys_scene

#endif // IPADDRESS_WIDGET_H