#ifndef CS96_BETA_PREVIEW_CONFIG_H
#define CS96_BETA_PREVIEW_CONFIG_H

#include <QDialog>
#include <functional>
#include <string>

class QCheckBox;
class QPushButton;
class QLabel;
class QLineEdit;
class QVBoxLayout;

namespace ys_scene {

class Cs96BetaPreviewConfig : public QDialog {
    Q_OBJECT

public:
    using LoadConfigCallback = std::function<bool(const std::string& ini_path,
        const std::string& csv_path)>;

    explicit Cs96BetaPreviewConfig(QWidget* parent = nullptr);
    ~Cs96BetaPreviewConfig() override = default;

    void SetPreviewEnabled(bool enabled);
    bool IsPreviewEnabled() const;

    void SetLoadConfigCallback(LoadConfigCallback cb);
    void SetConfigStatus(bool loaded, const std::string& status_msg);

signals:
    void PreviewEnabledChanged(bool enabled);

private slots:
    void OnBrowseIni();
    void OnBrowseCsv();
    void OnLoadConfig();

private:
    QCheckBox* cb_enable_preview_{nullptr};

    QLineEdit* le_ini_path_{nullptr};
    QPushButton* btn_browse_ini_{nullptr};
    QLineEdit* le_csv_path_{nullptr};
    QPushButton* btn_browse_csv_{nullptr};
    QPushButton* btn_load_config_{nullptr};
    QLabel* lbl_config_status_{nullptr};

    LoadConfigCallback load_config_callback_;
};

} 

#endif
