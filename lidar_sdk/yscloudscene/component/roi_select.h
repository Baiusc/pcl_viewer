#ifndef ROI_SELECT_H
#define ROI_SELECT_H

#include "ys_scene_def.h"
#include <QDialog>

namespace Ui {
class ROISelect;
}

namespace ys_scene {
class ROISelect : public QDialog
{
    Q_OBJECT

public:
    explicit ROISelect(QWidget* parent = nullptr);
    ~ROISelect();
    ROISelectType GetRoiSelectType();

signals:
    void ROISelectTypeSig(ROISelectType type, PointCloudROIA roi_select_data);
    void OptionalSlotNumberSig(int slot_number);
    void RawHistogramOffsetSig(int value);
    void RawHistogramDataProportion(double proportion);

private slots:
  void OnMaxCalOffsetSliderValueChanged(int value);
  void OnHistogramDataProportionValueChanged(int proportion);

private:
    Ui::ROISelect* ui;
    ROISelectType roi_select_type_;
    PointCloudROIA roi_select_;
    int optional_slot_number_ = 0;
};
}
#endif // ROI_SELECT_H
