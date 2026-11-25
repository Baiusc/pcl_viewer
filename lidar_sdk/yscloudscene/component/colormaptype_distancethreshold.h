#ifndef COLORMAPTYPE_DISTANCETHRESHOLD_H
#define COLORMAPTYPE_DISTANCETHRESHOLD_H

#include "component/ys_scene_def.h"
#include "component/colormapper/colormapper.h"
#include <QDialog>

namespace Ui {
class ColorMapTypeDistanceThreshold;
}
namespace ys_scene {

class ColorMapTypeDistanceThreshold : public QDialog
{
    Q_OBJECT

public:
    explicit ColorMapTypeDistanceThreshold(QWidget* parent = nullptr);
    ~ColorMapTypeDistanceThreshold();
    void GetDistanceThrehold();
    void GetAlgorithmItems();
    ColorMapper* GetColorMapper();

private slots:
    void HandleMapTypeToggle(bool checked);

signals:
    void DistanceShowThrehold(std::tuple<double, double> threhold);
    void AlgorithmItemEnableSig(AlgorithmItemEnable items);
    void ColorMapType(MappingMethod map_type);
    void LinesShowMode(int mode);

private:
    Ui::ColorMapTypeDistanceThreshold* ui;
};

} // namespace ys_scene

#endif // COLORMAPTYPE_DISTANCETHRESHOLD_H
