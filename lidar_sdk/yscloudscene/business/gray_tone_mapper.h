#ifndef YS_SCENE_GRAY_TONE_MAPPER_H_
#define YS_SCENE_GRAY_TONE_MAPPER_H_

#include "ys_gray_frame.h"

#include <QImage>
#include <QPoint>
#include <QString>
#include <cstdint>
#include <vector>

namespace ys_scene {
enum class GrayEchoSelection {
    KMaximum,
    KFirst,
    KSecond,
};

enum class GrayMappingMode {
    KAutomatic,
    KFixed,
};

enum class GrayImageOrientation {
    KVertical,   // x=SLOT，y=Pixel；120 行 × 20 列
    KHorizontal, // x=Pixel，y=SLOT；20 行 × 120 列
};

struct GrayDisplayOptions
{
    GrayEchoSelection echo{GrayEchoSelection::KMaximum};
    GrayMappingMode mapping{GrayMappingMode::KAutomatic};
    GrayImageOrientation orientation{GrayImageOrientation::KVertical};
    uint32_t black_level{0};
    uint32_t white_level{you_see::GrayProtocol::kMaxIntensity};
};

// 统一的强度输入；旧直方图链路可在下一阶段迁移至此结构。
struct IntensityFrame
{
    int width{0};
    int height{0};
    uint32_t device_ip{0};
    uint64_t session_id{0};
    uint16_t frame_id{0};
    std::vector<uint32_t> values;
    std::vector<uint8_t> valid_pixels;
};

uint32_t SelectGrayEcho(uint32_t echo1, uint32_t echo2, GrayEchoSelection selection);
IntensityFrame ExtractGrayIntensity(const you_see::GrayEchoFrame &frame, GrayEchoSelection selection);
// 灰阶映射后转置显示；不改变原始 SLOT 数据、像素顺序或 EMA 范围。
QImage OrientGrayImage(const QImage &image, GrayImageOrientation orientation);
// 显示坐标 -> (SLOT, Pixel)，无效坐标返回 (-1, -1)。
QPoint GrayDisplayToSourcePixel(int x, int y, GrayImageOrientation orientation,
                                size_t slot_count = you_see::GrayProtocol::kSlotsPerFrame,
                                size_t pixels = you_see::GrayProtocol::kPixelsPerSlot);
bool SaveGrayFrameCsv(const you_see::GrayEchoFrame &frame, const QString &path, const GrayDisplayOptions &options,
                      QString *error = nullptr);

class GrayToneMapper
{
  public:
    void Reset();
    QImage Map(const IntensityFrame &frame, const GrayDisplayOptions &options);
  private:
    bool initialized_{false};
    uint32_t device_ip_{0};
    uint64_t session_id_{0};
    uint16_t frame_id_{0};
    GrayDisplayOptions options_;
    double ema_min_{0.0};
    double ema_max_{1.0};
};
} // namespace ys_scene

#endif // YS_SCENE_GRAY_TONE_MAPPER_H_
