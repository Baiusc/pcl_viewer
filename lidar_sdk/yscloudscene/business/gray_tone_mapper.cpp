#include "gray_tone_mapper.h"

#include <QSaveFile>
#include <QTextStream>
#include <algorithm>
#include <cmath>
#include <limits>

namespace ys_scene {
uint32_t SelectGrayEcho(uint32_t echo1, uint32_t echo2, GrayEchoSelection selection)
{
    switch (selection) {
    case GrayEchoSelection::KFirst:
        return echo1;
    case GrayEchoSelection::KSecond:
        return echo2;
    default:
        return (std::max) (echo1, echo2);
    }
}

IntensityFrame ExtractGrayIntensity(const you_see::GrayEchoFrame &frame, GrayEchoSelection selection)
{
    const auto geometry = you_see::GetGrayGeometry(frame.format);
    IntensityFrame result;
    result.width = static_cast<int>(geometry.pixels);
    result.height = static_cast<int>(geometry.slot_count);
    result.device_ip = frame.device_ip;
    result.session_id = frame.session_id;
    result.frame_id = frame.frame_id;
    result.values.resize(geometry.slot_count * geometry.pixels);
    result.valid_pixels.resize(result.values.size());
    for (size_t i = 0; i < result.values.size(); ++i) {
        result.values[i] =
            geometry.echo_count == 1 ? frame.echo1[i] : SelectGrayEcho(frame.echo1[i], frame.echo2[i], selection);
        result.valid_pixels[i] = frame.received_slots.test(i / geometry.pixels) ? 1 : 0;
    }
    return result;
}

QImage OrientGrayImage(const QImage &image, GrayImageOrientation orientation)
{
    if (image.isNull() || orientation == GrayImageOrientation::KHorizontal) {
        return image;
    }
    if (image.format() != QImage::Format_Grayscale8) {
        return {};
    }
    QImage transposed(image.height(), image.width(), QImage::Format_Grayscale8);
    transposed.fill(0);
    for (int pixel = 0; pixel < image.width(); ++pixel) {
        auto *target_row = transposed.scanLine(pixel);
        for (int slot = 0; slot < image.height(); ++slot) {
            target_row[slot] = image.constScanLine(slot)[pixel];
        }
    }
    return transposed;
}

QPoint GrayDisplayToSourcePixel(int x, int y, GrayImageOrientation orientation, size_t slot_count, size_t pixels)
{
    const int slot = orientation == GrayImageOrientation::KVertical ? x : y;
    const int pixel = orientation == GrayImageOrientation::KVertical ? y : x;
    if (slot < 0 || pixel < 0 || slot >= static_cast<int>(slot_count) || pixel >= static_cast<int>(pixels)) {
        return QPoint(-1, -1);
    }
    return QPoint(slot, pixel);
}

bool SaveGrayFrameCsv(const you_see::GrayEchoFrame &frame, const QString &path, const GrayDisplayOptions &options,
                      QString *error)
{
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        if (error) {
            *error = file.errorString();
        }
        return false;
    }
    QTextStream stream(&file);
    const auto geometry = you_see::GetGrayGeometry(frame.format);
    stream << "# format=ys_gray_v1,width=" << geometry.pixels << ",height=" << geometry.slot_count
           << ",bits=" << (geometry.echo_count == 1 ? 22 : 20) << ",layout=slot_pixel\n";
    stream << "# device_ip=" << frame.device_ip << ",session_id=" << frame.session_id
           << ",echo_selection=" << static_cast<int>(options.echo) << ",mapping=" << static_cast<int>(options.mapping)
           << ",black_level=" << options.black_level << ",white_level=" << options.white_level
           << ",display_orientation="
           << (options.orientation == GrayImageOrientation::KVertical ? "vertical" : "horizontal") << '\n';
    stream << (geometry.echo_count == 1 ? "frame_id,slot,pixel,gray,valid\n"
                                        : "frame_id,slot,pixel,echo1,echo2,valid\n");
    for (size_t slot = 0; slot < geometry.slot_count; ++slot) {
        for (size_t pixel = 0; pixel < geometry.pixels; ++pixel) {
            const size_t index = slot * geometry.pixels + pixel;
            stream << frame.frame_id << ',' << slot << ',' << pixel << ',' << frame.echo1[index] << ','
                   << (geometry.echo_count == 2 ? QString::number(frame.echo2[index]) + ',' : QString())
                   << (frame.received_slots.test(slot) ? 1 : 0) << '\n';
        }
    }
    stream.flush();
    if (stream.status() != QTextStream::Ok || !file.commit()) {
        if (error) {
            *error = file.errorString();
        }
        return false;
    }
    return true;
}

void GrayToneMapper::Reset()
{
    initialized_ = false;
}

QImage GrayToneMapper::Map(const IntensityFrame &frame, const GrayDisplayOptions &options)
{
    if (frame.width <= 0 || frame.height <= 0 ||
        frame.values.size() != static_cast<size_t>(frame.width) * frame.height ||
        (!frame.valid_pixels.empty() && frame.valid_pixels.size() != frame.values.size()) ||
        (options.mapping == GrayMappingMode::KFixed && options.black_level >= options.white_level)) {
        return {};
    }
    const bool settings_changed = options.echo != options_.echo || options.mapping != options_.mapping ||
                                  options.black_level != options_.black_level ||
                                  options.white_level != options_.white_level;
    if (settings_changed || frame.device_ip != device_ip_ || frame.session_id != session_id_) {
        Reset();
    }
    uint32_t minimum = (std::numeric_limits<uint32_t>::max)();
    uint32_t maximum = 0;
    bool have_valid = false;
    for (size_t i = 0; i < frame.values.size(); ++i) {
        if (!frame.valid_pixels.empty() && !frame.valid_pixels[i]) {
            continue;
        }
        minimum = (std::min) (minimum, frame.values[i]);
        maximum = (std::max) (maximum, frame.values[i]);
        have_valid = true;
    }
    if (!have_valid) {
        minimum = 0;
    }
    if (options.mapping == GrayMappingMode::KAutomatic) {
        if (!initialized_) {
            ema_min_ = minimum;
            ema_max_ = maximum;
        }
        else if (frame.frame_id != frame_id_) {
            // 同一暂停帧重新显示时不推进 EMA，保持图像稳定。
            ema_min_ = 0.15 * minimum + 0.85 * ema_min_;
            ema_max_ = 0.15 * maximum + 0.85 * ema_max_;
        }
    }
    device_ip_ = frame.device_ip;
    session_id_ = frame.session_id;
    frame_id_ = frame.frame_id;
    options_ = options;
    initialized_ = true;
    const double black = options.mapping == GrayMappingMode::KFixed ? options.black_level : ema_min_;
    const double white = options.mapping == GrayMappingMode::KFixed ? options.white_level : ema_max_;
    const double range = white - black;
    QImage image(frame.width, frame.height, QImage::Format_Grayscale8);
    image.fill(0);
    for (int row = 0; row < frame.height; ++row) {
        auto *pixels = image.scanLine(row);
        for (int col = 0; col < frame.width; ++col) {
            const size_t index = static_cast<size_t>(row) * frame.width + col;
            if (!frame.valid_pixels.empty() && !frame.valid_pixels[index]) {
                continue;
            }
            const double value = range > 0.0 ? (static_cast<double>(frame.values[index]) - black) / range * 255.0
                                             : (frame.values[index] > 0 ? 255.0 : 0.0);
            pixels[col] = static_cast<uchar>(std::lround((std::clamp) (value, 0.0, 255.0)));
        }
    }
    return image;
}
} // namespace ys_scene
