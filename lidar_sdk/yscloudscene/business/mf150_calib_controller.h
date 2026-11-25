#ifndef YS_SCENE_MF150_CALIB_CONTROLLER_H_
#define YS_SCENE_MF150_CALIB_CONTROLLER_H_

#include <QObject>
#include <memory>
#include <string>
#include <vector>

#include "mf150_calibration_runner.h"
#include "ys_scene_def.h"

namespace ys_scene {

class OriginPointParser;
class Mf150PreviewConfig;

class Mf150CalibController : public QObject {
    Q_OBJECT

public:
    explicit Mf150CalibController(OriginPointParser* parser,
                                  Mf150PreviewConfig* dialog,
                                  QObject* parent = nullptr);

    void StartCalibration(float dist_real, const std::string& bin_path);
    bool CollectFrame(float dist_real);
    void RemoveCollectedFrame(int index);
    void ClearCollectedFrames();
    size_t GetCollectedFrameCount() const;
    void StartMultiCalibration(const std::string& bin_path);
    void SaveSingleRawData();
    void SaveMultiRawData();
    void SetRoiRect(int row, int col, int width, int height);
    void GetRoiRect(int& row, int& col, int& width, int& height) const;

signals:
    void CalibrationFinished(const std::vector<std::vector<float>>& calib_map);
    void CalibrationFailed(const std::string& error_msg);
    // 多点标定完成：新版每像素距离 LUT 表(shared_ptr 避免跨线程大表拷贝)
    void MultiCalibrationFinished(std::shared_ptr<SslDistCalibMulti> table);
    void MultiCalibrationFailed(const std::string& error_msg);

private:
    struct RawSample {
        float dist_real;
        std::string reflectivity;
        std::vector<std::vector<Mf150RawPixel>> raw_grid;
    };

    std::string MakeTimestamp() const;
    bool SaveRawSampleToFile(const RawSample& sample, const std::string& dir, int index) const;

    OriginPointParser* parser_;
    Mf150PreviewConfig* dialog_;
    std::vector<std::pair<float, Mf150CalibGrid>> collected_samples_;

    std::string multi_sync_save_dir_;
    int multi_sync_save_index_{0};

    RawSample last_single_raw_sample_;
    std::vector<RawSample> collected_raw_samples_;

    int roi_row_{0};
    int roi_col_{0};
    int roi_width_{360};
    int roi_height_{152};

    bool DoSyncSaveSingle(const std::vector<std::vector<Mf150RawPixel>>& grid,
                          size_t frame_number, uint64_t ms_timestamp, float dist_real);
    bool DoSyncSaveMulti(const std::vector<std::vector<Mf150RawPixel>>& grid,
                          size_t frame_number, uint64_t ms_timestamp, float dist_real);
};

} // namespace ys_scene

#endif // YS_SCENE_MF150_CALIB_CONTROLLER_H_
