#ifndef YS_SCENE_ORIGIN_POINT_PARSE_IMP_H_
#define YS_SCENE_ORIGIN_POINT_PARSE_IMP_H_

#include <QRunnable>
#include <QObject>
#include <memory>
#include "ys_scene_def.h"
#include "ys_lidar_validate_type.h"
#include "algorithm/repair_ground/include/algo_interface.h"
#include "algorithm/repair_ground/include/point_type.h"
#include "business/mf150_calibration_runner.h"
#include "business/cs96_beta_calib_params.h"

namespace you_see {};

namespace ys_scene {
class PointcloudParser;
class PickDataProvider;

class OriginPointParseImp : public QObject {
    Q_OBJECT
public:
    OriginPointParseImp(QObject* parent = nullptr);
    ~OriginPointParseImp();
    void SetPointcloudParser(PointcloudParser* parser);
	void SetAlgorithmRepairGroundEnable(int state);

public slots:
    void OriginPointProcessor(const OriginFramePtr& origin_frame,
        std::tuple<int, int, std::string, std::string> save_point_frames);
    void Mf150OriginPointProcessor(const OriginFramePtr& origin_frame,
        std::tuple<int, int, std::string, std::string> save_point_frames);
    void CS96OriginPointProcessor(const OriginFramePtr& origin_frame,
        std::tuple<int, int, std::string, std::string> save_point_frames);
	void OriginPoint2xyz(const OriginFramePtr& origin_frame);
    void Mf150OriginPoint2xyz(const OriginFramePtr& origin_frame);
    void Cs96BetaOriginPoint2xyz(const OriginFramePtr& origin_frame);
    void SetMf150PreviewConfig(bool enabled, int echo_index);
    void SetRoi(ROISelectType type, PointCloudROIA roi_select_data);
    void UpdateCalibrationParams(PsCalibrationParamsHoldT calibration_params);
    void UpdateCs96BetaCalibParams(std::shared_ptr<Cs96BetaCalibParams> params);
    void SetMf150CalibMap(const std::vector<std::vector<float>>& calib_map);
    void SetMf150CalibEnabled(bool enabled);
    // 多点标定模型(新版：每像素距离 LUT 查表)，跨线程经 shared_ptr 传递
    void SetMf150MultiCalibTable(std::shared_ptr<SslDistCalibMulti> table);
    void SetMf150MultiCalibEnabled(bool enabled);

signals:
    void PickDataReady(std::shared_ptr<PickDataProvider> provider);

private:
	bool LoadStandardCalibrationParams();
    SphericalFramePtr OriginToSpherical(const OriginFramePtr& origin_frame,
		const PointCloudROIA& roi);
    CloudFramePtr SphericalToCartesian(const SphericalFramePtr& spherical_frame);
    algo_lidar::PointMatrix OriginToPointMatrix(const OriginFramePtr& origin_frame, const PointCloudROIA& roi);
    CloudFramePtr PointMatrixToCartesian(const algo_lidar::PointMatrix& matrix, size_t frame_number);
    void SaveMf150Frame(const OriginFramePtr& origin_frame,
        int frame_index,
        const std::string& file_path,
        const std::string& prefix);
    void SaveCS96Slot800Frame(const OriginFramePtr& origin_frame,
        int frame_index,
        const std::string& file_path,
        const std::string& prefix);
    void SaveCS96Slot600Frame(const OriginFramePtr& origin_frame,
        const std::string& file_path);
    you_see::CartesianPoint Mf150PixelToXyz(uint32_t tof,
        uint16_t row, uint16_t col) const;
    std::vector<std::vector<Mf150RawPixel>> BuildMf150PixelGrid(
        const OriginFramePtr& origin_frame) const;
    algo_lidar::PointMatrix Mf150PixelGridToPointMatrix(
        const std::vector<std::vector<Mf150RawPixel>>& grid,
        int echo_index) const;

private:
    PointCloudROIA origin_point_roi_;
	PsCalibrationParamsHoldT calibration_params_;
    PsCalibrationParamsHoldT standard_calibration_params_;
    std::vector<std::vector<float>> mf150_calib_map_;
    bool mf150_calib_enabled_{ false };
    // 多点标定(新版 LUT 模型)：每像素距离查表插值；nullptr 表示未加载
    std::shared_ptr<const SslDistCalibMulti> mf150_multi_table_{ nullptr };
    bool mf150_multi_calib_enabled_{ false };
    bool mf150_preview_enabled_{ true };
    int mf150_preview_echo_index_{ 0 };
    PointcloudParser* pointcloud_parser_{ nullptr };
    bool algorithm_repair_ground_{ false };
    std::shared_ptr<std::vector<std::vector<Mf150RawPixel>>> latest_mf150_grid_;
    std::shared_ptr<Cs96BetaCalibParams> cs96_beta_calib_params_;
    bool cs96_beta_preview_enabled_{ false };
};

class OriginPointTask : public QRunnable {
public:
    OriginPointTask(const OriginFramePtr& frame,
        OriginPointParseImp* processor,
        std::tuple<int, int, std::string, std::string> save_point_frames,
        OriginPointOperation operation_type = OriginPointOperation::KOriginPointBothSaveTrans,
        bool is_mf150 = false,
        bool is_cs96_beta = false);
    void run() override;

private:
    OriginFramePtr frame_;
    OriginPointParseImp* processor_;
    std::tuple<int, int, std::string, std::string> save_origin_point_frames_;
    OriginPointOperation operation_type_;
    bool is_mf150_;
    bool is_cs96_beta_;
};

}// namespace ys_scene

#endif //YS_SCENE_ORIGIN_POINT_PARSE_IMP_H_
