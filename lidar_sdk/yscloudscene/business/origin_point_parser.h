#ifndef YS_SCENE_ORIGIN_POINT_PARSER_H_
#define YS_SCENE_ORIGIN_POINT_PARSER_H_

#include "ys_lidar_validate_user.h"
#include "ys_scene_def.h"
#include "business/cs96_gamma_raw.h"
#include "business/cs96_beta_calib_params.h"
#include "calib_dist_types.h" // 多点标定表 SslDistCalibMulti(每像素 LUT 模型)
#include <qglobal.h>
#include <QObject>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <unordered_map>
#include <vector>
#include <queue>
#include <optional>
#include <QList>

namespace ys_scene {
using namespace you_see;
class OriginPointParseImp;
class PointcloudParser;

class OriginPointParser : public QObject {
    Q_OBJECT
public:
    OriginPointParser(QObject* parent = nullptr);
    ~OriginPointParser();
    void Start();
    void Stop();
    void OriginPointReceive(void* src, uint32_t length);
    void OriginPointReceive2(const uint32_t handle, void* src, uint32_t length, void* context);
    void OriginPointReceiveMf150(const uint32_t handle, void* src, uint32_t length, void* context);
    void OriginPointParse();
    void OriginPointParse2();
    void SetSaveOriginPoint(std::tuple<int, int, std::string> save_origin);
    void SetSourceNode(const LidarInfo& node);
    void ClearSourceNode();
    bool IsMf150Detected() const { return mf150_detected_; }
    OriginFramePtr GetLastMf150Frame() const { return last_mf150_frame_; }
    void QueryPsCalibrationParamsT(const uint32_t handle);
    std::vector<OriginPointParseImp* > GetProcessors();
	static void OriginPointReceiveTrampoline(const uint32_t handle, void* src, uint32_t length, void* context);
    void SetPointcloudParser(PointcloudParser* parser);
    void SetAlgorithmRepairGroundEnable(int state);
    void SetMf150PreviewConfig(bool enabled, int echo_index);
    void SetMf150CalibMap(const std::vector<std::vector<float>>& calib_map);
    void SetMf150CalibEnabled(bool enabled);
    // 多点标定(新版每像素 LUT 模型)，跨线程队列投递
    void SetMf150MultiCalibTable(std::shared_ptr<SslDistCalibMulti> table);
    void SetMf150MultiCalibEnabled(bool enabled);
    void SetCs96BetaCalibParams(std::shared_ptr<Cs96BetaCalibParams> params);

public slots:
    void ResetManagementSession();
    void SetPaused(bool paused);

private:
    void CheckSaveOrigin();
    void DealOriginFrame(OriginFramePtr origin_frame);
    void DealCs96GammaOriginFrame(OriginFramePtr origin_frame);
    void DealMf150OriginFrame(OriginFramePtr origin_frame);
    bool IsNextFrame(size_t frame_number);
	static void UpdatePsCalibrationParamsTrampoline(const uint32_t handle,
                                        const PsCalibrationParamsQuery* ps_calibration_params,
                                        void *context);
    void UpdatePsCalibrationParams(const uint32_t handle,
        const PsCalibrationParamsQuery* ps_calibration_params,
        void* context);
    void DistributeCalibrationParamsToProcessors();

private:
    std::thread parse_thread_;
    std::vector<uint8_t> data_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::vector<OriginPointParseImp*> processors_;
    std::optional<size_t> first_origin_frame_;
    std::optional<size_t> second_origin_frame_;
    std::priority_queue<OriginFramePtr, std::vector<OriginFramePtr>,
        CloudFrameComparator<uint8_t>> complete_origin_frames_;
    std::tuple<int, int, std::string, std::string> save_origin_frames_;
    std::unordered_map<size_t, OriginFramePtr> incomplete_origin_frames_;
    Cs96GammaRawAssembler cs96_gamma_assembler_;
    std::queue<OriginFramePtr> complete_gamma_frames_;
    std::atomic<bool> finished{ false };
    std::atomic<size_t> next_frame_to_process_{ 0 };
    static constexpr std::chrono::microseconds frame_timeout_{ 80 };
    static constexpr int min_available_data_size_{ 50000 };
    std::atomic<size_t> last_blocked_origin_qs_{ 0 };
    std::atomic<size_t> mf150_dispatched_count_{ 0 };
    bool data_ready_;
    bool stop_;
    bool save_origin_point_;
    int balance_count_;
    static constexpr int processor_size_{ 4 };
    static constexpr int cal_{ 4 };
    int need_save_count_{ 0 };
    int left_frame_saving_{ 0 };
    bool mf150_detected_{ false };
    bool cs96beta_detected_{ false };
    bool mf150_preview_enabled_{ true };
    OriginFramePtr last_mf150_frame_;
	PsCalibrationParamsHoldT ps_calibration_params_;
    std::atomic<bool> paused_{false};
};
}// namespace ys_scene

#endif // YS_SCENE_ORIGIN_POINT_PARSER_H_
