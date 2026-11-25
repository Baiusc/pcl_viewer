#ifndef YS_SCENE_POINTCLOUD_PARSER_IMP_H_
#define YS_SCENE_POINTCLOUD_PARSER_IMP_H_

#include "ys_lidar_type.h"
#include "component/pointcloud/pointcloud_render_frame.h"
#include "component/scalar_field.hpp"
#include "component/color_types.hpp"
#include "component/ys_scene_def.h"
#include "ys_lidar_validate_type.h"
#include <QObject>
#include <QRunnable>
#include <array>
#include <memory>
#include <mutex>
#include <span>
#include <QVector3D>

namespace you_see {};
namespace ys_scene {
using namespace you_see;
class PointcloudBuild {
public:
	explicit PointcloudBuild(std::span<const Rgb, max_steps_> colors);
	~PointcloudBuild() = default;

public:
	[[nodiscard]] PointcloudRenderFramePtr PointcloudGen(
		CloudPureFramePtr& points,
		std::shared_ptr<std::vector<int>> slot_counts,
		const int& frame_index,
        std::uint64_t sequence = NextPointcloudRenderSequence());

	[[nodiscard]] PointcloudRenderFramePtr PointcloudGenFlat(
		CloudPureFramePtr& points,
		const int& frame_index,
        std::uint64_t sequence = NextPointcloudRenderSequence());

    void SetColorScale(std::span<const Rgb, max_steps_> colors);
	void SetDistanceThrehold(std::tuple<double, double> threhold);
	void SetMappingMethod(MappingMethod &mm);
	void SetDistance(int &value, PointDistance &distance_type);
	void SetDisColor(int &value, DistanceColor &color_type);
	void SetROI(ROISelectType type, PointCloudROIA roi_select_data);
	void SetLinesShowMode(int mode);
	void SetAllSidesDisplay(bool enable);

private:
	float MapRange(float value, float fromLow, float fromHigh, float toLow, float toHigh);
	float MapDistance2Color(float &distance);
	void DrawPrismFacesSingleColor(float &color, const int &frame_index);
	void Draw4FacesSingleColor(float& color, const int& frame_index);
	ScalarType Normalize(ScalarType val) const;
	inline const Rgb GetColor(double relativepos, unsigned steps);

private:
	double min_show_;
	double max_show_;
	PointCloudROIA point_roi_;

	MappingMethod mapping_method_;
	PointDistancePosA distance_pos_;
	DistanceColorValA distance_color_;
	you_see::PointcloudRecordType LinesShowMode_;
	bool all_sides_display_;

private:
    std::array<Rgb, max_steps_> color_scale_element_{};
	PrismFaces prism_faces_;

};

class PointcloudParserImp : public QObject {
	Q_OBJECT
public:
	explicit PointcloudParserImp(
		std::span<const Rgb, max_steps_> colors,
		QObject *parent = nullptr);
	~PointcloudParserImp();
    void SetColorScale(std::span<const Rgb, max_steps_> colors);

signals:
    void RenderFrameReady(ys_scene::PointcloudRenderFramePtr frame);

public slots:
	void PointcloudProcessor(CloudPureFramePtr data,
		std::shared_ptr<std::vector<int>> slot_counts,
		std::tuple<int, std::string> save_point_frames,
		int frame_index,
        std::uint64_t sequence = NextPointcloudRenderSequence());
	void PointcloudProcessorFlat(CloudPureFramePtr data,
		std::tuple<int, std::string> save_point_frames,
		int frame_index,
        std::uint64_t sequence = NextPointcloudRenderSequence());
	void SetDistanThrehold(std::tuple<double, double> threhold);
	void SetAlgoItems(AlgorithmItemEnable items);
	void SetMappingMethod(MappingMethod mm);
	void SetDistance(int value, PointDistance distance_type);
	void SetDisColor(int value, DistanceColor color_type);
	void SetROI(ROISelectType type, PointCloudROIA roi_select_data);
	void SetLinesShowMode(int mode);
	void SetAllSidesDisplay(bool enable);

private: 
	void PointcloudSave(CloudPureFramePtr& points,
						std::string save_directory,
						int frame_id);

private:
    [[nodiscard]] PointcloudBuild CaptureBuild() const;

    mutable std::mutex settings_mutex_;
	std::unique_ptr<PointcloudBuild> pointbuild_;
	AlgorithmItemEnable algo_items_;
};

class PointcloudTask : public QRunnable {
public:
	PointcloudTask(CloudPureFramePtr data,
		std::shared_ptr<std::vector<int>> slot_counts,
		PointcloudParserImp* processor,
		std::tuple<int, std::string> save_point_frames,
		int frame_index);
	~PointcloudTask();
	void run() override;

private:
	CloudPureFramePtr data_;
	std::shared_ptr<std::vector<int>> slot_counts_;
	PointcloudParserImp* processor_;
	std::tuple<int, std::string> save_point_frames_;
	int frame_index_;
    std::uint64_t sequence_{NextPointcloudRenderSequence()};
};

class PointcloudTaskFlat : public QRunnable {
public:
	PointcloudTaskFlat(CloudPureFramePtr data,
		PointcloudParserImp* processor,
		std::tuple<int, std::string> save_point_frames,
		int frame_index);
	~PointcloudTaskFlat();
	void run() override;

private:
	CloudPureFramePtr data_;
	PointcloudParserImp* processor_;
	std::tuple<int, std::string> save_point_frames_;
	int frame_index_;
    std::uint64_t sequence_{NextPointcloudRenderSequence()};
};
}; // namespace ys_scene 
#endif // YS_SCENE_POINTCLOUD_PARSER_IMP_H_
