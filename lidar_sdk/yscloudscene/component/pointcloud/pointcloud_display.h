#pragma once
#include "pointcloud_render_frame.h"
#include "pointcloud_render_mailbox.h"
#include "component/gui_parameters.h"
#include "rotation_indicator.h"
#include "ys_lidar_validate_type.h"
#include "component/pointcloud/picking/picked_point_info.h"
#include <QOpenGLShaderProgram>
#include <QWheelEvent>
#include <QMouseEvent>
#include <QKeyEvent>

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
#include <QtOpenGLWidgets/QOpenGLWidget>
#else
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#endif

#include <QOpenGLExtraFunctions>
#include <QOpenGLContext>
#include <QSurface>
#include <QRect>
#include <memory>
#include <QMatrix4x4>
#include <QVector2D>
#include <QQuaternion>
#include <QTimer>
#include <cmath>
#include <chrono>


namespace ys_scene {

class YsGui;
class PointcloudDisplay : public QOpenGLWidget, protected QOpenGLExtraFunctions
{
	Q_OBJECT

public:
	explicit PointcloudDisplay(QWidget *parent = nullptr);
	~PointcloudDisplay();
    void OnWheelEvent(float wheel_delta_deg);
    [[nodiscard]] std::shared_ptr<PointcloudRenderMailbox> GetRenderMailbox() const;

    struct RenderStatistics
    {
        std::size_t uploaded_frames{0};
        std::size_t created_buffer_pairs{0};
        std::size_t buffer_growths{0};
    };
    // GUI-thread diagnostic counters, also useful for checking upload/VBO reuse.
    [[nodiscard]] RenderStatistics GetRenderStatistics() const;


    //! 视图方向（参照 CloudCompare CC_VIEW_ORIENTATION）
    enum class ViewOrientation
    {
        Top,        //!< 俯视图（从 +Z 方向看）：X 轴朝前（屏幕上方）、Y 轴朝左、Z 轴朝上
        Bottom,     //!< 仰视图（从 -Z 方向看）
        Front,      //!< 前视图（从 -Y 方向看）
        Back,       //!< 后视图（从 +Y 方向看）
        Left,       //!< 左视图（从 -X 方向看）
        Right,      //!< 右视图（从 +X 方向看）
        IsoView1,   //!< 等轴测视图 1（前、右、上）
        IsoView2,   //!< 等轴测视图 2（后、左、上）
    };
    //! 一键切换视角（参照 CloudCompare setView：仅旋转视线方向，保持视野与旋转中心）
    void SetView(ViewOrientation orientation);

    void SetPickEngine(class SinglePickBase* engine);
    SinglePickBase* GetPickEngine() const;

public slots:
    void UpdateCloud(ys_scene::PointcloudRenderFramePtr frame);
    void SetPickData(std::shared_ptr<class PickDataProvider> provider);
    void UpdateRoiOverlay(const std::vector<float>& roi_vertices);
    void SetRoiVisible(bool visible);
	// 设置是否显示所有的转镜面的点云
	void SetShowAllMirrors(bool enable);

signals:
    void PickedPointInfoReady(const PickedPointInfo& info);
    void PickModeExited();
    void PickFreezeChanged(bool frozen);

protected:
	void initializeGL() override;
	void paintGL() override;
	void resizeGL(int w, int h) override;

protected:
	void mouseMoveEvent(QMouseEvent* event) override;
	void mousePressEvent(QMouseEvent* event) override;
	void mouseDoubleClickEvent(QMouseEvent* event) override;
	void mouseReleaseEvent(QMouseEvent* event) override;
	void wheelEvent(QWheelEvent* event) override;
	void paintEvent(QPaintEvent* event) override;
    void keyPressEvent(QKeyEvent* event) override;
	
private:
    void BuildCoorAxis(float length);
    void BuildPointdata();
    void PointPaintMethod2();
    const YsGui::ParamStruct& GetDisplayParameters() const;
    struct FrameData;
    void UploadPendingFrames();
    bool UploadFrame(FrameData& target, const PointcloudRenderFrame& frame);
    void DrawFrame(const FrameData& frame);
    void UpdateSceneBounds();
    // 视图操作（参照 CloudCompare ccGLWindow / ccViewportParameters 的实现）
    void SetPivotPoint(const QVector3D& point, bool autoUpdateCameraPos = false, bool verbose = false);
    void SetCameraCenter(const QVector3D& point);
    void MoveCamera(const QVector3D& v);
    void InvalidateViewport();
    void InvalidateVisualization();
    void EnsureValidMatrices();
    // 计算"短边视野约 ±25 米"所需的焦距（保证至少能看到 20 米距离）
    double ComputeInitialFocalDistance() const;
    // 配置初始视图：旋转中心在原点、雷达坐标惯例（X 前 Y 左 Z 上）、短边视野约 ±25 米（可看到 20m 距离）
    void ConfigureInitialView();
	void DrawAxis();
	double ComputeActualPixelSize() const;
    double ComputeDistanceToWidthRatio() const;
    QMatrix4x4 ComputeProjectionMatrix() const;
    QMatrix4x4 ComputeViewMatrix() const;
    QMatrix4x4 Ortho(double left, double right,
                    double bottom, double top,
                    double nearVal, double farVal) const;
    QMatrix4x4 FromToRotation(const QVector3D& from, const QVector3D& to);
    QVector3D ConvertMousePositionToOrientation(const QPointF& p);
    bool GetClick3DPos(int x, int y, QVector3D& P3D);
    void UpdateAutoPivotAtCenter();
    // 设备像素比辅助（矩阵/视口基于物理像素，交互输入为逻辑像素）
    inline double DevicePixelScale() const { return devicePixelRatioF(); }
    inline int GLViewportWidth() const { return qRound(width() * DevicePixelScale()); }
    inline int GLViewportHeight() const { return qRound(height() * DevicePixelScale()); }

    template <typename iType, typename oType>
    static bool Project(const QVector3D& input3D,
        const oType* modelview,
        const oType* projection,
        const int* viewport,
        QVector3D& output2D,
        bool* inFrustum = nullptr,
        const double* nearClippingDepth = nullptr,
        const double* farClippingDepth = nullptr)
    {
        //Modelview transform
        QVector4D Pm;
        {
            Pm.setX(static_cast<oType>(modelview[0] * input3D.x() + modelview[4] * input3D.y() + modelview[8] * input3D.z() + modelview[12]));
            Pm.setY(static_cast<oType>(modelview[1] * input3D.x() + modelview[5] * input3D.y() + modelview[9] * input3D.z() + modelview[13]));
            Pm.setZ(static_cast<oType>(modelview[2] * input3D.x() + modelview[6] * input3D.y() + modelview[10] * input3D.z() + modelview[14]));
            Pm.setW(static_cast<oType>(modelview[3] * input3D.x() + modelview[7] * input3D.y() + modelview[11] * input3D.z() + modelview[15]));
        };

        //Projection transform
        QVector4D Pp;
        {
            Pp.setX(static_cast<oType>(projection[0] * Pm.x() + projection[4] * Pm.y() + projection[8] * Pm.z() + projection[12] * Pm.w()));
            Pp.setY(static_cast<oType>(projection[1] * Pm.x() + projection[5] * Pm.y() + projection[9] * Pm.z() + projection[13] * Pm.w()));
            Pp.setZ(static_cast<oType>(projection[2] * Pm.x() + projection[6] * Pm.y() + projection[10] * Pm.z() + projection[14] * Pm.w()));
            Pp.setW(static_cast<oType>(projection[3] * Pm.x() + projection[7] * Pm.y() + projection[11] * Pm.z() + projection[15] * Pm.w()));
        };

        //The result normalizes between -1 and 1
        if (Pp.w() == 0.0)
        {
            return false;
        }

        //Perspective division
        Pp.setX(Pp.x() / Pp.w());
        Pp.setY(Pp.y() / Pp.w());
        Pp.setZ(Pp.z() / Pp.w());

        if (inFrustum)
        {
            //Check if the point is inside the frustum
            if (nearClippingDepth && -Pm.z() < *nearClippingDepth)
                *inFrustum = false;
            else if (farClippingDepth && -Pm.z() > *farClippingDepth)
                *inFrustum = false;
            else
                *inFrustum = (std::abs(Pp.x()) <= 1.0 && std::abs(Pp.y()) <= 1.0 && std::abs(Pp.z()) <= 1.0);
        }

        //Window coordinates
        //Map x, y to range 0-1
        output2D.setX((1.0 + Pp.x()) / 2 * viewport[2] + viewport[0]);
        output2D.setY((1.0 + Pp.y()) / 2 * viewport[3] + viewport[1]);
        //This is only correct when glDepthRange(0.0, 1.0)
        output2D.setZ((1.0 + Pp.z()) / 2);	//Between 0 and 1

        return true;
    }

	void updateProjectionMatrix();
	void updateViewMatrix();
	void updateModelMatrix();
    void createCoordinateCircle();
	void renderCoordinateCircle();
	void UpdateFps();
	void UpdateDataFps();
    void BuildPickSphere();
    void DrawRoiRect();
    std::vector<float> roi_vertices_;
	RotationIndicator rotation_indicator_;
	bool rotation_indicator_initialized_;

private:
    QOpenGLShaderProgram shaderProgram_mesh_;
    QOpenGLShaderProgram shaderProgram_axis_;
    QOpenGLShaderProgram shaderProgram_point_;
    QOpenGLShaderProgram shaderProgram_pick_sphere_;
    uint32_t VAO_meshline_{0};
    uint32_t VBO_meshline_{0};
    uint32_t VAO_axis_{0};
    uint32_t VBO_axis_{0};
    uint32_t VAO_point_{0};
    uint32_t VAO_pick_sphere_{ 0 };
    uint32_t VBO_pick_sphere_{ 0 };
    uint32_t EBO_pick_sphere_{ 0 };
    uint32_t pick_sphere_index_count_{ 0 };
    uint32_t VAO_roi_{ 0 };
    uint32_t VBO_roi_{ 0 };
    uint32_t roi_vertex_count_{ 0 };
    bool roi_visible_{ true };


    // display parameters
    std::shared_ptr<YsGui> gui_;

    // ============================================================
    // 视口参数（单一事实来源，参照 CloudCompare ccViewportParameters）
    // ============================================================
    //! 可视化矩阵（纯旋转，平移分量恒为 0）
    QMatrix4x4 view_rotation_;
    //! 相机中心（相机局部坐标系：x/y 为横向平移偏移，z 为沿视线方向的距离）
    QVector3D camera_center_;
    //! 旋转枢轴点（世界坐标系）
    QVector3D pivot_point_;
    //! 焦距（object-centered 模式下 = cameraCenter.z - pivotPoint.z）
    double focal_distance_;
    //! 视场角（水平方向，度）
    float fov_deg_;
    //! 相机纵横比
    double camera_aspect_ratio_;
    //! 透视视图开关
    bool perspective_view_;
    //! 是否以场景为中心（始终为 true）
    bool object_centered_view_;

    // ============================================================
    // 场景包围盒（由 UpdateCloud 更新，用于动态裁剪面与初始定位）
    // ============================================================
    QVector3D scene_bb_min_;
    QVector3D scene_bb_max_;
    double scene_bb_diag_;
    bool scene_bb_valid_;

    // ============================================================
    // 派生矩阵缓存（脏标记驱动，paintGL 时重建）
    // ============================================================
    QMatrix4x4 m_projectionMatrix;
	QMatrix4x4 m_viewMatrix;
	QMatrix4x4 m_modelMatrix;
    bool view_valid_;
    bool projection_valid_;

    // ============================================================
    // 交互状态机
    // ============================================================
    QPointF m_lastMousePos;
    bool m_mouseMoved;
    bool m_mouseButtonPressed;
    bool m_ignoreMouseReleaseEvent;
	bool m_leftButtonPressed;               // 左键是否按下
	bool m_showRotationCenter;              // 是否显示旋转中心
    //! 渲染结束后自动将屏幕中心点设为旋转枢轴（CloudCompare 行为，默认关闭）
    bool m_autoPickPivotAtCenter;
    //! 初始视图是否已配置（避免 resize 时反复重置用户的视角）
    bool initial_view_configured_;

    // ============================================================
    // 其他显示资源
    // ============================================================
    double m_circleRadiusMax;

	std::vector<float> m_circleVertices;
	std::vector<float> m_circleDistances;
	int m_circleTotalVertices;

	uint32_t fps_frame_count_{ 0 };
	float fps_value_{ 0.0f };
	std::chrono::steady_clock::time_point fps_last_time_;
	std::chrono::steady_clock::time_point fps_last_render_time_;
	uint32_t data_frame_count_{ 0 };
    RenderStatistics render_statistics_;
	float data_fps_value_{ 0.0f };
	std::chrono::steady_clock::time_point data_fps_last_time_;
	std::chrono::steady_clock::time_point data_last_frame_time_;

	QTimer* fps_decay_timer_{ nullptr };

private:
	// 存储多帧点云数据的结构
	struct FrameData {
		GLuint vbo_color = 0;
		GLuint vbo_cloud = 0;
		uint32_t point_count = 0;
        bool valid = false;
        std::size_t capacity = 0;
        QVector3D bounds_min;
        QVector3D bounds_max;
	};

	std::array<FrameData, static_cast<int>(you_see::PrismFaces::four_sided_mirror)> mirror_frames_;
	bool show_all_mirrors_{ false };
    FrameData single_frame_;
    bool cloud_view_fitted_{false};
    std::shared_ptr<PointcloudRenderMailbox> render_mailbox_{std::make_shared<PointcloudRenderMailbox>()};

    // 点选相关
    SinglePickBase* pick_engine_{nullptr};
    std::shared_ptr<PickDataProvider> pick_provider_;
    QPoint pick_start_pos_;
};

}; // namespace ys_scene 
