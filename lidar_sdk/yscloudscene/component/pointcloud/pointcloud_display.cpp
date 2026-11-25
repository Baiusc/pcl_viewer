#include "pointcloud_display.h"
#include "you_see_sdk/include/ys_lidar_type.h"
#include "log/logging.h"
#include "component/gl_trace.h"
#include "component/color_types.hpp"
#include "component/gui_parameters.h"
#include "component/ys_scene_const.h"
#include "component/pointcloud/picking/single_pick_base.h"

#include <QPainter>
#include <QCursor>
#include <QtMath>
#include <QKeyEvent>
#include <QThread>
#include <vector>
#include <array>
#include <limits>

namespace ys_scene {

// 透视模式下 zNear 相对 zFar 的最小比例（参照 CloudCompare zNearCoef）
static const double kZNearCoef = 0.005;

PointcloudDisplay::PointcloudDisplay(QWidget* parent)
    : QOpenGLWidget(parent),
    rotation_indicator_initialized_{ false },
    gui_{ std::make_shared<YsGui>() },
    view_rotation_(),
    camera_center_{ 0.0f, 0.0f, 1.0f },
    pivot_point_{ 0.0f, 0.0f, 0.0f },
    focal_distance_{ 1.0 },
    fov_deg_{ 50.0f },
    camera_aspect_ratio_{ 1.0 },
    perspective_view_{ true },
    object_centered_view_{ true },
    scene_bb_min_{ 0.0f, 0.0f, 0.0f },
    scene_bb_max_{ 0.0f, 0.0f, 0.0f },
    scene_bb_diag_{ 0.0 },
    scene_bb_valid_{ false },
    m_projectionMatrix(),
    m_viewMatrix(),
    m_modelMatrix(),
    view_valid_{ false },
    projection_valid_{ false },
    m_lastMousePos(),
    m_mouseMoved{ false },
    m_mouseButtonPressed{ false },
    m_ignoreMouseReleaseEvent{ false },
    m_leftButtonPressed{ false },
    m_showRotationCenter{ false },
    m_autoPickPivotAtCenter{ false },
    initial_view_configured_{ false },
    m_circleRadiusMax{ 120.0 }
{
    connect(render_mailbox_.get(), &PointcloudRenderMailbox::FramesAvailable,
        this, [this] { update(); }, Qt::QueuedConnection);
    setFocusPolicy(Qt::StrongFocus);
    view_rotation_.setToIdentity();
    m_modelMatrix.setToIdentity();
    // 初始化mirror_frames_数组
    for (auto& frame : mirror_frames_) {
        frame.vbo_cloud = 0;
        frame.vbo_color = 0;
        frame.point_count = 0;
        frame.valid = false;
    }

    fps_last_time_ = std::chrono::steady_clock::now();
    fps_last_render_time_ = fps_last_time_;
    data_fps_last_time_ = fps_last_time_;
    data_last_frame_time_ = fps_last_time_;

    fps_decay_timer_ = new QTimer(this);
    connect(fps_decay_timer_, &QTimer::timeout, this, [this]() {
        auto now = std::chrono::steady_clock::now();
        
        // Check Data FPS decay
        auto data_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - data_last_frame_time_);
        if (data_elapsed.count() > 1000) {
             data_fps_value_ = 0.0f;
             // Only update if we need to clear a non-zero value
             update();
        }

        // Check Render FPS decay
        auto render_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - fps_last_render_time_);
        if (render_elapsed.count() > 1000) {
            fps_value_ = 0.0f;
            update();
        }
    });
    fps_decay_timer_->start(500);
}

PointcloudDisplay::~PointcloudDisplay()

{
    render_mailbox_->Close();
    if (!context() || !isValid()) {
        return;
    }
    // 上下文绑定
    makeCurrent();
    // 释放从GPU内存中分配的缓冲对象，避免内存泄露
    glDeleteBuffers(1, &VBO_meshline_);
    glDeleteVertexArrays(1, &VAO_meshline_);
    glDeleteBuffers(1, &VBO_axis_);
    glDeleteVertexArrays(1, &VAO_axis_);
    glDeleteBuffers(1, &single_frame_.vbo_cloud);
    glDeleteBuffers(1, &single_frame_.vbo_color);
    glDeleteVertexArrays(1, &VAO_point_);

    // 释放mirror_frames_中的VBO
    for (const auto& frame : mirror_frames_) {
        if (frame.vbo_cloud != 0) {
            glDeleteBuffers(1, &frame.vbo_cloud);
            glDeleteBuffers(1, &frame.vbo_color);
        }
    }

    if (VAO_pick_sphere_ != 0) {
        glDeleteVertexArrays(1, &VAO_pick_sphere_);
        glDeleteBuffers(1, &VBO_pick_sphere_);
        glDeleteBuffers(1, &EBO_pick_sphere_);
    }

    if (VAO_roi_ != 0) {
        glDeleteVertexArrays(1, &VAO_roi_);
        glDeleteBuffers(1, &VBO_roi_);
    }

    // 释放shader
    shaderProgram_mesh_.release();
    shaderProgram_axis_.release();
    shaderProgram_point_.release();
    shaderProgram_pick_sphere_.release();
    // 上下文解绑
    doneCurrent();
}

void PointcloudDisplay::OnWheelEvent(float wheel_delta_deg)
{
    // 参照 CloudCompare onWheelEvent：缩放 == 相机沿视线方向前后移动
    double diag = (scene_bb_valid_ ? scene_bb_diag_ : m_circleRadiusMax * 2.0);
    if (diag < 1.0e-6) {
        diag = 1.0;
    }

    double delta = 0.0;
    if (perspective_view_) {
        // 透视模式：位移与"相机到旋转中心的距离"挂钩，并带速度自适应系数
        // （离场景越近速度越慢，便于精细操作；越远越快，便于快速拉远）
        // 注意：速度系数必须基于"距离"而不是动态 zNear——
        // zNear 随视角倾斜会因常驻距离圆环的深度跨度而骤变，
        // 导致同一缩放级别下不同视角的缩放速度不一致
        double distToPivot = (camera_center_ - pivot_point_).length();
        double speedRatio = 10.0 * std::max(distToPivot, 1.0e-9) / diag;
        double speedCoef = std::min(16.0, std::exp(speedRatio));
        delta = static_cast<double>(wheel_delta_deg) * diag / 250.0 / 8.0 * GetDisplayParameters().zoomSpeed;
        delta *= speedCoef;
    } else {
        // 正交模式：按焦距百分比等比缩放（永不越界、永不失效）
        delta = (std::abs(focal_distance_) / (wheel_delta_deg < 0.0 ? -20.0 : 20.0)) * GetDisplayParameters().zoomSpeed;
    }

    MoveCamera(QVector3D(0.0f, 0.0f, -static_cast<float>(delta)));
    update();
}

void PointcloudDisplay::SetView(ViewOrientation orientation)
{
    // 参照 CloudCompare ccGLUtils::GenerateViewMat：
    // 视角 = 绕竖直轴（Z）的方位角 + 绕水平轴（X）的俯仰角
    double vertAngle_deg = 0.0;
    double orthoAngle_deg = 0.0;
    switch (orientation)
    {
    case ViewOrientation::Top:     vertAngle_deg = 0.0;    orthoAngle_deg = 90.0;  break;
    case ViewOrientation::Bottom:  vertAngle_deg = -180.0; orthoAngle_deg = -90.0; break;
    case ViewOrientation::Front:   vertAngle_deg = 0.0;    orthoAngle_deg = 0.0;   break;
    case ViewOrientation::Back:    vertAngle_deg = -180.0; orthoAngle_deg = 0.0;   break;
    case ViewOrientation::Left:    vertAngle_deg = 90.0;   orthoAngle_deg = 0.0;   break;
    case ViewOrientation::Right:   vertAngle_deg = -90.0;  orthoAngle_deg = 0.0;   break;
    case ViewOrientation::IsoView1: vertAngle_deg = 45.0;  orthoAngle_deg = 45.0;  break;
    case ViewOrientation::IsoView2: vertAngle_deg = -45.0; orthoAngle_deg = -45.0; break;
    }

    // 基准视图矩阵 M_front：视线 +Y、屏幕右 +X、屏幕上 +Z（纯旋转，det=+1，右手系）
    // 注意：必须保证行列式为 +1（旋转），否则整个视图会变成镜像（左手系观感）
    QMatrix4x4 mFront;
    mFront(0, 0) = 1.0f;  mFront(0, 1) = 0.0f;  mFront(0, 2) = 0.0f;   // 行0 = ( 1,  0,  0) 屏幕右
    mFront(1, 0) = 0.0f;  mFront(1, 1) = 0.0f;  mFront(1, 2) = 1.0f;   // 行1 = ( 0,  0,  1) 屏幕上
    mFront(2, 0) = 0.0f;  mFront(2, 1) = -1.0f; mFront(2, 2) = 0.0f;   // 行2 = ( 0, -1,  0) 视线反方向

    QMatrix4x4 vertRot;
    vertRot.rotate(static_cast<float>(vertAngle_deg), QVector3D(0.0f, 0.0f, 1.0f));
    QMatrix4x4 horizRot;
    horizRot.rotate(static_cast<float>(orthoAngle_deg), QVector3D(1.0f, 0.0f, 0.0f));

    // viewMat = Rz(+90°) · Rx(ortho) · M_front · Rz(vert)
    // 前置 Rz(+90°)：雷达坐标系惯例——俯视图（Top）时
    // X 轴朝前（屏幕上方）、Y 轴朝左（屏幕左方）、Z 轴朝上（屏幕外）
    QMatrix4x4 viewMat = mFront * vertRot;
    viewMat = horizRot * viewMat;
    QMatrix4x4 userRot;
    userRot.rotate(90.0f, QVector3D(0.0f, 0.0f, 1.0f));
    viewMat = userRot * viewMat;

    view_rotation_ = viewMat;
    InvalidateVisualization();
    update();
}

std::shared_ptr<PointcloudRenderMailbox> PointcloudDisplay::GetRenderMailbox() const
{
    return render_mailbox_;
}

PointcloudDisplay::RenderStatistics PointcloudDisplay::GetRenderStatistics() const
{
    Q_ASSERT(QThread::currentThread() == thread());
    return render_statistics_;
}

void PointcloudDisplay::SetShowAllMirrors(bool enable)
{
    Q_ASSERT(QThread::currentThread() == thread());
    if (show_all_mirrors_ == enable) {
        return;
    }
    render_mailbox_->SetAllMirrors(enable);
    show_all_mirrors_ = enable;
    // Keep allocated GPU storage, but never revive frames from the previous mode.
    single_frame_.valid = false;
    single_frame_.point_count = 0;
    for (auto& frame : mirror_frames_) {
        frame.valid = false;
        frame.point_count = 0;
    }
    UpdateSceneBounds();
    update();
}

bool PointcloudDisplay::UploadFrame(FrameData& target, const PointcloudRenderFrame& frame)
{
    const auto count = frame.points->size();
    const auto max_draw_count = static_cast<std::size_t>((std::numeric_limits<GLsizei>::max)());
    const auto max_buffer_bytes = static_cast<std::size_t>((std::numeric_limits<GLsizeiptr>::max)());
    if (count > max_draw_count || count > max_buffer_bytes / sizeof(you_see::CartesianPoint) ||
        count > max_buffer_bytes / sizeof(Rgb)) {
        return false;
    }

    if (count != 0) {
        if (target.vbo_cloud == 0) {
            GLuint buffers[2]{};
            glGenBuffers(2, buffers);
            if (buffers[0] == 0 || buffers[1] == 0) {
                glDeleteBuffers(2, buffers);
                return false;
            }
            target.vbo_cloud = buffers[0];
            target.vbo_color = buffers[1];
            ++render_statistics_.created_buffer_pairs;
        }
        // Capacity only grows; smaller/empty frames keep storage for the next upload.
        if (count > target.capacity) {
            glBindBuffer(GL_ARRAY_BUFFER, target.vbo_cloud);
            glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(count * sizeof(you_see::CartesianPoint)),
                nullptr, GL_DYNAMIC_DRAW);
            glBindBuffer(GL_ARRAY_BUFFER, target.vbo_color);
            glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(count * sizeof(Rgb)), nullptr, GL_DYNAMIC_DRAW);
            target.capacity = count;
            ++render_statistics_.buffer_growths;
        }
        glBindBuffer(GL_ARRAY_BUFFER, target.vbo_cloud);
        glBufferSubData(GL_ARRAY_BUFFER, 0, static_cast<GLsizeiptr>(count * sizeof(you_see::CartesianPoint)),
            frame.points->data());
        glBindBuffer(GL_ARRAY_BUFFER, target.vbo_color);
        glBufferSubData(GL_ARRAY_BUFFER, 0, static_cast<GLsizeiptr>(count * sizeof(Rgb)), frame.colors.data());
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }
    target.point_count = static_cast<uint32_t>(count);
    target.bounds_min = frame.bounds_min;
    target.bounds_max = frame.bounds_max;
    target.valid = true;
    ++render_statistics_.uploaded_frames;
    data_last_frame_time_ = std::chrono::steady_clock::now();
    UpdateDataFps();
    return true;
}

void PointcloudDisplay::UploadPendingFrames()
{
    Q_ASSERT(QOpenGLContext::currentContext() == context());
    const auto pending = render_mailbox_->Take();
    bool changed = false;
    for (std::size_t i = 0; i < pending.size(); ++i) {
        if (pending[i]) {
            auto& target = show_all_mirrors_ ? mirror_frames_[i] : single_frame_;
            changed = UploadFrame(target, *pending[i]) || changed;
        }
    }
    if (changed) {
        UpdateSceneBounds();
    }
}

void PointcloudDisplay::UpdateSceneBounds()
{
    scene_bb_valid_ = false;
    scene_bb_min_ = {};
    scene_bb_max_ = {};
    const auto include = [this](const FrameData& frame) {
        if (!frame.valid || frame.point_count == 0) {
            return;
        }
        if (!scene_bb_valid_) {
            scene_bb_min_ = frame.bounds_min;
            scene_bb_max_ = frame.bounds_max;
            scene_bb_valid_ = true;
            return;
        }
        for (int axis = 0; axis < 3; ++axis) {
            scene_bb_min_[axis] = std::min(scene_bb_min_[axis], frame.bounds_min[axis]);
            scene_bb_max_[axis] = std::max(scene_bb_max_[axis], frame.bounds_max[axis]);
        }
    };
    if (show_all_mirrors_) {
        for (const auto& frame : mirror_frames_) {
            include(frame);
        }
    }
    else {
        include(single_frame_);
    }
    scene_bb_diag_ = (scene_bb_max_ - scene_bb_min_).length();
    projection_valid_ = false;
    if (scene_bb_valid_ && !cloud_view_fitted_) {
        double ratio = ComputeDistanceToWidthRatio();
        if (ratio <= 0.0 || !std::isfinite(ratio)) {
            ratio = 1.0;
        }
        double focal = std::max(scene_bb_diag_ * 1.2 / ratio, ComputeInitialFocalDistance());
        if (focal <= 0.0 || !std::isfinite(focal)) {
            focal = 50.0;
        }
        camera_center_.setZ(static_cast<float>(focal));
        focal_distance_ = focal;
        view_valid_ = false;
        initial_view_configured_ = true;
        cloud_view_fitted_ = true;
    }
}

void PointcloudDisplay::DrawFrame(const FrameData& frame)
{
    if (!frame.valid || frame.point_count == 0) {
        return;
    }
    shaderProgram_point_.bind();
    glBindVertexArray(VAO_point_);
    glBindBuffer(GL_ARRAY_BUFFER, frame.vbo_cloud);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(you_see::CartesianPoint), nullptr);
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, frame.vbo_color);
    glVertexAttribPointer(1, 3, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(Rgb), nullptr);
    glEnableVertexAttribArray(1);
    glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(frame.point_count));
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void PointcloudDisplay::initializeGL()
{
    initializeOpenGLFunctions();
    ys_scene::GLTrace::init(this);
    glEnable(GL_DEBUG_OUTPUT);
    glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
    glDebugMessageCallback(ys_scene::GLTrace::glDebugOutput, nullptr);
    glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, nullptr, GL_TRUE);

    // build point
    BuildPointdata();
    // set the clear screen color
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    // 检查深度缓冲区是否被正确配置
    int depthBits;
    glGetIntegerv(GL_DEPTH_BITS, &depthBits);
    LOG_INFO("Depth buffer bits: {}", depthBits);
    // link meshline shaders
    shaderProgram_mesh_.addShaderFromSourceFile(
        QOpenGLShader::Vertex, ":/opengl/shader/shader_mesh.vs");
    shaderProgram_mesh_.addShaderFromSourceFile(
        QOpenGLShader::Fragment, ":/opengl/shader/shader_mesh.fs");
    shaderProgram_mesh_.link();
    // link coordinate axis shaders
    shaderProgram_axis_.addShaderFromSourceFile(
        QOpenGLShader::Vertex, ":/opengl/shader/shader_axis.vs");
    shaderProgram_axis_.addShaderFromSourceFile(
        QOpenGLShader::Fragment, ":/opengl/shader/shader_axis.fs");
    shaderProgram_axis_.link();
    // link pointcloud shaders
    shaderProgram_point_.addShaderFromSourceFile(
        QOpenGLShader::Vertex, ":/opengl/shader/shader_point.vs");
    shaderProgram_point_.addShaderFromSourceFile(
        QOpenGLShader::Fragment, ":/opengl/shader/shader_point.fs");
    shaderProgram_point_.link();

    // 初始化旋转指示器
    rotation_indicator_.Initialize();
    rotation_indicator_initialized_ = true;

    createCoordinateCircle();

    // build axis
    BuildCoorAxis(1.0);

    // init pick sphere shader (Phong lighting)
    {
        shaderProgram_pick_sphere_.addShaderFromSourceCode(QOpenGLShader::Vertex, R"(
            #version 330 core
            layout(location = 0) in vec3 aPos;
            layout(location = 1) in vec3 aNormal;
            uniform mat4 projection;
            uniform mat4 view;
            uniform mat4 model;
            out vec3 Normal;
            out vec3 FragPos;
            void main() {
                FragPos = vec3(model * vec4(aPos, 1.0));
                Normal = mat3(transpose(inverse(model))) * aNormal;
                gl_Position = projection * view * model * vec4(aPos, 1.0);
            }
        )");
        shaderProgram_pick_sphere_.addShaderFromSourceCode(QOpenGLShader::Fragment, R"(
            #version 330 core
            out vec4 FragColor;
            in vec3 Normal;
            in vec3 FragPos;
            uniform vec3 viewPos;
            uniform vec3 lightDir;
            void main() {
                vec3 baseColor = vec3(1.0, 0.85, 0.0);
                vec3 norm = normalize(Normal);
                vec3 lightDirN = normalize(lightDir);
                float diff = max(dot(norm, lightDirN), 0.0);
                vec3 ambient = 0.35 * baseColor;
                vec3 diffuse = diff * baseColor;
                vec3 viewDir = normalize(viewPos - FragPos);
                vec3 reflectDir = reflect(-lightDirN, norm);
                float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);
                vec3 specular = 0.4 * spec * vec3(1.0);
                FragColor = vec4(ambient + diffuse + specular, 1.0);
            }
        )");
        shaderProgram_pick_sphere_.link();
    }

    // build pick sphere
    BuildPickSphere();

    QString versionString = QString::fromUtf8(reinterpret_cast<const char*>(glGetString(GL_VERSION)));
    QStringList parts = versionString.split(".");
    if (parts.size() >= 2) {
        int major = parts[0].toInt();
        int minor = parts[1].toInt();
        qDebug() << QString("OpenGL %1.%2").arg(major).arg(minor);
    }

    // 配置初始视图（旋转中心在原点、雷达坐标惯例 X 前 Y 左 Z 上、短边视野约 ±25 米）
    ConfigureInitialView();
}
void PointcloudDisplay::paintGL()
{
    UploadPendingFrames();
    UpdateFps();
    // 脏标记驱动：只在状态变化时重建矩阵（避免渲染矩阵累积浮点误差）
    EnsureValidMatrices();

    // 确保启用深度测试
    glEnable(GL_DEPTH_TEST);
    // 设置深度测试函数
    glDepthFunc(GL_LESS);
    // 设置清除颜色缓冲区时所使用的颜色
    glClearColor(0.015f, 0.015f, 0.015f, 1.0f);
    // 清除颜色缓冲区和深度缓冲区的内容
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    const QMatrix4x4& projection = m_projectionMatrix;
    const QMatrix4x4& view = m_viewMatrix;
    const QMatrix4x4& model = m_modelMatrix;

    // 绑定变换矩阵
    shaderProgram_mesh_.bind();
    shaderProgram_mesh_.setUniformValue("projection", projection);
    shaderProgram_mesh_.setUniformValue("view", view);
    shaderProgram_mesh_.setUniformValue("model", model);
    shaderProgram_axis_.bind();
    shaderProgram_axis_.setUniformValue("projection", projection);
    shaderProgram_axis_.setUniformValue("view", view);
    shaderProgram_axis_.setUniformValue("model", model);
    shaderProgram_point_.bind();
    shaderProgram_point_.setUniformValue("projection", projection);
    shaderProgram_point_.setUniformValue("view", view);
    shaderProgram_point_.setUniformValue("model", model);

    // draw meshline
    renderCoordinateCircle();

    // draw axis
    DrawAxis();

    // draw point
    if (show_all_mirrors_) {
        for (const auto& frame : mirror_frames_) {
            DrawFrame(frame);
        }
    }
    else {
        PointPaintMethod2();
    }

    // 绘制旋转中心标记（如果需要）
    QMatrix4x4 viewInv = m_viewMatrix.inverted();
    QVector3D cameraPos(viewInv(0, 3), viewInv(1, 3), viewInv(2, 3));

    if (m_showRotationCenter && rotation_indicator_initialized_) {
        float distanceToCenter = (pivot_point_ - cameraPos).length();

        rotation_indicator_.DrawRotationCenter(
            pivot_point_,
            m_projectionMatrix,
            m_viewMatrix,
            m_modelMatrix,
            cameraPos,
            distanceToCenter
        );
    }

    if (pick_engine_) {
        const auto& annotations = pick_engine_->GetAnnotations();
        double pixel_size_at_focal = ComputeActualPixelSize();
        for (const auto& ann : annotations) {
            float dist = (cameraPos - ann.pos).length();
            // 标注球保持约 20 像素的屏幕尺寸（随距离缩放）
            double pixel_size = pixel_size_at_focal;
            if (focal_distance_ > 1.0e-9) {
                pixel_size *= (dist / focal_distance_);
            }
            float sphere_radius = (20.0f * static_cast<float>(pixel_size)) / 2.0f;

            shaderProgram_pick_sphere_.bind();
            shaderProgram_pick_sphere_.setUniformValue("projection", m_projectionMatrix);
            shaderProgram_pick_sphere_.setUniformValue("view", m_viewMatrix);

            QMatrix4x4 model;
            model.translate(ann.pos);
            model.scale(sphere_radius);
            shaderProgram_pick_sphere_.setUniformValue("model", model);
            shaderProgram_pick_sphere_.setUniformValue("viewPos", cameraPos);
            shaderProgram_pick_sphere_.setUniformValue("lightDir", QVector3D(1, 1, 1).normalized());

            glBindVertexArray(VAO_pick_sphere_);
            glDrawElements(GL_TRIANGLES, pick_sphere_index_count_, GL_UNSIGNED_INT, 0);
            glBindVertexArray(0);
        }
    }

    DrawRoiRect();

    // 渲染结束后将屏幕中心点自动设为旋转枢轴（CloudCompare 行为，
    // 保证旋转始终围绕"当前正在看的东西"）
    UpdateAutoPivotAtCenter();
}

void PointcloudDisplay::UpdateCloud(ys_scene::PointcloudRenderFramePtr frame)
{
    Q_ASSERT(QThread::currentThread() == thread());
    render_mailbox_->Publish(std::move(frame));
}

void PointcloudDisplay::paintEvent(QPaintEvent* event)
{
    QOpenGLWidget::paintEvent(event);

    QPainter painter(this);
    painter.setRenderHint(QPainter::TextAntialiasing);
    painter.setPen(QPen(QColor(220, 220, 220)));
    painter.setFont(QFont("Microsoft YaHei", 12, QFont::Normal));

    // FPS
    QString fps_text = QString("R FPS %1 | D FPS %2")
        .arg(fps_value_, 0, 'f', 1)
        .arg(data_fps_value_, 0, 'f', 1);
    QFontMetrics fm(painter.font());
    int text_width = fm.horizontalAdvance(fps_text);
    int text_height = fm.height();
    int fps_x = (width() - text_width) / 2;
    int fps_y = height() - 10; // baseline
    painter.drawText(fps_x, fps_y, fps_text);

    // 矩阵基于物理像素，绘制基于逻辑像素，需要换算
    const double dpr = DevicePixelScale();
    QRect vp(0, 0, GLViewportWidth(), GLViewportHeight());

    // Distance labels
    painter.setPen(QPen(QColor(180, 180, 180)));
    painter.setFont(QFont("Microsoft YaHei", 11, QFont::Normal));
    QFontMetrics dist_fm(painter.font());
    for (int i = 0; i < m_circleDistances.size(); ++i) {
        int startIndex = i * 361;
        if (startIndex * 3 + 1 >= m_circleVertices.size()) break;
        QVector3D labelPos(m_circleVertices[startIndex * 3], m_circleVertices[startIndex * 3 + 1], 0.0f);
        QVector3D screenPos = labelPos.project(m_viewMatrix, m_projectionMatrix, vp);
        if (screenPos.z() > 1.0f) continue;
        float sx = screenPos.x() / dpr;
        float sy = (GLViewportHeight() - screenPos.y()) / dpr;
        if (sx < 0 || sx >= width() || sy < 0 || sy >= height()) {
            continue;
        }
        QString text = QString::number(m_circleDistances[i], 'f', 1) + "m";
        painter.drawText(qRound(sx), qRound(sy), text);
    }

    if (pick_engine_) {
        const auto& annotations = pick_engine_->GetAnnotations();
        for (const auto& ann : annotations) {
            QVector3D sp = ann.pos.project(m_viewMatrix, m_projectionMatrix, vp);
            if (sp.z() >= 1.0f) continue;

            int sx = qRound(sp.x() / dpr);
            int sy = qRound((GLViewportHeight() - sp.y()) / dpr);

            painter.save();
            painter.setPen(Qt::yellow);
            painter.setFont(QFont("Consolas", 10, QFont::Bold));
            painter.drawText(sx + 14, sy + 4, ann.label);
            painter.restore();
        }
    }
}
void PointcloudDisplay::resizeGL(int w, int h)
{
    glViewport(0, 0, GLViewportWidth(), GLViewportHeight());
    // 视口尺寸确定后，按真实尺寸配置初始视野（仅在无场景数据且未配置过时执行）
    if (!scene_bb_valid_ && !initial_view_configured_) {
        ConfigureInitialView();
    }
    // 纵横比变化需要重建投影矩阵
    InvalidateViewport();
    update();
}
void PointcloudDisplay::mouseMoveEvent(QMouseEvent* event)
{
    if (!(event->buttons() & (Qt::LeftButton | Qt::RightButton))) {
        m_lastMousePos = event->pos();
        return;
    }

    EnsureValidMatrices();
    const QPointF currentPos = QPointF(event->pos());

    if (event->buttons() & Qt::RightButton) {
        // 右键 = 平移（2D 平移）
        // 位移按焦平面"每像素世界尺寸"精确换算，保证 1:1 跟手手感
        QPointF diff = currentPos - m_lastMousePos;
        double pixSize = ComputeActualPixelSize() * DevicePixelScale();
        QVector3D u(static_cast<float>(diff.x() * pixSize),
            static_cast<float>(-diff.y() * pixSize),
            0.0f);
        if (object_centered_view_) {
            u = -u;
        }
        MoveCamera(u);
    } else if (event->buttons() & Qt::LeftButton) {
        // 左键 = 旋转（CloudCompare 式 arcball，球心跟随枢轴点的屏幕投影）
        m_showRotationCenter = true;

        static QVector3D s_lastOrientation;
        if (!m_mouseMoved) {
            // 首次移动才初始化起点，避免按下到移动之间的跳变
            s_lastOrientation = ConvertMousePositionToOrientation(m_lastMousePos);
        }
        QVector3D currentOrientation = ConvertMousePositionToOrientation(currentPos);
        QMatrix4x4 rotMat = FromToRotation(s_lastOrientation, currentOrientation);
        s_lastOrientation = currentOrientation;

        // 增量旋转（相机系下左乘）
        view_rotation_ = rotMat * view_rotation_;
        InvalidateVisualization();
    }

    m_mouseMoved = true;
    m_lastMousePos = currentPos;
    update();
}

void PointcloudDisplay::mousePressEvent(QMouseEvent* event)
{
    m_lastMousePos = event->pos();
    pick_start_pos_ = event->pos();
    m_mouseMoved = false;
    m_mouseButtonPressed = true;
    m_ignoreMouseReleaseEvent = false;

    if (event->buttons() & Qt::LeftButton) {
        m_leftButtonPressed = true;
        m_showRotationCenter = true;  // 左键按下时显示旋转中心
        update();  // 触发重绘以显示旋转中心
    }
}

void PointcloudDisplay::mouseDoubleClickEvent(QMouseEvent* event)
{
    if (event->button() != Qt::LeftButton) {
        return;
    }
    // 双击：把点击位置设为新的旋转枢轴（带相机补偿，画面不跳动）
    m_ignoreMouseReleaseEvent = true;

    makeCurrent();
    QVector3D worldPos;
    bool valid = GetClick3DPos(event->x(), event->y(), worldPos);
    doneCurrent();
    if (!valid) {
        return;
    }

    SetPivotPoint(worldPos, true, false);

    m_showRotationCenter = true;
    update();

    LOG_DEBUG("New rotation center set at: ({}, {}, {})",
        worldPos.x(),
        worldPos.y(),
        worldPos.z());

    // 短暂显示旋转中心后隐藏
    QTimer::singleShot(1000, this, [this]() {
        if (!m_leftButtonPressed) {
            m_showRotationCenter = false;
            update();
        }
    });
}

void PointcloudDisplay::mouseReleaseEvent(QMouseEvent* event)
{
    if (m_ignoreMouseReleaseEvent) {
        m_ignoreMouseReleaseEvent = false;
        return;
    }

    // 注意：不要用 m_mouseMoved 拦截拾取——点击选点时鼠标常有 1~3px 的微小抖动，
    // 会导致拾取被跳过（回归 bug）。仅按位移距离判断即可。
    m_mouseButtonPressed = false;
    m_mouseMoved = false;

    if (!(event->buttons() & Qt::LeftButton) && m_leftButtonPressed) {
        m_leftButtonPressed = false;
        m_showRotationCenter = false;  // 左键释放时隐藏旋转中心
        update();  // 触发重绘以隐藏旋转中心
    }

    if (event->button() == Qt::RightButton
        && (event->modifiers() & Qt::ShiftModifier)
        && pick_engine_) {
        pick_engine_->ClearSelection();
        SetPickEngine(nullptr);
        emit PickModeExited();
        update();
        return;
    }

    if (!pick_engine_ || !pick_provider_) {
        return;
    }

    QPoint delta = event->pos() - pick_start_pos_;
    if (delta.manhattanLength() >= 5) {
        return;
    }

    makeCurrent();

    QVector3D pick_point_3d;
    bool valid = GetClick3DPos(event->x(), event->y(), pick_point_3d);

    doneCurrent();

    if (!valid) {
        return;
    }

    const double dpr = DevicePixelScale();
    QPoint glPos(qRound(event->x() * dpr), GLViewportHeight() - 1 - qRound(event->y() * dpr));
    QSize viewport(GLViewportWidth(), GLViewportHeight());

    if (event->modifiers() & Qt::ShiftModifier) {
        if (pick_engine_->HasSelection()) {
            int idx = pick_engine_->FindNearestAnnotation(glPos, viewport,
                                                           m_projectionMatrix, m_viewMatrix);
            if (idx >= 0) {
                pick_engine_->RemoveAnnotation(idx);
                update();
                return;
            }
        }
    }

    pick_engine_->ExecutePick(glPos,
                              viewport,
                              pick_point_3d,
                              m_projectionMatrix,
                              m_viewMatrix);
    update();
}

void PointcloudDisplay::wheelEvent(QWheelEvent* event)
{
    // 缩放 == 相机沿视线移动（无 clamp、无边界，永不失效）
    int padDelta = event->angleDelta().y();
    if (padDelta != 0) {
        OnWheelEvent(padDelta / 8.0f); // 滚轮角度（度）
        event->accept();
    }
}

void PointcloudDisplay::BuildCoorAxis(float length)
{
    // 顶点数据 - 12个顶点
    // 坐标轴有三条线段：X轴、Y轴、Z轴，每个轴由两个点组成，因此一共有6个点
    // 每个点有3个坐标值（x、y、z）和3个颜色值（r、g、b），因此每个点有6个分量
    //  0   1   2   3   4   5
    // [x0, y0, z0, r0, g0, b0,
    //  x1, y1, z1, r1, g1, b1,
    //  x2, y2, z2, r2, g2, b2,
    //  ...
    // ]
    // X轴：起点 (0, 0, 0)，终点 (length, 0, 0)
    // Y轴：起点 (0, 0, 0)，终点 (0, length, 0)
    // Z轴：起点 (0, 0, 0)，终点 (0, 0, length)
    std::vector<float> axis_vertexs(36, 0.0);
    axis_vertexs[3] = 1.0;
    axis_vertexs[6] = length;
    axis_vertexs[9] = 1.0;
    axis_vertexs[16] = 1.0;
    axis_vertexs[19] = length;
    axis_vertexs[22] = 1.0;
    axis_vertexs[29] = 1.0;
    axis_vertexs[32] = length;
    axis_vertexs[35] = 1.0;
    // 创建VAO/VBO
    glGenVertexArrays(1, &VAO_axis_);
    glGenBuffers(1, &VBO_axis_);
    glCheckError();
    // 绑定VAO/VBO
    glBindVertexArray(VAO_axis_);
    glBindBuffer(GL_ARRAY_BUFFER, VBO_axis_);
    glBufferData(GL_ARRAY_BUFFER, axis_vertexs.size() * sizeof(float), &axis_vertexs[0], GL_STATIC_DRAW);
    glCheckError();
    // 指定顶点数组对象的顶点属性数据 - 索引、大小、类型、归一化、连续点字节偏移量、顶点缓冲对象首个属性起始位置
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    // 解绑VBO/VAO
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void PointcloudDisplay::BuildPointdata()
{
    // 顶点属性在上传后的 VBO 绑定时设置；无缓冲区时指定属性会产生 GL_INVALID_OPERATION。
    glGenVertexArrays(1, &VAO_point_);
}

void PointcloudDisplay::PointPaintMethod2()
{
    DrawFrame(single_frame_);
}

const YsGui::ParamStruct& PointcloudDisplay::GetDisplayParameters() const
{
    return YsGui::Parameters();
}

void PointcloudDisplay::SetPivotPoint(const QVector3D& point, bool autoUpdateCameraPos, bool verbose)
{
    if (autoUpdateCameraPos && object_centered_view_) {
        // 参照 CloudCompare setPivotPoint：改变枢轴点时补偿相机位置，保持画面不跳
        QVector3D pivotShift = pivot_point_ - point;
        QVector3D pivotShiftInCameraCS = view_rotation_.map(pivotShift);
        QVector3D newCameraCenter = camera_center_ + pivotShiftInCameraCS - pivotShift;
        if (!perspective_view_) {
            // 正交模式：保持缩放级别（焦距）不变
            newCameraCenter.setZ(static_cast<float>(focal_distance_) + point.z());
        }
        SetCameraCenter(newCameraCenter);
    }

    pivot_point_ = point;
    if (object_centered_view_) {
        focal_distance_ = camera_center_.z() - pivot_point_.z();
    }

    InvalidateViewport();
    InvalidateVisualization();

    if (verbose) {
        m_showRotationCenter = true;
        QTimer::singleShot(1000, this, [this]() {
            if (!m_leftButtonPressed) {
                m_showRotationCenter = false;
                update();
            }
        });
    }
}

void PointcloudDisplay::SetCameraCenter(const QVector3D& point)
{
    if ((camera_center_ - point).lengthSquared() != 0.0f) {
        camera_center_ = point;
        if (object_centered_view_) {
            focal_distance_ = camera_center_.z() - pivot_point_.z();
        }
        InvalidateViewport();
        InvalidateVisualization();
    }
}

void PointcloudDisplay::MoveCamera(const QVector3D& v)
{
    SetCameraCenter(camera_center_ + v);
}

void PointcloudDisplay::InvalidateViewport()
{
    projection_valid_ = false;
}

void PointcloudDisplay::InvalidateVisualization()
{
    view_valid_ = false;
}

void PointcloudDisplay::EnsureValidMatrices()
{
    if (!view_valid_) {
        updateViewMatrix();
    }
    if (!projection_valid_) {
        updateProjectionMatrix();
    }
}

double PointcloudDisplay::ComputeInitialFocalDistance() const
{
    // 目标：视野短边方向可见范围约 ±25 米（完整看到 20 米圆环）
    const double w = std::max(1.0, static_cast<double>(GLViewportWidth()));
    const double h = std::max(1.0, static_cast<double>(GLViewportHeight()));
    const double minAspect = std::min(1.0, h / w);
    double focal = 25.0 / (minAspect * std::tan(fov_deg_ * M_PI / 360.0));
    if (!std::isfinite(focal) || focal <= 0.0) {
        focal = 50.0;
    }
    return focal;
}

void PointcloudDisplay::ConfigureInitialView()
{
    // 视口尺寸未就绪时等待 resizeGL 再配置
    if (initial_view_configured_ || GLViewportWidth() <= 1 || GLViewportHeight() <= 1) {
        return;
    }
    // 旋转中心固定在原点（用户可通过双击调整）
    pivot_point_ = QVector3D(0.0f, 0.0f, 0.0f);
    // 初始朝向：雷达坐标惯例（X 前、Y 左、Z 上），俯视显示 XOY 平面（与视角切换中的 Top 视图保持一致）
    SetView(ViewOrientation::Top);
    // 短边视野约 ±25 米，保证至少能看到 20 米距离
    focal_distance_ = ComputeInitialFocalDistance();
    camera_center_ = QVector3D(0.0f, 0.0f, static_cast<float>(focal_distance_));
    view_valid_ = false;
    projection_valid_ = false;
    initial_view_configured_ = true;
}

void PointcloudDisplay::DrawAxis()
{
    shaderProgram_axis_.bind();
    glBindVertexArray(VAO_axis_);
    glLineWidth(2.0f);
    glDrawArrays(GL_LINES, 0, 6);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

double PointcloudDisplay::ComputeActualPixelSize() const
{
    // 焦平面处每个物理像素对应的世界尺寸（参照 CloudCompare computePixelSize）
    const int glWidth = GLViewportWidth();
    return (glWidth > 0 ? (focal_distance_ * ComputeDistanceToWidthRatio()) / glWidth : 1.0);
}

double PointcloudDisplay::ComputeDistanceToWidthRatio() const
{
    // 基于 FOV 的"距离/视野宽度"比例（参照 CloudCompare computeDistanceToWidthRatio）
    const double glWidth = GLViewportWidth();
    const double glHeight = GLViewportHeight();
    if (glWidth <= 0 || glHeight <= 0) {
        return 1.0;
    }

    double ar = std::min(1.0, glWidth / (glHeight * camera_aspect_ratio_)); // <= 1
    return (2.0 * std::tan(fov_deg_ * M_PI / 360.0)) / ar;
}

QMatrix4x4 PointcloudDisplay::ComputeProjectionMatrix() const
{
    // 动态裁剪面：把场景包围盒（8 角点）+ 旋转枢轴 + 距离圆环
    // 变换到相机坐标系求深度范围，保证近远裁剪面永远紧贴场景
    QMatrix4x4 viewmat = ComputeViewMatrix();

    double zMin = std::numeric_limits<double>::quiet_NaN();
    double zMax = zMin;

    auto updateDepthRange = [&zMin, &zMax, &viewmat](const QVector3D& P) {
        QVector3D Pd = viewmat * P;
        double z = -Pd.z(); // OpenGL 相机朝 -Z 方向
        if (std::isnan(zMin) || z < zMin) {
            zMin = z;
        }
        if (std::isnan(zMax) || z > zMax) {
            zMax = z;
        }
    };

    if (scene_bb_valid_) {
        const QVector3D bbMin = scene_bb_min_;
        const QVector3D bbMax = scene_bb_max_;
        std::array<QVector3D, 8> bbCorners{
            bbMin,
            QVector3D(bbMin.x(), bbMin.y(), bbMax.z()),
            QVector3D(bbMin.x(), bbMax.y(), bbMin.z()),
            QVector3D(bbMax.x(), bbMin.y(), bbMin.z()),
            bbMax,
            QVector3D(bbMax.x(), bbMax.y(), bbMin.z()),
            QVector3D(bbMax.x(), bbMin.y(), bbMax.z()),
            QVector3D(bbMin.x(), bbMax.y(), bbMax.z())
        };
        for (const QVector3D& P : bbCorners) {
            updateDepthRange(P);
        }
    }

    // 旋转枢轴符号（可能超出包围盒）
    updateDepthRange(pivot_point_);

    // 距离圆环（z=0 平面，常驻显示）
    {
        const double circleMax = std::max(m_circleRadiusMax, 0.0);
        updateDepthRange(QVector3D(static_cast<float>(circleMax), static_cast<float>(circleMax), 0.0f));
        updateDepthRange(QVector3D(static_cast<float>(-circleMax), static_cast<float>(circleMax), 0.0f));
        updateDepthRange(QVector3D(static_cast<float>(circleMax), static_cast<float>(-circleMax), 0.0f));
        updateDepthRange(QVector3D(static_cast<float>(-circleMax), static_cast<float>(-circleMax), 0.0f));
    }

    const double bbHalfDiag = std::max(scene_bb_diag_, m_circleRadiusMax * 2.0) / 2.0;
    const double epsilon = std::max(bbHalfDiag / 1000.0, 1.0e-6);

    // 纵横比（高度/宽度）
    const double ar = static_cast<double>(GLViewportHeight()) / GLViewportWidth();

    QMatrix4x4 projMatrix;
    double zNear = zMin;
    double zFar = zMax;

    if (perspective_view_) {
        double minZFar = static_cast<double>(1.0e-6) / kZNearCoef;
        if (zFar < minZFar || !std::isfinite(zFar)) {
            // 相机前方没有物体（或太近），使用兜底值
            zFar = minZFar;
            zNear = epsilon;
        } else {
            zNear = std::max(zMin, zFar * kZNearCoef);
        }

        // zNear 不能太接近 zFar（深度精度）
        double delta = zFar / 2;
        if (zFar - zNear < delta) {
            zNear -= delta / 2;
            zFar += delta / 2;
        }

        double xMax = zNear * std::tan(fov_deg_ * M_PI / 360.0);
        double yMax = xMax * ar;
        projMatrix.frustum(-xMax, xMax, -yMax, yMax, zNear, zFar);
    } else {
        zFar += epsilon;
        zNear -= epsilon;
        double xMax = std::abs(focal_distance_) * std::tan(fov_deg_ * M_PI / 360.0);
        double yMax = xMax * ar;
        projMatrix = Ortho(-xMax, xMax, -yMax, yMax, zNear, zFar);
    }

    return projMatrix;
}

QMatrix4x4 PointcloudDisplay::ComputeViewMatrix() const
{
    // 参照 CloudCompare computeViewMatrix：
    // V = T(rotCenter - camCenter) · R · T(-rotCenter)
    QMatrix4x4 viewmat;
    viewmat.setToIdentity();
    const QVector3D& rotationCenter = pivot_point_;

    // 原点移到旋转中心
    viewmat.translate(-rotationCenter);
    // 纯旋转
    viewmat = view_rotation_ * viewmat;
    // 恢复原点并抵消相机中心
    QVector4D t = viewmat.column(3);
    t += QVector4D(rotationCenter, 0.0f);
    t -= QVector4D(camera_center_, 0.0f);
    viewmat.setColumn(3, t);

    return viewmat;
}

QMatrix4x4 PointcloudDisplay::Ortho(double left, double right, double bottom, 
                                    double top, double nearVal, double farVal) const
{
    QMatrix4x4 matrix;
    double dx = (right - left);
    double dy = (top - bottom);
    double dz = (farVal - nearVal);
    if (dx != 0 && dy != 0 && dz != 0)
    {
        float* mat = matrix.data();
        // set OpenGL perspective projection matrix
        mat[0] = 2.0 / dx;
        mat[1] = 0;
        mat[2] = 0;
        mat[3] = 0;

        mat[4] = 0;
        mat[5] = 2.0 / dy;
        mat[6] = 0;
        mat[7] = 0;

        mat[8] = 0;
        mat[9] = 0;
        mat[10] = -2.0 / dz;
        mat[11] = 0;

        mat[12] = -(right + left) / dx;
        mat[13] = -(top + bottom) / dy;
        mat[14] = -(farVal + nearVal) / dz;
        mat[15] = 1.0;
    }
    else
    {
        matrix.setToIdentity();
    }

    return matrix;
}

QMatrix4x4 PointcloudDisplay::FromToRotation(const QVector3D& from, const QVector3D& to)
{
    // 计算将向量 from 旋转到 to 的旋转矩阵（列主序，作用于列向量）
    const QVector3D f = from.normalized();
    const QVector3D t = to.normalized();

    const float c = QVector3D::dotProduct(f, t);
    const float kEpsilon = 1.0e-6f;

    QMatrix4x4 result;
    result.setToIdentity();

    if (c > 1.0f - kEpsilon) {
        // 两向量几乎同向：单位阵
        return result;
    }

    if (c < -1.0f + kEpsilon) {
        // 两向量几乎反向：绕任意正交轴旋转 180°
        QVector3D axis = QVector3D::crossProduct(QVector3D(0.0f, 0.0f, 1.0f), f);
        if (axis.lengthSquared() < kEpsilon) {
            axis = QVector3D::crossProduct(QVector3D(0.0f, 1.0f, 0.0f), f);
        }
        axis.normalize();
        result.rotate(180.0f, axis);
        return result;
    }

    // 罗德里格斯公式：R = c·I + [v]x + vv^T·(1-c)/s^2
    const QVector3D v = QVector3D::crossProduct(f, t); // v = f × t, |v| = sin
    const float s2 = QVector3D::dotProduct(v, v);
    const float k = (1.0f - c) / s2;
    const float x = v.x(), y = v.y(), z = v.z();

    // 列主序填充
    result(0, 0) = c + x * x * k;
    result(1, 0) = z + y * x * k;
    result(2, 0) = -y + z * x * k;

    result(0, 1) = -z + x * y * k;
    result(1, 1) = c + y * y * k;
    result(2, 1) = x + z * y * k;

    result(0, 2) = y + x * z * k;
    result(1, 2) = -x + y * z * k;
    result(2, 2) = c + z * z * k;

    return result;
}

QVector3D PointcloudDisplay::ConvertMousePositionToOrientation(const QPointF& p)
{
    // CloudCompare 式 arcball：以"枢轴点屏幕投影"为球心（并限制在屏幕中央区域）
    const double xc = width() / 2.0;
    const double yc = height() / 2.0;
    const double dpr = DevicePixelScale();

    QVector3D Q2D;
    {
        int viewport[4] = { 0, 0, GLViewportWidth(), GLViewportHeight() };
        QVector3D proj2D;
        if (Project<float, float>(pivot_point_, m_viewMatrix.data(), m_projectionMatrix.data(),
                viewport, proj2D)) {
            // 投影结果在物理像素，转回逻辑像素
            double qx = proj2D.x() / dpr;
            double qy = proj2D.y() / dpr;
            // 虚拟球心始终保持在屏幕中央 1/4 ~ 3/4 区域
            qx = std::min(qx, 3.0 * width() / 4.0);
            qx = std::max(qx, width() / 4.0);
            qy = std::min(qy, 3.0 * height() / 4.0);
            qy = std::max(qy, height() / 4.0);
            Q2D.setX(static_cast<float>(qx));
            Q2D.setY(static_cast<float>(qy));
        } else {
            // 枢轴在相机后方：退化为屏幕中心
            return QVector3D(0.0f, 0.0f, 1.0f);
        }
    }

    // 鼠标坐标转 GL 屏幕坐标（y 向上）
    const double x = p.x();
    const double y = (height() - 1) - p.y();

    QVector3D v(static_cast<float>(x - Q2D.x()),
        static_cast<float>(y - Q2D.y()),
        0.0f);

    v.setX(static_cast<float>(std::max(std::min(v.x() / xc, 1.0), -1.0)));
    v.setY(static_cast<float>(std::max(std::min(v.y() / yc, 1.0), -1.0)));

    // 投影到单位球
    const double d2 = v.x() * v.x() + v.y() * v.y();
    if (d2 > 1.0) {
        const double d = std::sqrt(d2);
        v.setX(static_cast<float>(v.x() / d));
        v.setY(static_cast<float>(v.y() / d));
    } else {
        v.setZ(static_cast<float>(std::sqrt(1.0 - d2)));
    }

    return v;
}

bool PointcloudDisplay::GetClick3DPos(int x, int y, QVector3D& P3D)
{
    // 注意：调用者需保证 GL 上下文可用（paintGL 内或 makeCurrent 之后）
    EnsureValidMatrices();

    const double dpr = DevicePixelScale();
    const int glW = GLViewportWidth();
    const int glH = GLViewportHeight();

    const int px = qRound(x * dpr);
    const int py = qRound(y * dpr);
    const int glY = glH - 1 - py;

    GLfloat depthValue = 1.0f;
    glReadPixels(px, glY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depthValue);

    int foundX = px;
    int foundY = glY;

    if (depthValue >= 1.0f) {
        // 点击到背景：在邻域内寻找最近的（深度最小的）点
        const int searchRadiusPix = qRound(5 * dpr);
        const int rx = std::max(0, px - searchRadiusPix);
        const int ry = std::max(0, glY - searchRadiusPix);
        const int rw = std::min(glW - rx, 2 * searchRadiusPix + 1);
        const int rh = std::min(glH - ry, 2 * searchRadiusPix + 1);
        if (rw <= 0 || rh <= 0) {
            return false;
        }

        std::vector<GLfloat> depthBuffer(static_cast<size_t>(rw) * rh, 1.0f);
        glReadPixels(rx, ry, rw, rh, GL_DEPTH_COMPONENT, GL_FLOAT, depthBuffer.data());

        float minDepth = 1.0f;
        bool found = false;
        for (int dy = 0; dy < rh; ++dy) {
            for (int dx = 0; dx < rw; ++dx) {
                const float depth = depthBuffer[static_cast<size_t>(dy) * rw + dx];
                if (depth < minDepth && depth > 0.0f) {
                    minDepth = depth;
                    foundX = rx + dx;
                    foundY = ry + dy;
                    found = true;
                }
            }
        }
        if (!found) {
            return false;
        }
        depthValue = minDepth;
    }

    // 反投影到世界坐标（NDC → 裁剪 → 眼空间 → 世界空间）
    // 注意：使用 Qt 官方矩阵求逆（QMatrix4x4::inverted），
    // 不要使用自实现的高斯消元求逆（对旋转矩阵求逆有误）
    QVector3D ndcPos;
    ndcPos.setX((2.0f * foundX) / glW - 1.0f);
    ndcPos.setY((2.0f * foundY) / glH - 1.0f);
    ndcPos.setZ(2.0f * depthValue - 1.0f);

    QVector4D clipPos(ndcPos.x(), ndcPos.y(), ndcPos.z(), 1.0f);
    QVector4D eyePos = m_projectionMatrix.inverted() * clipPos;
    eyePos /= eyePos.w();
    QVector4D worldPos = m_viewMatrix.inverted() * eyePos;
    P3D = QVector3D(worldPos.x(), worldPos.y(), worldPos.z());
    return true;
}

void PointcloudDisplay::UpdateAutoPivotAtCenter()
{
    if (!m_autoPickPivotAtCenter || m_mouseMoved || m_mouseButtonPressed || pick_engine_) {
        return;
    }

    EnsureValidMatrices();

    const int glW = GLViewportWidth();
    const int glH = GLViewportHeight();

    GLfloat depth = 1.0f;
    glReadPixels(glW / 2, glH / 2, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
    if (depth >= 1.0f) {
        return; // 屏幕中心无有效点
    }

    // 反投影（NDC → 裁剪 → 眼空间 → 世界空间）
    QVector3D ndcPos;
    ndcPos.setX((2.0f * (glW / 2)) / glW - 1.0f);
    ndcPos.setY((2.0f * (glH / 2)) / glH - 1.0f);
    ndcPos.setZ(2.0f * depth - 1.0f);

    QVector4D clipPos(ndcPos.x(), ndcPos.y(), ndcPos.z(), 1.0f);
    QVector4D eyePos = m_projectionMatrix.inverted() * clipPos;
    eyePos /= eyePos.w();
    QVector4D worldPos = m_viewMatrix.inverted() * eyePos;
    QVector3D P(worldPos.x(), worldPos.y(), worldPos.z());

    if ((P - pivot_point_).lengthSquared() > 1.0e-8f) {
        SetPivotPoint(P, true, false);
    }
}

void PointcloudDisplay::updateProjectionMatrix()
{
    m_projectionMatrix = ComputeProjectionMatrix();
    projection_valid_ = true;
}

void PointcloudDisplay::updateViewMatrix()
{
    m_viewMatrix = ComputeViewMatrix();
    view_valid_ = true;
}

void PointcloudDisplay::updateModelMatrix()
{
    m_modelMatrix.setToIdentity();
}

void PointcloudDisplay::createCoordinateCircle()
{
    const float maxRadius = static_cast<float>(m_circleRadiusMax);
    float radius = 5.0f;
    int segments = 360;

    while (radius <= maxRadius) {
        for (int i = 0; i <= segments; ++i) {
            float theta = 2.0f * M_PI * float(i) / float(segments);
            float x = radius * cos(theta);
            float y = radius * sin(theta);
            m_circleVertices.push_back(x);
            m_circleVertices.push_back(y);
            m_circleVertices.push_back(0.0f);
        }
        m_circleDistances.push_back(radius);

        if (radius >= 30.0f && radius < 50.0f) {
            radius += 10.0f;
        }
        else if(radius >= 0.0f && radius < 30.0f){
            radius += 5.0f;
        }
        else if (radius >= 50.0f) {
            radius += 15.0f;
        }
    }

    m_circleTotalVertices = m_circleVertices.size() / 3;
    glGenVertexArrays(1, &VAO_meshline_);
    glGenBuffers(1, &VBO_meshline_);
    glCheckError();

    glBindVertexArray(VAO_meshline_);

    glBindBuffer(GL_ARRAY_BUFFER, VBO_meshline_);
    
    glBufferData(GL_ARRAY_BUFFER, m_circleVertices.size() * sizeof(float), m_circleVertices.data(), GL_STATIC_DRAW);
    // 设置顶点属性指针
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // 解绑 VBO 和 VAO
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}
void PointcloudDisplay::renderCoordinateCircle()
{
    glBindVertexArray(VAO_meshline_);
    
    for (int i = 0; i < m_circleDistances.size(); ++i) {
        shaderProgram_mesh_.bind();
        int startIndex = i * 361;
        glLineWidth(0.5f);
        int vertexCount = (i == m_circleDistances.size() - 1) ? (m_circleTotalVertices - startIndex) : 361;
        glDrawArrays(GL_LINE_LOOP, startIndex, vertexCount);
        glCheckError();
    }
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void PointcloudDisplay::UpdateFps()
{
    ++fps_frame_count_;
    auto now = std::chrono::steady_clock::now();
    fps_last_render_time_ = now; // Record last render time
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - fps_last_time_);
    if (elapsed.count() >= 1000) {
        fps_value_ = (fps_frame_count_ * 1000.0f) / static_cast<float>(elapsed.count());
        fps_frame_count_ = 0;
        fps_last_time_ = now;
    }
}

void PointcloudDisplay::UpdateDataFps()
{
    ++data_frame_count_;
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - data_fps_last_time_);
    if (elapsed.count() >= 1000) {
        data_fps_value_ = (data_frame_count_ * 1000.0f) / static_cast<float>(elapsed.count());
        data_frame_count_ = 0;
        data_fps_last_time_ = now;
    }
}

void PointcloudDisplay::SetPickEngine(SinglePickBase* engine)
{
    pick_engine_ = engine;

    if (engine) {
        render_mailbox_->SetFrozen(true);
        setCursor(Qt::CrossCursor);
        if (pick_provider_) {
            engine->SetProvider(pick_provider_);
        }
        emit PickFreezeChanged(true);
        return;
    }

    render_mailbox_->SetFrozen(false);
    setCursor(Qt::ArrowCursor);
    emit PickFreezeChanged(false);
}

SinglePickBase* PointcloudDisplay::GetPickEngine() const
{
    return pick_engine_;
}

void PointcloudDisplay::SetPickData(std::shared_ptr<PickDataProvider> provider)
{
    pick_provider_ = std::move(provider);

    if (pick_engine_) {
        pick_engine_->SetProvider(pick_provider_);
    }
}

void PointcloudDisplay::keyPressEvent(QKeyEvent* event)
{
    if (event->key() == Qt::Key_Escape && pick_engine_) {
        pick_engine_->ClearSelection();
        update();
        return;
    }

    QOpenGLWidget::keyPressEvent(event);
}

void PointcloudDisplay::BuildPickSphere()
{
    const int slices = 16;
    const int stacks = 16;

    std::vector<float> vertices;
    std::vector<unsigned int> indices;

    for (int i = 0; i <= stacks; ++i) {
        float phi = 3.1415926f * i / stacks;
        for (int j = 0; j <= slices; ++j) {
            float theta = 2.0f * 3.1415926f * j / slices;
            float nx = cosf(theta) * sinf(phi);
            float ny = sinf(theta) * sinf(phi);
            float nz = cosf(phi);
            vertices.push_back(nx);
            vertices.push_back(ny);
            vertices.push_back(nz);
            vertices.push_back(nx);
            vertices.push_back(ny);
            vertices.push_back(nz);
        }
    }

    for (int i = 0; i < stacks; ++i) {
        for (int j = 0; j < slices; ++j) {
            unsigned int first = i * (slices + 1) + j;
            unsigned int second = first + slices + 1;
            indices.push_back(first);
            indices.push_back(second);
            indices.push_back(first + 1);
            indices.push_back(second);
            indices.push_back(second + 1);
            indices.push_back(first + 1);
        }
    }

    pick_sphere_index_count_ = static_cast<uint32_t>(indices.size());

    glGenVertexArrays(1, &VAO_pick_sphere_);
    glGenBuffers(1, &VBO_pick_sphere_);
    glGenBuffers(1, &EBO_pick_sphere_);

    glBindVertexArray(VAO_pick_sphere_);

    glBindBuffer(GL_ARRAY_BUFFER, VBO_pick_sphere_);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO_pick_sphere_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
}

void PointcloudDisplay::UpdateRoiOverlay(const std::vector<float>& roi_vertices)
{
    roi_vertices_ = roi_vertices;
    roi_vertex_count_ = static_cast<uint32_t>(roi_vertices.size() / 6);

    makeCurrent();

    if (VAO_roi_ != 0) {
        glDeleteVertexArrays(1, &VAO_roi_);
        glDeleteBuffers(1, &VBO_roi_);
        VAO_roi_ = 0;
        VBO_roi_ = 0;
    }

    if (roi_vertices.empty()) {
        doneCurrent();
        return;
    }

    glGenVertexArrays(1, &VAO_roi_);
    glGenBuffers(1, &VBO_roi_);

    glBindVertexArray(VAO_roi_);
    glBindBuffer(GL_ARRAY_BUFFER, VBO_roi_);
    glBufferData(GL_ARRAY_BUFFER, roi_vertices.size() * sizeof(float), roi_vertices.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);

    doneCurrent();
}

void PointcloudDisplay::DrawRoiRect()
{
    if (!roi_visible_ || VAO_roi_ == 0 || roi_vertex_count_ == 0) {
        return;
    }

    shaderProgram_pick_sphere_.bind();
    shaderProgram_pick_sphere_.setUniformValue("projection", m_projectionMatrix);
    shaderProgram_pick_sphere_.setUniformValue("view", m_viewMatrix);

    QMatrix4x4 model;
    QVector3D center(0, 0, 0);
    model.translate(center);
    shaderProgram_pick_sphere_.setUniformValue("model", model);

    QMatrix4x4 viewInv = m_viewMatrix.inverted();
    QVector3D cameraPos(viewInv(0, 3), viewInv(1, 3), viewInv(2, 3));
    shaderProgram_pick_sphere_.setUniformValue("viewPos", cameraPos);
    shaderProgram_pick_sphere_.setUniformValue("lightDir", QVector3D(1, 1, 1).normalized());

    glLineWidth(2.0f);
    glBindVertexArray(VAO_roi_);
    glDrawArrays(GL_LINES, 0, roi_vertex_count_);
    glBindVertexArray(0);
}

void PointcloudDisplay::SetRoiVisible(bool visible)
{
    roi_visible_ = visible;
    update();
}

} // namespace ys_scene
