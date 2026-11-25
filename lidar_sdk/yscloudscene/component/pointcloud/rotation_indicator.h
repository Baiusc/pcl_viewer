#pragma once

#include <QOpenGLShaderProgram>
#include <QOpenGLExtraFunctions>
#include <QVector3D>
#include <QMatrix4x4>
#include <vector>

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
#include <QtOpenGLWidgets/QOpenGLWidget>
#else
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#endif

class RotationIndicator : protected QOpenGLExtraFunctions
{
public:
    // 构造函数和析构函数
    RotationIndicator();
    ~RotationIndicator();

    // 初始化函数 - 在OpenGL上下文中调用
    void Initialize();

    // 绘制旋转中心指示器
    void DrawRotationCenter(
        const QVector3D& rotation_center,
        const QMatrix4x4& projection_matrix,
        const QMatrix4x4& view_matrix,
        const QMatrix4x4& model_matrix,
        const QVector3D& camera_pos,
        float distance_to_center
    );

private:
    // 绘制黄色中心球
    void DrawCenterSphere(
        const QVector3D& rotation_center,
        const QMatrix4x4& projection_matrix,
        const QMatrix4x4& view_matrix,
        const QMatrix4x4& model_matrix,
        const QVector3D& camera_pos,
        float radius
    );

    // 绘制方向指示器
    void DrawDirectionIndicators(
        const QVector3D& rotation_center,
        const QMatrix4x4& projection_matrix,
        const QMatrix4x4& view_matrix,
        const QMatrix4x4& model_matrix,
        const QVector3D& camera_pos,
        float radius,
        float tube_radius
    );

    // 初始化着色器
    void InitializeCenterShader();
    void InitializeIndicatorShader();

    // 几何体生成函数
    void GenerateRing(
        std::vector<float>& vertices,
        std::vector<float>& colors,
        std::vector<float>& normals,
        const QVector3D& center,
        float radius,
        float tube_radius,
        int segments,
        int axis_plane,
        const QVector3D& color
    );

    void GenerateAxis(
        std::vector<float>& vertices,
        std::vector<float>& colors,
        std::vector<float>& normals,
        const QVector3D& center,
        float radius,
        float length,
        int segments,
        int axis,
        const QVector3D& color
    );

    void AddTriangle(
        std::vector<float>& vertices,
        std::vector<float>& colors,
        std::vector<float>& normals,
        const QVector3D& p1,
        const QVector3D& p2,
        const QVector3D& p3,
        const QVector3D& color,
        const QVector3D& n1,
        const QVector3D& n2,
        const QVector3D& n3
    );

    // 渲染函数
    void RenderGeometry(
        const std::vector<float>& vertices,
        const std::vector<float>& colors,
        const std::vector<float>& normals,
        const QMatrix4x4& projection_matrix,
        const QMatrix4x4& view_matrix,
        const QMatrix4x4& model_matrix,
        const QVector3D& camera_pos,
        QOpenGLShaderProgram& shader
    );

    // 成员变量
    QOpenGLShaderProgram center_shader_;
    QOpenGLShaderProgram indicator_shader_;
    bool center_shader_initialized_;
    bool indicator_shader_initialized_;
};