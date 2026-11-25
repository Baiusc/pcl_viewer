#include "rotation_indicator.h"
#include "log/logging.h"

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

RotationIndicator::RotationIndicator()
    : center_shader_initialized_(false),
    indicator_shader_initialized_(false)
{
}

RotationIndicator::~RotationIndicator()
{
    // 清理资源
}

void RotationIndicator::Initialize()
{
    initializeOpenGLFunctions();
    InitializeCenterShader();
    InitializeIndicatorShader();
}

void RotationIndicator::DrawRotationCenter(
    const QVector3D& rotation_center,
    const QMatrix4x4& projection_matrix,
    const QMatrix4x4& view_matrix,
    const QMatrix4x4& model_matrix,
    const QVector3D& camera_pos,
    float distance_to_center)
{
    // 固定的世界空间大小 - 不受缩放影响
    const float desired_screen_size_pixels = 50.0f;

    // 计算世界空间中的固定半径
    float fov_y_radians = 45.0f * M_PI / 180.0f;
    float pixel_size = 2.0f * tan(fov_y_radians / 2.0f) * distance_to_center / 1000.0f; // 假设窗口高度大约1000像素
    float sphere_radius = (desired_screen_size_pixels * pixel_size) / 2.0f;

    // 首先绘制中心球体
    DrawCenterSphere(
        rotation_center,
        projection_matrix,
        view_matrix,
        model_matrix,
        camera_pos,
        sphere_radius
    );

    // 然后绘制指示器
    float indicator_radius = sphere_radius * 5.0f; // 指示器比球体大5倍
    // 2D 模式：将 tube_radius 复用为起始偏移量 (start_offset)，即避开中心球体的距离
    float start_offset = sphere_radius * 1.0f; 

    DrawDirectionIndicators(
        rotation_center,
        projection_matrix,
        view_matrix,
        model_matrix,
        camera_pos,
        indicator_radius,
        start_offset
    );
}

void RotationIndicator::DrawCenterSphere(
    const QVector3D& rotation_center,
    const QMatrix4x4& projection_matrix,
    const QMatrix4x4& view_matrix,
    const QMatrix4x4& model_matrix,
    const QVector3D& camera_pos,
    float radius)
{
    // 球体参数
    const int slices = 32;
    const int stacks = 32;

    // 创建顶点数据
    std::vector<float> vertices;
    std::vector<float> normals;

    // 生成球体顶点和法线
    for (int i = 0; i <= stacks; ++i) {
        float phi = M_PI * float(i) / float(stacks);
        float sin_phi = sin(phi);
        float cos_phi = cos(phi);

        for (int j = 0; j <= slices; ++j) {
            float theta = 2.0f * M_PI * float(j) / float(slices);
            float sin_theta = sin(theta);
            float cos_theta = cos(theta);

            float x = cos_theta * sin_phi;
            float y = sin_theta * sin_phi;
            float z = cos_phi;

            vertices.push_back(rotation_center.x() + radius * x);
            vertices.push_back(rotation_center.y() + radius * y);
            vertices.push_back(rotation_center.z() + radius * z);

            normals.push_back(x);
            normals.push_back(y);
            normals.push_back(z);
        }
    }

    // 生成球体索引
    std::vector<unsigned int> indices;
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

    // 初始化OpenGL资源
    GLuint vao, vbo_position, vbo_normal, ebo;
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo_position);
    glGenBuffers(1, &vbo_normal);
    glGenBuffers(1, &ebo);

    // 设置VAO和各种缓冲
    glBindVertexArray(vao);

    glBindBuffer(GL_ARRAY_BUFFER, vbo_position);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, vbo_normal);
    glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(float), normals.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    // 保存当前OpenGL状态
    GLboolean depth_test, depth_mask, blend_enabled;
    GLint old_depth_func;
    glGetBooleanv(GL_DEPTH_TEST, &depth_test);
    glGetBooleanv(GL_DEPTH_WRITEMASK, &depth_mask);
    glGetIntegerv(GL_DEPTH_FUNC, &old_depth_func);
    glGetBooleanv(GL_BLEND, &blend_enabled);

    // 设置着色器和渲染球体
    center_shader_.bind();
    center_shader_.setUniformValue("projection", projection_matrix);
    center_shader_.setUniformValue("view", view_matrix);
    center_shader_.setUniformValue("model", model_matrix);
    center_shader_.setUniformValue("viewPos", camera_pos);
    center_shader_.setUniformValue("lightDir", QVector3D(1.0f, 1.0f, 1.0f).normalized());

    // 渲染策略：
    // 1. 首先渲染一个稍大的半透明外层，以创建"发光"效果
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_DEPTH_TEST); // 禁用深度测试确保外层始终可见

    // 创建外层顶点
    std::vector<float> outer_vertices;
    float outer_radius = radius * 1.2f;

    for (int i = 0; i <= stacks; ++i) {
        float phi = M_PI * float(i) / float(stacks);
        float sin_phi = sin(phi);
        float cos_phi = cos(phi);

        for (int j = 0; j <= slices; ++j) {
            float theta = 2.0f * M_PI * float(j) / float(slices);
            float sin_theta = sin(theta);
            float cos_theta = cos(theta);

            float x = cos_theta * sin_phi;
            float y = sin_theta * sin_phi;
            float z = cos_phi;

            outer_vertices.push_back(rotation_center.x() + outer_radius * x);
            outer_vertices.push_back(rotation_center.y() + outer_radius * y);
            outer_vertices.push_back(rotation_center.z() + outer_radius * z);
        }
    }

    glBindBuffer(GL_ARRAY_BUFFER, vbo_position);
    glBufferData(GL_ARRAY_BUFFER, outer_vertices.size() * sizeof(float), outer_vertices.data(), GL_STATIC_DRAW);

    center_shader_.setUniformValue("alpha", 0.2f); // 非常透明的外层
    glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);

    // 2. 然后渲染主体球体，启用深度测试但保持部分透明
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS); // 标准深度测试

    glBindBuffer(GL_ARRAY_BUFFER, vbo_position);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

    center_shader_.setUniformValue("alpha", 0.7f); // 部分透明
    glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);

    // 3. 最后渲染一个不透明的内核，增强中心亮度
    std::vector<float> inner_vertices;
    float inner_radius = radius * 0.7f;

    for (int i = 0; i <= stacks; ++i) {
        float phi = M_PI * float(i) / float(stacks);
        float sin_phi = sin(phi);
        float cos_phi = cos(phi);

        for (int j = 0; j <= slices; ++j) {
            float theta = 2.0f * M_PI * float(j) / float(slices);
            float sin_theta = sin(theta);
            float cos_theta = cos(theta);

            float x = cos_theta * sin_phi;
            float y = sin_theta * sin_phi;
            float z = cos_phi;

            inner_vertices.push_back(rotation_center.x() + inner_radius * x);
            inner_vertices.push_back(rotation_center.y() + inner_radius * y);
            inner_vertices.push_back(rotation_center.z() + inner_radius * z);
        }
    }

    glBindBuffer(GL_ARRAY_BUFFER, vbo_position);
    glBufferData(GL_ARRAY_BUFFER, inner_vertices.size() * sizeof(float), inner_vertices.data(), GL_STATIC_DRAW);

    center_shader_.setUniformValue("alpha", 1.0f); // 不透明
    glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);

    // 恢复OpenGL状态
    if (!blend_enabled) {
        glDisable(GL_BLEND);
    }

    glDepthFunc(old_depth_func);

    if (!depth_test) {
        glDisable(GL_DEPTH_TEST);
    }

    glDepthMask(depth_mask);

    // 清理资源
    glBindVertexArray(0);
    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(1, &vbo_position);
    glDeleteBuffers(1, &vbo_normal);
    glDeleteBuffers(1, &ebo);
}

void RotationIndicator::DrawDirectionIndicators(
    const QVector3D& rotation_center,
    const QMatrix4x4& projection_matrix,
    const QMatrix4x4& view_matrix,
    const QMatrix4x4& model_matrix,
    const QVector3D& camera_pos,
    float radius,
    float tube_radius)
{
    // 准备顶点数据
    std::vector<float> vertices;
    std::vector<float> colors;
    std::vector<float> normals; // 2D 模式下不需要法线，但为了兼容 RenderGeometry 接口保留

    // 生成三个轴的环状结构 (2D 圆圈) - Morandi Palette
    // X轴环(YZ平面) - 柔和红
    GenerateRing(vertices, colors, normals, rotation_center, radius, tube_radius, 120, 0, QVector3D(0.8f, 0.4f, 0.4f)); 
    // Y轴环(XZ平面) - 柔和绿
    GenerateRing(vertices, colors, normals, rotation_center, radius, tube_radius, 120, 1, QVector3D(0.4f, 0.7f, 0.4f)); 
    // Z轴环(XY平面) - 柔和蓝
    GenerateRing(vertices, colors, normals, rotation_center, radius, tube_radius, 120, 2, QVector3D(0.4f, 0.5f, 0.8f)); 

    // 生成三个坐标轴线段 (替代圆柱体) - Morandi Palette
    float axis_length = radius * 1.3f;
    // tube_radius在此处被复用为start_offset (避开中心球的距离)
    GenerateAxis(vertices, colors, normals, rotation_center, tube_radius, axis_length, 0, 0, QVector3D(0.8f, 0.4f, 0.4f)); // X轴
    GenerateAxis(vertices, colors, normals, rotation_center, tube_radius, axis_length, 0, 1, QVector3D(0.4f, 0.7f, 0.4f)); // Y轴
    GenerateAxis(vertices, colors, normals, rotation_center, tube_radius, axis_length, 0, 2, QVector3D(0.4f, 0.5f, 0.8f)); // Z轴

    // 渲染几何体
    RenderGeometry(
        vertices,
        colors,
        normals,
        projection_matrix,
        view_matrix,
        model_matrix,
        camera_pos,
        indicator_shader_
    );
}

void RotationIndicator::InitializeCenterShader()
{
    if (!center_shader_initialized_) {
        // 使用标准的3D光照顶点着色器
        center_shader_.addShaderFromSourceCode(
            QOpenGLShader::Vertex,
            "#version 330 core\n"
            "layout (location = 0) in vec3 aPos;\n"
            "layout (location = 1) in vec3 aNormal;\n"
            "uniform mat4 projection;\n"
            "uniform mat4 view;\n"
            "uniform mat4 model;\n"
            "out vec3 FragPos;\n"
            "out vec3 Normal;\n"
            "void main() {\n"
            "    FragPos = vec3(model * vec4(aPos, 1.0));\n"
            "    Normal = mat3(transpose(inverse(model))) * aNormal;\n"
            "    gl_Position = projection * view * model * vec4(aPos, 1.0);\n"
            "}\n"
        );

        // 片段着色器 - 使用Phong光照模型创建立体感
        center_shader_.addShaderFromSourceCode(
            QOpenGLShader::Fragment,
            "#version 330 core\n"
            "out vec4 FragColor;\n"
            "in vec3 FragPos;\n"
            "in vec3 Normal;\n"
            "uniform vec3 viewPos;\n"      // 相机位置
            "uniform vec3 lightDir;\n"     // 光源方向
            "uniform float alpha = 1.0;\n" // 透明度
            "void main() {\n"
            "    // 基础颜色（黄色）\n"
            "    vec3 baseColor = vec3(1.0, 1.0, 0.0);\n"
            "    // 环境光\n"
            "    float ambientStrength = 0.3;\n"
            "    vec3 ambient = ambientStrength * baseColor;\n"
            "    // 漫反射光\n"
            "    vec3 norm = normalize(Normal);\n"
            "    vec3 lightDirection = normalize(lightDir);\n"
            "    float diff = max(dot(norm, lightDirection), 0.0);\n"
            "    vec3 diffuse = diff * baseColor;\n"
            "    // 高光\n"
            "    float specularStrength = 0.7;\n"
            "    vec3 viewDir = normalize(viewPos - FragPos);\n"
            "    vec3 reflectDir = reflect(-lightDirection, norm);\n"
            "    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);\n"
            "    vec3 specular = specularStrength * spec * vec3(1.0);\n"
            "    // 最终颜色\n"
            "    vec3 result = ambient + diffuse + specular;\n"
            "    FragColor = vec4(result, alpha);\n"
            "}\n"
        );

        bool success = center_shader_.link();
        if (success) {
            center_shader_initialized_ = true;
            LOG_DEBUG("Center sphere shader initialized");
        }
        else {
            LOG_DEBUG("Failed to link center sphere shader: {}", center_shader_.log().toStdString());
        }
    }
}

void RotationIndicator::InitializeIndicatorShader()
{
    if (!indicator_shader_initialized_) {
        // 顶点着色器
        indicator_shader_.addShaderFromSourceCode(
            QOpenGLShader::Vertex,
            "#version 330 core\n"
            "layout (location = 0) in vec3 aPos;\n"
            "layout (location = 1) in vec3 aColor;\n"
            // "layout (location = 2) in vec3 aNormal;\n" // 2D 模式不需要法线
            "uniform mat4 projection;\n"
            "uniform mat4 view;\n"
            "uniform mat4 model;\n"
            "out vec3 Color;\n"
            "void main() {\n"
            "    Color = aColor;\n"
            "    gl_Position = projection * view * model * vec4(aPos, 1.0);\n"
            "}\n"
        );

        // 片段着色器 - 简单的纯色输出，无光照计算
        indicator_shader_.addShaderFromSourceCode(
            QOpenGLShader::Fragment,
            "#version 330 core\n"
            "in vec3 Color;\n"
            "out vec4 FragColor;\n"
            "void main() {\n"
            "    FragColor = vec4(Color, 1.0);\n"
            "}\n"
        );

        bool success = indicator_shader_.link();
        if (success) {
            indicator_shader_initialized_ = true;
            LOG_DEBUG("Direction indicator shader initialized");
        }
        else {
            LOG_DEBUG("Failed to link direction indicator shader: {}", indicator_shader_.log().toStdString());
        }
    }
}

void RotationIndicator::GenerateRing(
    std::vector<float>& vertices,
    std::vector<float>& colors,
    std::vector<float>& normals,
    const QVector3D& center,
    float radius,
    float tube_radius,
    int segments,
    int axis_plane, // 0=YZ(X轴), 1=XZ(Y轴), 2=XY(Z轴)
    const QVector3D& color)
{
    // 2D 模式：生成线段环 (GL_LINES)
    for (int i = 0; i < segments; i++) {
        float angle1 = 2.0f * M_PI * i / segments;
        float angle2 = 2.0f * M_PI * ((i + 1) % segments) / segments;

        QVector3D p1, p2;

        switch (axis_plane) {
        case 0: // YZ plane
            p1 = QVector3D(center.x(), center.y() + radius * cos(angle1), center.z() + radius * sin(angle1));
            p2 = QVector3D(center.x(), center.y() + radius * cos(angle2), center.z() + radius * sin(angle2));
            break;
        case 1: // XZ plane
            p1 = QVector3D(center.x() + radius * cos(angle1), center.y(), center.z() + radius * sin(angle1));
            p2 = QVector3D(center.x() + radius * cos(angle2), center.y(), center.z() + radius * sin(angle2));
            break;
        case 2: // XY plane
            p1 = QVector3D(center.x() + radius * cos(angle1), center.y() + radius * sin(angle1), center.z());
            p2 = QVector3D(center.x() + radius * cos(angle2), center.y() + radius * sin(angle2), center.z());
            break;
        }

        // 添加线段的两个端点
        vertices.push_back(p1.x()); vertices.push_back(p1.y()); vertices.push_back(p1.z());
        colors.push_back(color.x()); colors.push_back(color.y()); colors.push_back(color.z());
        normals.push_back(0); normals.push_back(0); normals.push_back(0); // 占位

        vertices.push_back(p2.x()); vertices.push_back(p2.y()); vertices.push_back(p2.z());
        colors.push_back(color.x()); colors.push_back(color.y()); colors.push_back(color.z());
        normals.push_back(0); normals.push_back(0); normals.push_back(0); // 占位
    }
}

void RotationIndicator::GenerateAxis(
    std::vector<float>& vertices,
    std::vector<float>& colors,
    std::vector<float>& normals,
    const QVector3D& center,
    float radius, // 此参数在2D模式下被复用为 start_offset
    float length,
    int segments,
    int axis, // 0=X轴, 1=Y轴, 2=Z轴
    const QVector3D& color)
{
    // 2D 模式：生成轴线和箭头 (GL_LINES)
    float start_offset = radius; // 复用参数
    
    QVector3D direction;
    QVector3D u, v; // 用于构建箭头的正交向量

    if (axis == 0) { // X轴
        direction = QVector3D(1, 0, 0);
        u = QVector3D(0, 1, 0);
        v = QVector3D(0, 0, 1);
    }
    else if (axis == 1) { // Y轴
        direction = QVector3D(0, 1, 0);
        u = QVector3D(1, 0, 0);
        v = QVector3D(0, 0, 1);
    }
    else { // Z轴
        direction = QVector3D(0, 0, 1);
        u = QVector3D(1, 0, 0);
        v = QVector3D(0, 1, 0);
    }

    // 正向轴线 (从球体表面延伸到长度末端)
    QVector3D pos_start = center + direction * start_offset;
    QVector3D pos_end = center + direction * length;

    // 负向轴线 (对称)
    QVector3D neg_start = center - direction * start_offset;
    QVector3D neg_end = center - direction * length;

    // Lambda helper to add a line
    auto add_line = [&](const QVector3D& p1, const QVector3D& p2) {
        vertices.push_back(p1.x()); vertices.push_back(p1.y()); vertices.push_back(p1.z());
        colors.push_back(color.x()); colors.push_back(color.y()); colors.push_back(color.z());
        normals.push_back(0); normals.push_back(0); normals.push_back(0);

        vertices.push_back(p2.x()); vertices.push_back(p2.y()); vertices.push_back(p2.z());
        colors.push_back(color.x()); colors.push_back(color.y()); colors.push_back(color.z());
        normals.push_back(0); normals.push_back(0); normals.push_back(0);
    };

    // 添加轴线
    add_line(pos_start, pos_end);
    add_line(neg_start, neg_end);

    // 添加箭头 (仅在正向末端)
    // 缩小箭头尺寸
    float arrow_len = length * 0.1f;    // 长度从 0.15 -> 0.1
    float arrow_width = length * 0.03f; // 宽度从 0.05 -> 0.03
    
    QVector3D arrow_base_center = pos_end - direction * arrow_len;

    // 箭头的四个底部点 (十字形)
    QVector3D base1 = arrow_base_center + u * arrow_width;
    QVector3D base2 = arrow_base_center - u * arrow_width;
    QVector3D base3 = arrow_base_center + v * arrow_width;
    QVector3D base4 = arrow_base_center - v * arrow_width;

    // 从箭头尖端连向底部四个点
    add_line(pos_end, base1);
    add_line(pos_end, base2);
    add_line(pos_end, base3);
    add_line(pos_end, base4);
    
    // 连接底部点形成一个闭合的底座
    add_line(base1, base3);
    add_line(base3, base2);
    add_line(base2, base4);
    add_line(base4, base1);

    // 增加底部对角线，使其看起来更"实心"一点
    add_line(base1, base2);
    add_line(base3, base4);
}

void RotationIndicator::AddTriangle(
    std::vector<float>& vertices,
    std::vector<float>& colors,
    std::vector<float>& normals,
    const QVector3D& p1,
    const QVector3D& p2,
    const QVector3D& p3,
    const QVector3D& color,
    const QVector3D& n1,
    const QVector3D& n2,
    const QVector3D& n3)
{
    // 2D 模式不再使用三角形绘制
}

void RotationIndicator::RenderGeometry(
    const std::vector<float>& vertices,
    const std::vector<float>& colors,
    const std::vector<float>& normals,
    const QMatrix4x4& projection_matrix,
    const QMatrix4x4& view_matrix,
    const QMatrix4x4& model_matrix,
    const QVector3D& camera_pos,
    QOpenGLShaderProgram& shader)
{
    // 创建VAO和VBO
    GLuint indicator_vao, indicator_vbo, indicator_color_vbo;
    glGenVertexArrays(1, &indicator_vao);
    glGenBuffers(1, &indicator_vbo);
    glGenBuffers(1, &indicator_color_vbo);

    glBindVertexArray(indicator_vao);

    // 位置属性
    glBindBuffer(GL_ARRAY_BUFFER, indicator_vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

    // 颜色属性
    glBindBuffer(GL_ARRAY_BUFFER, indicator_color_vbo);
    glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(float), colors.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

    // 2D 模式不需要法线属性

    // 保存当前OpenGL状态
    GLboolean depth_test, blend_enabled;
    GLint old_depth_func;
    glGetBooleanv(GL_DEPTH_TEST, &depth_test);
    glGetIntegerv(GL_DEPTH_FUNC, &old_depth_func);
    glGetBooleanv(GL_BLEND, &blend_enabled);

    // 设置渲染状态
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_DEPTH_TEST); // 2D 指示器通常不需要深度测试，确保它始终在最上层显示
    // glDepthFunc(GL_LEQUAL); 

    // 使用指示器着色器
    shader.bind();
    shader.setUniformValue("projection", projection_matrix);
    shader.setUniformValue("view", view_matrix);
    shader.setUniformValue("model", model_matrix);
    // shader.setUniformValue("viewPos", camera_pos); // 2D Shader 不需要相机位置

    // 设置线宽
    glLineWidth(2.0f); 

    // 绘制所有线段
    glDrawArrays(GL_LINES, 0, vertices.size() / 3);

    // 恢复OpenGL状态
    glLineWidth(1.0f); // 恢复默认线宽
    if (!blend_enabled) {
        glDisable(GL_BLEND);
    }
    glDepthFunc(old_depth_func);
    if (depth_test) {
        glEnable(GL_DEPTH_TEST);
    } else {
        glDisable(GL_DEPTH_TEST);
    }

    // 清理资源
    glBindVertexArray(0);
    glDeleteVertexArrays(1, &indicator_vao);
    glDeleteBuffers(1, &indicator_vbo);
    glDeleteBuffers(1, &indicator_color_vbo);
}
