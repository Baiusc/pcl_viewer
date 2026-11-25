#ifndef GL_TRACE_H_
#define GL_TRACE_H_
#include <QOpenGLFunctions>

#ifdef _WIN32
#ifndef APIENTRY
#define APIENTRY __stdcall
#endif
#else
#define APIENTRY
#endif

// 修改宏定义，使用 GLTrace 类
#define glCheckError() ys_scene::GLTrace::glCheckError_(__FILE__, __LINE__)

namespace ys_scene {
    class GLTrace {
    protected:
        static QOpenGLFunctions* gl;
    public:
        static void init(QOpenGLFunctions* functions) { gl = functions; }
        static void APIENTRY glDebugOutput(GLenum source, GLenum type, unsigned int id,
            GLenum severity, GLsizei length, const char* message, const void* userParam);
        static int glCheckError_(const char* file, int line);
    };
}//namespace ys_scene 

#endif // GL_TRACE_H_