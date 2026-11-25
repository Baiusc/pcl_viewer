#version 330 core

layout (location = 0) in vec4 aPos;
//layout (location = 1) in float aIntensity;
layout(location = 1) in vec3 aColor;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform float zoom;
uniform float basePointSize;

out vec3 ourColor;

void main()
{
    gl_Position = projection * view * model * vec4(aPos.x, aPos.y, aPos.z, 1.0);
    ourColor = aColor;
}