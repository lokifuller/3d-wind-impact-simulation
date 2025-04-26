#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in float aSpeed;
out float Speed;
uniform mat4 model, view, projection;
void main() {
    Speed = clamp(aSpeed, 0.0, 1.0);
    gl_Position = projection * view * model * vec4(aPos, 1.0);
}
