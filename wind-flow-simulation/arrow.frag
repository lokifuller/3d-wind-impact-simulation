#version 330 core
in float Speed;
out vec4 FragColor;

// Define stops: blue, green, yellow, orange, red

vec3 colormap(float t) {
    if (t < 0.25) return mix(vec3(0,0,1), vec3(0,1,0), t/0.25);
    else if (t < 0.5) return mix(vec3(0,1,0), vec3(1,1,0), (t-0.25)/0.25);
    else if (t < 0.75) return mix(vec3(1,1,0), vec3(1,0.5,0), (t-0.5)/0.25);
    else return mix(vec3(1,0.5,0), vec3(1,0,0), (t-0.75)/0.25);
}

void main() {
    vec3 color = colormap(Speed);
    FragColor = vec4(color, 1.0);
}
