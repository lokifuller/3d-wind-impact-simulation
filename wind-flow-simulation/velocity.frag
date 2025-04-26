#version 330 core
in float vSpeed;
out vec4 FragColor;
void main(){
    vec3 color;
    if(vSpeed<0.5){
        float t=vSpeed/0.5;
        color=mix(vec3(0,1,0),vec3(1,1,0),t);
    } else {
        float t=(vSpeed-0.5)/0.5;
        color=mix(vec3(1,1,0),vec3(1,0,0),t);
    }
    FragColor=vec4(color,1.0);
}