#version 330 core
layout (location = 0) in vec3 aPos;      // Position
layout (location = 1) in vec3 aNormal;   // Normal
layout (location = 2) in vec2 aTexCoords; // Texture Coordinates

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

out vec3 FragPos;
out vec3 Normal;
out vec2 TexCoords;

void main()
{
    // Calculate fragment position in world space
    FragPos = vec3(model * vec4(aPos, 1.0));
    
    // Transform normal vectors
    Normal = mat3(transpose(inverse(model))) * aNormal;
    
    // Pass texture coordinates to fragment shader
    TexCoords = aTexCoords;
    
    // Calculate final position
    gl_Position = projection * view * model * vec4(aPos, 1.0);
}