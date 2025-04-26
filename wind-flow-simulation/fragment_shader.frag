#version 330 core

// Output color of the fragment
out vec4 FragColor;

// Input from vertex shader
in vec3 FragPos;
in vec3 Normal;
in vec2 TexCoords;

// Uniforms
uniform sampler2D texture_diffuse1;  // Diffuse texture
uniform vec3 lightPos = vec3(0.0, 180.0, 300.0);  // Light position
uniform vec3 viewPos = vec3(0.0, 90.0, 400.0);    // Camera position
uniform vec3 lightColor = vec3(1.0, 1.0, 1.0);   // Light color
uniform vec3 objectColor = vec3(0.7, 0.7, 0.7);   // Object color

void main()
{
    // Ambient lighting
    float ambientStrength = 0.3;
    vec3 ambient = ambientStrength * lightColor;

    // Diffuse lighting
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(lightPos - FragPos);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * lightColor;

    // Specular lighting
    float specularStrength = 0.5;
    vec3 viewDir = normalize(viewPos - FragPos);
    vec3 reflectDir = reflect(-lightDir, norm);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 64);
    vec3 specular = specularStrength * spec * lightColor;

    // Combine ambient + diffuse + specular
    vec3 finalColor = (ambient + diffuse + specular) * objectColor;

    // Final color
    FragColor = vec4(finalColor, 1.0);
}
