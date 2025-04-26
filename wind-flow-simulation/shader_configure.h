#pragma once

#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

class Shader {
public:
    GLuint ID;

    // Constructor reads and builds the shader.
    Shader(const char* vert_path, const char* frag_path) {
        std::string vert_string;
        std::string frag_string;
        std::ifstream vert_stream(vert_path);
        std::ifstream frag_stream(frag_path);
        std::stringstream vss, fss;

        if (vert_stream.is_open()) {
            vss << vert_stream.rdbuf();
            vert_string = vss.str();
            std::cout << "File: " << vert_path << " opened successfully.\n\n";
        }
        else {
            std::cout << "ERROR! File: " << vert_path << " could not be opened.\n\n";
        }

        if (frag_stream.is_open()) {
            fss << frag_stream.rdbuf();
            frag_string = fss.str();
            std::cout << "File: " << frag_path << " opened successfully.\n\n";
        }
        else {
            std::cout << "ERROR! File: " << frag_path << " could not be opened.\n\n";
        }

        const char* vert_pointer = vert_string.c_str();
        const char* frag_pointer = frag_string.c_str();

        GLuint vert_shad, frag_shad;
        // Create and compile vertex shader.
        vert_shad = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vert_shad, 1, &vert_pointer, NULL);
        glCompileShader(vert_shad);
        Check_Shaders_Program(vert_shad, "vert_shader");

        // Create and compile fragment shader.
        frag_shad = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(frag_shad, 1, &frag_pointer, NULL);
        glCompileShader(frag_shad);
        Check_Shaders_Program(frag_shad, "frag_shader");

        // Link shaders.
        ID = glCreateProgram();
        glAttachShader(ID, vert_shad);
        glAttachShader(ID, frag_shad);
        glLinkProgram(ID);
        Check_Shaders_Program(ID, "shader_program");

        glDeleteShader(vert_shad);
        glDeleteShader(frag_shad);
    }

    // Activate the shader.
    void use() {
        glUseProgram(ID);
    }

    // Utility uniform functions.
    void setInt(const std::string& name, int value) const {
        glUniform1i(glGetUniformLocation(ID, name.c_str()), value);
    }

    void setMat4(const std::string& name, const glm::mat4& mat) const {
        glUniformMatrix4fv(glGetUniformLocation(ID, name.c_str()), 1, GL_FALSE, glm::value_ptr(mat));
    }

private:
    // Utility function for checking shader compilation/linking errors.
    void Check_Shaders_Program(GLuint type, const std::string& name) {
        int success;
        int error_log_size;
        char info_log[2000];

        if (name == "vert_shader" || name == "frag_shader") {
            glGetShaderiv(type, GL_COMPILE_STATUS, &success);
            if (!success) {
                glGetShaderInfoLog(type, 1024, &error_log_size, info_log);
                std::cout << "\n--- Shader Compilation Error (" << name << "):\n"
                    << info_log << "\nError Log Length: " << error_log_size << "\n\n";
            }
        }
        else {
            glGetProgramiv(type, GL_LINK_STATUS, &success);
            if (!success) {
                glGetProgramInfoLog(type, 1024, &error_log_size, info_log);
                std::cout << "\n--- Program Link Error (" << name << "):\n"
                    << info_log << "\nError Log Length: " << error_log_size << "\n";
            }
        }
    }
};
