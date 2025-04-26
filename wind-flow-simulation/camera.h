#ifndef CAMERA_H
#define CAMERA_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <GLFW/glfw3.h>

class Camera {
public:
    glm::vec3 Position;
    glm::vec3 Front;
    glm::vec3 Up;

    // Euler angles.
    float Yaw;
    float Pitch;

    // For mouse input.
    float LastX;
    float LastY;
    bool FirstMouse;

    // Constructor.
    Camera(glm::vec3 position, glm::vec3 up, float yaw, float pitch);

    // Returns the view matrix.
    glm::mat4 GetViewMatrix() const;

    // Processes keyboard input.
    void ProcessKeyboard(GLFWwindow* window, float deltaTime, float groundLevel);

    // Processes mouse movement.
    void ProcessMouseMovement(float xpos, float ypos);

private:

    void updateCameraVectors();
};

#endif // CAMERA_H