// camera.cpp
#include "camera.h"
#include <cmath>

Camera::Camera(glm::vec3 position, glm::vec3 up, float yaw, float pitch)
    : Position(position), Up(up), Yaw(yaw), Pitch(pitch), FirstMouse(true)
{
    updateCameraVectors();
    LastX = LastY = 0;
}

glm::mat4 Camera::GetViewMatrix() const {
    return glm::lookAt(Position, Position + Front, Up);
}

void Camera::ProcessKeyboard(GLFWwindow* window, float deltaTime, float groundLevel)
{
    float cameraSpeed = 1.0f * deltaTime;
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        Position += cameraSpeed * Front;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        Position -= cameraSpeed * Front;

    glm::vec3 cameraRight = glm::normalize(glm::cross(Front, Up));
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        Position -= cameraSpeed * cameraRight;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        Position += cameraSpeed * cameraRight;

    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
        Position.y += cameraSpeed;
    if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
        Position.y -= cameraSpeed;

    if (Position.y < groundLevel)
        Position.y = groundLevel;
}

void Camera::ProcessMouseMovement(float xpos, float ypos)
{
    if (FirstMouse)
    {
        LastX = xpos;
        LastY = ypos;
        FirstMouse = false;
    }

    float xoffset = xpos - LastX;
    float yoffset = LastY - ypos;
    LastX = xpos;
    LastY = ypos;

    float sensitivity = 0.1f;
    xoffset *= sensitivity;
    yoffset *= sensitivity;

    Yaw += xoffset;
    Pitch += yoffset;

    if (Pitch > 89.0f)  Pitch = 89.0f;
    if (Pitch < -89.0f) Pitch = -89.0f;

    glm::vec3 front;
    front.x = cos(glm::radians(Yaw)) * cos(glm::radians(Pitch));
    front.y = sin(glm::radians(Pitch));
    front.z = sin(glm::radians(Yaw)) * cos(glm::radians(Pitch));
    Front = glm::normalize(front);
}

void Camera::updateCameraVectors() {
    glm::vec3 front;
    front.x = cos(glm::radians(Yaw)) * cos(glm::radians(Pitch));
    front.y = sin(glm::radians(Pitch));
    front.z = sin(glm::radians(Yaw)) * cos(glm::radians(Pitch));
    Front = glm::normalize(front);
}
