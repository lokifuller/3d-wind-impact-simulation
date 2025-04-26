#include "ray_triangle.h"
#include <cmath>

bool rayTri(const glm::vec3& o,const glm::vec3& d,const Triangle& t,float& tt) 
{
    const float EPS = 1e-6f;

    glm::vec3 e1 = t.v1 - t.v0;
    glm::vec3 e2 = t.v2 - t.v0;

    glm::vec3 p = glm::cross(d, e2);
    float det = glm::dot(e1, p);
    if (std::fabs(det) < EPS) return false;
    float invDet = 1.0f / det;

    glm::vec3 tv = o - t.v0;
    float uPar = glm::dot(tv, p) * invDet;
    if (uPar < 0.0f || uPar > 1.0f) return false;

    glm::vec3 q = glm::cross(tv, e1);
    float vPar = glm::dot(d, q) * invDet;
    if (vPar < 0.0f || uPar + vPar > 1.0f) return false;

    tt = glm::dot(e2, q) * invDet;
    return (tt >= 0.0f);
}
