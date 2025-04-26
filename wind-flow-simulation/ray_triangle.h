#ifndef RAY_TRIANGLE_H
#define RAY_TRIANGLE_H

#include <glm/glm.hpp>

/// A single triangle, defined by its three vertices.
struct Triangle {
    glm::vec3 v0, v1, v2;
};

/**
 * Ray–triangle intersection (Möller–Trumbore).
 * @param o   Ray origin.
 * @param d   Ray direction.
 * @param t   Triangle to test against.
 * @param tt  On hit, the distance along the ray to the intersection.
 * @return    True if the ray hits the triangle.
 */
bool rayTri(const glm::vec3& o, const glm::vec3& d, const Triangle& t, float& tt);

#endif