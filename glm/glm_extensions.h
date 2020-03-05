#pragma once

#include <glm/glm.hpp>
#include <glm/gtx/closest_point.hpp>
#include <glm/gtx/intersect.hpp>
#include <glm/gtc/epsilon.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/projection.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <glm/gtx/transform.hpp>

#include <tuple>


struct compare_glm_vec3 {
    bool operator() (const glm::vec3& lhs, const glm::vec3& rhs) const {
        return std::make_tuple(lhs.x, lhs.y, lhs.z) < std::make_tuple(rhs.x, rhs.y, rhs.z);
    }
};
