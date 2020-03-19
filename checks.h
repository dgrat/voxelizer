#pragma once

#include "mesh/polyhedron.h"
#include <glm/glm.hpp>
#include <iostream>
#include "xml_config.h"


namespace checks {
    namespace raycast {
        bool is_in(const glm::vec3 &pos, const mesh::polyhedron &poly);
    }
    namespace intersection_3d {
        bool is_shell(const glm::vec3 &pos, const mesh::polyhedron &poly, const cfg::shape_settings &settings);
        bool face_in_hexahedron(const std::array<glm::vec3, 3> &face, const glm::vec3 voxel_center, const glm::vec3 &half_box_size);
    };
    namespace intersection_2d {
    };
};
