#pragma once

#include "mesh/polyhedron.h"
#include <glm/glm.hpp>
#include <iostream>
#include "xml_config.h"

struct intersection_t {
    float _distance = 0.f;
    bool _valid = true;
};

namespace checks {
    namespace raycast {
        bool is_in(const glm::vec3 &pos, const mesh::polyhedron_flt &poly);
        intersection_t is_intersecting(const glm::vec3 &pos, const glm::vec3 &dir, const std::array<glm::vec3, 3> &face);
        void fill_x(
             const glm::vec3 &pos,
             const glm::ivec3 &voxel_pos,
             const glm::vec3 &voxel_size,
             const mesh::polyhedron_flt &poly,
             const glm::ivec3 &dim,
             std::vector<int8_t> &buffer
            );
    }
    namespace intersection_3d {
        bool is_shell(const glm::vec3 &pos, const mesh::polyhedron_flt &poly, const cfg::shape_settings &settings);
        bool face_in_hexahedron(const std::array<glm::vec3, 3> &face, const glm::vec3 pos, const glm::vec3 &box_hlf);
    };
    namespace intersection_2d {
        bool face_in_hexahedron(const std::array<glm::vec3, 3> &face, const glm::vec3 pos, const glm::vec3 &box_hlf);
    };
};
