#pragma once

#include "glm_ext/glm_extensions.h"

#include <tuple>
#include <vector>
#include <array>
#include "stl/stl_import.h"
#include "mesh/polyhedron.h"
#include "checks.h"
#include "xml_config.h"

struct voxel_t {
    glm::vec3 _position;
    bool _is_in_mesh = false;

    friend bool operator< (const voxel_t& lhs, const voxel_t& rhs) {
        return std::make_tuple(lhs._position.x, lhs._position.y, lhs._position.z) < std::make_tuple(rhs._position.x, rhs._position.y, rhs._position.z);
    }
};

struct build_stl_cube {
    static std::vector<stl::face> mesh(const glm::vec3 &pos, const glm::vec3 cube_size) {
        auto get_corners = [=](const glm::vec3 &size) {
            std::array<glm::vec3, 8> arr = {
                glm::vec3(pos.x - size.x/2, pos.y - size.y/2, pos.z - size.z/2),
                glm::vec3(pos.x + size.x/2, pos.y - size.y/2, pos.z - size.z/2),
                glm::vec3(pos.x + size.x/2, pos.y + size.y/2, pos.z - size.z/2),
                glm::vec3(pos.x - size.x/2, pos.y + size.y/2, pos.z - size.z/2),
                glm::vec3(pos.x - size.x/2, pos.y - size.y/2, pos.z + size.z/2),
                glm::vec3(pos.x + size.x/2, pos.y - size.y/2, pos.z + size.z/2),
                glm::vec3(pos.x + size.x/2, pos.y + size.y/2, pos.z + size.z/2),
                glm::vec3(pos.x - size.x/2, pos.y + size.y/2, pos.z + size.z/2)
            };
            return arr;
        };

        auto c = get_corners(cube_size);
        return {
            { glm::vec3(0), c[1], c[0], c[3] },
            { glm::vec3(0), c[3], c[2], c[1] },

            { glm::vec3(0), c[5], c[1], c[2] },
            { glm::vec3(0), c[2], c[6], c[5] },

            { glm::vec3(0), c[5], c[6], c[7] },
            { glm::vec3(0), c[7], c[4], c[5] },

            { glm::vec3(0), c[3], c[0], c[4] },
            { glm::vec3(0), c[4], c[7], c[3] },

            { glm::vec3(0), c[6], c[2], c[3] },
            { glm::vec3(0), c[3], c[7], c[6] },

            { glm::vec3(0), c[0], c[1], c[5] },
            { glm::vec3(0), c[5], c[4], c[0] }
        };
    }
};

