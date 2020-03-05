#pragma once

#include "mesh/polyhedron.h"
#include <glm/glm.hpp>
#include <iostream>
#include "xml_config.h"


namespace checks {
    namespace convex {
        //! if euler characteristic is 2 (convex)
        //! if all normals point away or all normals point towards the point,
        //! the point is certainly in (I guess, only for convex shapes true)
        bool is_in(const glm::vec3 &pos, const mesh::polyhedron &poly);
    };
    namespace nonconvex {


        inline bool axis_test_x01(const glm::vec2 &p1, const glm::vec2 &p2, const glm::vec2 &half_box_size) {
            glm::vec2 edge = p2 - p1;
            glm::vec2 abs_edge = glm::abs(edge);

            float d1 = edge.x * p1.y - edge.y * p1.x;
            float d2 = edge.x * p2.y - edge.y * p2.x;

            printf("p1: %f; p2: %f\n", d1, d2);

            float min = d1 < d2 ? d1 : d2;
            float max = d1 >= d2 ? d1 : d2;

            printf("min: %f; max: %f\n", min, max);

            float rad = abs_edge.x * half_box_size.x + abs_edge.y * half_box_size.y;
            bool res = min > rad || max < -rad ? false : true;
            printf("rad: %f %d\n", rad, res);

            return res;
        }

        namespace raycast {
            bool is_in(const glm::vec3 &pos, const mesh::polyhedron &poly);
            bool is_shell(const glm::vec3 &pos, const mesh::polyhedron &poly, const cfg::shape_settings &settings);
        };
    };
};

