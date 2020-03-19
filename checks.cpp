#include "checks.h"
#include <chrono>
#include "glm_ext/glm_extensions.h"


namespace checks {
    constexpr float precision_bias = 100*FLT_EPSILON;
    
    struct plane_t {
        glm::vec3 normal;
        float d;
    };
    
    
    namespace raycast {
        template <typename ... VECS>
        bool point_collision(const glm::vec3 &ray_start, const glm::vec3 ray_direction, const float distance,
                                const glm::vec3 &v1, const glm::vec3 &v2, const glm::vec3 &v3) {
            const glm::vec3 ray_trg = ray_start + (ray_direction * distance);
            for(const auto &v : {v1,v2,v3}) {
                if(glm::length(ray_trg - v) < precision_bias) {
                    return true;
                }
            }
            return false;
        }

        // check whether and where a ray crosses an edge from a face
        bool edge_collision(const glm::vec3 &pos, const glm::vec3 ray_direction, const mesh::polyhedron &poly) {
            // 1) Calculate angle between the ray and an arbitrary vector used for a projection of the later points
            const glm::vec3 ref = glm::vec3(0,0,1);
            const glm::vec3 axis = glm::cross(ref, ray_direction);
            const float angle = glm::orientedAngle(ray_direction, ref, axis);
            const glm::mat3 rotation = std::abs(angle) > precision_bias ? (glm::mat3)glm::rotate(angle, axis) : glm::mat3(1);

            // 2) go over all edges
            for(const auto &e : poly._edges) {
                const glm::vec3 &e1 = poly._vertices._vertex_arr.at(e.first._id1)._position;
                const glm::vec3 &e2 = poly._vertices._vertex_arr.at(e.first._id2)._position;

                // 3) rotate all points
                const glm::vec2 rot_e1 = rotation * e1;
                const glm::vec2 rot_e2 = rotation * e2;
                const glm::vec2 p = rotation * pos;

                // 5) check whether the ray is close to the edge
                const glm::vec2 pt_on_line = glm::closestPointOnLine(p, rot_e1, rot_e2);
                if(glm::length(pt_on_line - p) < precision_bias) {
                    return true;
                }
            }
            return false;
        }

        std::vector<float> check_intersections(const glm::vec3 &pos, const glm::vec3 &dir, const mesh::polyhedron &poly) {
            const auto &vertex_arr =  poly._vertices._vertex_arr;
            const auto &index_buf = poly._indices._buffer;
            const size_t faces = poly._indices._buffer.size() / poly._indices._stride;

            std::vector<float> inters_dist;
            for(size_t face_id = 0; face_id < faces; face_id++) {
                const glm::ivec3 id = glm::ivec3(face_id) * poly._indices._stride + glm::ivec3(0,1,2);

                const size_t vid1 = index_buf.at(id.x);
                const size_t vid2 = index_buf.at(id.y);
                const size_t vid3 = index_buf.at(id.z);

                glm::vec2 bary_pos(0);
                float distance = 0;
                bool is_inters = glm::intersectRayTriangle(
                    pos,
                    dir,
                    vertex_arr.at(vid1)._position,
                    vertex_arr.at(vid2)._position,
                    vertex_arr.at(vid3)._position,
                    bary_pos,
                    distance
                );

                //! BUG in glm 9.9
                // the intersectRayTriangle behaves like a line intersection,
                // always returning two opposing intersecting faces
                const bool positive = distance > 0 ? true : false;
                if(is_inters && positive) {
                    // try other ray if there is a point collision
                    if(point_collision(pos, dir, distance, vertex_arr.at(vid1)._position, vertex_arr.at(vid2)._position, vertex_arr.at(vid3)._position)) {
                        continue;
                    }
                    inters_dist.push_back(distance);
                }
            }
            return inters_dist;
        }

        bool is_in(const glm::vec3 &pos, const mesh::polyhedron &poly) {
            const float x = ((float) rand() / (RAND_MAX));
            const float y = ((float) rand() / (RAND_MAX));
            const float z = ((float) rand() / (RAND_MAX));
            const std::vector<glm::vec3> dirs = {
                glm::normalize(glm::vec3(x,y,z)),
                { 1,0,0 },
                { 0,1,0 },
                { 0,0,1 },
                { -1,0,0 },
                { 0,-1,0 },
                { 0,0,-1 },
                glm::normalize(glm::vec3(x,y,0)),
                glm::normalize(glm::vec3(x,0,z)),
                glm::normalize(glm::vec3(0,y,z)),
                glm::normalize(glm::vec3(-x,-y,0)),
                glm::normalize(glm::vec3(-x,0,-z)),
                glm::normalize(glm::vec3(0,-y,-z)),
            };

            for(const auto &dir : dirs) {
                if(edge_collision(pos, dir, poly)) continue;
                const int cur_inters = check_intersections(pos, dir, poly).size();
                if(cur_inters % 2 == 0) return false;
                if(cur_inters % 2 == 1) return true;
            }
            return false;
        }
    };
        
    namespace intersection_3d {
        bool axis_test_x02(const glm::vec3 &e, const glm::vec3 &voxel_center, const std::array<glm::vec3, 3> &face, const glm::vec3 &half_box_size) {
            auto v0 = face[0] - voxel_center;
            auto v2 = face[2] - voxel_center;

            float fa = std::abs(e.z);
            float fb = std::abs(e.y);

            float p1 = e.z * v0.y - e.y * v0.z;
            float p3 = e.z * v2.y - e.y * v2.z;

            float min = p1 > p3 ? p3 : p1;
            float max = p1 > p3 ? p1 : p3;

            float rad = fa * half_box_size.y + fb * half_box_size.z;
            if (min > rad || max < -rad) {
                return true;
            }
            return false;
        }
        bool axis_test_x01(const glm::vec3 &e, const glm::vec3 &voxel_center, const std::array<glm::vec3, 3> &face, const glm::vec3 &half_box_size) {
            auto v0 = face[0] - voxel_center;
            auto v1 = face[1] - voxel_center;

            float fa = std::abs(e.z);
            float fb = std::abs(e.y);

            float p1 = e.z * v0.y - e.y * v0.z;
            float p2 = e.z * v1.y - e.y * v1.z;

            float min = p1 > p2 ? p2 : p1;
            float max = p1 > p2 ? p1 : p2;

            float rad = (fa + fb) * half_box_size.z;
            if (min > rad || max < -rad) {
                return true;
            }
            return false;
        }

        bool axis_test_y02(const glm::vec3 &e, const glm::vec3 &voxel_center, const std::array<glm::vec3, 3> &face, const glm::vec3 &half_box_size) {
            auto v0 = face[0] - voxel_center;
            auto v2 = face[2] - voxel_center;

            float fa = std::abs(e.z);
            float fb = std::abs(e.x);

            float p1 = -e.z * v0.x + e.x * v0.z;
            float p2 = -e.z * v2.x + e.x * v2.z;

            float min = p1 > p2 ? p2 : p1;
            float max = p1 > p2 ? p1 : p2;

            float rad = (fa + fb) * half_box_size.z;
            if (min > rad || max < -rad) {
                return true;
            }
            return false;
        }
        bool axis_test_y01(const glm::vec3 &e, const glm::vec3 &voxel_center, const std::array<glm::vec3, 3> &face, const glm::vec3 &half_box_size) {
            auto v0 = face[0] - voxel_center;
            auto v1 = face[1] - voxel_center;

            float fa = std::abs(e.z);
            float fb = std::abs(e.x);

            float p1 = -e.z * v0.x + e.x * v0.z;
            float p2 = -e.z * v1.x + e.x * v1.z;

            float min = p1 > p2 ? p2 : p1;
            float max = p1 > p2 ? p1 : p2;

            float rad = (fa + fb) * half_box_size.z;
            if (min > rad || max < -rad) {
                return true;
            }
            return false;
        }

        bool axis_test_z12(const glm::vec3 &e, const glm::vec3 &voxel_center, const std::array<glm::vec3, 3> &face, const glm::vec3 &half_box_size) {
            auto v1 = face[1] - voxel_center;
            auto v2 = face[2] - voxel_center;

            float fa = std::abs(e.y);
            float fb = std::abs(e.x);

            float p1 = e.y * v1.x - e.x * v1.y;
            float p2 = e.y * v2.x - e.x * v2.y;

            float min = p1 > p2 ? p2 : p1;
            float max = p1 > p2 ? p1 : p2;

            float rad = (fa + fb) * half_box_size.z;
            if (min > rad || max < -rad) {
                return true;
            }
            return false;
        }
        bool axis_test_z01(const glm::vec3 &e, const glm::vec3 &voxel_center, const std::array<glm::vec3, 3> &face, const glm::vec3 &half_box_size) {
            auto v1 = face[0] - voxel_center;
            auto v2 = face[1] - voxel_center;

            float fa = std::abs(e.y);
            float fb = std::abs(e.x);

            float p1 = e.y * v1.x - e.x * v1.y;
            float p2 = e.y * v2.x - e.x * v2.y;

            float min = p1 > p2 ? p2 : p1;
            float max = p1 > p2 ? p1 : p2;

            float rad = (fa + fb) * half_box_size.z;
            if (min > rad || max < -rad) {
                return true;
            }
            return false;
        }

        bool allGreaterThan(const std::array<float, 3> &v, const float c) {
            for(auto &p : v) {
                if(p < c) return false;
            }
            return true;
        }

        bool allSmallerThan(const std::array<float, 3> &v, const float c) {
            for(auto &p : v) {
                if(p > c) return false;
            }
            return true;
        }

        bool plane_box_overlap(const plane_t &plane, const glm::vec3 &voxel_center, const glm::vec3 &halfboxsize) {
            glm::vec3 vmax = voxel_center;
            glm::vec3 vmin = voxel_center;
            for (int dim = 0; dim < 3; dim++) {
                if (plane.normal[dim] > 0) {
                    vmin[dim] += -halfboxsize[dim];
                    vmax[dim] += halfboxsize[dim];
                }
                else {
                    vmin[dim] += halfboxsize[dim];
                    vmax[dim] += -halfboxsize[dim];
                }
            }

            if (glm::dot(plane.normal, vmin) + plane.d > 0) {
                return false;
            }
            if (glm::dot(plane.normal, vmax) + plane.d >= 0) {
                return true;
            }
            return false;
        }

        //! return true if outside
        //! return false if maybe inside
        bool face_in_hexahedron(const std::array<glm::vec3, 3> &face, const glm::vec3 voxel_center, const glm::vec3 &half_box_size) {
            const glm::vec3 e1 = face[2] - face[1];
            const glm::vec3 e2 = face[0] - face[2];
            const glm::vec3 norm = glm::cross(e1, e2);
            if(!plane_box_overlap({norm, -glm::dot(norm, face[0])}, voxel_center, half_box_size)) {
                return false;
            }

            const glm::vec3 e0 = face[1] - face[0];
            if(axis_test_x02(e0, voxel_center, face, half_box_size)) return false;
            if(axis_test_x02(e1, voxel_center, face, half_box_size)) return false;
            if(axis_test_x01(e2, voxel_center, face, half_box_size)) return false;

            if(axis_test_y02(e0, voxel_center, face, half_box_size)) return false;
            if(axis_test_y02(e1, voxel_center, face, half_box_size)) return false;
            if(axis_test_y01(e2, voxel_center, face, half_box_size)) return false;

            if(axis_test_z12(e0, voxel_center, face, half_box_size)) return false;
            if(axis_test_z01(e1, voxel_center, face, half_box_size)) return false;
            if(axis_test_z12(e2, voxel_center, face, half_box_size)) return false;

            const auto max = voxel_center + half_box_size;
            if(allGreaterThan({face[0].x, face[1].x, face[2].x}, max.x)) return false;
            if(allGreaterThan({face[0].y, face[1].y, face[2].y}, max.y)) return false;
            if(allGreaterThan({face[0].z, face[1].z, face[2].z}, max.z)) return false;

            const auto min = voxel_center - half_box_size;
            if(allSmallerThan({face[0].x, face[1].x, face[2].x}, min.x)) return false;
            if(allSmallerThan({face[0].y, face[1].y, face[2].y}, min.y)) return false;
            if(allSmallerThan({face[0].z, face[1].z, face[2].z}, min.z)) return false;

            return true;
        }

        // for voxel shell calculation we invert the test
        // we do not check wheter a voxel is in the mesh, but whether a face is in the bbox of the voxel
        bool is_shell(const glm::vec3 &pos, const mesh::polyhedron &poly, const cfg::shape_settings &settings) {
            const auto &vertex_arr =  poly._vertices._vertex_arr;
            const auto &index_buf = poly._indices._buffer;
            const size_t faces = poly._indices._buffer.size() / poly._indices._stride;
            const glm::vec3 half_box_size = settings._voxel_size / 2.f;

            for(size_t face_id = 0; face_id < faces; face_id++) {
                // walk over faces
                const glm::ivec3 id = glm::ivec3(face_id) * poly._indices._stride + glm::ivec3(0,1,2);
                const size_t vid1 = index_buf.at(id.x);
                const size_t vid2 = index_buf.at(id.y);
                const size_t vid3 = index_buf.at(id.z);

                // face vertices
                const std::array<glm::vec3, 3> face = {
                    vertex_arr.at(vid1)._position,
                    vertex_arr.at(vid2)._position,
                    vertex_arr.at(vid3)._position
                };
                if(face_in_hexahedron(face, pos, half_box_size)) return true;
            }

            return false;
        }
    };
    
    namespace intersection_2d {
    };
};

