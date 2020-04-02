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
        bool edge_collision(const glm::vec3 &pos, const glm::vec3 ray_direction, const mesh::polyhedron_flt &poly) {
            // 1) Calculate angle between the ray and an arbitrary vector used for a projection of the later points
            const glm::vec3 ref = glm::vec3(0,0,1);
            const glm::vec3 axis = glm::cross(ref, ray_direction);
            const float angle = glm::orientedAngle(ray_direction, ref, axis);
            const glm::mat3 rotation = std::abs(angle) > precision_bias ? (glm::mat3)glm::rotate(angle, axis) : glm::mat3(1);

            // 2) go over all edges
            for(const auto &e : poly._edges) {
                const glm::vec3 &e1 = poly._vertices._vertex_arr.at(e.first._id1);
                const glm::vec3 &e2 = poly._vertices._vertex_arr.at(e.first._id2);

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

        std::set<float> ray_intersections_safe(const glm::vec3 &pos, const glm::vec3 &dir, const mesh::polyhedron_flt &poly) {
            const auto &vertex_arr =  poly._vertices._vertex_arr;
            const auto &index_buf = poly._indices._buffer;
            const size_t faces = poly._indices._buffer.size() / poly._indices._stride;

            std::set<float> inters_dist;
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
                    vertex_arr.at(vid1),
                    vertex_arr.at(vid2),
                    vertex_arr.at(vid3),
                    bary_pos,
                    distance
                );

                //! BUG in glm 9.9
                // the intersectRayTriangle behaves like a line intersection,
                // always returning two opposing intersecting faces
                const bool positive = distance > 0 ? true : false;
                if(is_inters && positive) {
                    inters_dist.insert(distance);
                }
            }
            return inters_dist;
        }

        float sign (const glm::vec2 &p1, const glm::vec2 &p2, const glm::vec2 &p3) {
            return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
        }

        bool pt_in_triangle (const glm::vec2 &pt, const glm::vec2 &v1, const glm::vec2 &v2, const glm::vec2 &v3) {
            float d1 = sign(pt, v1, v2);
            float d2 = sign(pt, v2, v3);
            float d3 = sign(pt, v3, v1);
            bool has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
            bool has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);
            return !(has_neg && has_pos);
        }

        float calc_z(const glm::vec3 &pt, const glm::vec3 &v1, const glm::vec3 &v2, const glm::vec3 &v3) {
            const glm::vec3 e1 = glm::normalize(v2 - v1);
            const glm::vec3 e2 = glm::normalize(v3 - v1);

            // plane equation is: ax + by + cz = k
            // a,b,c is n.x, n.y, and n.z
            const glm::vec3 n = glm::cross(e1, e2);
            const float k = v1.x*n.x + v1.y*n.y + v1.z*n.z;

            // plane equation solved for z
            float on_plane = (k - n.y*pt.y - n.z*pt.z) / n.x;
            return on_plane - pt.x;
        }

        enum direction {
            x_dir,
            y_dir,
            z_dir
        };

        std::set<float> ray_intersections_fast(const glm::vec3 &pos, const mesh::polyhedron_flt &poly, const direction dir) {
            const auto &vertex_arr =  poly._vertices._vertex_arr;
            const auto &index_buf = poly._indices._buffer;
            const size_t faces = poly._indices._buffer.size() / poly._indices._stride;

            std::set<float> inters_dist;
            for(size_t face_id = 0; face_id < faces; face_id++) {
                const glm::ivec3 id = glm::ivec3(face_id) * poly._indices._stride + glm::ivec3(0,1,2);

                const size_t vid1 = index_buf.at(id.x);
                const size_t vid2 = index_buf.at(id.y);
                const size_t vid3 = index_buf.at(id.z);

                const glm::vec3 &v1 = vertex_arr.at(vid1);
                const glm::vec3 &v2 = vertex_arr.at(vid2);
                const glm::vec3 &v3 = vertex_arr.at(vid3);

                glm::vec2 bary_pos(0);
                float distance = 0;
                switch(dir) {
                case x_dir:
                    if(!pt_in_triangle(pos.yz(), v1.yz(), v2.yz(), v3.yz())) break;
                    glm::intersectRayTriangle(pos, {1,0,0}, v1, v2, v3, bary_pos, distance);
                    inters_dist.insert(distance);
                    break;
                case y_dir:
                    if(!pt_in_triangle(pos.xz(), v1.xz(), v2.xz(), v3.xz())) break;
                    glm::intersectRayTriangle(pos, {0,1,0}, v1, v2, v3, bary_pos, distance);
                    inters_dist.insert(distance);
                    break;
                case z_dir:
                    if(!pt_in_triangle(pos.xy(), v1.xy(), v2.xy(), v3.xy())) break;
                    glm::intersectRayTriangle(pos, {0,0,1}, v1, v2, v3, bary_pos, distance);
                    inters_dist.insert(distance);
                    break;
                default:
                    break;
                }
            }
            return inters_dist;
        }

        void fill_x(
             const glm::vec3 &pos,
             const glm::ivec3 &voxel_pos,
             const glm::vec3 &voxel_size,
             const mesh::polyhedron_flt &poly,
             const glm::ivec3 &dim,
             std::vector<int8_t> &buffer
            )
        {
            std::set<float> intersections = ray_intersections_fast(pos, poly, x_dir);
            bool is_in = intersections.size() % 2 == 1 ? true : false;
            int from = voxel_pos.x;
            for(float inters : intersections) {
                const int to = inters / voxel_size.x + voxel_pos.x;
                for(int x = from; x < to; x++) {
                    const size_t id = x * dim.y * dim.z + voxel_pos.y * dim.z + voxel_pos.z;
                    buffer[id] = is_in;
                }
                is_in = !is_in;
                from = to;
            }
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
        bool is_shell(const glm::vec3 &pos, const mesh::polyhedron_flt &poly, const cfg::shape_settings &settings) {
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
                    vertex_arr.at(vid1),
                    vertex_arr.at(vid2),
                    vertex_arr.at(vid3)
                };
                if(face_in_hexahedron(face, pos, half_box_size)) return true;
            }

            return false;
        }
    };
    
    namespace intersection_2d {
        bool edge_cutting_rectangle(const std::array<glm::vec2, 3> &triangle, const glm::vec2 &pos, const glm::vec2 &box_hlf) {
            const std::array<glm::vec2, 3> edges = {
                triangle[2] - triangle[0],
                triangle[2] - triangle[1],
                triangle[1] - triangle[0]
            };
            
            for(const auto &e : edges) {
                // line equation
                const float m = e.y / e.x;
                const float n = e.y - m * e.x;
                // solved line equations for rectangle intersections
                const float px = (box_hlf.y - n) / m;
                const float nx = (-box_hlf.y - n) / m;
                const float py = box_hlf.x * m + n;
                const float ny = -box_hlf.x * m + n;
                
                int c = 0;
                if(std::abs(px) < box_hlf.x) 
                    c++;
                
                if(std::abs(nx) < box_hlf.x) 
                    c++;
                
                if(std::abs(py) < box_hlf.y) 
                    c++;
                    
                if(std::abs(ny) < box_hlf.y) 
                    c++;
                
                if(c < 2) return false;
            }
            return true;
        }
        
        bool face_in_hexahedron(const std::array<glm::vec3, 3> &face, const glm::vec3 pos, const glm::vec3 &box_hlf) {
            const std::array<glm::vec3, 3> f = {
                face[0]-pos,
                face[1]-pos,
                face[2]-pos
            };
            
            const glm::vec2 b_x = { box_hlf.y, box_hlf.z };            
            const std::array<glm::vec2, 3> f_x = {
                f[0].yz(),
                f[1].yz(),
                f[2].yz()
            };
            if(!edge_cutting_rectangle(f_x, { 0, 0 }, b_x)) return false;
            
            
            const glm::vec2 b_y = { box_hlf.x, box_hlf.z };
            const std::array<glm::vec2, 3> f_y = {
                f[0].xz(),
                f[1].xz(),
                f[2].xz()
            };
            if(!edge_cutting_rectangle(f_y, { 0, 0 }, b_y)) return false;
            
            
            const glm::vec2 b_z = { box_hlf.x, box_hlf.y };
            const std::array<glm::vec2, 3> f_z = {
                f[0].xy(),
                f[1].xy(),
                f[2].xy()
            };
            if(!edge_cutting_rectangle(f_z, { 0, 0 }, b_z)) return false;

            
            return true;
        }
    };
};

