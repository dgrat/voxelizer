#pragma once

#include "glm_ext/glm_extensions.h"

#include "mesh/polyhedron.h"
#include "checks.h"
#include "xml_config.h"
#include "buffer.h"
#include "timer.h"
#include "enums.h"
#include "octree/tree.h"

#include <set>
#include <vector>

namespace rasterize {
    template<typename base_t>
    struct voxel_arr {
        glm::ivec3 _arr_dim;
        buffer3d<int8_t> _voxels;
        mesh::polyhedron<base_t> _mesh; // rescaled
        size_t _num_voxels;
    };

    template<typename vec_t>
    float area(const vec_t &v1, const vec_t &v2, const vec_t &v3) {
        float le1 = glm::length(v2-v1);
        float le2 = glm::length(v3-v1);
        float le3 = glm::length(v3-v2);
        float p = 0.5 * (le1 + le2 + le3);
        float area = std::sqrt(p * (p - le1) * (p - le2) * (p - le3));
        return area;
    }
    
    template<typename base_t>
    mesh::polyhedron<base_t> prepare_index_buffers(
        const mesh::polyhedron<base_t> &in_mesh, 
        const glm::ivec3 &in_scale,
        buffer3d<mesh::face<id_t>> &out_xy, 
        buffer3d<mesh::face<id_t>> &out_yz,
        buffer3d<mesh::face<id_t>> &out_xz
    ) 
    {           
        benchmark::timer tmp("prepare_index_buffers()");
            
        mesh::polyhedron<base_t> mesh;
        mesh = mesh::polyhedron<base_t>::normalized(in_mesh);
        mesh = mesh::polyhedron<base_t>::scaled(mesh, in_scale);
        const glm::ivec3 dim = glm::ceil(mesh.dim());

        auto &indices = mesh._indices;
        out_xy = buffer3d<mesh::face<id_t>>(dim.x, dim.y, 0);
        out_yz = buffer3d<mesh::face<id_t>>(dim.y, dim.z, 0);
        out_xz = buffer3d<mesh::face<id_t>>(dim.x, dim.z, 0);
        
        // run over target voxel coordinates 
        // and check whether a faces cuts a posiion in one of the planes
        size_t num_elems = 0;
        for(const mesh::face<id_t> &f : indices._buffer) {
            const auto &v1 = mesh._vertices[f[0]];
            const auto &v2 = mesh._vertices[f[1]];
            const auto &v3 = mesh._vertices[f[2]];

            constexpr float area_bias = 0.1;
            const float a_xz = area(v1.xz(), v2.xz(), v3.xz());
            const float a_xy = area(v1.xy(), v2.xy(), v3.xy());
            const float a_yz = area(v1.yz(), v2.yz(), v3.yz());
            
            // calc bbox around the face
            // we use the bbox to create a simple search buffer (for later rasterization)
            const glm::ivec3 min = glm::floor(glm::min(glm::min(v1, v2), v3));
            const glm::ivec3 max = glm::ceil(glm::max(glm::max(v1, v2), v3));

            // using the bbox: 
            // insert the face indices into the buffers
            // do this for each major plane..
            // xy
            if(a_xy > area_bias) {
                for(int x = min.x; x < max.x; x++)
                for(int y = min.y; y < max.y; y++) {
                    out_xy[x][y].push_back(f);
                    num_elems += 3;
                }
            }
            // yz
            if(a_yz > area_bias) {
                for(int y = min.y; y < max.y; y++)
                for(int z = min.z; z < max.z; z++) {
                    out_yz[y][z].push_back(f);
                    num_elems += 3;
                }
            }
            // xz
            if(a_xz > area_bias) {
                for(int x = min.x; x < max.x; x++)
                for(int z = min.z; z < max.z; z++) {                 
                    out_xz[x][z].push_back(f);
                    num_elems += 3;
                }
            }
        }
        float s_mb = (float)((num_elems * sizeof(id_t)) / std::pow(1024, 2));
        std::cout << "search buffers are " << (size_t)s_mb << " MBytes" << std::endl;
        return mesh;
    }
    
    //! generates a voxel mesh from an arbitrary polyhedron
    //! is very fast but memory consumption does not scale well
    template<typename base_t>
    class all_fast {
        using id_t = typename mesh::polyhedron<base_t>::index_t;
        
        mesh::polyhedron<base_t> _polyhedron;
        buffer3d<mesh::face<id_t>> _xy_plane_buffer;
        buffer3d<mesh::face<id_t>> _yz_plane_buffer;
        buffer3d<mesh::face<id_t>> _xz_plane_buffer;
        
    public: 
        all_fast(const mesh::polyhedron<base_t> &poly, glm::ivec3 dim) {
            _polyhedron = prepare_index_buffers(poly, dim, _xy_plane_buffer, _yz_plane_buffer, _xz_plane_buffer);
        }
        
        voxel_arr<base_t> rasterize() const {
            const glm::ivec3 dim = glm::ceil(_polyhedron.dim());
            voxel_arr<base_t> res = { dim, buffer3d<int8_t>(dim.x, dim.y, dim.z, 0), _polyhedron };
        
            // create timer object
            benchmark::timer tmp("rasterize()");
            
#pragma omp parallel for
            for(int y = 0; y < dim.y; y++)
            for(int z = 0; z < dim.z; z++) {
                std::set<int> intersections = checks::raycast::get_intersections<yzx>(glm::ivec2(y,z), _polyhedron._vertices, _yz_plane_buffer[y][z]);
                bool is_in = intersections.size() % 2 == 0 ? false : true;
                
                int from = 0;
                for(int inters : intersections) {
                    inters = constrain(0, dim.x-1, inters); // ensure we do not exceed array boundaries
                    
                    res._voxels[inters][y][z] = voxel_type::shell;                    
                    for(int i = from; i < inters; i++) {
                        inters = inters < dim.x ? inters : dim.x-1;
                        if(res._voxels[i][y][z] == voxel_type::shell) continue;
                        res._voxels[i][y][z] = is_in; // initially set voxel to 1
                    }
                    from = inters+1;
                    is_in = !is_in;
                }
            }
#pragma omp parallel for
            for(int x = 0; x < dim.x; x++)
            for(int z = 0; z < dim.z; z++) {
                std::set<int> intersections = checks::raycast::get_intersections<xzy>(glm::ivec2(x,z), _polyhedron._vertices, _xz_plane_buffer[x][z]);
                bool is_in = intersections.size() % 2 == 0 ? false : true;
                
                int from = 0;
                for(int inters : intersections) {
                    inters = constrain(0, dim.y-1, inters); // ensure we do not exceed array boundaries
                    
                    res._voxels[x][inters][z] = voxel_type::shell;
                    for(int i = from; i < inters; i++) {
                        inters = inters < dim.y ? inters : dim.y-1;
                        if(res._voxels[x][i][z] == voxel_type::shell) continue;
                        res._voxels[x][i][z] <<= is_in; // first bit shift; now the value should be either 0 or voxel_type::interior
                    }
                    from = inters+1;
                    is_in = !is_in;
                }
            }
#pragma omp parallel for
            for(int x = 0; x < dim.x; x++)
            for(int y = 0; y < dim.y; y++) {
                std::set<int> intersections = checks::raycast::get_intersections<xyz>(glm::ivec2(x,y), _polyhedron._vertices, _xy_plane_buffer[x][y]);
                bool is_in = intersections.size() % 2 == 0 ? false : true;
                
                int from = 0;
                for(int inters : intersections) {
                    inters = constrain(0, dim.z-1, inters); // ensure we do not exceed array boundaries
                    
                    res._voxels[x][y][inters] = voxel_type::shell;
                    for(int i = from; i < inters; i++) {
                        inters = inters < dim.z ? inters : dim.z-1;
                        if(res._voxels[x][y][i] == voxel_type::shell) continue;
                        res._voxels[x][y][i] <<= is_in; // second bit shift; now the value should be either 0 or voxel_type::interior
                    }
                    from = inters+1;
                    is_in = !is_in;
                }
            }
            
            return res;
        }
    };
    
    //! buffer for the intersections
    //! used by rasterize::all_rle 
    //! for memory optimization
    struct intersections {
        //! xy plane
        buffer3d<int> xy;
        //! yz plane
        buffer3d<int> yz;
        //! xz plane
        buffer3d<int> xz;
    };
    
    //! generates a voxel mesh from an arbitrary polyhedron
    //! uses fast run length encoding to minimize the size
    template<typename base_t>
    class all_oct {
        using id_t = typename mesh::polyhedron<base_t>::index_t;
        using payload_t = point<glm::vec<2, base_t>, mesh::face<id_t>>;
        
        mesh::polyhedron<base_t> _polyhedron;
        tree<quad::boundary, payload_t> _xy_tree;
        tree<quad::boundary, payload_t> _yz_tree;
        tree<quad::boundary, payload_t> _xz_tree;
        
    public: 
        all_oct(const mesh::polyhedron<base_t> &poly, glm::ivec3 scale) {
            benchmark::timer t("Generate quad trees");
            
            _polyhedron = mesh::polyhedron<base_t>::normalized(poly);
            _polyhedron = mesh::polyhedron<base_t>::scaled(_polyhedron, scale);
            
            glm::vec3 dim = glm::ceil(_polyhedron.dim());
            glm::vec3 dim_half = dim / 2.f;
            
            printf("dim: %f %f %f\n", dim.x, dim.y, dim.z);
            printf("dim_half: %f %f %f\n", dim_half.x, dim_half.y, dim_half.z);
            
            _xy_tree = tree<quad::boundary, payload_t>(quad::boundary(dim_half.xy(), dim_half.xy()));
            _yz_tree = tree<quad::boundary, payload_t>(quad::boundary(dim_half.yz(), dim_half.yz()));
            _xz_tree = tree<quad::boundary, payload_t>(quad::boundary(dim_half.xz(), dim_half.xz()));
            
            // build tree
            int i = 0;
            for (const auto &f : _polyhedron._indices._buffer) {
                const glm::vec<3, base_t> &v1 = _polyhedron._vertices[f[0]];
                const glm::vec<3, base_t> &v2 = _polyhedron._vertices[f[1]];
                const glm::vec<3, base_t> &v3 = _polyhedron._vertices[f[2]];
                
                _xy_tree.insert(point((v1.xy()), f));
                _xy_tree.insert(point((v2.xy()), f));
                _xy_tree.insert(point((v3.xy()), f));

                _yz_tree.insert(point((v1.yz()), f));
                _yz_tree.insert(point((v2.yz()), f));
                _yz_tree.insert(point((v3.yz()), f));   
                
                _xz_tree.insert(point((v1.xz()), f));
                _xz_tree.insert(point((v2.xz()), f));
                _xz_tree.insert(point((v3.xz()), f));
            }
            //test_tree(_xy_tree);
            //test_tree(_yz_tree);
            //test_tree(_xz_tree);
        }
        
        voxel_arr<base_t> rasterize() const {
            // create timer object
            benchmark::timer tmp("rasterize()");
            const glm::ivec3 dim = glm::ceil(_polyhedron.dim());
            
/*
            intersections r;
            r.yz = buffer3d<int>(dim.y, dim.z, 0);
#pragma omp parallel for
            for(int y = 0; y < dim.y; y++)
            for(int z = 0; z < dim.z; z++) {
                std::set<int> intersections = checks::raycast::get_intersections<yzx>(glm::ivec2(y,z), _polyhedron._vertices, _yz_tree[glm::vec2(y,z)]);
                std::copy(intersections.begin(), intersections.end(), std::back_inserter(r.yz[y][z]));
            }
            r.xz = buffer3d<int>(dim.x, dim.z, 0);
#pragma omp parallel for
            for(int x = 0; x < dim.x; x++)
            for(int z = 0; z < dim.z; z++) {
                std::set<int> intersections = checks::raycast::get_intersections<xzy>(glm::ivec2(x,z), _polyhedron._vertices, _xz_tree[glm::vec2(x,z)]);
                std::copy(intersections.begin(), intersections.end(), std::back_inserter(r.xz[x][z]));
            }
            r.xy = buffer3d<int>(dim.x, dim.y, 0);
#pragma omp parallel for
            for(int x = 0; x < dim.x; x++)
            for(int y = 0; y < dim.y; y++) {
                std::set<int> intersections = checks::raycast::get_intersections<xyz>(glm::ivec2(x,y), _polyhedron._vertices, _xy_tree[glm::vec2(x,y)]);
                std::copy(intersections.begin(), intersections.end(), std::back_inserter(r.xy[x][y]));
            }
            return r;
*/

            voxel_arr<base_t> res = { dim, buffer3d<int8_t>(dim.x, dim.y, dim.z, 0), _polyhedron };

#pragma omp parallel for
            for(int y = 0; y < dim.y; y++)
            for(int z = 0; z < dim.z; z++) {
                std::set<int> intersections = checks::raycast::get_intersections<yzx>(glm::ivec2(y,z), _polyhedron._vertices, _yz_tree[glm::vec2(y,z)]);
                bool is_in = intersections.size() % 2 == 0 ? false : true;
                
                int from = 0;
                for(int inters : intersections) {
                    inters = constrain(0, dim.x-1, inters); // ensure we do not exceed array boundaries
                    
                    res._voxels[inters][y][z] = voxel_type::shell;                    
                    for(int i = from; i < inters; i++) {
                        inters = inters < dim.x ? inters : dim.x-1;
                        if(res._voxels[i][y][z] == voxel_type::shell) continue;
                        res._voxels[i][y][z] = is_in; // initially set voxel to 1
                    }
                    from = inters+1;
                    is_in = !is_in;
                }
            }
#pragma omp parallel for
            for(int x = 0; x < dim.x; x++)
            for(int z = 0; z < dim.z; z++) {
                std::set<int> intersections = checks::raycast::get_intersections<xzy>(glm::ivec2(x,z), _polyhedron._vertices, _xz_tree[glm::vec2(x,z)]);
                bool is_in = intersections.size() % 2 == 0 ? false : true;
                
                int from = 0;
                for(int inters : intersections) {
                    inters = constrain(0, dim.y-1, inters); // ensure we do not exceed array boundaries
                    
                    res._voxels[x][inters][z] = voxel_type::shell;
                    for(int i = from; i < inters; i++) {
                        inters = inters < dim.y ? inters : dim.y-1;
                        if(res._voxels[x][i][z] == voxel_type::shell) continue;
                        res._voxels[x][i][z] <<= is_in; // first bit shift; now the value should be either 0 or voxel_type::interior
                    }
                    from = inters+1;
                    is_in = !is_in;
                }
            }
#pragma omp parallel for
            for(int x = 0; x < dim.x; x++)
            for(int y = 0; y < dim.y; y++) {
                std::set<int> intersections = checks::raycast::get_intersections<xyz>(glm::ivec2(x,y), _polyhedron._vertices, _xy_tree[glm::vec2(x,y)]);
                bool is_in = intersections.size() % 2 == 0 ? false : true;
                
                int from = 0;
                for(int inters : intersections) {
                    inters = constrain(0, dim.z-1, inters); // ensure we do not exceed array boundaries
                    
                    res._voxels[x][y][inters] = voxel_type::shell;
                    for(int i = from; i < inters; i++) {
                        inters = inters < dim.z ? inters : dim.z-1;
                        if(res._voxels[x][y][i] == voxel_type::shell) continue;
                        res._voxels[x][y][i] <<= is_in; // second bit shift; now the value should be either 0 or voxel_type::interior
                    }
                    from = inters+1;
                    is_in = !is_in;
                }
            }
            return res;
        }
    };
    
    //! generates a voxel mesh from an arbitrary polyhedron
    template<typename base_t>
    class shell_only {
        mesh::polyhedron<base_t> _polyhedron;
        
    public:
        shell_only(const mesh::polyhedron<base_t> &poly, const glm::ivec3 &dim) {
            _polyhedron = poly;
            _polyhedron.normalize();
            _polyhedron.scale(dim);
        }
        
        voxel_arr<base_t> rasterize() const {
            benchmark::timer tmp("rasterize()");
            
            const glm::ivec3 dim = glm::ceil(_polyhedron.dim());
            voxel_arr res = { dim, buffer3d<int8_t>(dim.x, dim.y, dim.z, 0), _polyhedron };
            
            const size_t stride = _polyhedron._indices._stride;
            const auto &index_buf = _polyhedron._indices._buffer;
            const auto &vertex_buffer = _polyhedron._vertices;
            
            for(size_t face_id = 0; face_id < index_buf.size() / stride; face_id++) {
                const glm::ivec3 id = glm::ivec3(face_id) * _polyhedron._indices._stride;
                const uint32_t vid1 = index_buf[id.x+0];
                const uint32_t vid2 = index_buf[id.y+1];
                const uint32_t vid3 = index_buf[id.z+2];

                std::array<glm::vec<3, base_t>, 3> face = {
                    vertex_buffer[vid1],
                    vertex_buffer[vid2],
                    vertex_buffer[vid3]
                };

                const glm::vec3 lmin = glm::floor(glm::min(face[2], glm::min(face[0], face[1]))) - glm::vec3(0.5f);
                const glm::vec3 lmax = glm::ceil(glm::max(face[2], glm::max(face[0], face[1]))) + glm::vec3(0.5f);
                const glm::ivec3 lsteps = lmax - lmin;

                face[0] -= lmin;
                face[1] -= lmin;
                face[2] -= lmin;
                
                const glm::ivec3 offs = lmin;

                for(int x = 0; x < lsteps.x; x++)
                for(int y = 0; y < lsteps.y; y++)
                for(int z = 0; z < lsteps.z; z++) {
                    int nx = x + lmin.x;
                    int ny = y + lmin.y;
                    int nz = z + lmin.z;
                    
                    if(res._voxels[nx][ny][nz]) continue;
                    if(checks::intersection_3d::face_in_hexahedron(face, {x,y,z}, glm::vec3(0.5))) {
                        res._voxels[nx][ny][nz] = voxel_type::shell;
                        res._num_voxels++;
                    }
                }
            }

            std::cout << "created " << (float)res._num_voxels/1000000 << " M voxels" << std::endl;
            return res;
        }
    };
};
