#pragma once

#include "glm_ext/glm_extensions.h"

#include "mesh/polyhedron.h"
#include "stl/stl_import.h"
#include "checks.h"
#include "xml_config.h"
#include <filesystem>
#include <fstream>
#include "rules.h"

#include <set>
#include <map>
#include <vector>
#include <tuple>


namespace voxelize {

    struct voxel_arr {
        glm::ivec3 _arr_dim;
        glm::vec3 _offset;
        cfg::shape_settings _setting;
        std::vector<int8_t> _voxels;
        size_t _num_voxels;
    };
    
    //! generates a voxel mesh from an arbitrary polyhedron
    template<typename rule_t> class grid {
        mesh::polyhedron _polyhedron;
        stl::bbox _bbox;                // bounding box of the mesh

    public:
        grid(const mesh::polyhedron &poly) {
            _polyhedron = poly;
            _bbox = poly.bounding_box();
        }

        voxel_arr hexaeders_in(const cfg::shape_settings &setting) const {
            const glm::vec3 from = glm::floor(_bbox._min - 0.5f);
            const glm::vec3 to = glm::ceil(_bbox._max + 0.5f);
            const glm::ivec3 steps = glm::abs(to - from) / setting._voxel_size;
            const size_t voxels = static_cast<size_t>(steps.x) * steps.y * steps.z;
            voxel_arr res = { steps, glm::vec3(0), setting, std::vector<int8_t>(voxels, -1) };

            int progress = 0;
//#pragma omp parallel for
            for(int y = 0; y < steps.y; y++) {
            for(int z = 0; z < steps.z; z++) {
                const glm::vec3 pos = from + glm::vec3(0,y,z) * setting._voxel_size;
                checks::raycast::fill_x(pos, {0,y,z}, setting._voxel_size, _polyhedron, steps, res._voxels);
            }

//#pragma omp critical
{
            std::cout << "progress: " << ++progress << "/" << steps.x << std::endl;
}
            }

            return res;
        }
    };


/*
    //! generates a voxel mesh from an arbitrary polyhedron
    template<typename rule_t> class grid {
        mesh::polyhedron _polyhedron;
        stl::bbox _bbox;                // bounding box of the mesh

    public:
        grid(const mesh::polyhedron &poly) {
            _polyhedron = poly;
            _bbox = poly.bounding_box();
        }

        voxel_arr hexaeders_in(const cfg::shape_settings &setting) const {
            const auto &v =  _polyhedron._vertices._vertex_arr;
            const auto &index_buf = _polyhedron._indices._buffer;
            const size_t faces = _polyhedron._indices._buffer.size() / _polyhedron._indices._stride;

            const stl::bbox bbox = _polyhedron.bounding_box();
            const glm::vec3 &gmin = glm::floor(bbox._min / setting._voxel_size - glm::vec3(1));
            const glm::vec3 &gmax = glm::ceil(bbox._max / setting._voxel_size + glm::vec3(1));
            const glm::ivec3 gsteps = gmax - gmin;
            const size_t gvoxels = static_cast<size_t>(gsteps.x) * gsteps.y * gsteps.z;

            voxel_arr res = { gsteps, gmin*setting._voxel_size, setting, std::vector<uint8_t>(gvoxels) };
            size_t voxels = 0;
            const auto start = std::chrono::steady_clock::now();
            for(size_t face_id = 0; face_id < faces; face_id++) {
                const glm::ivec3 id = glm::ivec3(face_id) * _polyhedron._indices._stride;
                const uint32_t vid1 = index_buf.at(id.x+0);
                const uint32_t vid2 = index_buf.at(id.y+1);
                const uint32_t vid3 = index_buf.at(id.z+2);

                std::array<glm::vec3, 3> face = {
                    v[vid1]._position / setting._voxel_size,
                    v[vid2]._position / setting._voxel_size,
                    v[vid3]._position / setting._voxel_size
                };

                const glm::vec3 lmin = glm::floor(glm::min(face[2], glm::min(face[0], face[1])));
                const glm::vec3 lmax = glm::ceil(glm::max(face[2], glm::max(face[0], face[1])) + glm::vec3(0.5));
                const glm::ivec3 lsteps = lmax - lmin;

                face[0] -= lmin;
                face[1] -= lmin;
                face[2] -= lmin;

                const glm::ivec3 offs = lmin - gmin;

                for(int x = 0; x < lsteps.x; x++)
                for(int y = 0; y < lsteps.y; y++)
                for(int z = 0; z < lsteps.z; z++) {
                    const glm::ivec3 i = glm::ivec3(x,y,z) + offs;
                    const size_t id = i.x * gsteps.y * gsteps.z + i.y * gsteps.z + i.z;
                    
                    if(res._voxels[id]) continue;
                    if(checks::intersection_3d::face_in_hexahedron(face, {x,y,z}, glm::vec3(0.5))) {
                        res._voxels[id] = true;
                        res._num_voxels++;
                    }
                }
            }

            const auto end = std::chrono::steady_clock::now();
            std::cout << "time elapsed: " << std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count() << std::endl;
            std::cout << "created " << (float)res._num_voxels/1000000 << " M voxels" << std::endl;
            return res;
        }
    };
*/
    template<typename rule_t>
    class voxelizer {
        cfg::xml_project _project_cfg;
        std::vector<voxel_arr> _rasterizer_res;

    public:
        voxelizer(const cfg::xml_project &cfg) {
            _project_cfg = cfg;
        }

        void to_stl(const std::string &out_file) {
            for(const voxel_arr &arr : _rasterizer_res) {
                auto stlf = stl::format::open(out_file);
                for(int i = 0; i < 80; i++)
                    stl::format::append(stlf, char(0));
                uint32_t faces = arr._num_voxels * 12;
                stl::format::append(stlf, faces);

                const auto &vox_size = arr._arr_dim;
                for(int x = 0; x < arr._arr_dim.x; x++)
                for(int y = 0; y < arr._arr_dim.y; y++)
                for(int z = 0; z < arr._arr_dim.z; z++) {
                    const size_t i = x * vox_size.y * vox_size.z  + y * vox_size.z + z;
                    if(!arr._voxels[i]) continue;
                    
                    const glm::vec3 pos = glm::vec3(x,y,z) * arr._setting._voxel_size + arr._offset;
                    for(stl::face &f : rule_t::mesh(pos, arr._setting)) {
                        stl::format::append(stlf, f);
                    }
                }
                stl::format::close(stlf);
            }
        }
        
        void to_vox(const std::string &out_file) {
            struct v3uint8_t {
                uint8_t x = 0;
                uint8_t y = 0;
                uint8_t z = 0;
            };
            
            // saves a voxel file (.slab.vox format, can be imported by MagicaVoxel)
            std::vector<v3uint8_t> palette(256); // RGB palette
            palette[123] = { 127, 0, 127 };
            palette[124] = { 255, 0, 0 };
            palette[125] = { 0, 255, 0 };
            palette[126] = { 0, 0, 255 };
            palette[127] = { 255, 255, 255 };
            
            for(const voxel_arr &arr : _rasterizer_res) {
                std::ofstream f(out_file, std::ios::out | std::ios::binary);
                const auto &vox_size = arr._arr_dim;
                f.write((const char*)&vox_size, sizeof(glm::ivec3));
                
                for(int x = 0; x < vox_size.x; x++)
                for(int y = 0; y < vox_size.y; y++)
                for(int z = 0; z < vox_size.z; z++) {
                    const size_t i = x * vox_size.y * vox_size.z  + y * vox_size.z + z;
                    uint8_t v = arr._voxels[i];
                    uint8_t pal = v == 1 ? 127 : 255;
                    f.write((const char*)&pal, 1);
                }

                f.write((const char*)&palette[0], palette.size()*sizeof(v3uint8_t));
                f.close();
                break;
            }
        }

        void clear() {
            _rasterizer_res.clear();
        }

        void run() {
            const std::filesystem::path path = _project_cfg.project_path();
            for(const auto &shape : _project_cfg.shapes()) {
                stl::format stl;
                const std::filesystem::path file = path / shape._file;
                stl.load(file.string());

                auto polyhedron = stl.to_polyhedron(stl.faces());
                grid<rule_t> grid_voxelizer(polyhedron);

                const auto start = std::chrono::steady_clock::now();
                const voxel_arr voxels = grid_voxelizer.hexaeders_in(shape);
                _rasterizer_res.push_back(voxels);

                const auto end = std::chrono::steady_clock::now();
                std::cout << "time elapsed: " << std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count() << std::endl;
            }
        }
    };
};

