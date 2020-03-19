#pragma once

#include "glm_ext/glm_extensions.h"

#include "mesh/polyhedron.h"
#include "stl/stl_import.h"
#include "checks.h"
#include "xml_config.h"
#include <filesystem>
#include "rules.h"

#include <set>
#include <map>
#include <vector>
#include <tuple>


namespace voxelize {
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

        std::vector<voxel_t> hexaeders_in(const cfg::shape_settings &setting) const {
            const glm::vec3 from = glm::floor(_bbox._min - 0.5f);
            const glm::vec3 to = glm::ceil(_bbox._max + 0.5f);
            const glm::ivec3 steps = glm::abs(to - from) / setting._voxel_size;
            const size_t voxels = static_cast<size_t>(steps.x) * steps.y * steps.z;
            std::vector<voxel_t> buffer(voxels);

            int progress = 0;
//#pragma omp parallel for
            for(int x = 0; x < steps.x; x++) {
            for(int y = 0; y < steps.y; y++) {
            for(int z = 0; z < steps.z; z++) {
                const glm::vec3 pos = from + glm::vec3(x,y,z) * setting._voxel_size;
                buffer[x * steps.y * steps.z  + y * steps.z + z] = rule_t::evaluate(pos, _polyhedron, setting);
            }
            }

//#pragma omp critical
{
            std::cout << "progress: " << ++progress << "/" << steps.x << std::endl;
}
            }
            return buffer;
        }
    };
*/
    struct voxel_arr {
        glm::ivec3 _arr_dim;
        glm::vec3 _offset;
        cfg::shape_settings _setting;
        std::vector<bool> _voxels;
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
            const auto &v =  _polyhedron._vertices._vertex_arr;
            const auto &index_buf = _polyhedron._indices._buffer;
            const size_t faces = _polyhedron._indices._buffer.size() / _polyhedron._indices._stride;

            const stl::bbox bbox = _polyhedron.bounding_box();
            const glm::vec3 &gmin = bbox._min / setting._voxel_size - glm::vec3(1);
            const glm::vec3 &gmax = bbox._max / setting._voxel_size + glm::vec3(1);
            const glm::ivec3 gsteps = glm::ceil(gmax) - glm::floor(gmin);
            const size_t gvoxels = static_cast<size_t>(gsteps.x) * gsteps.y * gsteps.z;

            voxel_arr res = { gsteps, gmin*setting._voxel_size, setting, std::vector<bool>(gvoxels) };
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

                const glm::vec3 lmin = glm::min(face[2], glm::min(face[0], face[1])) - glm::vec3(0.5);
                const glm::vec3 lmax = glm::max(face[2], glm::max(face[0], face[1])) + glm::vec3(0.5);
                const glm::ivec3 lsteps = glm::ceil(lmax) - glm::floor(lmin);

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

