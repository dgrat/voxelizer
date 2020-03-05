#pragma once

#include "glm/glm_extensions.h"

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
#pragma omp parallel for
            for(int x = 0; x < steps.x; x++) {
            for(int y = 0; y < steps.y; y++) {
            for(int z = 0; z < steps.z; z++) {
                const glm::vec3 pos = from + glm::vec3(x,y,z) * setting._voxel_size;
                buffer[x * steps.y * steps.z  + y * steps.z + z] = rule_t::evaluate(pos, _polyhedron, setting);
            }
            }

#pragma omp critical
{
            std::cout << "progress: " << ++progress << "/" << steps.x << std::endl;
}
            }
            return buffer;
        }
    };

    template<typename rule_t> class voxelizer {
        cfg::xml_project _project_cfg;
        std::multimap<voxel_t, cfg::shape_settings> _voxels;

    public:
        voxelizer(const cfg::xml_project &cfg) {
            _project_cfg = cfg;
        }

        void to_stl(const std::string &out_file) {
            std::vector<stl::face> voxel_faces;
            for(auto &e : _voxels) {
                const voxel_t &voxel = e.first;
                const cfg::shape_settings &setting = e.second;

                for(auto &f : rule_t::mesh(voxel._position, setting))
                    voxel_faces.push_back(f);
            }
            stl::format::save(voxel_faces, out_file);
        }

        void clear() {
            _voxels.clear();
        }

        void run() {
            const std::filesystem::path path = _project_cfg.project_path();
            for(const auto &shape : _project_cfg.shapes()) {
                stl::format stl;
                const std::filesystem::path file = path / shape._file;
                stl.load(file);

                const auto polyhedron = stl.to_polyhedron(stl.faces());
                grid<rule_t> grid_voxelizer(polyhedron);

                const auto start = std::chrono::steady_clock::now();

                std::vector<voxel_t> voxels = grid_voxelizer.hexaeders_in(shape);
                for(auto &p : voxels) {
                    if(p._is_in_mesh == false) continue;
                    _voxels.insert({p, shape});
                }

                const auto end = std::chrono::steady_clock::now();
                std::cout << "time elapsed: " << std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count() << std::endl;
            }
        }
    };
};

