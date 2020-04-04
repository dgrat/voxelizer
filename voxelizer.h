#pragma once

#include "glm_ext/glm_extensions.h"

#include "mesh/polyhedron.h"
#include "stl/stl_import.h"
#include "checks.h"
#include "xml_config.h"
#include "rules.h"
#include "buffer.h"
#include "timer.h"
#include "enums.h"
#include "rasterizer.h"

#include <set>
#include <map>
#include <vector>
#include <tuple>
#include <filesystem>
#include <fstream>


namespace voxelize {
    template<typename rule_t>
    class voxelizer {
        cfg::xml_project _project_cfg;
        std::vector<rasterize::voxel_arr> _rasterizer_res;

    public:
        voxelizer(const cfg::xml_project &cfg) {
            _project_cfg = cfg;
        }

        void to_stl(const std::string &out_file) {
            for(const rasterize::voxel_arr &arr : _rasterizer_res) {
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
                    if(arr._voxels[x][y][z] != voxel_type::shell) continue;
                    
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
            
            for(const rasterize::voxel_arr &arr : _rasterizer_res) {
                std::ofstream f(out_file, std::ios::out | std::ios::binary);
                const auto &vox_size = arr._arr_dim;
                f.write((const char*)&vox_size, sizeof(glm::ivec3));

                for(int x = 0; x < vox_size.x; x++)
                for(int y = 0; y < vox_size.y; y++)
                for(int z = 0; z < vox_size.z; z++) {
                    uint8_t v = arr._voxels[x][y][z];
                    uint8_t pal = 255;
                    
                    switch(v) {
                        case voxel_type::interior:
                            pal = 127;
                            break;
                        case voxel_type::shell:
                            pal = 123;
                            break;
                        default:
                            break;
                    };
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
            for(const cfg::shape_settings &shape : _project_cfg.shapes()) {
                stl::format stl;
                const std::filesystem::path file = path / shape._file;
                stl.load(file.string());

                mesh::polyhedron<float> p_flt = stl.to_polyhedron(stl.faces());
                //rasterize::shell_only r(p_flt, shape);
                rasterize::all_fast r(p_flt, shape);
                const rasterize::voxel_arr voxels = r.rasterize();
                _rasterizer_res.push_back(voxels);
            }
        }
    };
};

