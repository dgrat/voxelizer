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
#include "vox_file.h"

#include <string>
#include <set>
#include <map>
#include <vector>
#include <tuple>
#include <filesystem>
#include <fstream>


namespace voxelize {
    class voxelizer {
        cfg::xml_project _project_cfg;
        
        struct voxel_data_t {
            mesh::polyhedron<float> mesh;
            cfg::shape_settings cfg;
            stl::bbox<float> bbox;
            rasterize::voxel_arr<float> voxels;
        };
        std::vector<voxel_data_t> _rasterizer_res;
        
        stl::bbox<float> _prj_bbox;
        
    private:
        //! calculates the project bbox
        //! buffers the meshes after creation
        void project_bbox() {
            const std::filesystem::path path = _project_cfg.project_path();
            
            glm::vec3 glob_min(FLT_MAX);
            glm::vec3 glob_max(-FLT_MAX);
            for(const cfg::shape_settings &shape : _project_cfg.shapes()) {
                const std::filesystem::path file = path / shape._file;
                
                std::cout << file << std::endl;
                
                stl::format stl(file.string());
                mesh::polyhedron<float> p_flt = stl.to_polyhedron(stl.faces());
                stl::bbox<float> bbox = p_flt.bounding_box();
                
                glob_min = glm::min(glob_min, bbox._min);
                glob_max = glm::max(glob_max, bbox._max);
                
                _rasterizer_res.push_back({p_flt, shape, bbox});
            }
            _prj_bbox = { glob_min, glob_max };
        }
        
    public:
        voxelizer(const cfg::xml_project &cfg) {
            _project_cfg = cfg;
        }
        
        template<typename rule_t>
        void to_stl(const std::string &out_dir) {
            int file_id = 0;
            for(const voxel_data_t &mdata : _rasterizer_res) {
                const rasterize::voxel_arr<float> &arr = mdata.voxels;
                
                auto p = std::filesystem::path(out_dir) / (std::to_string(file_id++) + ".stl");                
                auto stlf = stl::format::open(p.string());
                for(int i = 0; i < 80; i++)
                    stl::format::append(stlf, char(0));

                // first count hull cubes
                int num_cubes = 0;
                for(int x = 0; x < arr._arr_dim.x; x++)
                for(int y = 0; y < arr._arr_dim.y; y++)
                for(int z = 0; z < arr._arr_dim.z; z++) {
                    if(arr._voxels[x][y][z] != voxel_type::shell) continue;
                    num_cubes++;
                }
                uint32_t faces = num_cubes * 12;
                stl::format::append(stlf, faces);
                
                const glm::vec3 proj_dim = (_prj_bbox._max - _prj_bbox._min);
                const float scalef = _project_cfg.max_grid_size().x / glm::compMax(proj_dim);                
                const glm::vec3 offset = glm::round((mdata.bbox._min - _prj_bbox._min) * scalef);
                
                // now write faces of hull cubes into stl
                for(int x = 0; x < arr._arr_dim.x; x++)
                for(int y = 0; y < arr._arr_dim.y; y++)
                for(int z = 0; z < arr._arr_dim.z; z++) {
                    if(arr._voxels[x][y][z] != voxel_type::shell) continue;
                    for(stl::face &f : rule_t::mesh(glm::vec3(x,y,z)+offset, glm::vec3(1))) {
                        stl::format::append(stlf, f);
                    }
                }
                stl::format::close(stlf);
            }
        }
        
        void to_vox(const std::string &out_file) {
            for(const voxel_data_t &mdata : _rasterizer_res) {
                const rasterize::voxel_arr<float> &arr = mdata.voxels;
                
                const auto &vox_size = arr._arr_dim;
                
                vox::chunk::MAIN chunk_main;                
                vox::chunk::SIZE chunk_size(vox_size);   
                vox::chunk::XYZI chunk_xyzi;
                
                for(int x = 0; x < vox_size.x; x++)
                for(int y = 0; y < vox_size.y; y++)
                for(int z = 0; z < vox_size.z; z++) {
                    uint8_t pal = 255;
                    uint8_t v = arr._voxels[x][y][z];
                    switch(v) {
                        case voxel_type::interior:
                            pal = 127;
                            chunk_xyzi.data.push_back(vox::xyzi_t(x,y,z,pal));
                            break;
                        case voxel_type::shell:
                            pal = 123;
                            chunk_xyzi.data.push_back(vox::xyzi_t(x,y,z,pal));
                            break;
                        default:
                            break;
                    };
                }
                
                chunk_main.models.push_back({chunk_size, chunk_xyzi});
                
                std::ofstream f(out_file, std::ios::out | std::ios::binary);
                chunk_main.write(f);
                f.close();
            }
        }

        void clear() {
            _rasterizer_res.clear();
        }

        void run() {
            // calc the meshes and the project bbox
            project_bbox();
            const glm::vec3 proj_dim = (_prj_bbox._max - _prj_bbox._min);
            const float scalef = _project_cfg.max_grid_size().x / glm::compMax(proj_dim);
            printf ("scalef %f proj_dim %f %f %f\n", scalef, proj_dim.x, proj_dim.y, proj_dim.z);
            
            for(voxel_data_t &mdata : _rasterizer_res) {
                // calculate uniorm scale factor (+/- 1 voxel)
                // keeping the size ratio of each voxel model in the project constant to each other
                const glm::vec3 mesh_dim = (mdata.bbox._max - mdata.bbox._min) * scalef;
                const float dim = glm::round(glm::compMax(mesh_dim));
                
                mdata.voxels = rasterize::all_fast(mdata.mesh, glm::ivec3(dim)).rasterize();
                //mdata.voxels = rasterize::shell_only(mdata.mesh, _project_cfg.max_grid_size()).rasterize();
            }
        }
    };
};

