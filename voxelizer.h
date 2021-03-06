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
            
            // rasterizer results
            rasterize::voxel_arr<float> voxels;         // high memory usage
            rasterize::intersections intersections;     // intersections only
        };
        std::vector<voxel_data_t> _rasterizer_res;
        
        stl::bbox<float> _prj_bbox;
        float _max_grid_size = 0;
        float _voxel_size = 0;
        
    private:
        //! calculates the project bbox
        //! buffers the meshes after creation
        void project_bbox() {
            const std::filesystem::path path = _project_cfg.project_path();
            
            glm::vec3 glob_min(FLT_MAX);
            glm::vec3 glob_max(-FLT_MAX);
            for(const cfg::shape_settings &shape : _project_cfg.shapes()) {
                const std::filesystem::path file = path / shape._file_in;
                const stl::format stl(file.string());
                
                mesh::polyhedron<float> p_flt = stl.to_polyhedron(stl.faces());
                const stl::bbox<float> bbox = p_flt.bounding_box();
                
                glob_min = glm::min(glob_min, bbox._min);
                glob_max = glm::max(glob_max, bbox._max);
                
                _rasterizer_res.push_back({p_flt, shape, bbox});
            }
            _prj_bbox = { glob_min, glob_max };
        }
        
        int _proj_file_cntr = 0;
        std::filesystem::path make_fname(const voxel_data_t &d, std::string ext) {
            std::filesystem::path p;
            if(d.cfg._file_out.empty()) {
                p = std::filesystem::path(_project_cfg.target_dir()) / (std::to_string(_proj_file_cntr++) + ext);
            }
            else {
                p = std::filesystem::path(_project_cfg.target_dir()) / (d.cfg._file_out + ext);
            }
            return p;
        }
        
    public:
        voxelizer(const cfg::xml_project &cfg) {
            _project_cfg = cfg;
            
            // calc the meshes and the project bbox
            project_bbox();
            
            if(_project_cfg.grid_size_defined()) {
                _max_grid_size = _project_cfg.max_grid_size();
                _voxel_size = _project_cfg.voxel_size();
            }
            else if(_project_cfg.voxel_size_defined()) {
                _voxel_size = _project_cfg.voxel_size();
                const float max_bbox_edge = glm::compMax(_prj_bbox._max - _prj_bbox._min);
                _max_grid_size = glm::ceil(max_bbox_edge / _voxel_size);
            }
        }
        
        void to_fs_obj() {
            for(const voxel_data_t &mdata : _rasterizer_res) {
                const std::filesystem::path p = make_fname(mdata, ".obj");
                mdata.mesh.to_obj(p.string());
            }
        }
        
        template<typename rule_t>
        void to_fs_stl() {
            const glm::vec3 proj_dim = (_prj_bbox._max - _prj_bbox._min);
            const float scalef = _max_grid_size / glm::compMax(proj_dim);  
            
            for(const voxel_data_t &mdata : _rasterizer_res) {
                const rasterize::voxel_arr<float> &arr = mdata.voxels;
                
                const std::filesystem::path p = make_fname(mdata, ".stl");
                benchmark::timer t("voxelizer::to_stl() - " + p.string() + " export took");
                                
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
                               
                glm::vec3 offset;
                if(_project_cfg.voxel_size_defined()) {
                    offset = glm::round(mdata.bbox._min * scalef) * _voxel_size;
                }
                else if(_project_cfg.grid_size_defined()) {
                    offset = glm::round((mdata.bbox._min - _prj_bbox._min) * scalef) * _voxel_size;
                }
                
                // now write faces of hull cubes into stl
                for(int x = 0; x < arr._arr_dim.x; x++)
                for(int y = 0; y < arr._arr_dim.y; y++)
                for(int z = 0; z < arr._arr_dim.z; z++) {
                    if(arr._voxels[x][y][z] != voxel_type::shell) continue;
                    for(stl::face &f : rule_t::mesh(glm::vec3(x,y,z)*_voxel_size+offset, glm::vec3(_voxel_size))) {
                        stl::format::append(stlf, f);
                    }
                }
                stl::format::close(stlf);
            }
        }
        
        template<array_order order = row_major>
        std::vector<uint8_t> to_bytes() const {
            glm::vec3 prj_dim_unit = _prj_bbox._max - _prj_bbox._min;
            const float scalef = _max_grid_size / glm::compMax(prj_dim_unit);   
            
            float voxel_size;
            if(_project_cfg.voxel_size_defined()) {
                voxel_size = _voxel_size;
            }
            else if(_project_cfg.grid_size_defined()) {
                voxel_size = glm::compMax(prj_dim_unit) / _max_grid_size;
            }
            else {
                std::cerr << "to_bytes() - No valid voxel size" << std::endl;
                return {};
            }
            const glm::ivec3 prj_dim_voxels = glm::ceil(prj_dim_unit / voxel_size);
            const glm::ivec3 prj_dim_bound = prj_dim_voxels - 1;
            std::vector<uint8_t> buffer(glm::compMul(prj_dim_voxels), 0);
            
            for(const voxel_data_t &mdata : _rasterizer_res) {
                benchmark::timer t("voxelizer::to_bytes() - " + mdata.cfg._file_in + " export took");
                
                const glm::vec3 offset = glm::round(mdata.bbox._min * scalef) * _voxel_size;    
                const rasterize::voxel_arr<float> &arr = mdata.voxels;
                const auto &vox_size = arr._arr_dim;
                const auto &cfg = mdata.cfg;
                
                for(int x = 0; x < vox_size.x; x++)
                for(int y = 0; y < vox_size.y; y++)
                for(int z = 0; z < vox_size.z; z++) {
                    // calculate position in project wide voxel model
                    // position in project unit
                    const glm::vec3 pos_unit = glm::vec3(x,y,z)*voxel_size+offset;
                    // voxel coordinate
                    glm::ivec3 pos_vox = glm::round(pos_unit / voxel_size);
                    // ensure boundaries are not exceeded
                    pos_vox = constrain(glm::ivec3(0), prj_dim_bound, pos_vox);
                    // 1d index
                    size_t id;
                    if constexpr(order == row_major) {
                        id = pos_vox.z * prj_dim_voxels.x * prj_dim_voxels.y + pos_vox.y * prj_dim_voxels.x + pos_vox.z;
                    }
                    else {
                        id = pos_vox.x * prj_dim_voxels.y * prj_dim_voxels.z + pos_vox.y * prj_dim_voxels.z + pos_vox.x;
                    }
                    
                    switch(arr._voxels[x][y][z]) {
                        case voxel_type::interior:
                            buffer[id] = cfg._material_inside;
                            break;
                        case voxel_type::shell:
                            buffer[id] = cfg._material_shell;
                            break;
                        default:
                            break;
                    };
                }
            }
            return buffer;
        }
        
        
        template<array_order order = row_major>
        void to_fs_bytes() const {
            std::string trg_file = "unamed_prj.raw";
            if(!_project_cfg.raw_fname().empty()) {
                trg_file = _project_cfg.raw_fname();
            }
            const std::filesystem::path p = std::filesystem::path(_project_cfg.target_dir()) / trg_file;
            benchmark::timer t("voxelizer::to_fs_bytes() - " + p.string() + " export took");
            
            std::vector<uint8_t> buffer = to_bytes();
            std::ofstream f(p, std::ios::out | std::ios::binary);
            f.write((char*)&buffer[0], buffer.size());
            f.close();
        }
/*
        void to_fs_vox(const std::string &out_file) {
            for(const voxel_data_t &mdata : _rasterizer_res) {
                const rasterize::voxel_arr<float> &arr = mdata.voxels;
                
                const auto &vox_size = arr._arr_dim;
                const auto &cfg = mdata.cfg;
                
                vox::chunk::MAIN chunk_main;                
                vox::chunk::SIZE chunk_size(vox_size);   
                vox::chunk::XYZI chunk_xyzi;
                
                for(int x = 0; x < vox_size.x; x++)
                for(int y = 0; y < vox_size.y; y++)
                for(int z = 0; z < vox_size.z; z++) {
                    uint8_t v = arr._voxels[x][y][z];
                    switch(v) {
                        case voxel_type::interior:
                            chunk_xyzi.data.push_back(vox::xyzi_t(x, y, z, cfg._material_inside));
                            break;
                        case voxel_type::shell:
                            chunk_xyzi.data.push_back(vox::xyzi_t(x, y, z, cfg._material_shell));
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
*/
        void clear() {
            _rasterizer_res.clear();
        }

        void run() {
            if(!_project_cfg.voxel_size_defined() && !_project_cfg.grid_size_defined()) {
                std::cerr << "Neither maximum grid size, nor voxel size defined in *.xml file" << std::endl;
                return;
            }
            
            const glm::vec3 proj_dim = (_prj_bbox._max - _prj_bbox._min);
            const float scalef = _max_grid_size / glm::compMax(proj_dim);
            
            for(voxel_data_t &mdata : _rasterizer_res) {
                // calculate uniorm scale factor (+/- 1 voxel)
                // keeping the size ratio of each voxel model in the project constant to each other
                const glm::vec3 mesh_dim = (mdata.bbox._max - mdata.bbox._min) * scalef;
                const float dim = glm::round(glm::compMax(mesh_dim));
                
                std::cout << "Process file: " << mdata.cfg._file_in << std::endl;
                
                //mdata.voxels = rasterize::all_fast(mdata.mesh, glm::ivec3(dim)).rasterize();
                mdata.voxels = rasterize::all_oct(mdata.mesh, glm::ivec3(dim)).rasterize();
                //mdata.voxels = rasterize::shell_only(mdata.mesh, _max_grid_size).rasterize();
            }
        }
    };
};

