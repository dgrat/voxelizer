#pragma once

#include "glm_ext/glm_extensions.h"

#include <pugixml.hpp>
#include <string>
#include <iostream>
#include <filesystem>
#include <vector>
#include <regex>
#include <set>


namespace cfg {
    //! shape settings
    struct shape_settings {
        std::string _file_in;
        std::string _file_out;
        int _merge_priority;
        int _material_inside;
        int _material_shell;
    };

    //! project settings
    class xml_project {
        std::string                 _project_file = "";
        std::vector<shape_settings> _shapes;
        
        
        int                         _grid_size;     // maximum number of voxels (either along: w, h, d)
        float                       _voxel_size;    // alternatively use voxel size
        
        std::string                 _target_dir = "";
        std::string                 _raw_fname = "";
        
        // internal state members
        bool                        _voxel_size_defined = false;
        bool                        _grid_size_defined = false;
        
        
        static bool endsWithIgnoreCase(const std::string& str, const std::string& suffix) {
            return std::regex_search(str, std::regex(std::string(suffix) + "$", std::regex_constants::icase));
        }

        pugi::xml_parse_result read_project() {
            pugi::xml_document doc;
            pugi::xml_parse_result result = doc.load_file(_project_file.c_str());
            if (result) {
                for (pugi::xml_node tool : doc.child("project").children("stl")) {
                    const std::string file_in = tool.attribute("file_in").as_string();
                    const std::string file_out = tool.attribute("file_out").as_string();
                    const int prio = tool.attribute("merge_priority").as_int();
                    const int mat_in = tool.attribute("material_inside").as_int();
                    const int mat_out = tool.attribute("material_outside").as_int();
                    _shapes.push_back({file_in, file_out, prio, mat_in, mat_out});
                }
                pugi::xml_node g = doc.child("project").child("grid");
                if(!g.empty()) {
                    _grid_size_defined = true;
                    _grid_size = g.attribute("max_voxels").as_int();
                    _voxel_size = g.attribute("stl_cube_size").as_float();
                }
                pugi::xml_node r = doc.child("project").child("voxel_size");
                if(!r.empty()) {
                    _voxel_size_defined = true;
                    _voxel_size = r.attribute("cube_size").as_float();
                }
                pugi::xml_node t = doc.child("project").child("target");
                if(!t.empty()) {
                    _target_dir = t.attribute("dir_out").as_string();
                    _raw_fname = t.attribute("raw_fname").as_string();
                }
            }
            return result;
        }

    public:
        const bool voxel_size_defined() const {
            return _voxel_size_defined;
        }
        const bool grid_size_defined() const {
            return _grid_size_defined;
        }
        const float voxel_size() const {
            return _voxel_size;
        }
        const int max_grid_size() const {
            return _grid_size;
        }
        const std::vector<shape_settings> &shapes() const {
            return _shapes;
        }
        const std::string target_dir() const {
            return _target_dir;
        }
        const std::string raw_fname() const {
            return _raw_fname;
        }
        const std::string project_file() const {
            return _project_file;
        }
        const std::string project_path() const {
            return std::filesystem::path(_project_file).parent_path().string();
        }

        void init(const std::string &project_dir) {
            if(!std::filesystem::exists(project_dir)) {
                std::cerr << "xml_project::init() - invalid project config: " << project_dir << std::endl;
                return;
            }
            
            std::vector<std::string> xml_files;
            for (const auto& entry : std::filesystem::directory_iterator(project_dir)) {
                if(endsWithIgnoreCase(entry.path().string(), ".xml")) {
                    std::cout << "add file: " << entry.path().string() << std::endl;
                    xml_files.push_back(entry.path().string());
                }
            }
            if(xml_files.size() > 0) {
                _project_file = xml_files.at(0);
                read_project();
            }
        }

        xml_project() = default;
        xml_project(const std::string &project_dir) {
            init(project_dir);
            if(!std::filesystem::exists(_target_dir)) {
                std::cout << _target_dir << " does not exist. Create new directory" << std::endl; 
                std::filesystem::create_directory(_target_dir);
            }
        }
    };
};
