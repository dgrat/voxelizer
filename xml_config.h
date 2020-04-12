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
        std::string _file;
        std::string _priority;
        std::string _material;

        // for std::set
        friend bool operator< (const shape_settings &lhs, const shape_settings &rhs) {
            return lhs._file < rhs._file;
        }
    };

    //! project settings
    class xml_project {
        std::string                 _project_file = "";
        std::set<shape_settings>    _shapes;
        int                         _grid_size; // maximum number of voxels (either along: w, h, d)
        float                       _voxel_size; // alternatively use voxel size
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
                    const std::string file = tool.attribute("file").as_string();
                    const std::string prio = tool.attribute("priority").as_string();
                    const std::string mat = tool.attribute("material").as_string();
                    _shapes.insert({file, prio, mat});
                }
                pugi::xml_node g = doc.child("project").child("grid");
                if(!g.empty()) {
                    _grid_size_defined = true;
                    const int s = g.attribute("max_voxels").as_int();
                    const float e = g.attribute("stl_voxel_size").as_float();
                    _grid_size = s;
                    _voxel_size = e;
                }
                pugi::xml_node r = doc.child("project").child("voxel_size");
                if(!r.empty()) {
                    _voxel_size_defined = true;
                    const float e = r.attribute("edge_length").as_float();
                    _voxel_size = e;
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
        const std::set<shape_settings> &shapes() const {
            return _shapes;
        }
        const std::string project_file() const {
            return _project_file;
        }
        const std::string project_path() const {
            return std::filesystem::path(_project_file).parent_path().string();
        }

        void init(const std::string &project_dir) {
            std::vector<std::string> xml_files;
            for (const auto& entry : std::filesystem::directory_iterator(project_dir)) {
                if(endsWithIgnoreCase(entry.path().string(), ".xml")) {
                    xml_files.push_back(entry.path().string());
                }
            }
            if(xml_files.size() == 1) {
                _project_file = xml_files.at(0);
                read_project();
            }
        }

        xml_project() = default;
        xml_project(const std::string &project_dir) {
            this->init(project_dir);
        }
    };
};
