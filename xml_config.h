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
        std::string _project_file = "";
        std::set<shape_settings> _shapes;
        glm::ivec3 _max_grid_size;

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
                float x = g.attribute("max_x").as_int();
                float y = g.attribute("max_y").as_int();
                float z = g.attribute("max_z").as_int();
                _max_grid_size = {x,y,z};
            }
            return result;
        }

    public:
        const glm::ivec3 &max_grid_size() const {
            return _max_grid_size;
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
