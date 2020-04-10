#include "stl_import.h"
#include "../mesh/polyhedron.h"
#include "../glm_ext/glm_extensions.h"

#include <functional>
#include <iostream>
#include <fstream>

namespace stl {
    //! Estimates global factor to scale the whole model to 0 < x < 1
    template<typename base_t>
    base_t bbox<base_t>::scale() const {
        base_t max(-FLT_MAX);
        glm::vec3 dif = _max - _min;

        for(int i = 0; i < 3; i++) {
            max = dif[i] > max ? dif[i] : max;
        }
        return base_t(1) / max;
    }

    //! Estimates offset to scale the whole model to 0 <= x <= 1
    template<typename base_t>
    glm::vec<3, base_t> bbox<base_t>::offset() const {
        return _min * base_t(-1);
    }

    std::vector<face> format::normalized(const bbox<float> &b, const std::vector<face> &faces, const glm::vec3 &transl) {
        const float scale = b.scale();
        const glm::vec3 offs = b.offset();

        std::vector<face> res;
        for(face f : faces) {
            glm::vec3 v1 = (f._vert_1 + offs) * scale + transl;
            glm::vec3 v2 = (f._vert_2 + offs) * scale + transl;
            glm::vec3 v3 = (f._vert_3 + offs) * scale + transl;
            res.push_back( { f._norm, v1, v2, v3 } );
        }

        return res;
    }

    std::vector<face> format::remove_offset(const bbox<float> &b, const std::vector<face> &faces) {
        const glm::vec3 offs = b.offset();
        const glm::vec3 dim = (b._max - b._min) / 2.f;

        std::vector<face> res;
        for(face f : faces) {
            f._vert_1 += offs - dim;
            f._vert_2 += offs - dim;
            f._vert_3 += offs - dim;
            res.push_back( f );
        }

        return res;
    }

    bbox<float> format::estimate_bbox(const std::vector<face> &faces) {
        glm::vec3 min(FLT_MAX);
        glm::vec3 max(-FLT_MAX);

        for(auto face : faces) {
            min = glm::min(face._vert_1, min);
            min = glm::min(face._vert_2, min);
            min = glm::min(face._vert_3, min);

            max = glm::max(face._vert_1, max);
            max = glm::max(face._vert_2, max);
            max = glm::max(face._vert_3, max);
        }
        return { min, max };
    }

    format::format(const std::string &file) {
        if(!load(file)) {
            std::cerr << file << ": could not be loaded" << std::endl; 
        }
    }

    void format::close(std::ofstream &f) {
        f.close();
    }
    
    std::ofstream format::open(const std::string &file) {
        std::remove(file.c_str());
        std::ofstream f(file, std::ios::out | std::ios::binary | std::ios::app);
        return f;
    }
    
    bool format::load(const std::string &filename) {
        std::ifstream ifs;
        ifs.open (filename.c_str(), std::ios::in | std::ios::binary);

        if(!ifs.is_open()) return false;

        // read header
        ifs.read(reinterpret_cast<char *>(&_header[0]), 80);
        // read number of faces
        size_t num_faces = 0;
        ifs.read(reinterpret_cast<char *>(&num_faces), 4);
        // read the faces
        _faces.resize(num_faces);
        ifs.read(reinterpret_cast<char *>(&_faces[0]), num_faces*sizeof(face));

        ifs.close();
        return true;
    }

    void format::save(const std::vector<face> &faces, const std::string &file) {
        std::ofstream stl_file(file, std::ios::out | std::ios::binary);
        std::array<char, 80> header = { 0 };
        stl_file.write((char*)(&header[0]), header.size());
        const uint32_t n_faces = faces.size();
        stl_file.write((char*)(&n_faces), sizeof(uint32_t));
        stl_file.write((char*)(faces.data()), sizeof(stl::face)*n_faces);
        stl_file.close();
    }

    const std::vector<face> &format::faces() const {
        return _faces;
    }

    mesh::polyhedron<float> format::to_polyhedron(const std::vector<face> &faces) {
        mesh::polyhedron<float> mesh;
        auto &vertex_arr =  mesh._vertices;

        // put vertices into a hash table
        int i = 0;
        for(const auto &f : faces) {
            vertex_arr.push_back(f._vert_1);
            vertex_arr.push_back(f._vert_2);
            vertex_arr.push_back(f._vert_3);

            int id_v1 = i*3+0;
            int id_v2 = i*3+1;
            int id_v3 = i*3+2;
            
            mesh._indices.add(id_v1);
            mesh._indices.add(id_v2);
            mesh._indices.add(id_v3);

            mesh._edges[mesh::edge(id_v1, id_v2)] += 1;
            mesh._edges[mesh::edge(id_v2, id_v3)] += 1;
            mesh._edges[mesh::edge(id_v3, id_v1)] += 1;
            
            i++;
        }
        return mesh;
    }
};
