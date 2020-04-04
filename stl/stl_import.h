#pragma once

#include "../glm_ext/glm_extensions.h"

#include <limits>
#include <vector>
#include <fstream>
#include <cstdio>
#include <iostream>
#include <cassert>
#include <set>
#include <map>

namespace mesh {
    template<class T>
    struct polyhedron;
}

namespace stl {
    #pragma pack(push, 1)
    struct face {
        glm::vec3 _norm;
        glm::vec3 _vert_1;
        glm::vec3 _vert_2;
        glm::vec3 _vert_3;
        uint16_t _attribute = 0;

        face() = default;
        face(const glm::vec3 &n, const glm::vec3 &p1, const glm::vec3 &p2, const glm::vec3 &p3) {
            if(glm::length(n) < std::numeric_limits<float>::epsilon()) {
                const glm::vec3 u = p2 - p1;
                const glm::vec3 v = p3 - p1;
                _norm = glm::cross(u, v);
            }
            else {
                _norm = n;
            }
            _vert_1 = p1;
            _vert_2 = p2;
            _vert_3 = p3;
        }
    };
    #pragma pack(pop)

    template<typename base_t>
    struct bbox {
        glm::vec<3, base_t> _min = glm::vec3(FLT_MAX);
        glm::vec<3, base_t> _max = glm::vec3(-FLT_MAX);
    
        base_t scale() const;
        glm::vec<3, base_t> offset() const;

        //! For iterating over vertex lists
        //! pass a vertex and calculate the bounding box on the fly
        void extend(const glm::vec<3, base_t> &v) {
            _min = glm::min(_min, v);
            _max = glm::max(_max, v);
        }
    };

    class format {
        // small check to guarantee sanity
        static_assert(sizeof(face) == sizeof(glm::vec3) * sizeof(float) + sizeof(uint16_t), "size mismatch: face not compatible with stl format");
       
        uint8_t _header[80] = { 0 };
        std::vector<face> _faces;
       
        public:
            format() = default;
            format(const std::string &file);

            static mesh::polyhedron<float> to_polyhedron(const std::vector<face> &faces);
            static void save(const std::vector<face> &, const std::string &filename);
            static std::ofstream open(const std::string &file);
            static void close(std::ofstream &f);
            template<typename T>
            static void append(std::ofstream &f, const T & data) {
                f.write((char*)(&data), sizeof(T));
            }
            
            bool load(const std::string &filename);
            const std::vector<face> &faces() const;
                   
            //! calculates a bounding box
            static bbox<float> estimate_bbox(const std::vector<face> &);
            //! this functions centers the mesh automatically around around {0,0,0}
            static std::vector<face> remove_offset(const bbox<float> &, const std::vector<face> &);
            //! this functions centers the mesh automatically around around {0,0,0}
            //! and scales it down to 0 <= {x,y,z} <= 1
            static std::vector<face> normalized(const bbox<float> &, const std::vector<face> &, const glm::vec3 &transl = glm::vec3(0));
           
            // operator overload to access face
            face& operator[](std::size_t idx) { return _faces[idx]; }
            const face& operator[](std::size_t idx) const { return _faces[idx]; }
    };
};
