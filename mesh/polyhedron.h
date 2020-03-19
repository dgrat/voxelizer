#pragma once

#include "../glm_ext/glm_extensions.h"

#include "../stl/stl_import.h"

#include <utility>
#include <vector>
#include <tuple>
#include <map>
#include <fstream>
#include <limits>


namespace mesh {
    struct vertex {
        // flooating point vertex position
        glm::vec3 _position;
        glm::vec3 _normal;
        
        vertex(const glm::vec3& p, const glm::vec3& n = glm::vec3(0)) {
            _position = p;
            _normal = n;
        }

        // for std::set
        friend bool operator< (const vertex &lhs, const vertex &rhs) {
            return std::tie(lhs._position.x, lhs._position.y, lhs._position.z) < std::tie(rhs._position.x, rhs._position.y, rhs._position.z);
        }
    };

    struct edge {
        size_t _id1;
        size_t _id2;

        edge(const size_t v1, const size_t v2) {
            _id1 = std::min(v1, v2);
            _id2 = std::max(v1, v2);
        }
        // for std::set
        friend bool operator< (const edge &lhs, const edge &rhs) {
            return std::make_tuple(lhs._id1, lhs._id2) < std::make_tuple(rhs._id1, rhs._id2);
        }
    };

    struct index_buffer {
        int _stride = 3;              // stride of std::vector
        std::vector<size_t> _buffer; // face indices

        void add(const size_t index) {
            _buffer.push_back(index);
        }
    };

    template <typename T>
    struct vertex_buf {
        /*number of adjescent faces*/
        std::map<T, int> _vertex_map;
        /*direct access*/
        std::vector<T> _vertex_arr;

        void build_buffer() {
            _vertex_arr.clear();
            for(const auto &v : _vertex_map) {
                _vertex_arr.push_back(v.first);
            }
        }
    };

    //! Yet another index based mesh
    //! Each vertex is stored exactly once within a hash table
    //! The face indices are stored separately and evaluated for error detection
    struct polyhedron {       
        vertex_buf<vertex> _vertices;           // floating point vertex representation        
        index_buffer _indices;          // face indices
        std::map<edge, int> _edges;     // edges with number of adjacent faces
        
        //! in case the mesh is convex, the tests are cheaper :D
        //! x = V - E + F see: https://en.wikipedia.org/wiki/Euler_characteristic
        int euler_characteristic() const;

        //! export the current mesh as obj
        //! (with vertex normals)
        void to_obj(const std::string &file) const;

        stl::bbox bounding_box() const;
    }; 
};
