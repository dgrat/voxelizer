#pragma once

#include "../glm_ext/glm_extensions.h"
#include "../stl/stl_import.h"
#include "../buffer.h"

#include <utility>
#include <vector>
#include <tuple>
#include <map>
#include <fstream>
#include <limits>
#include <chrono>


namespace mesh {
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

    template <typename T>
    struct vertex_buf {
        using vec_t = glm::vec<3, T>;
        using arr_t = std::vector<vec_t>;

        /*direct access*/
        arr_t _vertex_arr;
    };

    template <typename index_t>
    struct face {
        std::array<index_t, 3> _ids;

        operator const std::array<index_t, 3> &() const {
            return _ids;
        }
        operator std::array<index_t, 3> &() {
            return _ids;
        }

        index_t &operator [] (const size_t id) {
            assert(id < 3);
            return _ids[id];
        }
        const index_t &operator [] (const size_t id) const {
            assert(id < 3);
            return _ids[id];
        }
        
        face() {
            _ids = { 0 };
        }
        face(index_t v1, index_t v2, index_t v3) {
            if(v1 > v2)
                std::swap(v1, v2);
            if(v2 > v3)
                std::swap(v2, v3);
            if(v1 > v2)
                std::swap(v1, v2);
            
            _ids = { v1, v2, v3 };
        }
    };
    
    //! axis aligned index buffer for faster voxelization
    template<typename index_t>
    struct index_buffer {
        std::vector<face<index_t>> _buffer;    // face indices

        operator const std::vector<face<index_t>> &() const {
            return _buffer;
        }
        operator std::vector<face<index_t>> &() {
            return _buffer;
        }

        face<index_t> &operator [] (const size_t id) {
            assert(id < _buffer.size());
            return _buffer[id];
        }
        const face<index_t> &operator [] (const size_t id) const {
            assert(id < _buffer.size());
            return _buffer[id];
        }
        
        void add(const face<index_t> &f) {
            _buffer.push_back(f);
        }
    };
    
    //! Yet another index based mesh
    //! Each vertex is stored exactly once within a hash table
    //! The face indices are stored separately and evaluated for error detection
    template<typename base_t>
    class polyhedron {        
    public:
        using index_t = uint32_t;
        using vec_t = glm::vec<3, base_t>;
        using arr_t = std::vector<vec_t>;
        
        arr_t _vertices; // floating point vertex representation
        index_buffer<index_t> _indices; // face indices
        std::map<edge, int> _edges; // edges with number of adjacent faces
        
        //! scales the mesh to dim and returns the new scale for each axis while keeping original ratio
        void scale(const base_t dim);
        static polyhedron<base_t> scaled(const polyhedron<base_t> &in, const base_t dim);
        
        //! scales the mesh to dim and returns the new scale for each axis while keeping original ratio
        void scale(const glm::vec<3, base_t> &dim);
        static polyhedron<base_t> scaled(const polyhedron<base_t> &in, const glm::vec<3, base_t> &dim);
        
        //! removes any offset and rescales to:
        //! 0 <= [x|y|z] <= 1
        //! but is keeping original scale factors
        void normalize();
        static polyhedron<base_t> normalized(const polyhedron<base_t> &in);
        
        //! check mesh for issues based on edges
        bool issues() const;
        
        //! export the current mesh as obj
        //! (with vertex normals)
        void to_obj(const std::string &file) const;
        //! estimates the boundingbox (slow)
        stl::bbox<base_t> bounding_box() const;
        glm::vec<3, base_t> dim() const;
    }; 
};

#include "polyhedron.tpp"
