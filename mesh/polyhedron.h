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
        using vec_t = glm::vec<3, T>;
        using map_t = std::map<vec_t, int, compare_glm_vec3>;
        using arr_t = std::vector<vec_t>;

        /*number of adjescent faces*/
        map_t _vertex_map;
        /*direct access*/
        arr_t _vertex_arr;

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
    template<typename base_t>
    struct polyhedron {       
        vertex_buf<base_t> _vertices;           // floating point vertex representation
        index_buffer _indices;          // face indices
        std::map<edge, int> _edges;     // edges with number of adjacent faces
        
        //! shifts the origin to 0 and rescales the mesh to match the scale <dim>
        //! and may convert the the mesh to a new type (float -> int)
        template<typename other_t>
        polyhedron<other_t> prepare_for_rasterization(const glm::vec<3,other_t> &new_size) {
            polyhedron<other_t> poly;
            if(_vertices._vertex_arr.size() < 1) return poly;

            const stl::bbox bbox = bounding_box();
            const glm::vec3 cur_size = bbox._max - bbox._min;

            std::vector<size_t> ind_buf = _indices._buffer;
            vertex_buf<other_t> &vbuffer = poly._vertices;
            std::map<edge, int> &ebuffer = poly._edges;

            // constexpr functor to calc the new position of the vertex
            auto calc_pos = [=](const glm::vec<3,base_t> &p) constexpr {
                constexpr bool is_int = std::is_integral<other_t>::value;
                if constexpr(is_int)
                    return glm::round((p - bbox._min) / cur_size * (glm::vec3)new_size);
                else
                    return (p - bbox._min) / cur_size * (glm::vec3)new_size;
            };

            // build unique list
            for(size_t i = 0; i < _vertices._vertex_arr.size(); i++) {
                glm::vec<3,other_t> v = calc_pos(_vertices._vertex_arr[i]);
                vbuffer._vertex_map[v] = 0;
            }
            vbuffer.build_buffer();

            // vertex ids changed, build new index list
            for(size_t i = 0; i < ind_buf.size(); i++) {
                size_t &cur_ind = ind_buf[i];
                glm::vec<3,base_t> v_old = _vertices._vertex_arr[cur_ind];
                glm::vec<3,other_t> v_new = calc_pos(v_old);
                size_t new_ind = std::distance(vbuffer._vertex_map.begin(), vbuffer._vertex_map.find(v_new));
                cur_ind = new_ind;
            }
            // remove invalid faces
            std::vector<size_t> tmp;
            for(size_t i = 0; i < ind_buf.size()/3; i++) {
                const size_t id_v1 = ind_buf[i*_indices._stride+0];
                const size_t id_v2 = ind_buf[i*_indices._stride+1];
                const size_t id_v3 = ind_buf[i*_indices._stride+2];

                // skip faces which are or have been invalidated ..
                // .. in the rescale process or before
                if(id_v1 == id_v2) continue;
                if(id_v1 == id_v3) continue;
                if(id_v2 == id_v3) continue;

                tmp.push_back(id_v1);
                tmp.push_back(id_v2);
                tmp.push_back(id_v3);

                // re-calc how often the vertices are used
                vbuffer._vertex_map[vbuffer._vertex_arr[id_v1]] += 1;
                vbuffer._vertex_map[vbuffer._vertex_arr[id_v2]] += 1;
                vbuffer._vertex_map[vbuffer._vertex_arr[id_v3]] += 1;

                // recalc the edge lists
                ebuffer[edge(id_v1, id_v2)] += 1;
                ebuffer[edge(id_v1, id_v3)] += 1;
                ebuffer[edge(id_v2, id_v3)] += 1;
            }
            poly._indices._buffer = tmp;
            return poly;
        }

        //! in case the mesh is convex, the tests are cheaper :D
        //! x = V - E + F see: https://en.wikipedia.org/wiki/Euler_characteristic
        int euler_characteristic() const {
            return _vertices._vertex_arr.size() - _edges.size() + (_indices._buffer.size() / _indices._stride);
        }

        //! export the current mesh as obj
        //! (with vertex normals)
        void to_obj(const std::string &file) const {
            std::ofstream obj_file(file);
            for(auto v : this->_vertices._vertex_arr) {
                obj_file << "v " << v.x << " " << v.y << " " << v.z << std::endl;
            }

            const size_t faces = this->_indices._buffer.size() / this->_indices._stride;
            for(size_t i = 0; i < faces; i++) {
                size_t id1 = i * this->_indices._stride + 0;
                size_t id2 = i * this->_indices._stride + 1;
                size_t id3 = i * this->_indices._stride + 2;

                size_t vid1 = this->_indices._buffer.at(id1) + 1;
                size_t vid2 = this->_indices._buffer.at(id2) + 1;
                size_t vid3 = this->_indices._buffer.at(id3) + 1;

                obj_file << "f " << vid1 << " " << vid2 << " " << vid3 << std::endl;
            }
            obj_file.close();
        }

        stl::bbox bounding_box() const {
            stl::bbox bbox;
            for(const auto &v : this->_vertices._vertex_arr) {
                bbox.extend(v);
            }
            return bbox;
        }
    }; 

    using polyhedron_flt = polyhedron<float>;
    using polyhedron_int = polyhedron<int>;
};
