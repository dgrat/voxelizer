#include "../timer.h"

namespace mesh {
    template<typename base_t>
    void polyhedron<base_t>::scale(const base_t dim) {
        benchmark::timer tmp("scale()");
        const stl::bbox bbox = bounding_box();
        const glm::vec<3, base_t> cur_size = bbox._max - bbox._min;
        
        for(size_t i = 0; i < _vertices.size(); i++) {
            _vertices[i] *= dim;
        }
        _dim = cur_size * dim;
    }

    template<typename base_t>
    void polyhedron<base_t>::scale(const glm::vec<3, base_t> &dim) {
        benchmark::timer tmp("scale()");
        const stl::bbox<base_t> bbox = bounding_box();
        const glm::vec<3, base_t> cur_size = bbox._max - bbox._min;

        for(size_t i = 0; i < _vertices.size(); i++) {
            _vertices[i] *= dim;
        }
        _dim = cur_size * dim;
    }
    
    template<typename base_t>
    void polyhedron<base_t>::normalize() {
        constexpr bool is_flt = std::is_floating_point<base_t>::value;
        static_assert(is_flt, "normalize(): type must be floating point");
    
        benchmark::timer tmp("normalize()");
        const stl::bbox<base_t> bbox = bounding_box();
        const glm::vec<3, base_t> cur_size = bbox._max - bbox._min;
        const base_t comp_max = glm::compMax(cur_size);
                
        // constexpr functor to calc the new position of the vertex
        auto calc_pos = [=](const glm::vec<3,base_t> &p) constexpr {
            return (p - bbox._min) / comp_max;
        };

        for(size_t i = 0; i < _vertices.size(); i++) {
            _vertices[i] = calc_pos(_vertices[i]);
        }
        
        // return new dimensions of the model
        _dim = cur_size / comp_max;
    }

    template<typename base_t>
    polyhedron<base_t> polyhedron<base_t>::scaled(const polyhedron<base_t> &in, const base_t dim) {
        polyhedron<base_t> res = in;
        res.scale(dim);
        return res;
    }
    
    template<typename base_t>
    polyhedron<base_t> polyhedron<base_t>::scaled(const polyhedron<base_t> &in, const glm::vec<3, base_t> &dim) {
        polyhedron<base_t> res = in;
        res.scale(dim);
        return res;
    }
    
    template<typename base_t>
    polyhedron<base_t> polyhedron<base_t>::normalized(const polyhedron<base_t> &in){
        polyhedron<base_t> res = in;
        res.normalize();
        return res;
    }
    
    template<typename base_t>
    void polyhedron<base_t>::to_obj(const std::string &file) const {
        std::ofstream obj_file(file);
        for(auto v : this->_vertices) {
            obj_file << "v " << v.x << " " << v.y << " " << v.z << std::endl;
        }

        const size_t faces = this->_indices._buffer.size() / this->_indices._stride;
        for(size_t i = 0; i < faces; i++) {
            index_t id1 = i * this->_indices._stride + 0;
            index_t id2 = i * this->_indices._stride + 1;
            index_t id3 = i * this->_indices._stride + 2;

            index_t vid1 = this->_indices._buffer.at(id1) + 1;
            index_t vid2 = this->_indices._buffer.at(id2) + 1;
            index_t vid3 = this->_indices._buffer.at(id3) + 1;

            obj_file << "f " << vid1 << " " << vid2 << " " << vid3 << std::endl;
        }
        obj_file.close();
    }

    template<typename base_t>
    stl::bbox<base_t> polyhedron<base_t>::bounding_box() {
        stl::bbox<base_t> bbox;
        for(const auto &v : this->_vertices) {
            bbox.extend(v);
        }
        _dim = bbox._max - bbox._min;
        return bbox;
    }  
    
    template<typename base_t>
    bool polyhedron<base_t>::issues() const {
        for(const auto &p : _edges) {
            const size_t _face_count = p.second;
            if(_face_count != 2) return false;
        }
        return true;
    }
};
