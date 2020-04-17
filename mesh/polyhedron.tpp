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
    }

    template<typename base_t>
    void polyhedron<base_t>::scale(const glm::vec<3, base_t> &dim) {
        benchmark::timer tmp("scale()");
        const stl::bbox<base_t> bbox = bounding_box();
        const glm::vec<3, base_t> cur_size = bbox._max - bbox._min;

        for(size_t i = 0; i < _vertices.size(); i++) {
            _vertices[i] *= dim;
        }
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

        for(auto &f : _indices._buffer) {
            obj_file << "f " << f[0]+1 << " " << f[1]+1 << " " << f[2]+1 << std::endl;
        }
        obj_file.close();
    }    
    
    template<typename base_t>
    stl::bbox<base_t> polyhedron<base_t>::bounding_box() const {
        stl::bbox<base_t> bbox;
        for(const auto &v : this->_vertices) {
            bbox.extend(v);
        }
        return bbox;
    }
    
    template<typename base_t>
    glm::vec<3, base_t> polyhedron<base_t>::dim() const {
        auto tmp = bounding_box();
        return tmp._max - tmp._min; 
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
