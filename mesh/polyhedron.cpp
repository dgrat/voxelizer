#include "polyhedron.h"
#include <limits>


namespace mesh {   
    //! in case the mesh is convex, the tests are cheaper :D
    //! x = V - E + F see: https://en.wikipedia.org/wiki/Euler_characteristic
    int polyhedron::euler_characteristic() const {
        return _vertices._vertex_arr.size() - _edges.size() + (_indices._buffer.size() / _indices._stride);
    }

    //! export the current mesh as obj
    //! (with vertex normals)
    void polyhedron::to_obj(const std::string &file) const {
        std::ofstream obj_file(file);
        for(auto v : this->_vertices._vertex_arr) {
            obj_file << "v " << v._position.x << " " << v._position.y << " " << v._position.z << std::endl;
            obj_file << "vn " << v._normal.x << " " << v._normal.y << " " << v._normal.z << std::endl;
        }

        const size_t faces = this->_indices._buffer.size() / this->_indices._stride;
        for(size_t i = 0; i < faces; i++) {
            size_t id1 = i * this->_indices._stride + 0;
            size_t id2 = i * this->_indices._stride + 1;
            size_t id3 = i * this->_indices._stride + 2;

            size_t vid1 = this->_indices._buffer.at(id1) + 1;
            size_t vid2 = this->_indices._buffer.at(id2) + 1;
            size_t vid3 = this->_indices._buffer.at(id3) + 1;

            obj_file << "f " << vid1 << "//" << vid1 << " " << vid2 << "//" << vid2 << " " << vid3 << "//" << vid3 << std::endl;
        }
        obj_file.close();
    }

    stl::bbox polyhedron::bounding_box() const {
        stl::bbox bbox;
        for(const auto &v : this->_vertices._vertex_arr) {
            bbox.extend(v._position);
        }
        return bbox;
    }
};
