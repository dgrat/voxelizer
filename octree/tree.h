#pragma once

#include <list>
#include <vector>
#include "../glm_ext/glm_extensions.h"

namespace octa {
    template <typename pos_t, typename material_t>
    struct point {
        using vec_t = glm::vec<3, pos_t>;
        vec_t _pos;
        material_t _material = 0;
        
        point() = default;
        point(const vec_t &pos, const material_t mat) {
            _pos = pos;
            _material = mat;
        }
    };
    
    template <typename pos_t, typename material_t>
    struct boundary {
        using vec_t = glm::vec<3, pos_t>;
        
        vec_t _center; // center position
        vec_t _half_box_size; // half box size
        
        boundary(const vec_t &min, const vec_t &max) {
            _half_box_size = max - min;
            _center = min + _half_box_size / 2.f;
        }
        
        boundary(const vec_t &cnt, const pos_t wx, const pos_t wy, const pos_t wz) {
            _center = cnt;
            _half_box_size = {wx,wy,wz};
        }
        
        bool is_in(const point<pos_t, material_t> &pt) const {
            const bool high = glm::all(glm::lessThanEqual(pt, _center + _half_box_size));
            const bool low = glm::all(glm::greaterThanEqual(pt, _center - _half_box_size));
            return (low && high);
        }
        
        bool intersects(const boundary &other) const {
            const vec_t other_min = other._center - other._half_box_size;
            const vec_t other_max = other._center + other._half_box_size;
            const vec_t this_min = _center - _half_box_size;
            const vec_t this_max = _center + _half_box_size;
            
            if(other_max.x < this_min.x) return false;
            if(other_max.y < this_min.y) return false;
            if(other_max.z < this_min.z) return false;
            
            if(other_min.x > this_max.x) return false;
            if(other_min.y > this_max.y) return false;
            if(other_min.z > this_max.z) return false;
            
            return true;
        }
        
        //! return list of new boundaries for tree sub-division
        std::vector<boundary<pos_t, material_t>> divide() const {
            const vec_t dim_half = _half_box_size / 2.f;
            // clockwise -z/2
            vec_t new_cnt_1 = _center;
            new_cnt_1.x += dim_half.x;
            new_cnt_1.y += dim_half.y;
            new_cnt_1.z -= dim_half.z;
            
            vec_t new_cnt_2 = _center;
            new_cnt_2.x += dim_half.x;
            new_cnt_2.y -= dim_half.y;
            new_cnt_2.z -= dim_half.z;
            
            vec_t new_cnt_3 = _center;
            new_cnt_3.x -= dim_half.x;
            new_cnt_3.y -= dim_half.y;
            new_cnt_3.z -= dim_half.z;
            
            vec_t new_cnt_4 = _center;
            new_cnt_4.x -= dim_half.x;
            new_cnt_4.y += dim_half.y;
            new_cnt_4.z -= dim_half.z;
            
            // clockwise +z/2
            vec_t new_cnt_5 = _center;
            new_cnt_5.x += dim_half.x;
            new_cnt_5.y += dim_half.y;
            new_cnt_5.z += dim_half.z;
            
            vec_t new_cnt_6 = _center;
            new_cnt_6.x += dim_half.x;
            new_cnt_6.y -= dim_half.y;
            new_cnt_6.z += dim_half.z;
            
            vec_t new_cnt_7 = _center;
            new_cnt_7.x -= dim_half.x;
            new_cnt_7.y -= dim_half.y;
            new_cnt_7.z += dim_half.z;
            
            vec_t new_cnt_8 = _center;
            new_cnt_8.x -= dim_half.x;
            new_cnt_8.y += dim_half.y;
            new_cnt_8.z += dim_half.z;
            
            return {
                { new_cnt_1, dim_half },
                { new_cnt_2, dim_half },
                { new_cnt_3, dim_half },
                { new_cnt_4, dim_half },
                
                { new_cnt_5, dim_half },
                { new_cnt_6, dim_half },
                { new_cnt_7, dim_half },
                { new_cnt_8, dim_half }
            };
        }
    };
    
    template <typename pos_t, typename material_t>
    class tree {
        using pt_t = point<pos_t, material_t>;
        
    private:
        // boundary of the tree volume
        // defined by a center position and [w, h, l] dimension (half box size)
        boundary<pos_t, material_t> _boundary;
        
        // list of points
        std::vector<point<pos_t, material_t>> _points = {};
        
        // tree branches
        std::vector<tree<pos_t, material_t>> _branches = {};
        
        // maximum capacity
        size_t _capacity = 8;
        
    protected:
        void divide() {
            for(const boundary<pos_t, material_t> &new_boundary : _boundary.divide()) {
                _branches.push_back(boundary<pos_t, material_t>(new_boundary, _capacity));
            }
        }
        
        void recursive_find(const boundary<pos_t, material_t> &boundary_in, std::vector<pt_t> &points_in_out) {
            if(!_boundary.intersects(boundary_in)) {
                return;
            }
            std::copy(_points.begin(), _points.end(), std::back_inserter(points_in_out));
            // forward the point to all branches
            for(tree<pos_t, material_t> &b : _branches) {
                b.recursive_find(boundary_in, points_in_out);
            }
        }
        
    public:
        tree(const boundary<pos_t, material_t> &boundary, const size_t capacity = 8) {
            _boundary = boundary;
            _capacity = capacity;
        }
        
        std::vector<pt_t> find(const boundary<pos_t, material_t> &boundary) {
            std::vector<pt_t> res = {};
            recursive_find(boundary, res);
            return res;
        }
        
        bool insert(const point<pos_t, material_t> &pt) {
            // point not in boundary
            if(!_boundary.is_in(pt)) {
                return false;
            }
            // push point in if not full
            if(_points.size() < _capacity) {
                _points.push_back(pt);
            }
            else {
                // if full then subdivide
                if(_branches.size() == 0) {
                    divide();
                }
                // forward the point to all branches
                for(tree<pos_t, material_t> &b : _branches) {
                    if(b.insert(pt)) break;
                }
            }
            return true;
        }
    };
};
