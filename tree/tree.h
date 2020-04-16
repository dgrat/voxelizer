#pragma once

#include <list>
#include <limits>
#include <vector>
#include "../timer.h"
#include "../glm_ext/glm_extensions.h"
#include "../stl/stl_import.h"
#include "../mesh/polyhedron.h"

namespace test {
    struct draw_data {
        // opencv stuff
        // corners of the current boundary
        struct rect_t {
            glm::vec2 low;
            glm::vec2 high;
        } rect;
        // points in the grid
        std::vector<glm::vec2> points;
    };   
};

namespace quad {
    struct boundary {
        using vec_t = glm::vec2;
        vec_t _pos;
        vec_t _dim;
        vec_t _initial_dim;
        
        boundary() = default;
        boundary(const vec_t &cnt, const vec_t &dim) {
            _pos = cnt;
            _dim = dim;
            _initial_dim = dim;
        }
        boundary(const vec_t &cnt, const vec_t &dim, const vec_t &initial_dim) {
            _pos = cnt;
            _dim = dim;
            _initial_dim = initial_dim;
        }
        
        bool is_in(const vec_t &pt) const {
            const bool high = glm::all(glm::lessThanEqual(pt, _pos + _dim));
            const bool low = glm::all(glm::greaterThanEqual(pt, _pos - _dim));
            return (low && high);
        }
        
        bool intersects(const boundary &other) const {
            const vec_t other_max = other._pos + other._dim;
            const vec_t this_min = _pos - _dim;
            
            //printf("intersects: [%f,%f]\n", other._pos.x, other._pos.y);
            
            if(other_max.x < this_min.x) return false;
            if(other_max.y < this_min.y) return false;
            
            const vec_t other_min = other._pos - other._dim;
            const vec_t this_max = _pos + _dim;
            if(other_min.x > this_max.x) return false;
            if(other_min.y > this_max.y) return false;
            
            return true;
        }
        
        //! return list of new boundaries for tree sub-division
        std::vector<boundary> divide() const {
            const vec_t dim_half = _dim / 2.f;
            vec_t new_cnt_1 = _pos;
            new_cnt_1.x += dim_half.x;
            new_cnt_1.y += dim_half.y;
            
            vec_t new_cnt_2 = _pos;
            new_cnt_2.x += dim_half.x;
            new_cnt_2.y -= dim_half.y;
            
            vec_t new_cnt_3 = _pos;
            new_cnt_3.x -= dim_half.x;
            new_cnt_3.y -= dim_half.y;
            
            vec_t new_cnt_4 = _pos;
            new_cnt_4.x -= dim_half.x;
            new_cnt_4.y += dim_half.y;
            
            return {
                { new_cnt_1, dim_half, _initial_dim },
                { new_cnt_2, dim_half, _initial_dim },
                { new_cnt_3, dim_half, _initial_dim },
                { new_cnt_4, dim_half, _initial_dim }
            };
        }
    };
};

template <typename pos_t, typename payload_t>
struct point {
    using vec_t = pos_t;
    
    pos_t _pos;
    payload_t _payload;
    
    point() = default;
    point(const pos_t &pos, const payload_t &val) {
        _pos = pos;
        _payload = val;
    }
    
    // conversion operator
    operator payload_t &() {
        return _payload;
    }
    operator const payload_t &() const {
        return _payload;
    }
    
    // access operator
    auto &operator[] (const size_t id) {
        return _payload[id];
    }
    const auto &operator[] (const size_t id) const {
        return _payload[id];
    }
};

template <typename boundary_t, typename pt_t>
class tree {
    using pos_t = typename pt_t::vec_t;
    
public:
    // boundary of the tree volume
    // defined by a center position and [w, h, l] dimension (half box size)
    boundary_t _boundary;
    
    // list of points
    std::vector<pt_t> _points = {};
    
    // tree branches
    std::vector<tree<boundary_t, pt_t>> _branches = {};
    
    // maximum capacity
    // default is three points (because we want a new boundary for each face)
    size_t _capacity;
    
protected:
    void divide() {
        // avoid getting boundaries too small
        size_t cap = _capacity;
/*
        if(glm::any(glm::lessThanEqual(_boundary._dim / pos_t(2), pos_t(1)))) {
            cap = std::numeric_limits<uint16_t>::max();
        }
*/
        for(const boundary_t &new_boundary : _boundary.divide()) {
            _branches.push_back(tree<boundary_t, pt_t>(new_boundary, cap));
        }
    }
    
    void recursive_find(const boundary_t &boundary_in, std::vector<pt_t> &points_in_out) const {        
        if(!_boundary.intersects(boundary_in)) {
            return;
        }
        std::copy(_points.begin(), _points.end(), std::back_inserter(points_in_out));
        // forward the point to all branches
        for(const tree<boundary_t, pt_t> &b : _branches) {
            b.recursive_find(boundary_in, points_in_out);
        }
    }

    void recursive_find(const pos_t &pos_in, std::vector<pt_t> &points_in_out) const {        
        if(!_boundary.is_in(pos_in)) {
            return;
        }
        std::copy(_points.begin(), _points.end(), std::back_inserter(points_in_out));
        // forward the point to all branches
        for(const tree<boundary_t, pt_t> &b : _branches) {
            b.recursive_find(pos_in, points_in_out);
        }
    }
    
public:
    tree() = default;
    tree(const boundary_t &boundary, size_t capacity = 1) {
        _boundary = boundary;
        _capacity = capacity;
    }

    std::vector<pt_t> operator[](const pos_t &pos) const {
        return find(pos);
    }
    
    std::vector<pt_t> operator[](const boundary_t &boundary) const {
        return find(boundary);
    }
    
    //! find all entries whose boundaries surround <pos>
    std::vector<pt_t> find(const pos_t &pos) const {        
        std::vector<pt_t> res = {};
        recursive_find(pos, res);
        return res;
    }
    
    //! find all entries whose boundaries surround <pos>
    std::vector<pt_t> find(const pos_t &pos, const pos_t &search_dist) const {        
        std::vector<pt_t> res = {};
        recursive_find(boundary_t(pos, search_dist), res);
        return res;
    }
    
    //! find all entries whose boundaries intersect with <boundary>
    std::vector<pt_t> find(const boundary_t &boundary) const {
        std::vector<pt_t> res = {};
        recursive_find(boundary, res);
        return res;
    }
    
    //! insert a new point (and payload) into the tree
    bool insert(const pt_t &pt) {
        // point not in boundary
        if(!_boundary.is_in(pt._pos)) {
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
            for(tree<boundary_t, pt_t> &b : _branches) {
                if(b.insert(pt)) break;
            }
        }
        return true;
    }
    
// ifdef test ..
    void recursive_get_tree(std::vector<test::draw_data> &corners) const {
        const auto &c = _boundary._pos;
        const auto &d = _boundary._dim;
        
        test::draw_data dta;
        dta.rect.low = c-d;
        dta.rect.high = c+d;
        for(auto v : _points)
            dta.points.push_back(v._pos);
        
        corners.push_back(dta);
        // forward the point to all branches
        for(const tree<boundary_t, pt_t> &b : _branches) {
            b.recursive_get_tree(corners);
        }
    }
};

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

using namespace cv;

template <typename boundary_t, typename pt_t>
void test_tree(const tree<boundary_t, pt_t> &t) {
    /// Windows names
    char tree_window[] = "Drawing 1: Atom";

    float scale_f = 1;
    float offs = 16;
    
    int w = t._boundary._dim.x*2 * scale_f + 2*offs;
    int h = t._boundary._dim.y*2 * scale_f + 2*offs;
    
    std::cout << "w: " << t._boundary._dim.x << " h: " << t._boundary._dim.y << " w: " << w << " h: " << h << std::endl;
    
    /// Create black empty images
    Mat tree_image = Mat( h, w, CV_8UC4, Scalar(255,255,255,0) );    
    std::vector<test::draw_data> dta;
    
    t.recursive_get_tree(dta);

    for(auto e : dta) {
        auto f = e.rect.low*scale_f+offs;
        auto s = e.rect.high*scale_f+offs;
        Point p1(f.x, f.y);
        Point p2(s.x, s.y);
        
        int pos = rand() % 3;
        int pos2 = rand() % 3;
        
        Scalar color( 0, 0, 0 );
        Scalar lighter( 32, 32, 32 );
        color[pos] = 196;
        color[pos2] = 96;

        // draw rectangle
        rectangle(
            tree_image,
            p1,
            p2,
            color+lighter,
            CV_FILLED
        );
        
        // draw points
        for(auto c : e.points) {            
            circle(
                tree_image,
                Point(c.x*scale_f+offs, c.y*scale_f+offs),
                scale_f/4,
                color,
                CV_FILLED
            );
        }
    }
    
    imshow( tree_window, tree_image );
    waitKey( 0 );
}
