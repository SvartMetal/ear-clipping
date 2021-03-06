#pragma once

#include "geom/primitives/contour.h"
#include "io/point.h"
#include "io/segment.h"

#include <list>

#include <boost/variant/variant.hpp>

namespace geom {
namespace structures {

    using geom::structures::point_type;

    struct vertex_type : point_type {

        vertex_type(int32 x, int32 y)
            : point_type(x, y)
        {}

        vertex_type(const point_type& point)
            : point_type(point.x, point.y)
        {}

        vertex_type& operator = (vertex_type const & v) 
        {
            x = v.x;
            y = v.y;
            convex = v.convex;
            ear = v.ear;
            return *this;
        }

        bool convex = true;
        bool ear = false;
    };

    template <typename T> class dc_list : public std::list<T> {
    public:

        typedef typename std::list<T>::iterator list_iterator;
        class Iterator;
        typedef Iterator iterator;
        typedef Iterator const_iterator;

        dc_list() = default;

        dc_list(std::list<T> && pts) 
            : std::list<T>(std::move(pts))
        {}

        class Iterator : public list_iterator {
        public:

            Iterator & operator = (Iterator const & iter) {
                auto temp(iter);
                swap(temp);
                return *this;
            }

            Iterator(dc_list & list, list_iterator const & iter) 
                : list_iterator(iter), list_(list) 
            {}

            Iterator prev() {
                Iterator temp(*this);
                validate(temp);
                if (temp != list_.begin()) {
                    --temp;
                    return temp;
                }
                temp = list_.end();
                --temp;
                return temp;
            }

            Iterator next() {
                Iterator temp(*this);
                validate(temp);
                ++temp;
                if (temp == list_.end()) {
                    return list_.begin(); 
                }
                return temp;
            }

        private:

            void swap(Iterator & iter) {
                list_iterator::operator = (iter);
                std::swap(list_, iter.list_);
            }

            void validate(Iterator & iter) {
                if (iter == list_.end()) {
                    throw std::runtime_error("Attempt to use invalid iterator.");
                }
            }

            dc_list & list_;
        };

        Iterator begin() {
            return Iterator(*this, std::list<T>::begin());
        }
        
        Iterator end() {
            return Iterator(*this, std::list<T>::end());
        }

    };

}}

namespace geom {
namespace structures {

    struct contour_builder_type
    {
        void add_point(point_type const & pt)
        {
            pts_.push_back(pt);
        }
        
        contour_type get_result()
        {
            return contour_type(std::move(pts_));
        }
        
    private:
        std::vector<point_type> pts_;
    };    
    
}}

namespace visualization {

    using geom::structures::segment_type;

    struct drawer_type;

    void draw(drawer_type & drawer, std::vector<segment_type> const & segments);
    void draw(drawer_type & drawer, geom::structures::contour_type const & cnt, bool draw_vertices = false);
}

namespace geom {
namespace algorithms {
namespace triangulation {

    using geom::structures::point_type;
    using geom::structures::contour_type;
    using geom::structures::segment_type;

    std::vector<segment_type> ear_clipping(std::list<point_type> const pts);

}}}

namespace geom {
namespace algorithms {
namespace intersection {

    using geom::structures::point_type;
    using geom::structures::segment_type;

    bool is_intersection(std::list<point_type> const & pts, segment_type const & sgm);
}}}

