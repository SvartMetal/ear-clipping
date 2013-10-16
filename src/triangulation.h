#pragma once

#include "geom/primitives/contour.h"
#include "io/point.h"
#include "io/segment.h"

#include <list>

namespace geom {
namespace structures {

    using geom::structures::point_type;

    class dc_list : public std::list<point_type> {
    public:

        typedef std::list<point_type>::iterator list_iterator;
        class Iterator;
        typedef Iterator iterator;

        dc_list() = default;
        dc_list(std::list<point_type> && pts);

        class Iterator : public list_iterator {
        public:

            Iterator & operator = (const Iterator & iter) {
                auto temp(iter);
                swap(temp);
                return *this;
            }

            Iterator(dc_list & list, const list_iterator & iter) 
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
            return Iterator(*this, std::list<point_type>::begin());
        }
        
        Iterator end() {
            return Iterator(*this, std::list<point_type>::end());
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

    void draw(drawer_type & drawer, const std::vector<segment_type> & segments);
    void draw(drawer_type & drawer, const geom::structures::contour_type & cnt, bool draw_vertices = false);
}

namespace geom {
namespace algorithms {
namespace triangulation {

    using geom::structures::point_type;
    using geom::structures::contour_type;
    using geom::structures::segment_type;

    std::vector<segment_type> ear_clipping(const std::list<point_type> pts);

}}}

namespace geom {
namespace algorithms {
namespace convex_hull {

    using geom::structures::point_type;
    using geom::structures::contour_type;

    contour_type andrews(std::vector<point_type> pts); 
}}}

