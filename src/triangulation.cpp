#include "stdafx.h"

#include "triangulation.h"
#include "visualization/viewer_adapter.h"
#include <boost/optional/optional.hpp>

using geom::structures::point_type;
using geom::structures::segment_type;
using geom::structures::dc_list; 

namespace geom {
namespace predicates {
    
    enum turn_type
    {
        COLLINEAR = 0, LEFT, RIGHT
    };
    
    namespace
    {
        template<typename Scalar>
        int sgn(Scalar x)
        {
            if (x == 0)
                return 0;
            else
                return x < 0 ? -1 : 1;
        }
    }

    turn_type turn( point_type const & a,
                    point_type const & b,
                    point_type const & c )
    {
        auto v1 = b - a;
        auto v2 = c - a;
        
        auto sign = sgn(v1 ^ v2);
        
        switch (sign)
        {
            case -1: return RIGHT;
            case  1: return LEFT;
            default: return COLLINEAR;
        }
    }
}}


namespace visualization {

    void draw(drawer_type & drawer, const std::vector<segment_type> & segments) 
    {
        for (const auto & line : segments) 
        {
            drawer.draw_line(line);
        }
    }
}

namespace geom {
namespace structures {
    
    contour_type::contour_type(std::vector<point_type> && pts)
        : pts_(std::move(pts))
    {}
    
}}

namespace geom {
namespace util {

    void add_edges(dc_list<segment_type> & segments, dc_list<point_type> & pts) 
    {
        auto end = pts.end();
        for (auto it = pts.begin(); it != end; ++it) 
        {
            segments.push_back(segment_type(*it, *it.next()));
        }
    }
}}

namespace geom {
namespace algorithms {
namespace triangulation {


    predicates::turn_type get_default_turn(dc_list<point_type> & list) 
    {
        using namespace predicates;
        using namespace structures;

        auto end = list.end();
        int left = 0;
        int right = 0;
        for (auto iter = list.begin(); iter != end; ++iter) 
        {
            auto current_turn = turn(*iter.prev(), *iter, *iter.next());
            if (current_turn != COLLINEAR) 
            {
                if (current_turn == LEFT) 
                {
                    ++left;
                } 
                else 
                {
                    ++right;
                }
            }

        }

        return left > right ? LEFT : RIGHT;
    }
    
    bool is_inside_triangle(point_type p, point_type a, point_type b, point_type c) 
    {
        using namespace predicates;

        auto pab = turn(p, a, b);
        auto pbc = turn(p, b, c);
        auto pca = turn(p, c, a);
        
        if (pab && pbc && pca) 
        {
            return pab == pbc && pbc == pca ? true : false;
        }
        return false;
    }

    std::vector<segment_type> ear_clipping(std::list<point_type> pts) 
    {

        std::vector<segment_type> edges;

        pts.erase(boost::unique<boost::return_found>(pts), pts.end());

        dc_list<point_type> list;
        list.assign(pts.begin(), pts.end());

        auto default_turn = get_default_turn(list);

        auto v_iter = list.begin();
        while (list.size() > 3) 
        {
            auto a(*v_iter.prev()), b(*v_iter), c(*v_iter.next());
            v_iter = v_iter.next();

            if (predicates::turn(a, b, c) != default_turn) 
            {
                continue;
            }

            bool is_ear = true;
            for (auto p : list) 
            {
                if (p != a && p != b && p != c && is_inside_triangle(p, a, b, c)) 
                {
                    is_ear = false;
                    break;
                }
            }

            if (!is_ear) continue;

            edges.push_back(segment_type(a, c));
            list.erase(v_iter.prev());
        }

        return edges;
    }

}}}

namespace geom {
namespace algorithms {
namespace intersection {

    using geom::predicates::turn_type;
    using geom::predicates::turn;
    using geom::structures::range_type;

    bool is_point(const segment_type & s) {
        return s[0] == s[1];
    }

    segment_type get_segment(const point_type & pt1 , const point_type & pt2) {
        int32 x1(pt1.x);
        int32 y1(pt1.y);
        int32 x2(pt2.x);
        int32 y2(pt2.y);

        int32 a_end;
        int32 b_end;

        #define GENERATE(f, res, first, second) \
        if (x1 == x2) { res = f(first, second); } else if (x1 < x2) { res = first; } else { res = second; }

        GENERATE(std::min, a_end, y1, y2);
        GENERATE(std::max, b_end, y2, y1);

        #undef GENERATE

        point_type a(std::min(x1, x2), a_end);
        point_type b(std::max(x1, x2), b_end);

        return segment_type(a, b);
    }

    enum intersection_type {
        NONE = 0, POINT, LINE
    };

    boost::optional<range_type> intersect_range(const range_type & r1, const range_type & r2) {
        int32 ax = std::min(r1.inf, r1.sup);
        int32 bx = std::max(r1.inf, r1.sup);
        int32 cx = std::min(r2.inf, r2.sup);
        int32 dx = std::max(r2.inf, r2.sup);

        if (std::max(ax, cx) > std::min(bx, dx)) {
            return boost::optional<range_type>();
        }
        if (r1.inf < r1.sup && r2.inf < r2.sup) 
        {
            return boost::optional<range_type>
            (
                range_type(std::max(ax, cx), std::min(bx, dx))
            );
        } 
        else 
        {
            return boost::optional<range_type>
            (
                range_type(std::min(bx, dx), std::max(ax, cx))
            );
        }
    }

    intersection_type intersect_segment(segment_type& s1_, segment_type& s2_) {
        segment_type s1(get_segment(s1_[0], s1_[1]));
        segment_type s2(get_segment(s2_[0], s2_[1]));

        turn_type turn1(turn(s1[0], s1[1], s2[0]));
        turn_type turn2(turn(s1[0], s1[1], s2[1]));
        turn_type turn3(turn(s2[0], s2[1], s1[0]));
        turn_type turn4(turn(s2[0], s2[1], s1[1]));

        if (turn1 == turn2 && turn1 == 0 && turn3 == turn4 && turn3 == 0) 
        {
            auto rx
            (
                intersect_range
                (
                    range_type(s1[0].x, s1[1].x), 
                    range_type(s2[0].x, s2[1].x)
                )
            );
            auto ry
            (
                intersect_range
                (
                    range_type(s1[0].y, s1[1].y), 
                    range_type(s2[0].y, s2[1].y)
                )
            );
            if (rx && ry) 
            {
                segment_type res
                (
                    point_type(rx.get().inf, ry.get().inf),
                    point_type(rx.get().sup, ry.get().sup)
                );
                if (is_point(res))
                {
                    return POINT;
                } 
                else 
                {
                    return LINE;
                }
            } 
            else 
            {
                return NONE;
            }
        }
        else
        {
            if (turn1 == turn2 || turn3 == turn4) 
            {
                return NONE;
            } 
            else 
            {
                return POINT;

            } 
        }
    }

    using geom::structures::dc_list;
    using geom::util::add_edges;

    bool is_intersect(const std::list<point_type> & pts_) 
    {
        dc_list<point_type> pts;
        pts.assign(pts_.begin(), pts_.end());
        dc_list<segment_type> segments;

        add_edges(segments, pts);

        for (auto outer = segments.begin(); outer != segments.end(); ++outer) 
        {
            dc_list<segment_type>::iterator temp(outer);
            if (++temp == segments.end()) 
            {
                break;
            }
            ++temp;
            for (dc_list<segment_type>::iterator inner = temp; inner != segments.end(); ++inner)
            {
                if (intersect_segment(*inner, *outer) != NONE && *inner.next() != *outer) 
                {
                    return true;
                }
            }
        }

        return false;
    }

}}}


