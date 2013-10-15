#include "stdafx.h"

#include "triangulation.h"
#include "visualization/viewer_adapter.h"

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

dc_list::dc_list(std::list<point_type> && pts) 
    : std::list<point_type>(std::move(pts))
{}

namespace visualization {

    void draw(drawer_type & drawer, const std::vector<segment_type> & segments) {
        for (const auto & line : segments) {
            drawer.draw_line(line);
        }
    }
}

namespace geom {
namespace structures {
    
    contour_type::contour_type(std::vector<point_type> && pts)
        : pts_(std::move(pts))
    {}
    
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

namespace geom {
namespace algorithms {
namespace convex_hull {

    template<class Iter, class Func>
    void find_chain(Iter beg, Iter end, Func f)
    {
        using namespace geom::predicates;

        std::vector<Iter> stack;
        stack.push_back(beg);
        while (++beg != end)
        {
            size_t N = stack.size();

            while ((N > 1) && (turn(*stack[N - 2], *stack[N - 1], *beg) == RIGHT))
            {
                stack.pop_back();
                --N;
            }
            stack.push_back(beg);
        }
        for (auto it = stack.begin() + 1, end = stack.end() - 1; it != end; ++it)
            f(**it);
    }

    contour_type andrews(std::vector<point_type> vec)
    {
        boost::sort(vec);

        std::list<point_type> pts;
        pts.assign(vec.begin(), vec.end());

        pts.erase(boost::unique<boost::return_found>(pts), pts.end());

        if (pts.size() < 2) {
            throw std::logic_error("not enough points to build convex hull");
        }

        geom::structures::contour_builder_type builder;

        auto f = [&] (point_type const & pt) { builder.add_point(pt); };

        builder.add_point(pts.front());
        find_chain(pts.begin(), pts.end(), f);
        builder.add_point(pts.back());
        find_chain(pts.rbegin(), pts.rend(), f);

        return builder.get_result();
    }
    
}}}

namespace geom {
namespace algorithms {
namespace triangulation {

    void add_edges(std::vector<segment_type> & segments, dc_list & list) {
        auto end = list.end();
        for (auto iter = list.begin(); iter != end; ++iter) {
            segments.push_back(segment_type(*iter, *iter.next()));
        }
    }

    predicates::turn_type get_default_turn(dc_list & list) {
        using namespace predicates;
        using namespace structures;

        auto end = list.end();
        int left = 0;
        int right = 0;
        for (auto iter = list.begin(); iter != end; ++iter) {
            auto current_turn = turn(*iter.prev(), *iter, *iter.next());
            if (current_turn != COLLINEAR) {
                if (current_turn == LEFT) {
                    ++left;
                } else {
                    ++right;
                }
            }

        }

        return left > right ? LEFT : RIGHT;
    }
    
    bool is_inside_triangle(point_type p, point_type a, point_type b, point_type c) {
        using namespace predicates;

        auto pab = turn(p, a, b);
        auto pbc = turn(p, b, c);
        auto pca = turn(p, c, a);
        
        if (pab && pbc && pca) {
            return pab == pbc && pbc == pca ? true : false;
        }
        return false;
    }

    std::vector<segment_type> ear_clipping(std::list<point_type> pts) {

        std::vector<segment_type> edges;

        pts.erase(boost::unique<boost::return_found>(pts), pts.end());

        dc_list list;
        list.assign(pts.begin(), pts.end());
        add_edges(edges, list);

        auto default_turn = get_default_turn(list);

        auto v_iter = list.begin();
        while (list.size() > 3) {
            auto a(*v_iter.prev()), b(*v_iter), c(*v_iter.next());
            v_iter = v_iter.next();

            if (predicates::turn(a, b, c) != default_turn) {
                continue;
            }

            bool is_ear = true;
            for (auto p : list) {
                if (p != a && p != b && p != c && is_inside_triangle(p, a, b, c)) {
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


