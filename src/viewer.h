#include "io/point.h"

#include "triangulation.h"
#include "visualization/viewer_adapter.h"
#include <boost/optional/optional.hpp>

using namespace visualization;
using geom::structures::point_type;
using geom::structures::segment_type;
using geom::structures::contour_type;

struct triangulation_viewer : viewer_adapter
{
    void draw(drawer_type & drawer)     const;
    void print(printer_type & printer)  const;

    bool on_double_click(point_type const & pt);
    bool on_move(point_type const & pt);
    bool on_key(int key);

private:
    std::list<point_type>                       pts_; 
    std::unique_ptr<std::vector<segment_type>>  segments_;
    boost::optional<point_type>                 last_point_;
    bool                                        draw_trigger_ = false;
};

void triangulation_viewer::draw(drawer_type & drawer) const
{
    drawer.set_color(Qt::green);
    for (point_type const & pt : pts_)
        drawer.draw_point(pt, 3);

    if (!pts_.empty()) 
    {
        drawer.set_color(Qt::red);
        geom::structures::contour_builder_type builder;

        for (auto point : pts_) 
        {
            builder.add_point(point);
        }

        if (!draw_trigger_ && last_point_) 
        {
            builder.add_point(last_point_.get());
        }

        visualization::draw(drawer, builder.get_result());
    }

    if (segments_)
    {
        drawer.set_color(Qt::blue);
        visualization::draw(drawer, *segments_);
    }

}

void triangulation_viewer::print(printer_type & printer) const
{
    printer.corner_stream() << "Points num: " << pts_.size() << endl;
    if (segments_)
        printer.corner_stream() <<"Triangulation edges num: " << segments_->size();
}

bool triangulation_viewer::on_double_click(point_type const & pt)
{
    pts_.push_back(pt);
    segments_.reset();
    draw_trigger_ = false;
    return true;
}

bool triangulation_viewer::on_move(point_type const & pt) 
{
    last_point_ = pt;
    return true;
}

bool triangulation_viewer::on_key(int key)
{
    switch (key)
    {
    case Qt::Key_Return: 
        if (pts_.size() >= 3 && !geom::algorithms::intersection::is_intersect(pts_))
        {
            segments_.reset(new std::vector<segment_type>(
                geom::algorithms::triangulation::ear_clipping(pts_)
            ));
            draw_trigger_ = true;
            return true;
        }
        break;
    case Qt::Key_S:
        {
            std::string filename = QFileDialog::getSaveFileName(
                get_wnd(), 
                "Save Points"
            ).toStdString();
            if (filename != "")
            {
                std::ofstream out(filename.c_str());
                boost::copy(pts_, std::ostream_iterator<point_type>(out, "\n"));
            }
        }
        break;
    case Qt::Key_L:
        {
            std::string filename = QFileDialog::getOpenFileName(
                get_wnd(), 
                "Load Points"
            ).toStdString();
            if (filename != "")
            {
                std::ifstream in(filename.c_str());
                std::istream_iterator<point_type> beg(in), end;
                pts_.assign(beg, end);
                segments_.reset();
                return true;
            }
        }
        break;
    case Qt::Key_D:
        {
            pts_.assign(pts_.end(), pts_.end());
            segments_.reset();
            draw_trigger_ = false;
        }
    }
    return false;
}
