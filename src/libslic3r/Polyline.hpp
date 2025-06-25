#ifndef slic3r_Polyline_hpp_
#define slic3r_Polyline_hpp_

#include "libslic3r.h"
#include "Line.hpp"
#include "MultiPoint.hpp"
#include <string>
#include <vector>
//BBS: new necessary header file
#include "ArcFitter.hpp"

namespace Slic3r {

class Polyline;
class ThickPolyline;
typedef std::vector<Polyline> Polylines;
typedef std::vector<ThickPolyline> ThickPolylines;

class Polyline : public MultiPoint {
public:
    Polyline() {};
    Polyline(const Polyline& other) : MultiPoint(other.points), fitting_result(other.fitting_result) {}
    Polyline(Polyline &&other) noexcept : MultiPoint(std::move(other.points)), fitting_result(std::move(other.fitting_result))  {}
    Polyline(std::initializer_list<Point> list) : MultiPoint(list) {
        fitting_result.clear();
    }
    explicit Polyline(const Point &p1, const Point &p2) {
        points.reserve(2);
        points.emplace_back(p1);
        points.emplace_back(p2);
        fitting_result.clear();
    }
    explicit Polyline(const Points &points) : MultiPoint(points) {
        fitting_result.clear();
    }
    // explicit Polyline(const Points3 &points) {
    //     this->points.reserve(points.size());
    //     for (const Point3 &point : points) {
    //         this->points.emplace_back(point.x(), point.y());
    //     }
    //     fitting_result.clear();
    // }
    explicit Polyline(Points &&points) : MultiPoint(std::move(points)) {
        fitting_result.clear();
    }
    Polyline& operator=(const Polyline& other) {
        points = other.points;
        fitting_result = other.fitting_result;
        return *this;
    }
    Polyline& operator=(Polyline&& other) {
        points = std::move(other.points);
        fitting_result = std::move(other.fitting_result);
        return *this;
    }
	static Polyline new_scale(const std::vector<Vec2d> &points) {
		Polyline pl;
		pl.points.reserve(points.size());
		for (const Vec2d &pt : points)
			pl.points.emplace_back(Point::new_scale(pt(0), pt(1)));
        //BBS: new_scale doesn't support arc, so clean
        pl.fitting_result.clear();
		return pl;
    }

    void append(const Point &point) {
        //BBS: don't need to append same point
        if (!this->empty() && this->last_point() == point)
            return;
        MultiPoint::append(point);
        append_fitting_result_after_append_points();
    }

    void append_before(const Point& point) {
        //BBS: don't need to append same point
        if (!this->empty() && this->first_point() == point)
            return;
        if (this->size() == 1) {
            this->fitting_result.clear();
            MultiPoint::append(point);
            MultiPoint::reverse();
        } else {
            this->reverse();
            this->append(point);
            this->reverse();
        }
    }

    void append(const Points &src) {
        //BBS: don't need to append same point
        if (!this->empty() && !src.empty() && this->last_point() == src[0])
            this->append(src.begin() + 1, src.end());
        else
            this->append(src.begin(), src.end());
    }
    void append(const Points3 &src) {
        //BBS: don't need to append same point
        if (!this->empty() && !src.empty() && this->last_point() == src[0].to_point())
            this->append(src.begin() + 1, src.end());
        else
            this->append(src.begin(), src.end());
    }
    void append(const Points::const_iterator &begin, const Points::const_iterator &end) {
        //BBS: don't need to append same point
        if (!this->empty() && begin != end && this->last_point() == *begin)
            MultiPoint::append(begin + 1, end);
        else
            MultiPoint::append(begin, end);
        append_fitting_result_after_append_points();
    }
    void append(const Points3::const_iterator &begin, const Points3::const_iterator &end) {
        //BBS: don't need to append same point
        if (!this->empty() && begin != end && this->last_point() == begin->to_point())
            MultiPoint::append(begin + 1, end);
        else
            MultiPoint::append(begin, end);
        append_fitting_result_after_append_points();
    }
    void append(Points &&src)
    {
        MultiPoint::append(std::move(src));
        append_fitting_result_after_append_points();
    }
    void append(const Polyline& src);
    void append(const Polyline3& src);
    void append(Polyline&& src);

    Polyline rebase_at(size_t idx);

    Point& operator[](Points::size_type idx) { return this->points[idx]; }
    const Point& operator[](Points::size_type idx) const { return this->points[idx]; }

    const Point& last_point() const override { return this->points.back(); }
    const Point& leftmost_point() const;
    Lines lines() const override;

    void clear() { MultiPoint::clear(); this->fitting_result.clear(); }
    void reverse();
    void clip_end(double distance);
    void clip_start(double distance);
    void extend_end(double distance);
    void extend_start(double distance);
    Points equally_spaced_points(double distance) const;
    void simplify(double tolerance);
//    template <class T> void simplify_by_visibility(const T &area);
    void split_at(Point &point, Polyline* p1, Polyline* p2) const;
    bool split_at_index(const size_t index, Polyline* p1, Polyline* p2) const;
    bool split_at_length(const double length, Polyline *p1, Polyline *p2) const;
    bool is_straight() const;
    bool is_closed() const { return this->points.front() == this->points.back(); }

    //BBS: store arc fitting result
    std::vector<PathFittingData> fitting_result;
    //BBS: simplify points by arc fitting
    void simplify_by_fitting_arc(double tolerance);
    //BBS:
    Polylines equally_spaced_lines(double distance) const;

private:
    void append_fitting_result_after_append_points();
    void append_fitting_result_after_append_polyline(const Polyline& src);
    void reset_to_linear_move();
    bool split_fitting_result_before_index(const size_t index, Point &new_endpoint, std::vector<PathFittingData>& data) const;
    bool split_fitting_result_after_index(const size_t index, Point &new_startpoint, std::vector<PathFittingData>& data) const;
};

inline bool operator==(const Polyline &lhs, const Polyline &rhs) { return lhs.points == rhs.points; }
inline bool operator!=(const Polyline &lhs, const Polyline &rhs) { return lhs.points != rhs.points; }

// Don't use this class in production code, it is used exclusively by the Perl binding for unit tests!
#ifdef PERL_UCHAR_MIN
class PolylineCollection
{
public:
    Polylines polylines;
};
#endif /* PERL_UCHAR_MIN */

extern BoundingBox get_extents(const Polyline &polyline);
extern BoundingBox get_extents(const Polylines &polylines);

// Return True when erase some otherwise False.
bool remove_same_neighbor(Polyline &polyline);
bool remove_same_neighbor(Polylines &polylines);

inline double total_length(const Polylines &polylines) {
    double total = 0;
    for (const Polyline &pl : polylines)
        total += pl.length();
    return total;
}

inline Lines to_lines(const Polyline &poly)
{
    Lines lines;
    if (poly.points.size() >= 2) {
        lines.reserve(poly.points.size() - 1);
        for (Points::const_iterator it = poly.points.begin(); it != poly.points.end()-1; ++it)
            lines.push_back(Line(*it, *(it + 1)));
    }
    return lines;
}

inline Lines to_lines(const Polylines &polys)
{
    size_t n_lines = 0;
    for (size_t i = 0; i < polys.size(); ++ i)
        if (polys[i].points.size() > 1)
            n_lines += polys[i].points.size() - 1;
    Lines lines;
    lines.reserve(n_lines);
    for (size_t i = 0; i < polys.size(); ++ i) {
        const Polyline &poly = polys[i];
        for (Points::const_iterator it = poly.points.begin(); it != poly.points.end()-1; ++it)
            lines.push_back(Line(*it, *(it + 1)));
    }
    return lines;
}

inline Polylines to_polylines(const std::vector<Points> &paths)
{
    Polylines out;
    out.reserve(paths.size());
    for (const Points &path : paths)
        out.emplace_back(path);
    return out;
}

inline Polylines to_polylines(std::vector<Points> &&paths)
{
    Polylines out;
    out.reserve(paths.size());
    for (const Points &path : paths)
        out.emplace_back(std::move(path));
    return out;
}

inline void polylines_append(Polylines &dst, const Polylines &src)
{
    dst.insert(dst.end(), src.begin(), src.end());
}

inline void polylines_append(Polylines &dst, Polylines &&src)
{
    if (dst.empty()) {
        dst = std::move(src);
    } else {
        std::move(std::begin(src), std::end(src), std::back_inserter(dst));
        src.clear();
    }
}

// Merge polylines at their respective end points.
// dst_first: the merge point is at dst.begin() or dst.end()?
// src_first: the merge point is at src.begin() or src.end()?
// The orientation of the resulting polyline is unknown, the output polyline may start
// either with src piece or dst piece.
template<typename PointsType>
inline void polylines_merge(PointsType &dst, bool dst_first, PointsType &&src, bool src_first)
{
    if (dst_first) {
        if (src_first)
            std::reverse(dst.begin(), dst.end());
        else
            std::swap(dst, src);
    } else if (! src_first)
        std::reverse(src.begin(), src.end());
    // Merge src into dst.
    append(dst, std::move(src));
}

const Point& leftmost_point(const Polylines &polylines);

bool remove_degenerate(Polylines &polylines);

// Returns index of a segment of a polyline and foot point of pt on polyline.
std::pair<int, Point> foot_pt(const Points &polyline, const Point &pt);
std::pair<int, Point> foot_pt(const Points3 &polyline, const Point &pt);

class Polyline3 : public MultiPoint3
{
public:
    Polyline3() {};
    Polyline3(const Polyline3& other) : MultiPoint3(other.points), fitting_result(other.fitting_result) {}
    Polyline3(Polyline3 &&other) noexcept : MultiPoint3(std::move(other.points)), fitting_result(std::move(other.fitting_result))  {}
    Polyline3(std::initializer_list<Point3> list) : MultiPoint3(list) {
        fitting_result.clear();
    }
    explicit Polyline3(const Point3 &p1, const Point3 &p2) {
        points.reserve(2);
        points.emplace_back(p1);
        points.emplace_back(p2);
        fitting_result.clear();
    }
    explicit Polyline3(const Point &p1, const Point &p2) {
        points.reserve(2);
        points.emplace_back(p1.x(), p1.y(), 0);
        points.emplace_back(p2.x(), p2.y(), 0);
        fitting_result.clear();
    }
    explicit Polyline3(const Points3 &points) : MultiPoint3(points) {
        fitting_result.clear();
    }
    explicit Polyline3(Points3 &&points) : MultiPoint3(std::move(points)) {
        fitting_result.clear();
    }
    explicit Polyline3(const Polyline &other) {
        this->points.reserve(other.points.size());
        for (const Point &point : other.points) {
            this->points.emplace_back(point.x(), point.y(), 0);
        }
        fitting_result.clear();
    }
    Polyline3& operator=(const Polyline3& other) {
        points = other.points;
        fitting_result = other.fitting_result;
        return *this;
    }
    Polyline3& operator=(Polyline3&& other) {
        points = std::move(other.points);
        fitting_result = std::move(other.fitting_result);
        return *this;
    }

    static Polyline3 new_scale(const std::vector<Vec3d> &points) {
		Polyline3 pl;
		pl.points.reserve(points.size());
		for (const Vec3d &pt : points)
			pl.points.emplace_back(Point3::new_scale(pt(0), pt(1), pt(2)));
        //BBS: new_scale doesn't support arc, so clean
        pl.fitting_result.clear();
		return pl;
    }

    void append(const Point3 &point) {
        //BBS: don't need to append same point
        if (!this->empty() && this->last_point() == point)
            return;
        MultiPoint3::append(point);
        append_fitting_result_after_append_points();
    }

    void append(const Point &point) {
        this->append(Point3(point, 0));
    }

    void append_before(const Point3& point) {
        //BBS: don't need to append same point
        if (!this->empty() && this->first_point() == point)
            return;
        if (this->size() == 1) {
            this->fitting_result.clear();
            MultiPoint3::append(point);
            MultiPoint3::reverse();
        } else {
            this->reverse();
            this->append(point);
            this->reverse();
        }
    }

    void append(const Points3 &src) {
        //BBS: don't need to append same point
        if (!this->empty() && !src.empty() && this->last_point() == src[0])
            this->append(src.begin() + 1, src.end());
        else
            this->append(src.begin(), src.end());
    }
    void append(const Points &src) {
        this->points.reserve(this->points.size() + src.size());
        for (const Point &point : src) {
            this->points.emplace_back(point.x(), point.y(), 0);
        }
    }
    void append(const Points3::const_iterator &begin, const Points3::const_iterator &end) {
        //BBS: don't need to append same point
        if (!this->empty() && begin != end && this->last_point() == *begin)
            MultiPoint3::append(begin + 1, end);
        else
            MultiPoint3::append(begin, end);
        append_fitting_result_after_append_points();
    }
    void append(Points3 &&src)
    {
        MultiPoint3::append(std::move(src));
        append_fitting_result_after_append_points();
    }
    void append(const Polyline3& src);
    void append(Polyline3&& src);

    void append(const Polyline& src);

    Polyline rebase_at(size_t idx);
    Point3& operator[](Points::size_type idx) { return this->points[idx]; }
    const Point3& operator[](Points::size_type idx) const { return this->points[idx]; }

    const Point3& last_point() const override { return this->points.back(); }
    const Point3& leftmost_point() const;
    Lines3 lines() const override;

    void clear() { MultiPoint3::clear(); this->fitting_result.clear(); }
    void reverse();
    void clip_end(double distance);
    void clip_start(double distance);
    void extend_end(double distance);
    void extend_start(double distance);
    Points equally_spaced_points(double distance) const;
    void simplify(double tolerance);
    void split_at(Point &point, Polyline3* p1, Polyline3* p2) const;
    void split_at(Point3 &point, Polyline3* p1, Polyline3* p2) const;
    bool split_at_index(const size_t index, Polyline3* p1, Polyline3* p2) const;
    bool split_at_length(const double length, Polyline3 *p1, Polyline3 *p2) const;
    bool is_straight() const;
    bool is_closed() const { return this->points.front() == this->points.back(); }
    
    
    //BBS: store arc fitting result
    std::vector<PathFittingData> fitting_result;
    //BBS: simplify points by arc fitting
    void simplify_by_fitting_arc(double tolerance);
    //BBS:
    Polylines equally_spaced_lines(double distance) const;

    Polyline to_polyline() const;

private:
    void append_fitting_result_after_append_points();
    void append_fitting_result_after_append_polyline(const Polyline3& src);
    void reset_to_321linear_move();
    bool split_fitting_result_before_index(const size_t index, Point3 &new_endpoint, std::vector<PathFittingData> &data) const;
    bool split_fitting_result_after_index(const size_t index, Point3 &new_startpoint, std::vector<PathFittingData> &data) const;
};

class ThickPolyline : public Polyline {
public:
    ThickPolyline() : endpoints(std::make_pair(false, false)) {}
    ThickLines thicklines() const;
    void reverse() {
        Polyline::reverse();
        std::reverse(this->width.begin(), this->width.end());
        std::swap(this->endpoints.first, this->endpoints.second);
    }
    void clear() {
        Polyline::clear();
        width.clear();
    }
    ThickPolyline rebase_at(size_t idx);
    coordf_t get_width_at(size_t point_idx) const;

    std::vector<coordf_t> width;
    std::pair<bool,bool>  endpoints;
};

typedef std::vector<Polyline3> Polylines3;

inline ThickPolylines to_thick_polylines(Polylines&& polylines, const coordf_t width)
{
    ThickPolylines out;
    out.reserve(polylines.size());
    for (Polyline& polyline : polylines) {
        out.emplace_back();
        out.back().width.assign((polyline.points.size() - 1) * 2, width);
        out.back().points = std::move(polyline.points);
    }
    return out;
}



}

#endif
