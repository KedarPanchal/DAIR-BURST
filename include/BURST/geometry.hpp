#ifndef BURST_GEOMETRIC_TYPES_HPP
#define BURST_GEOMETRIC_TYPES_HPP

#include <concepts>
#include <initializer_list>
#include <numeric>
#include <optional>
#include <ranges>

#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Vector_2.h>
#include <CGAL/Bbox_2.h>
#include <CGAL/General_polygon_set_2.h>
#include <CGAL/enum.h>
#include <CGAL/Aff_transformation_2.h>

#include <boost/container/small_vector.hpp>

#include "kernel.hpp"
#include "numeric.hpp"
#include "logging.hpp"

namespace BURST::geometry {

    // -- GEOMETRIC TYPES ------------------------------------------------------
    
    // Primitive types
    using Point2D = Kernel::Point_2;
    using Segment2D = Kernel::Segment_2;
    using Line2D = Kernel::Line_2;
    using Ray2D = Kernel::Ray_2;
    using MonotoneCurve2D = CurvedTraits::X_monotone_curve_2;
    using Polygon2D = CGAL::Polygon_2<Kernel>;
    using HoledPolygon2D = CGAL::Polygon_with_holes_2<Kernel>;
    using CurvilinearPolygon2D = CurvedTraits::General_polygon_2;
    using HoledCurvilinearPolygon2D = CGAL::General_polygon_with_holes_2<CurvilinearPolygon2D>;
    using Vector2D = CGAL::Vector_2<Kernel>;
    
    // Composite/complex types
    using CurvilinearPolygonSet2D = CGAL::General_polygon_set_2<CurvedTraits>;
    using LinearPolygonSet2D = CGAL::General_polygon_set_2<LinearTraits>;
    using BoundingBox2D = CGAL::Bbox_2;
    using WindingOrder = CGAL::Orientation;

    // Iterator types
    using vertex_iterator = Polygon2D::Vertex_const_iterator;
    using edge_iterator = Polygon2D::Edge_const_iterator;

    // Transformation types
    using Transformation = CGAL::Aff_transformation_2<Kernel>;


    // -- GEOMETRIC CONCEPTS ---------------------------------------------------

    // Checks if a type is a valid path type, which can be constructed from a start and end point
    // It can either be a Segment2D or a curve type that's convertible to an X_monotone_curve_2
    template <typename T>
    concept valid_path_type = requires (geometry::Point2D start, geometry::Point2D end) {
        T{start, end};
    } && (std::same_as<Segment2D, T> || std::convertible_to<T, CurvedTraits::X_monotone_curve_2>);
    
    // Checks if a type is a valid trajectory type, which can be constructed from an origin point and a direction vector
    template <typename T>
    concept valid_trajectory_type = requires (geometry::Point2D origin, geometry::Vector2D direction) {
        T{origin, direction};
    };

    // Checks is a collection is a valid input collection for constructing a polygon
    // Prior to C++23, std::initializer_list doesn't satisfy the condition for sized_range, so an explicit check is included here
    template <typename C, typename V>
    concept valid_geometric_collection = 
        std::ranges::sized_range<C> && std::same_as<std::remove_cv_t<std::ranges::range_value_t<C>>, V> ||
        std::same_as<C, std::initializer_list<V>>;


    // -- HELPER FUNCTIONS -----------------------------------------------------

    // Utility function to construct a polygon off of any collection of points
    template <valid_geometric_collection<Point2D> C>
    std::optional<Polygon2D> construct_polygon(const C& points, CGAL::Orientation expected_orientation = CGAL::COUNTERCLOCKWISE) {
        // Can't make a polygon with 2 or fewer points
        if (std::ranges::size(points) <= 2) {
            BURST_ERROR("Cannot construct a polygon with 2 or fewer points, collection is degenerate");
            return std::nullopt;
        }

        // Check for self-intersection, overall simplicity, and non-degeneracy of the polygon and return nullopt if any of these conditions are violated
        if (!CGAL::is_simple_2(std::ranges::begin(points), std::ranges::end(points), LinearTraits{})) {
            BURST_ERROR("Cannot construct a polygon from the given collection of points for one of the following reasons: the polygon is self-intersecting, not simple, or degenerate");
            return std::nullopt;
        }

        // Create the polygon from the input points and return it
        Polygon2D polygon{std::ranges::begin(points), std::ranges::end(points)};
        // If the polygon is not oriented as expected, reverse the orientation to ensure it's a valid polygon for CGAL
        if (polygon.orientation() != expected_orientation) polygon.reverse_orientation();

        return polygon;
    }

    inline std::optional<Polygon2D> construct_polygon(const std::initializer_list<Point2D>& points, CGAL::Orientation expected_orientation = CGAL::COUNTERCLOCKWISE) {
        return construct_polygon<std::initializer_list<Point2D>>(points, expected_orientation);
    }

    template <valid_geometric_collection<Point2D> C>
    std::optional<Point2D> average(const C& points) {
        // Can't compute the average of an empty collection
        if (std::ranges::size(points) == 0) {
            BURST_ERROR("Cannot compute the average of an empty collection of points");
            return std::nullopt;
        }

        Point2D sum = std::accumulate(points.begin(), points.end(), Point2D{0, 0}, [](const Point2D& acc, const Point2D& point) {
            return Point2D{acc.x() + point.x(), acc.y() + point.y()};
        });
        return Point2D{sum.x() / std::ranges::size(points), sum.y() / std::ranges::size(points)};
    }

    inline std::optional<Point2D> average(const std::initializer_list<Point2D>& points) {
        return average<std::initializer_list<Point2D>>(points);
    }

    template <typename T, typename F>
        requires requires(const F& from_point) {
            T{from_point.x(), from_point.y()};
        } 
    T convert_point(const F& from_point) {
        return T{from_point.x(), from_point.y()};
    }

    template <typename T, typename F, typename ConvertFn>
        requires requires(const F& from_point, const ConvertFn& convert_fn) {
            T{convert_fn(from_point.x()), convert_fn(from_point.y())};
        } 
    T convert_point(const F& from_point, const ConvertFn& convert_fn) {
        return T{convert_fn(from_point.x()), convert_fn(from_point.y())};
    }
 
    template <valid_path_type P>
    MonotoneCurve2D construct_curve(const P& path) {
        if constexpr (std::same_as<P, Segment2D>) {
            return CurvedTraits::X_monotone_curve_2{path.source(), path.target()};
        } else {
            return static_cast<CurvedTraits::X_monotone_curve_2>(path); 
        }
    }
       
    inline std::optional<CurvilinearPolygon2D> construct_circle(const numeric::fscalar& radius, const Point2D& center) {
        // Only construct circles with positive radius
        if (radius <= 0) {
            BURST_ERROR("Cannot construct a circle with non-positive radius");
            return std::nullopt;
        }
        CGAL::Circle_2<Kernel> circle = CGAL::Circle_2<Kernel>{center, radius * radius};

        boost::container::small_vector<CurvedTraits::X_monotone_curve_2, 2> semicircles{
            MonotoneCurve2D{circle, CurvedTraits::Point_2{center.x() - radius, center.y()}, CurvedTraits::Point_2{center.x() + radius, center.y()}, CGAL::COUNTERCLOCKWISE},
            MonotoneCurve2D{circle, CurvedTraits::Point_2{center.x() + radius, center.y()}, CurvedTraits::Point_2{center.x() - radius, center.y()}, CGAL::COUNTERCLOCKWISE}
        };
        
        return CurvilinearPolygon2D{semicircles.begin(), semicircles.end()};
    }

    inline Point2D midpoint(const Point2D& a, const Point2D& b) {
        return Point2D{(a.x() + b.x())/2, (a.y() + b.y())/2};
    }

}

#endif
