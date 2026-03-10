#ifndef BURST_GEOMETRIC_TYPES_HPP
#define BURST_GEOMETRIC_TYPES_HPP

#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Vector_2.h>
#include <CGAL/Bbox_2.h>
#include <CGAL/General_polygon_set_2.h>
#include <CGAL/enum.h>
#include <CGAL/Aff_transformation_2.h>
#include <initializer_list>

#include "kernel_types.hpp"

namespace BURST::geometry {

    // -- GEOMETRIC TYPES ------------------------------------------------------
    
    // Primitive types
    using Point2D = Kernel::Point_2;
    using Segment2D = Kernel::Segment_2;
    using Line2D = Kernel::Line_2;
    using Ray2D = Kernel::Ray_2;
    using Polygon2D = CGAL::Polygon_2<Kernel>;
    using HoledPolygon2D = CGAL::Polygon_with_holes_2<Kernel>;
    using CurvilinearPolygon2D = CurvedTraits::General_polygon_2;
    using HoledCurvilinearPolygon2D = CGAL::General_polygon_with_holes_2<CurvilinearPolygon2D>;
    using Vector2D = CGAL::Vector_2<Kernel>;
    
    // Composite/complex types
    using CurvilinearPolygonSet2D = CGAL::General_polygon_set_2<CurvedTraits>;
    using BoundingBox2D = CGAL::Bbox_2;
    using WindingOrder = CGAL::Orientation;

    // Iterator types
    using vertex_iterator = Polygon2D::Vertex_const_iterator;
    using edge_iterator = Polygon2D::Edge_const_iterator;

    // Transformation types
    using Transformation = CGAL::Aff_transformation_2<Kernel>;


    // -- GEOMETRIC CONCEPTS ---------------------------------------------------

    // Checks if a type is a valid path type, which can be constructed from a start and end point
    template <typename T>
    concept valid_path_type = requires (geometry::Point2D start, geometry::Point2D end) {
        T{start, end};
    };
    
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
    std::optional<Polygon2D> construct_polygon(C points) {
        // Can't make a polygon with 2 or fewer points
        if (points.size() <= 2) return std::nullopt; 

        // Create the polygon from the input points and return it
        Polygon2D polygon{points.begin(), points.end()};
        // If the polygon is not oriented counterclockwise, reverse the orientation to ensure it's a valid polygon for CGAL
        if (polygon.orientation() != CGAL::COUNTERCLOCKWISE) polygon.reverse_orientation();

        // Check for self-intersection, overall simplicity, and non-degeneracy of the polygon and return nullopt if any of these conditions are violated
        return CGAL::is_valid_polygon(polygon, LinearTraits{}) ? std::optional<Polygon2D>{polygon} : std::nullopt;
    }

}

#endif
