#ifndef FUNCTIONS_HPP
#define FUNCTIONS_HPP

#include <CGAL/Arrangement_2.h>
#include <CGAL/Arr_naive_point_location.h>

#include <sstream>

#include "types.hpp"

namespace BURST {

    // Type conversion functions
    template <typename FT>
    hpscalar to_high_precision(const FT& value) {
        std::stringstream ss;
        ss << std::setprecision(HP_PRECISION) << value;
        return hpscalar(ss.str());
    }

    // Polygon utility functions
    inline bool curved_has_point(const ClosedCurve2D& closed_curve, const AlgebraicPoint2D& point) {
        // Create a point location object for the arrangement
        CGAL::Arr_naive_point_location<ClosedCurve2D> point_location{closed_curve};
        
        // Locate the point and check if it's on the boundary or vertex of the polygon
        auto location = point_location.locate(point);
        return std::holds_alternative<halfedge_iterator<ClosedCurve2D>>(location)
            || std::holds_alternative<vertex_iterator<ClosedCurve2D>>(location);
    }
    
    inline bool curved_has_point(const edge_iterator<ClosedCurve2D>& edge, const AlgebraicPoint2D& point) {
        ConicTraits traits;
        auto compare_x = traits.compare_x_2_object();
        auto compare_y_at_x = traits.compare_y_at_x_2_object();

        // Check if the point is within the x-range of the edge
        if (compare_x(edge->curve().left(), point) == CGAL::SMALLER || compare_x(edge->curve().right(), point) == CGAL::LARGER) return false;
        // Check if the point is on the edge
        return compare_y_at_x(point, edge->curve()) == CGAL::EQUAL;
    }

}

#endif
