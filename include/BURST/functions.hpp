#ifndef FUNCTIONS_HPP
#define FUNCTIONS_HPP

#include <CGAL/Arrangement_2.h>
#include <CGAL/Arr_naive_point_location.h>

#include <variant>
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
    inline bool curved_has_point(const curve_iterator& start, const curve_iterator& end, const Point2D& point) {
        // Create an arrangement to contain the polygon
        CGAL::Arrangement_2<ConicTraits> arrangement;
        CGAL::insert(arrangement, start, end);

        // Create a point location object for the arrangement
        CGAL::Arr_naive_point_location<CGAL::Arrangement_2<ConicTraits>> point_location{arrangement};
        
        // Locate the point and check if it's on the boundary or vertex of the polygon
        auto location = point_location.locate(point);
        return std::holds_alternative<CGAL::Arrangement_2<ConicTraits>::Halfedge_const_handle>(location)
            || std::holds_alternative<CGAL::Arrangement_2<ConicTraits>::Vertex_const_handle>(location);
    }

    inline bool curved_has_point(const curve_iterator& edge, const Point2D& point) {
        static ConicTraits traits;
        return traits.contains_point(*edge, point);
    }

}

#endif
