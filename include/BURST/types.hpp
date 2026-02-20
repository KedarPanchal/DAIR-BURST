#ifndef TYPES_HPP
#define TYPES_HPP

#include <CGAL/Cartesian.h>
#include <CGAL/CORE_algebraic_number_traits.h>
#include <CGAL/Gps_traits_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Arr_conic_traits_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Graphics_scene.h>
#include <CGAL/Basic_viewer.h>
#include <CGAL/enum.h>
#include <CGAL/Graphics_scene_options.h>
#include <CGAL/IO/Color.h>

#include <boost/multiprecision/mpfr.hpp>

namespace BURST {
    // Top-level
    using NumberTraits = CGAL::CORE_algebraic_number_traits;
    using AlgebraicKernel = CGAL::Cartesian<NumberTraits::Algebraic>; // Used for computation
    using RationalKernel = CGAL::Cartesian<NumberTraits::Rational>; // Used for polygon construction
    using ConicTraits = CGAL::Arr_conic_traits_2<RationalKernel, AlgebraicKernel, NumberTraits>; // Used for creating curved polygons
    using GeneralPolygonTraits = CGAL::Gps_traits_2<ConicTraits>; // Used for polygon operations on curved polygons
    namespace bmp = boost::multiprecision;
    constexpr unsigned int HP_PRECISION = 100; // 100 decimal digits of precision for high-precision scalar type
    
    // Scalar/numeric types
    using fscalar = AlgebraicKernel::FT;
    using rscalar = RationalKernel::FT;
    using hpscalar = bmp::number<bmp::mpfr_float_backend<HP_PRECISION>>; 
    
    // Geometric types
    using Point2D = AlgebraicKernel::Point_2;
    using Segment2D = AlgebraicKernel::Segment_2;
    using Line2D = AlgebraicKernel::Line_2;
    using Ray2D = AlgebraicKernel::Ray_2;
    using Polygon2D = CGAL::Polygon_2<RationalKernel>;
    using CurvedPolygon2D = GeneralPolygonTraits::Polygon_2;
    using CurvedPolygonArrangement2D = CGAL::Arrangement_2<ConicTraits>;
    using winding_order = CGAL::Orientation;
    using Vector2D = CGAL::Vector_2<AlgebraicKernel>;

    // Iterator types
    using vertex_iterator = Polygon2D::Vertex_const_iterator;
    using edge_iterator = Polygon2D::Edge_const_iterator;
    using curve_iterator = CurvedPolygon2D::Curve_const_iterator;
    
    // Rendering types
    using scene = CGAL::Graphics_scene;
    using polygon_options = CGAL::Graphics_scene_options<Polygon2D, edge_iterator, edge_iterator, void*>;
    using curved_polygon_options = CGAL::Graphics_scene_options<
        CurvedPolygonArrangement2D,
        CurvedPolygonArrangement2D::Vertex_const_handle,
        CurvedPolygonArrangement2D::Halfedge_const_handle,
        CurvedPolygonArrangement2D::Face_const_handle
    >;
    using color = CGAL::IO::Color;

}
#endif
