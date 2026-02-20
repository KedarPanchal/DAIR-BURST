#ifndef TYPES_HPP
#define TYPES_HPP

#include <CGAL/Cartesian.h>
#include <CGAL/CORE_algebraic_number_traits.h>
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
    namespace bmp = boost::multiprecision;
    constexpr unsigned int HP_PRECISION = 100; // 100 decimal digits of precision for high-precision scalar type
    
    // Scalar/numeric types
    using fscalar = AlgebraicKernel::FT;
    using rscalar = RationalKernel::FT;
    using hpscalar = bmp::number<bmp::mpfr_float_backend<HP_PRECISION>>; 
    
    // Geometric types
    template <typename Kernel>
    using Point2D = typename Kernel::Point_2;
    template <typename Kernel>
    using Segment2D = typename Kernel::Segment_2;
    template <typename Kernel>
    using Line2D = typename Kernel::Line_2;
    template <typename Kernel>
    using Ray2D = typename Kernel::Ray_2;
    using Polygon2D = CGAL::Polygon_2<RationalKernel>;
    using ClosedCurve2D = CGAL::Arrangement_2<ConicTraits>;
    using winding_order = CGAL::Orientation;
    using Vector2D = CGAL::Vector_2<AlgebraicKernel>;

    // Iterator types
    template <typename Shape>
    using vertex_iterator = typename Shape::Vertex_const_iterator;
    template <typename Shape>
    using halfedge_iterator = typename Shape::Halfedge_const_iterator;
    template <typename Shape>
    using edge_iterator = typename Shape::Edge_const_iterator;
    
    // Rendering types
    using scene = CGAL::Graphics_scene;
    template <typename Shape, typename VertexIter = vertex_iterator<Shape>, typename EdgeIter = edge_iterator<Shape>, typename FaceIter = void*>
    using shape_options = CGAL::Graphics_scene_options<Shape, VertexIter, EdgeIter, FaceIter>;
    using polygon_options = shape_options<Polygon2D>;
    using closed_curve_options = shape_options<ClosedCurve2D, vertex_iterator<ClosedCurve2D>, edge_iterator<ClosedCurve2D>, ClosedCurve2D::Face_const_handle>;
    using color = CGAL::IO::Color;

}
#endif
