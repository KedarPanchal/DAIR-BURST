#ifndef TYPES_HPP
#define TYPES_HPP

#include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Vector_2.h>
#include <CGAL/Aff_transformation_2.h>
#include <CGAL/Graphics_scene.h>
#include <CGAL/Basic_viewer.h>
#include <CGAL/enum.h>
#include <CGAL/Graphics_scene_options.h>
#include <CGAL/IO/Color.h>

#include <boost/multiprecision/mpfr.hpp>

#include <sstream>
#include <limits.h>

namespace BURST {
    // Top-level
    using Kernel = CGAL::Exact_predicates_exact_constructions_kernel_with_sqrt;
    namespace bmp = boost::multiprecision;
    constexpr unsigned int HP_PRECISION = 100; // 100 decimal digits of precision for high-precision scalar type
    
    // Scalar/numeric types
    using fscalar = Kernel::FT;
    using rscalar = Kernel::RT;
    using hpscalar = bmp::number<bmp::mpfr_float_backend<HP_PRECISION>>; 
    
    // Geometric types
    using Point_2 = Kernel::Point_2;
    using Segment_2 = Kernel::Segment_2;
    using Line_2 = Kernel::Line_2;
    using Ray_2 = Kernel::Ray_2;
    using Polygon_2 = CGAL::Polygon_2<Kernel>;
    using winding_order = CGAL::Orientation;
    using Vector_2 = CGAL::Vector_2<Kernel>;

    // Iterator types
    using vertex_iterator = Polygon_2::Vertex_const_iterator;
    using edge_iterator = Polygon_2::Edge_const_iterator;
    
    // Transformation types
    using Transformation = CGAL::Aff_transformation_2<Kernel>;
    
    // Rendering types
    using scene = CGAL::Graphics_scene;
    using polygon_options = CGAL::Graphics_scene_options<Polygon_2, vertex_iterator, vertex_iterator, void*>;
    using color = CGAL::IO::Color;

    // Type conversion functions
    template <typename FT>
    hpscalar to_high_precision(const FT& value) {
        std::ostringstream str_representation;
        str_representation << std::setprecision(100) << value; // 100-decimal precision string
        return hpscalar{str_representation.str()}; // Construct high-precision scalar from string
    }
}
#endif
