#ifndef KERNEL_TYPES_HPP
#define KERNEL_TYPES_HPP

#include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>
#include <CGAL/Gps_circle_segment_traits_2.h>

namespace BURST {

    // Top-level kernel
    using Kernel = CGAL::Exact_predicates_exact_constructions_kernel_with_sqrt;
    using CurvedTraits = CGAL::Gps_circle_segment_traits_2<Kernel>;

}

#endif
