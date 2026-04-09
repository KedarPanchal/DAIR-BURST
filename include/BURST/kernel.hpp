#ifndef BURST_KERNEL_TYPES_HPP
#define BURST_KERNEL_TYPES_HPP

#include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>
#include <CGAL/Gps_segment_traits_2.h>
#include <CGAL/Gps_circle_segment_traits_2.h>
#include <CGAL/Graphics_scene.h>
#include <CGAL/IO/Color.h>

namespace BURST {

    // Top-level kernel
    using Kernel = CGAL::Exact_predicates_exact_constructions_kernel_with_sqrt;
    using LinearTraits = CGAL::Gps_segment_traits_2<Kernel>;
    using CurvedTraits = CGAL::Gps_circle_segment_traits_2<Kernel>;

    // Graphics types
    using Scene = CGAL::Graphics_scene;
    using Color = CGAL::IO::Color;
}

#endif
