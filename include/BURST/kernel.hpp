#ifndef BURST_KERNEL_TYPES_HPP
#define BURST_KERNEL_TYPES_HPP

#include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>
#include <CGAL/Gps_segment_traits_2.h>
#include <CGAL/Gps_circle_segment_traits_2.h>

namespace BURST {

    /**
     * @brief Exact geometric kernel used throughout BURST.
     *
     * Provides exact predicates and exact constructions (including square roots), which keeps
     * incidence and orientation tests reliable when composing CGAL algorithms on polygon sets,
     * offsets, and arrangements.
     */
    using Kernel = CGAL::Exact_predicates_exact_constructions_kernel_with_sqrt;

    /**
     * @brief Traits for linear (segment) geometry in generalized polygon set operations.
     *
     * Use this when working with straight-line polygons and boolean operations that only involve
     * segments under the shared @ref Kernel.
     */
    using LinearTraits = CGAL::Gps_segment_traits_2<Kernel>;

    /**
     * @brief Traits for circular arcs and segments in generalized polygon set operations.
     *
     * Enables curvilinear boundaries (e.g. circular arcs) while remaining consistent with
     * @ref Kernel number types and predicates.
     */
    using CurvedTraits = CGAL::Gps_circle_segment_traits_2<Kernel>;
}

#endif
