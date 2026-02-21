#ifndef NUMERIC_TYPES_HPP
#define NUMERIC_TYPES_HPP

#include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>
#include <boost/multiprecision/mpfr.hpp>

#include <sstream>

#include "kernel_types.hpp"

// This namespace contains numeric types for CGAL
namespace BURST::numeric {

    // Top-level types and constants
    constexpr unsigned int HP_PRECISION = 100; // 100 decimal digits of precision for high-precision scalar type
                                               
    // Scalar/numeric types
    using fscalar = Kernel::FT;
    using rscalar = Kernel::RT;
    using hpscalar = boost::multiprecision::number<boost::multiprecision::mpfr_float_backend<HP_PRECISION>>;

    // Type conversion functions
    template <typename FT>
    hpscalar to_high_precision(const FT& value) {
        std::ostringstream str_representation;
        str_representation << std::setprecision(100) << value; // 100-decimal precision string
        return hpscalar{str_representation.str()}; // Construct high-precision scalar from string
    }
}

#endif
