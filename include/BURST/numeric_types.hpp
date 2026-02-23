#ifndef BURST_NUMERIC_TYPES_HPP
#define BURST_NUMERIC_TYPES_HPP

#include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>
#include <boost/multiprecision/mpfr.hpp>

#include <sstream>

#include "kernel_types.hpp"

// This namespace contains numeric types for CGAL
namespace BURST::numeric {
    
    // Internal implementations not intended for public use
    namespace detail {
        template <typename T, typename = void>
        struct is_valid_sqrt_type : std::false_type {};
        // Recognize types with a0(), a1(), and root() member functions
        template <typename T>
        struct is_valid_sqrt_type<T, std::void_t<decltype(std::declval<T>().a0()), decltype(std::declval<T>().a1()), decltype(std::declval<T>().root())>> : std::true_type {};
    }

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
    
    template <typename SqrtType>
    inline fscalar sqrt_to_fscalar(const SqrtType& value) {
    static_assert(detail::is_valid_sqrt_type<SqrtType>::value, "The type provided to numeric::to_fscalar must have a0(), a1(), and root() member functions that return the components of the square root in the form a0 + a1 * sqrt(root)");
        // Get exact values of components
        fscalar a0 = value.a0();
        fscalar a1 = value.a1();
        fscalar root = value.root();

        return a0 + a1 * CGAL::sqrt(root);
    }
}

#endif
