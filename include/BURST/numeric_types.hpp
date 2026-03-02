#ifndef BURST_NUMERIC_TYPES_HPP
#define BURST_NUMERIC_TYPES_HPP

#include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>
#include <boost/multiprecision/mpfr.hpp>

#include <sstream>
#include <iomanip>

#include "kernel_types.hpp"

// This namespace contains numeric types for CGAL
namespace BURST::numeric {

    template <typename T>
    concept valid_sqrt_type = requires (T value) {
        { value.a0() } -> std::convertible_to<Kernel::FT>;
        { value.a1() } -> std::convertible_to<Kernel::FT>;
        { value.root() } -> std::convertible_to<Kernel::FT>;
    };

    // Top-level types and constants
    constexpr unsigned int HP_PRECISION = 100; // 100 decimal digits of precision for high-precision scalar type
                                               
    // Scalar/numeric types
    using fscalar = Kernel::FT;
    using rscalar = Kernel::RT;
    using hpscalar = boost::multiprecision::number<boost::multiprecision::mpfr_float_backend<HP_PRECISION>>;
    
    // Helper functions
    template <typename FT>
    FT abs(const FT& value) {
        return value < 0 ? -value : value;
    }
    template <typename FT>
    hpscalar to_high_precision(const FT& value) {
        std::ostringstream str_representation;
        str_representation << std::setprecision(100) << value; // 100-decimal precision string
        return hpscalar{str_representation.str()}; // Construct high-precision scalar from string
    }
    
    template <valid_sqrt_type SqrtType>
    inline fscalar sqrt_to_fscalar(const SqrtType& value) {
        // Get exact values of components
        fscalar a0 = value.a0();
        fscalar a1 = value.a1();
        fscalar root = value.root();

        return a0 + a1 * CGAL::sqrt(root);
    }

    // Utility classes
    
    /*
     * Custom random number distribution that generates the same number for every RNG, which is useful for testing
     * This allows for templating the rotation model and avoiding inheritance
     */
    class flat_distribution {
    public:
        flat_distribution(double = 0.0, double = 0.0) {} // Dummy constructor to match the interface of std::uniform_real_distribution
        template <typename RNG> double operator() (RNG& rng) const {
            return 1.0;
        }
        double min() const { return 1.0; }
        double max() const { return 1.0; }
    };


}

#endif
