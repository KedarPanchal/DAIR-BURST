#ifndef BURST_NUMERIC_TYPES_HPP
#define BURST_NUMERIC_TYPES_HPP

#include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>
#include <boost/multiprecision/mpfr.hpp>

#include <concepts>
#include <sstream>
#include <iomanip>

#include "kernel_types.hpp"

// This namespace contains numeric types for CGAL
namespace BURST::numeric {
    
    // -- NUMERIC CONCEPTS -----------------------------------------------------

    // Checks if a type is a valid square root number type, which models the form a0 + a1 * sqrt(root) where a0, a1, and root are convertible to the kernel's field type
    template <typename T>
    concept valid_sqrt_type = requires (T value) {
        { value.a0() } -> std::convertible_to<Kernel::FT>;
        { value.a1() } -> std::convertible_to<Kernel::FT>;
        { value.root() } -> std::convertible_to<Kernel::FT>;
    };

    // Checks if a type is a valid random number generator, which must be callable with no arguments and return a value convertible to unsigned int
    template <typename R>
    concept valid_rng = requires(R rng) {
        {rng()} -> std::convertible_to<unsigned int>;
    };

    /*
     * Checks if a type is a valid random number distribution, which must be:
     * Default constructible
     * Have min() and max() functions that return values convertible to double
     * Be callable with a random number generator to produce a value convertible to double
     */
    template <typename D, typename R>
    concept valid_distribution = valid_rng<R> && requires(D dist, R rng) {
        {D{}};
        {dist.min()} -> std::convertible_to<double>;
        {dist.max()} -> std::convertible_to<double>;
        {dist(rng)} -> std::convertible_to<double>;
    };


    // -- NUMERIC CONSTANTS ----------------------------------------------------

    constexpr unsigned int HP_PRECISION = 100; // 100 decimal digits of precision for high-precision scalar type


    // -- NUMERIC TYPES --------------------------------------------------------

    using fscalar = Kernel::FT;
    using rscalar = Kernel::RT;
    using hpscalar = boost::multiprecision::number<boost::multiprecision::mpfr_float_backend<HP_PRECISION>>;
   

    // -- NUMERIC FUNCTIONS ----------------------------------------------------

    // Computes the absolute value of a number
    template <typename FT>
    FT abs(const FT& value) {
        return value < 0 ? -value : value;
    }

    // Converts a number to a high-precision scalar
    template <typename FT>
    hpscalar to_high_precision(const FT& value) {
        std::ostringstream str_representation;
        str_representation << std::setprecision(100) << value; // 100-decimal precision string
        return hpscalar{str_representation.str()}; // Construct high-precision scalar from string
    }

    // Converts a number to an fscalar
    template <typename N>
    fscalar to_fscalar(const N& value) {
        std::ostringstream str_representation;
        str_representation << std::setprecision(100) << value;
        return fscalar{str_representation.str()};
    }
    
    // Converts a number of the form a0 + a1 * sqrt(root) to an fscalar
    template <valid_sqrt_type SqrtType>
    inline fscalar sqrt_to_fscalar(const SqrtType& value) {
        // Get exact values of components
        fscalar a0 = value.a0();
        fscalar a1 = value.a1();
        fscalar root = value.root();

        return a0 + a1 * CGAL::sqrt(root);
    }

    // -- UTILITY TYPES --------------------------------------------------------
    
    /*
     * Custom random number distribution that generates the same number for every RNG, which is useful for testing
     * This allows for templating the rotation model and avoiding inheritance
     */
    class flat_distribution {
    public:
        flat_distribution(double = 0.0, double = 0.0) {} // Dummy constructor to match the interface of std::uniform_real_distribution
        template <typename RNG> double operator() (RNG&) const {
            return 1.0;
        }
        double min() const { return 1.0; }
        double max() const { return 1.0; }
    };

}

#endif
