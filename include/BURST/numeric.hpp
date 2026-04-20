#ifndef BURST_NUMERIC_TYPES_HPP
#define BURST_NUMERIC_TYPES_HPP

#include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>
#include <boost/multiprecision/mpfr.hpp>

#include <concepts>
#include <sstream>
#include <iomanip>

#include "kernel.hpp"

/**
 * @file numeric.hpp
 * @brief Scalar types, numeric concepts, and conversions used alongside CGAL's exact kernel.
 *
 * Field types from @ref BURST::Kernel are the primary runtime scalars; additional MPFR-backed
 * high-precision scalars support trigonometry and analysis where extended precision is helpful.
 */

namespace BURST::numeric {
    
    /**
     * @brief Matches CGAL-style square-root field elements `a0 + a1 * sqrt(root)`.
     *
     * Satisfied when `a0()`, `a1()`, and `root()` are convertible to the kernel field type,
     * which is how several CGAL exact types expose irrational coordinates.
     */
    template <typename T>
    concept valid_sqrt_type = requires (T value) {
        { value.a0() } -> std::convertible_to<Kernel::FT>;
        { value.a1() } -> std::convertible_to<Kernel::FT>;
        { value.root() } -> std::convertible_to<Kernel::FT>;
    };

    /**
     * @brief Minimal random number generator concept for templated noise models.
     *
     * Requires a default-callable engine whose result is convertible to `unsigned int`, matching
     * common PRNGs such as `std::mt19937`.
     */
    template <typename R>
    concept valid_rng = requires(R rng) {
        {rng()} -> std::convertible_to<unsigned int>;
    };

    /**
     * @brief Distribution interface compatible with @ref valid_rng engines.
     *
     * Default-constructible, exposes `min()` / `max()` as `double`-convertible bounds, and
     * produces `double`-convertible samples when invoked with a compatible RNG—similar to
     * `std::uniform_real_distribution`.
     *
     * @tparam D Distribution type.
     * @tparam R Engine type satisfying @ref valid_rng.
     */
    template <typename D, typename R>
    concept valid_distribution = valid_rng<R> && requires(D dist, R rng) {
        {D{}};
        {dist.min()} -> std::convertible_to<double>;
        {dist.max()} -> std::convertible_to<double>;
        {dist(rng)} -> std::convertible_to<double>;
    };


    /** @brief Decimal precision (digits) used by @ref hpscalar. */
    constexpr unsigned int HP_PRECISION = 100;


    /** @brief Primary numeric type from @ref BURST::Kernel (`Kernel::FT`). */
    using fscalar = Kernel::FT;

    /** 
     * @brief Ring type from @ref BURST::Kernel (`Kernel::RT`).
     * 
     * Largely unused in the library, but exposed for external use.
     */
    using rscalar = Kernel::RT;

    /**
     * @brief High-precision floating scalar based on MPFR with @ref HP_PRECISION decimal digits.
     *
     * Used where extended precision beyond the kernel field is desired (e.g. trigonometric
     * evaluation) while still interoperable with conversions back to @ref fscalar where needed.
     */
    using hpscalar = boost::multiprecision::number<boost::multiprecision::mpfr_float_backend<HP_PRECISION>>;
   

    /**
     * @brief Absolute value for exact scalar types supporting ordered comparison.
     * @tparam FT Numeric type comparable to zero.
     */
    template <typename FT>
    FT abs(const FT& value) {
        return value < 0 ? -value : value;
    }

    /**
     * @brief Convert a value to @ref hpscalar for high-precision representation.
     * @tparam FT Source scalar type.
     */
    template <typename FT>
    hpscalar to_high_precision(const FT& value) {
        if constexpr (std::same_as<FT, hpscalar>) {
            return value; // No conversion needed
        } else {
            std::ostringstream str_representation;
            str_representation << std::setprecision(100) << value; // 100-decimal precision string
            return hpscalar{str_representation.str()}; // Construct high-precision scalar from string
        }
    }

    /**
     * @brief Convert a numeric value to @ref fscalar via high-precision decimal serialization.
     *
     * Useful for bringing external or high-precision values into the exact kernel field type in a
     * controlled way.
     *
     * @tparam N Source type streamable with sufficient precision.
     */
    template <typename N>
    fscalar to_fscalar(const N& value) {
        std::ostringstream str_representation;
        str_representation << std::setprecision(100) << value;
        return fscalar{str_representation.str()};
    }
    
    /**
     * @brief Convert a @ref valid_sqrt_type value to @ref fscalar.
     */
    template <valid_sqrt_type SqrtType>
    inline fscalar sqrt_to_fscalar(const SqrtType& value) {
        // Get exact values of components
        fscalar a0 = value.a0();
        fscalar a1 = value.a1();
        fscalar root = value.root();

        return a0 + a1 * CGAL::sqrt(root);
    }

    /**
     * @brief Human-readable decimal string for a scalar at high fixed precision.
     */
    template <typename N>
    std::string to_string(const N& value) {
        std::ostringstream str_representation;
        str_representation << std::setprecision(100) << value;
        return str_representation.str();
    }

    /**
     * @brief Deterministic “distribution” that always returns `1.0` for any RNG draw.
     *
     * Satisfies the interface expected by @ref models::RotationModel templates so tests can fix
     * noise without substituting a different model type. Constructor parameters are ignored and
     * exist only to mirror `std::uniform_real_distribution`'s shape.
     */
    class flat_distribution {
    public:
        flat_distribution(double = 0.0, double = 0.0) {}
        /** @brief Returns `1.0` regardless of `rng`. */
        template <typename RNG> double operator() (RNG&) const {
            return 1.0;
        }
        /** @brief Lower bound of the (degenerate) range. */
        double min() const { return 1.0; }
        /** @brief Upper bound of the (degenerate) range. */
        double max() const { return 1.0; }
    };

}

#endif
