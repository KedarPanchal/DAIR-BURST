#ifndef BURST_LOGGING_HPP
#define BURST_LOGGING_HPP

/**
 * @file logging.hpp
 * @brief Compile-time configurable logging macros for BURST diagnostics.
 *
 * Define `BURST_DISABLE_WARNINGS` and/or `BURST_DISABLE_ERRORS` before including this header to
 * silence the corresponding macros (they become no-ops). When enabled, messages include the
 * literal text, source file, and line number to aid debugging during simulation setup and
 * geometry validation.
 */

#ifndef BURST_DISABLE_WARNINGS
#include <string_view>
#include <source_location>
#include <iostream>

/** @brief Emit a warning to an arbitrary `std::ostream`. */
inline void burst_warning_to(const std::string_view& msg, std::ostream& stream, const std::source_location& location = std::source_location::current()) {
    stream << "[BURST] Warning:\t" << msg << '\n'
           << "[BURST] File:\t" << location.file_name() << '\n'
           << "[BURST] Line:\t" << location.line() << '\n';
}

/** @brief Emit a warning to `stderr`. */
inline void burst_warning(const std::string_view& msg, const std::source_location& location = std::source_location::current()) {
    burst_warning_to(msg, std::cerr, location);
}

#else

// Define no-op functions if warnings are disabled to prevent compile errors

inline void burst_warning_to(const std::string_view& msg, std::ostream& stream, const std::source_location& location = std::source_location::current()) {}
inline void burst_warning(const std::string_view& msg, const std::source_location& location = std::source_location::current()) {}

#endif

#ifndef BURST_DISABLE_ERRORS
#include <string_view>
#include <source_location>
#include <iostream>

/** @brief Emit an error to an arbitrary `std::ostream`. */
inline void burst_error_to(const std::string_view& msg, std::ostream& stream, const std::source_location& location = std::source_location::current()) {
    stream << "[BURST] Error:\t" << msg << '\n'
           << "[BURST] File:\t" << location.file_name() << '\n'
           << "[BURST] Line:\t" << location.line() << '\n';
}

/** @brief Emit an error to `stderr`. */
inline void burst_error(const std::string_view& msg, const std::source_location& location = std::source_location::current()) {
    burst_error_to(msg, std::cerr, location);
}

#else

// Define no-op functions if errors are disabled to prevent compile errors
inline void burst_error_to(const std::string_view& msg, std::ostream& stream, const std::source_location& location = std::source_location::current()) {}
inline void burst_error(const std::string_view& msg, const std::source_location& location = std::source_location::current()) {}

#endif

#endif
