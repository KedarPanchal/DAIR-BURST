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
#include <cstdio>

/** @brief Emit a warning to an arbitrary `FILE*` stream (typically `stderr`). */
#define BURST_WARNING_TO(msg, stream) do { \
    ::fprintf(stream, "[BURST] Warning:\t%s\n[BURST] File:\t%s\n[BURST] Line:\t%d\n", (msg), __FILE__, __LINE__); \
} while(0)

/** @brief Emit a warning to `stderr`. */
#define BURST_WARNING(msg) BURST_WARNING_TO(msg, stderr)

#else

// Define no-op macros if warnings are disabled to prevent compile errors
#define BURST_WARNING_TO(msg, stream) do {} while(0)
#define BURST_WARNING(msg) do {} while(0)

#endif

#ifndef BURST_DISABLE_ERRORS
#include <cstdio>

/** @brief Emit an error to an arbitrary `FILE*` stream (typically `stderr`). */
#define BURST_ERROR_TO(msg, stream) do { \
    ::fprintf(stream, "[BURST] Error:\t%s\n[BURST] File:\t%s\n[BURST] Line:\t%d\n", (msg), __FILE__, __LINE__); \
} while(0)

/** @brief Emit an error to `stderr`. */
#define BURST_ERROR(msg) BURST_ERROR_TO(msg, stderr)

#else

// Define no-op macros if errors are disabled to prevent compile errors
#define BURST_ERROR_TO(msg, stream) do {} while(0)
#define BURST_ERROR(msg) do {} while(0)

#endif

#endif
