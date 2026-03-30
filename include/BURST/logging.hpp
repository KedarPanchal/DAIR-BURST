#ifndef BURST_LOGGING_HPP
#define BURST_LOGGING_HPP

#ifndef BURST_DISABLE_WARNINGS
#include <cstdio>

#define BURST_WARNING_TO(msg, stream) do { \
    ::fprintf(stream, "[BURST] Warning\t%s\nFile:\t%s\nLine:\t%d\n", (msg), __FILE__, __LINE__); \
} while(0)

#define BURST_WARNING(msg) BURST_WARNING_TO(msg, stderr)

#else

// Define no-op macros if warnings are disabled to prevent compile errors
#define BURST_WARNING_TO(msg, stream) do {} while(0)
#define BURST_WARNING(msg) do {} while(0)

#endif

#ifndef BURST_DISABLE_ERRORS
#include <cstdio>

#define BURST_ERROR_TO(msg, stream) do { \
    ::fprintf(stream, "[BURST] Error\t%s\nFile:\t%s\nLine:\t%d\n", (msg), __FILE__, __LINE__); \
} while(0)

#define BURST_ERROR(msg) BURST_ERROR_TO(msg, stderr)

#else

// Define no-op macros if errors are disabled to prevent compile errors
#define BURST_ERROR_TO(msg, stream) do {} while(0)
#define BURST_ERROR(msg) do {} while(0)

#endif

#endif
