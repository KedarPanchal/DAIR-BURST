#ifndef BURST_LOGGING_HPP
#define BURST_LOGGING_HPP

#ifndef BURST_DISABLE_WARNINGS
#include <stdio.h>

#define BURST_WARNING_TO(msg, stream) do { \
    ::fprintf(stream, "[BURST]\t%s\nFile:\t%s\nLine:\t%d\n", msg, __FILE__, __LINE__); \
} while(0)

#define BURST_WARNING(msg) BURST_WARNING_TO(msg, stderr)

#endif

#endif
