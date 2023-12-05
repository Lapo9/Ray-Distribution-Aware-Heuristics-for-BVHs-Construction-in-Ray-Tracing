#pragma once

#define TIMING 1

#if TIMING
#define TIME(body) body
#else
#define TIME(body)
#endif // TIMING
