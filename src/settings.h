/** @file */
#pragma once

#define TIMING 1 /**< When is set, during the runtime there will be routines to collect data about performance */

#if TIMING
#define TIME(body) body
#else
#define TIME(body)
#endif // TIMING

#define TOLERANCE 0.01f
