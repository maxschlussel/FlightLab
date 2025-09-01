#pragma once

#include <stdio.h>

#include "src/core/state_vector.h"

/**
 * Simple CSV logger for simulation output.
 */
typedef struct {
    FILE* fp;
}Logger;

Logger loggerInit(const char* filename);

void loggerLogState(Logger* logger, double time, const StateVector* X);

void loggerClose(Logger* logger);
