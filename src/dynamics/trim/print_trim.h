#pragma once

#include "src/core/constants.h"
#include "src/dynamics/aerodynamics.h"
#include "src/dynamics/eom.h"
#include "src/dynamics/trim/trim.h"
#include "src/dynamics/trim/fmin_search.h"


const char* getTrimModeString(TrimModes m);

const char* getFminStatusString(FminSearchStatus s);

void getCurrentTimeString(char* buffer, size_t len);

void printTrimReport(
    const TrimRefs* refs,
    const NMOptions* opts,
    const NMResult* result,
    const AeroData* aeroData,
    const AircraftParams* acParams,
    const double* X_trim,
    const double* U_trim,
    const char* scenarioFile,
    const char* filename_txt, // optional
    const char* filename_csv, // optional
    int print_console
);
