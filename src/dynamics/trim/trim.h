#pragma once

#define N_TRIM_STATES 14


typedef enum {
    TRIM_STRAIGHT_LEVEL,
    TRIM_CLIMB_RATE,
    TRIM_GAMMA_ANGLE,
    TRIM_COORD_TURN,
    TRIM_ENGINE_OUT,
    TRIM_CUSTOM
} TrimModes;


typedef struct {
    TrimModes trimMode;

    double airspeed_ref;
    double altitude_ref;
    double climb_rate_ref;
    double gamma_ref;
    double turn_rate_ref;
    double roll_angle_ref;
    double pitch_angle_ref;
    double throttle_ref;
} TrimRefs;


typedef struct {
    int saveToFile;
    char* fileName;
    int printToScreen;
} TrimSolveOptions;

typedef double (*CostFunction)(const double* Z, int n);

void solveTrim(double* Z0, TrimRefs* trimRefs, TrimSolveOptions* tirmOpts, double* Z_return);

CostFunction getCostFunction(const TrimRefs* trimRefs);

double costStraightAndLevel(const double* Z,int n);
