#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>

#include "src/dynamics/trim/print_trim.h"


// ============================================================
//  Main Report Function
// ============================================================
void printTrimReport(
    const TrimRefs* refs,
    const NMOptions* opts,
    const NMResult* result,
    const AeroData* aeroData,
    const AircraftModel* acModel,
    const double* X_trim,
    const double* U_trim,
    const char* scenarioFile,
    const char* filename_txt, // optional
    const char* filename_csv, // optional
    int print_console
) {
    FILE* f_txt = NULL;
    FILE* f_csv = NULL;
    char timeStr[64];
    getCurrentTimeString(timeStr, sizeof(timeStr));

    if (filename_txt) f_txt = fopen(filename_txt, "w");
    if (filename_csv) f_csv = fopen(filename_csv, "w");

    // Define a unified FILE* array for looping (stdout + files)
    FILE* outputs[3];
    int nOut = 0;
    if (print_console) outputs[nOut++] = stdout;
    if (f_txt)         outputs[nOut++] = f_txt;

    for (int k = 0; k < nOut; ++k) {
        FILE* out = outputs[k];

        fprintf(out,
            "============================================================\n"
            "                 AIRCRAFT TRIM SOLVER REPORT\n"
            "============================================================\n"
            "Aircraft model:       %s\n"
            "Scenario file:        %s\n"
            "Date & Time:          %s\n"
            "------------------------------------------------------------\n"
            "Trim Mode:            %s\n"
            "Solver:               fminsearch (Nelder–Mead)\n"
            "Convergence Status:   %s\n"
            "Iterations:           %d\n"
            "Function Evaluations: %d\n"
            "Final Cost:           %.6e\n"
            "Tolerance (TolX):     %.3e\n"
            "Tolerance (TolFun):   %.3e\n"
            "\n",
            acModel->name,
            scenarioFile,
            timeStr,
            getTrimModeString(refs->trimMode),
            getFminStatusString(result->status),
            result->iterations,
            result->feval_count,
            result->f_opt,
            opts->TolX,
            opts->TolFun
        );
        
        fprintf(out,
            "------------------------------------------------------------\n"
            "Target References:\n"
            "------------------------------------------------------------\n"
            "  Airspeed_ref      : %8.3f [m/s]\n"
            "  Altitude_ref      : %8.3f [m]\n"
            "  Climb_rate_ref    : %8.3f [m/s]\n"
            "  Gamma_ref         : %8.3f [rad]\n"
            "  Turn_rate_ref     : %8.3f [rad/s]\n"
            "  Roll_angle_ref    : %8.3f [rad]\n"
            "  Pitch_angle_ref   : %8.3f [rad]\n"
            "  Throttle_ref      : %8.3f [0–1]\n"
            "\n",
            refs->airspeed_ref,
            refs->altitude_ref,
            refs->climb_rate_ref,
            refs->gamma_ref,
            refs->turn_rate_ref,
            refs->roll_angle_ref,
            refs->pitch_angle_ref,
            refs->throttle_ref
        );
        
        fprintf(out,
            "------------------------------------------------------------\n"
            "Trimmed State Vector (X_trim)\n"
            "------------------------------------------------------------\n"
            "  u    : %8.3f [m/s]       v  : %8.3f [m/s]     w   : %8.3f [m/s]\n"
            "  p    : %8.4f [rad/s]     q  : %8.4f [rad/s]   r   : %8.4f [rad/s]\n"
            "  phi  : %8.3f [deg]    theta : %8.3f [deg]   psi   : %8.3f [deg]\n\n",
            X_trim[0], X_trim[1], X_trim[2],
            X_trim[3], X_trim[4], X_trim[5],
            X_trim[6]*rad2deg, X_trim[7]*rad2deg, X_trim[8]*rad2deg
        );
        
        fprintf(out,
            "------------------------------------------------------------\n"
            "Trimmed Control Inputs (U_trim)\n"
            "------------------------------------------------------------\n"
            "  Elevator   : %8.4f [rad]\n"
            "  Aileron    : %8.4f [rad]\n"
            "  Rudder     : %8.4f [rad]\n"
            "  Throttle1   : %8.4f [0–1]\n"
            "  Throttle2   : %8.4f [0–1]\n\n",
            U_trim[0],
            U_trim[1],
            U_trim[2],
            U_trim[3],
            U_trim[4]
        );
        
        fprintf(out,
            "------------------------------------------------------------\n"
            "Aerodynamic Coefficients, Parameters and Dynamics @ Trim\n"
            "------------------------------------------------------------\n"
            "  Velocity          : %10.3f [m/s]\n"
            "  q_inf             : %10.3f [Pa]\n"
            "  Alpha             : %10.3f [deg]\n"
            "  Beta              : %10.3f [deg]\n"
            "  Lift              : %10.3f [N]\n"
            "  Drag              : %10.3f [N]\n"
            "  Side Force        : %10.3f [N]\n"
            "  Roll Moment (L)   : %10.4f [N·m]\n"
            "  Pitch Moment (M)  : %10.4f [N·m]\n"
            "  Yaw Moment (N)    : %10.4f [N·m]\n"
            "  CL                : %10.4f [-]\n"
            "  CD                : %10.4f [-]\n"
            "  CY                : %10.4f [-]\n"
            "  Cm                : %10.4f [-]\n"
            "  Cl                : %10.4f [-]\n"
            "  Cn                : %10.4f [-]\n\n",
            aeroData->velocity, aeroData->q_inf, aeroData->alpha, aeroData->beta,
            aeroData->Lift, aeroData->Drag, aeroData->SideForce,
            aeroData->RollingMoment, aeroData->PitchingMoment, aeroData->YawingMoment,
            aeroData->CL, aeroData->CD, aeroData->CY,
            aeroData->Cm, aeroData->Cl, aeroData->Cn
        );
        
    }

    // CSV output (optional, easier for postprocessing)
    if (f_csv) {
        fprintf(f_csv, "Parameter,Value,Units\n");
        fprintf(f_csv, "TrimMode,%s,\n", getTrimModeString(refs->trimMode));
        fprintf(f_csv, "Airspeed_ref,%.3f,m/s\n", refs->airspeed_ref);
        fprintf(f_csv, "Altitude_ref,%.3f,m\n", refs->altitude_ref);
        fprintf(f_csv, "Climb_rate_ref,%.3f,m/s\n", refs->climb_rate_ref);
        fprintf(f_csv, "Gamma_ref,%.3f,rad\n", refs->gamma_ref);
        fprintf(f_csv, "Turn_rate_ref,%.3f,rad/s\n", refs->turn_rate_ref);
        fprintf(f_csv, "Pitch_angle_ref,%.3f,rad\n", refs->pitch_angle_ref);
        fprintf(f_csv, "Throttle_ref,%.3f,ratio\n", refs->throttle_ref);

        fprintf(f_csv, "Result_Status,%s,\n", getFminStatusString(result->status));
        fprintf(f_csv, "Iterations,%d,\n", result->iterations);
        fprintf(f_csv, "Final_Cost,%.6e,\n", result->f_opt);

        fprintf(f_csv, "\nState Variable,Value,Units\n");
        fprintf(f_csv, "u,%.4f,m/s\nv,%.4f,m/s\nw,%.4f,m/s\n", X_trim[0], X_trim[1], X_trim[2]);
        fprintf(f_csv, "p,%.5f,rad/s\nq,%.5f,rad/s\nr,%.5f,rad/s\n", X_trim[3], X_trim[4], X_trim[5]);
        fprintf(f_csv, "phi,%.4f,rad\ntheta,%.4f,rad\npsi,%.4f,rad\n", X_trim[6], X_trim[7], X_trim[8]);
        fprintf(f_csv, "elevator,%.4f,rad\naileron,%.4f,rad\nrudder,%.4f,rad\nthrottle,%.4f,\n",
            U_trim[0], U_trim[1], U_trim[2], U_trim[3]);
                    fprintf(f_csv, "\nAero Forces & Moments,N/A,\n");
        fprintf(f_csv, "Lift,%.4f,N\nDrag,%.4f,N\nSide,%.4f,N\n", aeroData->Lift, aeroData->Drag, aeroData->SideForce);
        fprintf(f_csv, "RollMoment,%.4f,Nm\nPitchMoment,%.4f,Nm\nYawMoment,%.4f,Nm\n",
                aeroData->RollingMoment, aeroData->PitchingMoment, aeroData->YawingMoment);
    }

    if (f_txt) fclose(f_txt);
    if (f_csv) fclose(f_csv);
}


typedef struct {
    int nx; // number of states
    int nu; // number of control inputs
    int ny; // number of outputs
    double* A; // size nx * nx
    double* B; // size nx * nu
    double* C; // size ny * nx
    double* D; // size ny * nu
} LinearModel;

//---------------------------------------------
// Helper to print matrix
//---------------------------------------------
static void printMatrix(FILE* out, const char* name, const double* M, int rows, int cols)
{
    fprintf(out, "%s = [\n", name);
    for (int i = 0; i < rows; ++i) {
        fprintf(out, "  ");
        for (int j = 0; j < cols; ++j)
            fprintf(out, "%+10.6e ", M[i * cols + j]);
        fprintf(out, "\n");
    }
    fprintf(out, "];\n\n");
}

//---------------------------------------------
// Append to trim report (if linear model exists)
//---------------------------------------------
void appendLinearizationReport(
    FILE* f_txt,
    const LinearModel* linModel
)
{
    if (!linModel || !linModel->A) return;

    fprintf(f_txt,
        "\n============================================================\n"
        "               LINEARIZED STATE-SPACE MODEL\n"
        "============================================================\n"
        "State dimension (nx): %d\n"
        "Input dimension (nu): %d\n"
        "Output dimension (ny): %d\n"
        "------------------------------------------------------------\n\n",
        linModel->nx, linModel->nu, linModel->ny
    );

    printMatrix(f_txt, "A", linModel->A, linModel->nx, linModel->nx);
    printMatrix(f_txt, "B", linModel->B, linModel->nx, linModel->nu);

    if (linModel->C && linModel->ny > 0)
        printMatrix(f_txt, "C", linModel->C, linModel->ny, linModel->nx);
    if (linModel->D && linModel->ny > 0)
        printMatrix(f_txt, "D", linModel->D, linModel->ny, linModel->nu);

    fprintf(f_txt,
        "------------------------------------------------------------\n"
        "Eigenvalue Analysis (approximate stability check):\n"
        "------------------------------------------------------------\n"
    );

    // Optional: quick real-only stability check (approximate)
    for (int i = 0; i < linModel->nx; ++i) {
        double eig = linModel->A[i * linModel->nx + i]; // crude approximation (diagonal)
        fprintf(f_txt, "  λ[%02d] ≈ %+8.4f\n", i + 1, eig);
    }

    fprintf(f_txt,
        "------------------------------------------------------------\n"
        "Interpretation:\n"
        "  - Negative real eigenvalues indicate stable modes.\n"
        "  - Positive real parts indicate unstable divergence.\n"
        "============================================================\n\n"
    );
}


// Helper: format time
void getCurrentTimeString(char* buffer, size_t len) {
    time_t t = time(NULL);
    struct tm* tm_info = localtime(&t);
    strftime(buffer, len, "%Y-%m-%d %H:%M:%S", tm_info);
}


// Helper: interpret solver status
const char* getFminStatusString(FminSearchStatus s) {
    switch (s) {
        case FMIN_STATUS_SUCCESS:       return "SUCCESS";
        case FMIN_STATUS_MAX_ITER:      return "MAX_ITER";
        case FMIN_STATUS_MAX_FUNC_EVAL: return "MAX_FUNC_EVAL";
        case FMIN_STATUS_FAILURE:       return "FAILURE";
        default:                        return "UNKNOWN";
    }
}


// Helper: interpret trim mode
const char* getTrimModeString(TrimModes m) {
    switch (m) {
        case TRIM_STRAIGHT_LEVEL: return "STRAIGHT_LEVEL";
        case TRIM_CLIMB_RATE:     return "CLIMB_RATE";
        case TRIM_GAMMA_ANGLE:    return "GAMMA_ANGLE";
        case TRIM_COORD_TURN:     return "COORD_TURN";
        case TRIM_ENGINE_OUT:     return "ENGINE_OUT";
        case TRIM_CUSTOM:         return "CUSTOM";
        default:                  return "UNKNOWN";
    }
}
