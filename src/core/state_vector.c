#include "src/core/state_vector.h"


void statevec_to_array(const StateVector* X, double x[N_STATE_VEC]){
    x[0] = X->u; x[1] = X->v; x[2] = X->w;
    x[3] = X->p; x[4] = X->q; x[5] = X->r;
    x[6] = X->phi; x[7] = X->theta; x[8] = X->psi;
    x[9] = X->x; x[10] = X->y; x[11] = X->z;
}


void array_to_statevec(const double x[N_STATE_VEC], StateVector* X){
    X->u = x[0]; X->v = x[1]; X->w = x[2];
    X->p = x[3]; X->q = x[4]; X->r = x[5];
    X->phi = x[6]; X->theta = x[7]; X->psi = x[8];
    X->x = x[9]; X->y = x[10]; X->z = x[11];
}