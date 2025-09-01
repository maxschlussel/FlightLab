#include "src/core/constants.h"
#include "src/io/logger.h"


Logger loggerInit(const char* filename){
    Logger logger = {0};

    logger.fp = fopen(filename, "w");

    if (!logger.fp) {
        printf("Error: could not open logger file.\n");
        return logger;
    }

    // Write CSV header
    fprintf(logger.fp, "time,u,v,w,p,q,r,phi,theta,psi,x,y,z\n");
    
    return logger;
}


void loggerLogState(Logger* logger, double time, const StateVector* X){
    fprintf(logger->fp, "%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
        time,
        X->u,
        X->v,
        X->w,
        X->p,
        X->q,
        X->r,
        X->phi * rad2deg,
        X->theta * rad2deg,
        X->psi * rad2deg,
        X->x,
        X->y,
        X->z
    );
}


void loggerClose(Logger* logger){
    fclose(logger->fp);
}