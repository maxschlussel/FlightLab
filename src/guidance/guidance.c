#include <stdio.h>

#include "src/guidance/guidance.h"


void handleGuidance(const GuidanceRefs* g) {
    switch (g->mode) {

        case GUIDANCE_NONE:
            // Module in development...
            break;
            
        case GUIDANCE_WAYPOINT:
            break;

        case GUIDANCE_LOITER:
            break;

        case GUIDANCE_PATH_FOLLOW:
            break;

        case GUIDANCE_HEADING_ALT_VEL:
            break;

        case GUIDANCE_GLIDEPATH:
            break;

        case GUIDANCE_ATTITUDE:
            break;

        default:
            break;
    }
}


GuidanceRefs initGuidanceNone(void){
    GuidanceRefs g = {
        .mode = GUIDANCE_NONE
    };
}


void updateGuidanceRefs(GuidanceRefs* g){
    // [1] Chose guidance mode
    g->mode = GUIDANCE_NONE;

    // [2] Set guidance refs
    handleGuidance(g);
}

