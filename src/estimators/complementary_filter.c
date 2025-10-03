#include "src/estimators/complementary_filter.h"
#include "src/math/utils.h"

double complementaryFilter(double lowFreqSignal, double highFreqSignal, double alpha){
    return weightedAverage(lowFreqSignal, highFreqSignal, alpha);
}
