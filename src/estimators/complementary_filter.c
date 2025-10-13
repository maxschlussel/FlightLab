#include "src/estimators/complementary_filter.h"
#include "src/math/utils.h"

double complementaryFilter(double highFreqSignal, double lowFreqSignal, double alpha){
    return weightedAverage(highFreqSignal, lowFreqSignal, alpha);
}
