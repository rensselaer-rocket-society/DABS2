#ifndef LPF_H
#define LPF_H

#include "matrixmath.hpp"
#include "math.h"

#define LPF_STEP_ERROR_AT_SAMPLE(err,n) (1-powf((err),1.0f/((n)+1)))

template <class T>
class LowPassFilter {
private:
    float alpha;
public:
    T value;

    LowPassFilter(float alph, const T& startVal)
    {
        value = startVal;
        alpha = alph;
    }
    void ingest(const T& sample)
    {
        value += alpha*(sample - value);
    }
};

#endif