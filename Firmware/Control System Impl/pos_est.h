#include "matrixmath.hpp"

namespace Control {

class PositionEstimator {
public:
    float K;
    Vec<2> state;
    Square<2> covar;
    
    inline float getAltitude() const {
        return state[1];
    }
    inline float getEps() const {
        return 0.5*state[0]*state[0];
    }

    PositionEstimator();
    void reset();
    void predict(float accel, float cost, float step);
    void update(float alt);
};

}