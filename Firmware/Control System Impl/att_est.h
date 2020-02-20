#ifndef ATT_EST_H
#define ATT_EST_H

#include "matrixmath.hpp"
#include "quaternion.hpp"

namespace Control {

class AttitudeEstimator {
    Vec<3> inertialMag;
    void resetq(const Vec<3>& aerr);
public:
    Square<3> covar;
    Quaternion attitude;
    float cosineTheta;


    AttitudeEstimator();
    void reset();
    void setReference(const Vec<3>& refMag);
    void predict(const Vec<3>& gyro, float step);
    void update(const Vec<3>& magneto);

};

}

#endif