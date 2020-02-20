#include "att_est.h"

#define GYRO_VAR 1e-5
#define MAG_VAR 0.1

namespace Control {

static inline Square<3> crossMat(const Vec<3>& v) {
    Square<3> ret;
    ret.at(0,0) = ret.at(1,1) = ret.at(2,2) = 0;
    ret.at(1,0) = v[2];
    ret.at(0,1) = -v[2];
    ret.at(0,2) = v[1];
    ret.at(2,0) = -v[1];
    ret.at(2,1) = v[0];
    ret.at(1,2) = -v[0];
    return ret;
}
static inline float tanc(float x) {
    if(fabs(x)<1e-4) return 1;
    return tanf(x)/x;
}

AttitudeEstimator::AttitudeEstimator()
{
    reset();
}

void AttitudeEstimator::reset()
{
    attitude = Quaternion(1,0,0,0);
    covar.fill(0);
    cosineTheta = 1;
}


void AttitudeEstimator::setReference(const Vec<3>& refMag)
{
    inertialMag = refMag/norm(refMag);
}

inline void AttitudeEstimator::resetq(const Vec<3>& aerr)
{

    Quaternion dq(2,aerr);
    attitude *= dq;
    attitude.normalize();
    cosineTheta = 1 - 2 * (attitude[1]*attitude[1] + attitude[2]*attitude[2]);
 
}

void AttitudeEstimator::predict(const Vec<3>& gyro, float step)
{
    float scaling = step*tanc(0.5*norm(gyro)*step);
    Vec<3> aerr = scaling * gyro;

    resetq(aerr);

    float addedvar = scaling*scaling*GYRO_VAR;
    covar += addedvar;
}

void AttitudeEstimator::update(const Vec<3>& magneto)
{
    float B2 = normsquare(magneto);
    Vec<3> z = magneto/sqrtf(B2);
    Vec<3> predict = attitude.invrotate(inertialMag);
    Square<3> H = crossMat(predict);
    Square<3> Ht = H.transpose();

    Square<3> R = -MAG_VAR/B2 * Square<3>::eye();


    Square<3> S = covar_map(H,covar) + R;
    Square<3> K = covar*Ht*inv(S);

    Vec<3> aerr = K * (z-predict);
    covar = (Square<3>::eye() - K*H)*covar;

    resetq(aerr);
}

}