#include "att_est.h"
#include "conf.h"
#include "Arduino.h"

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
    if(fabs(x)<1e-5) return 1;
    return tanf(x)/x;
}

void AttitudeEstimator::init(const Quaternion& q0)
{
    attitude = q0;
    covar.fill(0);
    ang_vel.fill(0);
    // covar = Square<3>::eye()*1e-6;
    cosineTheta = 1 - 2 * (q0[1]*q0[1] + q0[2]*q0[2]);
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
    ang_vel = gyro;
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
    // printMat(Serial,Ht);

    Vec<3> motion_blur_dir = crossp(ang_vel,magneto);
    Square<3> R = (MAG_VAR * Square<3>::eye() + MAG_BLUR_VAR * outerprod(motion_blur_dir,motion_blur_dir))/B2;
    // printMat(Serial,covar*1e6);

    Square<3> S = covar_map(H,covar) + R;
    Square<3> K = covar*Ht*inv(S);
    // printMat(Serial,S*1e6);

    Vec<3> aerr = K * (z-predict);
    // printMat(Serial,aerr*1e3);
    covar = (Square<3>::eye() - K*H)*covar;

    resetq(aerr);
}

}