#include "pos_est.h"
#include "conf.h"

#include <Arduino.h>

namespace Control {

void PositionEstimator::init(float alt,float velocity)
{
    Vec<2> v0_map = vec2(1,SAMPLE_TIME_US/1e6f);
    covar = V0_VAR*outerprod(v0_map,v0_map);
    state = vec2(alt,velocity);
    K = 0;
    gravity = G_CONST;
}

void PositionEstimator::setGravity(float grav)
{
    gravity = grav;
}

void PositionEstimator::predict(float accel, float cost, float step)
{
    Square<2> F = Square<2>::eye();
    F.at(1,0) = step*cost;

    state = F*state;
    state[0] -= step*(accel+cost*gravity);
    state[1] -= 0.5*step*step*(cost*accel + gravity);

    covar = covar_map(F,covar);
    float q00 = ACCEL_VAR*step*step;
    float scalef = 0.5*cost*step;
    float q01 = q00*scalef;
    float q11 = q01*scalef;

    covar.at(0,0) += q00;
    covar.at(0,1) += q01;
    covar.at(1,0) += q01;
    covar.at(1,1) += q11;

    K = accel/getEps();
}

void PositionEstimator::update(float alt)
{
    float S = covar.at(1,1) + ALT_VAR;
    Vec<2> Ht;
    Ht[0] = 0;
    Ht[1] = 1;
    Mat<1,2> H = Ht.transpose();

    Vec<2> K = covar*Ht/S;

    // Serial.println(alt-state.at(1,0));
    // printVecHorz(Serial,K);
    // printVecHorz(Serial,K*(alt-state.at(1,0)));

    state += K*(alt-state.at(1,0));
    covar = (Square<2>::eye() - K*H)*covar;
}

}