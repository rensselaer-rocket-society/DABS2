#include "pos_est.h"

#define G_CONST 9.81
#define ACCEL_VAR 1e-3
#define ALT_VAR 0.1

namespace Control {

PositionEstimator::PositionEstimator()
{
    reset();
}

void PositionEstimator::reset()
{
    covar.fill(0);
    state.fill(0);
}

void PositionEstimator::predict(float accel, float cost, float step)
{
    Square<2> F = Square<2>::eye();
    F.at(1,0) = step*cost;

    state = F*state;
    state[0] -= step*(accel+cost*G_CONST);
    state[1] -= 0.5*step*step*(cost*accel + G_CONST);

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
    Ht.at(0,0) = 0;
    Ht.at(1,0) = 1;
    Mat<1,2> H = Ht.transpose();

    Vec<2> K = covar*Ht/S;

    state += K*(alt-state.at(1,0));
    covar = (Square<2>::eye() - K*H)*covar;
}

}