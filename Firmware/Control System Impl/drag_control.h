#ifndef DRAG_CONTROL_H
#define DRAG_CONTROL_H

#include "matrixmath.hpp"
#include "pos_est.h"
#include "att_est.h"

namespace Control {

extern PositionEstimator p_est;
extern AttitudeEstimator a_est;

extern float targetK;
extern float currentProjection;


void Estimator_IMU(uint32_t step_ms, const Vec<3>& a, const Vec<3>& g, const Vec<3>& m);
void Estimator_Altimeter(float altitude);

void Init(float ground_alt, const Vec<3>& grav_vec,const Vec<3>& B);
float Controller_EvalControlLaw();


}

#endif