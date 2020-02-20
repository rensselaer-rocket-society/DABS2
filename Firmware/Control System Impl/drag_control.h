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

void setReferenceMagneticField(const Vec<3>& B);

void on_IMU(const Vec<3>& a, const Vec<3>& g, const Vec<3>& m);
void on_Altimeter(float altitude);

void Tasks();

}

#endif