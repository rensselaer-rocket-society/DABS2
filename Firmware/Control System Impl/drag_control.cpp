#include <math.h>
#include "drag_control.h"

#define G_CONST 9.81
#define CONVERGENCE_MIN 0.1
#define MAX_ITERATIONS 10

#define APOGEE_TARGET 100

namespace Control {


PositionEstimator p_est;
AttitudeEstimator a_est;

bool new_data = false;

float targetK = 0.1;
float currentProjection = 0;


static float ds(float K, float eps, float g_eff) {
    return logf(1+K*eps/g_eff)/K;
}


static void updateDragCalcs()
{
    float eps = p_est.getEps();
    float cost = a_est.cosineTheta;
    float g_eff = cost*G_CONST;

    float z = p_est.getAltitude();
    currentProjection = z + cost*ds(p_est.K, eps, g_eff);


    // Iterative root finder to get K needed for target ds
    float target = (APOGEE_TARGET - z)/cost;
    float K = targetK;
    if(K <= 0) targetK = 0.01;
    float curr = ds(K,eps,g_eff);
    uint32_t iterations = 0;
    while(fabs(curr-target)>CONVERGENCE_MIN && iterations < MAX_ITERATIONS)
    {
        float newK = K*(1+(curr-target)/(curr - 1/(g_eff/eps + K)));
        if(newK <= 0) newK = K/2;
        K = newK;
        curr = ds(K,eps,g_eff);
        ++iterations;
    }

    if(iterations != MAX_ITERATIONS) targetK = K;
}

static void updateFlapControl() {
    float Kerror = targetK - p_est.K;


}


void setReferenceMagneticField(const Vec<3>& B){
    a_est.setReference(B);
}

void on_IMU(const Vec<3>& a, const Vec<3>& g, const Vec<3>& m){
    float step = 0.01; //TODO Acutally time this?

    a_est.predict(g,step);
    a_est.update(m);

    p_est.predict(-a[2],a_est.cosineTheta,step);

    new_data = true;
}
void on_Altimeter(float altitude){
    p_est.update(altitude);

    new_data = true;
}

void Setup()
{
    p_est.reset();
    a_est.reset();
}

void Tasks()
{

    if(new_data)
    {
        updateDragCalcs();
        updateFlapControl();
        new_data = false;
    }

}

}