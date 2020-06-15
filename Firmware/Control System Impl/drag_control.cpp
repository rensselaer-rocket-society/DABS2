#include <math.h>
#include "drag_control.h"
#include "conf.h"
#include <Arduino.h>

namespace Control {


PositionEstimator p_est;
AttitudeEstimator a_est;
Vec<3> gyro_bias;
float ground_alt = 0;


float targetK = BURNOUT_K_EST;
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

void Estimator_IMU(uint32_t step_us, const Vec<3>& a, const Vec<3>& g){
    float step = step_us/1e6f;

    p_est.predict(-a[2],a_est.cosineTheta,step);
    a_est.predict(g-gyro_bias,step);
}
void Estimator_Magnetometer(const Vec<3>& m)
{
    a_est.update(m);
}

void Estimator_Altimeter(float altitude){
    p_est.update(altitude-ground_alt);
}

void Init(float g_alt, const Vec<3>& grav_vec, const Vec<3>& B, const Vec<3>& g_bias)
{
    ground_alt = g_alt;
    gyro_bias = g_bias;
    p_est.init(0,0);
    p_est.setGravity(norm(grav_vec));

    // Since we don't care about the inertial heading, just find the
    // simplest rotation that rotates grav_vec to khat (pitch directly back up)
    float ang = acos(grav_vec[2]/norm(grav_vec)); // This is acos(k_hat dot grav)
    Vec<3> axis = vec3(grav_vec[1],-grav_vec[0],0); // Simplified grav_vec x k_hat
    axis /= norm(axis);
    Quaternion orientation(cos(ang/2.0f), sin(ang/2.0f)*axis);

    // printMat(Serial,orientation.rotate(grav_vec));
    // printMat(Serial,grav_vec);
    // printMat(Serial,orientation.invrotate(vec3(0,0,norm(grav_vec))));

    // while(1);

    a_est.init(orientation);
    a_est.setReference(orientation.rotate(B));
}

float Controller_EvalControlLaw()
{
    updateDragCalcs();
    float Kerror = targetK - p_est.K;
    return Kerror;
}

}