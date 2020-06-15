#ifndef CONF_H
#define CONF_H

#include "math.h"

#define G_CONST 9.81f

//--------------------------------------------------
//       Parameters you are likely to change
//--------------------------------------------------
// Desired apogee in meters AGL
#define APOGEE_TARGET 300.0f
// Expected Acceleration at liftoff 
#define LIFTOFF_ACCEL (12.0f*G_CONST)
// Minimum angle to vertical at burnout which will prevent flap deployment (degrees)
#define NO_DEPLOY_ANGLE 15.0f
// Estimate of drag parameter K at burnout (to seed iterative solver)
#define BURNOUT_K_EST 0.1f
//--------------------------------------------------


// ---- Launch Detect Parameters ----
// Time (s) from ignition to launch detect
// More time increases robustness to spurious accelerations, but means the estimator
// will take longer to converge since it has to 'catch up' more
#define LAUNCH_DETECT_TIME 0.1f
// Fraction of ignition acceleration to use for threshold
// Lower makes time to detect more consistent with changes in actual liftoff accel, but makes it triggerable with smaller but sustained accels.
// Higher makes it less suceptible to smaller acceleration triggeers, but increases risk of not reaching threshold and variation in time to detect
#define LAUNCH_THRESHOLD_FRACTION 0.75f

// ---- Pad Idle Filter parameters: ----
// Seconds for reference convergence after going upright on pad
#define REF_CONVERGENCE_TIME 10.0f
// Allowable relative error after reference convergence time
// (due to LPF exponetial convergence, not sensor noise)
#define FILT_CONVERGENCE_FRACTION 0.01f 

// ---- Parameters for iterative root finder ----
#define CONVERGENCE_MIN 0.1 // Max allowable altitude error with current iterated K (m)
#define MAX_ITERATIONS 10 // Max iterations before failing out (to avoid hangup)

// ---- Parameters for sensor/processing rates ----
#define SAMPLE_TIME_US 4250 // Time between updates
// At least one sensor should have decimation 1
#define ALTIMETER_DECIMATION 16 // Number of updates between polls of altimeter
#define IMU_DECIMATION 1 // Number of updates between polls to accelerometer
#define MAGNETOMETER_DECIMATION 3 // Number of updates between polls to magnetometer

#define GYRO_DENSITY (7e-3f*M_PI/180.0) // 7 mdps/sqrt(Hz) using http://www.emcu.it/MKT/MEMSconsumerQ12015/STM_MEMS_Q12015.pdf
#define ACCEL_DENSITY (150e-6f*G_CONST)  // 150ug/sqrt(Hz) using LIS3DSH datasheet
#define MAG_RMS (0.32f) // 3.2 mgauss RMS converted to uT (using LIS3MDL datasheet)
#define MAG_BLUR_RMS (SAMPLE_TIME_US*MAGNETOMETER_DECIMATION/6e6f) // Std. Dev of difference between current time and sample time, use half sample period as 3
#define ALT_RMS (0.95) // 


#define GYRO_VAR (GYRO_DENSITY*GYRO_DENSITY*76 + 1e-5) // Gyroscope noise density * 76 Hz bandwidth plus process noise
#define ACCEL_VAR (ACCEL_DENSITY*ACCEL_DENSITY*105 + 1e-3) // Accelerometer noise density times 105 Hz bandwidth plus process noise
#define MAG_VAR (MAG_RMS*MAG_RMS) 
#define MAG_BLUR_VAR (MAG_BLUR_RMS*MAG_BLUR_RMS) 
#define ALT_VAR (ALT_RMS*ALT_RMS) 
#define V0_VAR (1*1) // Variance of initial velocity for position covarinace

// --- Derived constants ---
#define LAUNCH_ACCEL_THRESHOLD (LAUNCH_THRESHOLD_FRACTION*(LIFTOFF_ACCEL+G_CONST))
#define SAMPLE_FREQ (1e6f/SAMPLE_TIME_US)

#endif