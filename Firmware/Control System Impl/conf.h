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
#define REF_CONVERGENCE_TIME 60.0f
// Allowable relative error after reference convergence time
// (due to LPF exponetial convergence, not sensor noise)
#define FILT_CONVERGENCE_FRACTION 0.01f 

// ---- Parameters for iterative root finder ----
#define CONVERGENCE_MIN 0.1 // Max allowable altitude error with current iterated K (m)
#define MAX_ITERATIONS 10 // Max iterations before failing out (to avoid hangup)

// ---- Parameters for sensor/processing rates ----
#define SAMPLE_TIME_MS 10 // Time between updates
// At least one sensor should have decimation 1
#define ALTIMETER_DECIMATION 10 // Number of updates between polls of altimeter
#define IMU_DECIMATION 1 // Number of updates between polls to accelerometer


// --- Derived constants ---
#define LAUNCH_ACCEL_THRESHOLD (LAUNCH_THRESHOLD_FRACTION*(LIFTOFF_ACCEL+G_CONST))
#define SAMPLE_FREQ (1000.0f/SAMPLE_TIME_MS)