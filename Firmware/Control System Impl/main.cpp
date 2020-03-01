#include <Arduino.h>
#include <SD.h>
#include "matrixmath.hpp"
#include "quaternion.hpp"
#include "pos_est.h"
#include "att_est.h"
#include "drag_control.h"
#include "conf.h"
#include "led_readout.h"
#include "lpf.hpp"

enum {
    STATE_PAD_IDLE,
    STATE_LAUNCHING,
    STATE_DRAG_CONTROL,
    STATE_DESCENT
} program_state; // State machine state enum

// Auto counting veriables (count every ms)
elapsedMillis sampling_timer = 0;
elapsedMillis imu_step = 0;
uint32_t sample_n = 0; // Sample counter

struct {
    Vec<3> accel;
    Vec<3> gyro;
    Vec<3> magneto;
    float alt;
} sensor_vals; // Holds latest sensor readings

// Filters to get averaged reference directions during pad idle
LowPassFilter<Vec<3>> gravityFilter(
    LPF_STEP_ERROR_AT_SAMPLE(FILT_CONVERGENCE_FRACTION,REF_CONVERGENCE_TIME*SAMPLE_FREQ/IMU_DECIMATION),
    vec3(0,0,0)
);
LowPassFilter<Vec<3>> magFilter(gravityFilter); // Same parameters as ground filter
LowPassFilter<float> groundFilter(
    LPF_STEP_ERROR_AT_SAMPLE(FILT_CONVERGENCE_FRACTION,REF_CONVERGENCE_TIME*SAMPLE_FREQ/ALTIMETER_DECIMATION),
    0.0f
);
// Launch Detect Filter
LowPassFilter<float> launchFilter(LPF_STEP_ERROR_AT_SAMPLE(0.75,LAUNCH_DETECT_TIME*SAMPLE_FREQ/IMU_DECIMATION),0.0f);

const char* SD_LOG_FILE = "log.csv";
File loggerFile;

ReadoutLED blinker(LED_BUILTIN,LED_ACTIVE_HIGH);

void blinkError(int code)
{
    while(1)
    {
        blinker.blinkN(code);
        delay(LED_DIGIT_SPACE_MS);
    }
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    if (!SD.begin(BUILTIN_SDCARD)) {
        blinkError(1);
    }
    else {
        loggerFile = SD.open(SD_LOG_FILE, FILE_WRITE);
        if(!loggerFile){
            blinkError(2);
        }
    }


}

#define ESTIMATOR_RUNNING ((program_state==STATE_LAUNCHING) || (program_state==STATE_DRAG_CONTROL))

void processIMU()
{
    //TODO poll imu
    if(ESTIMATOR_RUNNING){
        Control::Estimator_IMU(imu_step,sensor_vals.accel,sensor_vals.gyro,sensor_vals.magneto);
    }
    else if(program_state == STATE_PAD_IDLE){
        gravityFilter.ingest(sensor_vals.accel);
        magFilter.ingest(sensor_vals.magneto);
        launchFilter.ingest(sensor_vals.accel[2]);
    }
    imu_step = 0;
}

void processAltimeter()
{
    //TODO poll altimeter
    if(ESTIMATOR_RUNNING){
        Control::Estimator_Altimeter(sensor_vals.alt);
    }
    else if(program_state == STATE_PAD_IDLE){
        groundFilter.ingest(sensor_vals.alt);
    }
}

void commandFlaps()
{
    // Default to close as fast as possible
    // Limit check will stop it when fully retracted
    float deploy_speed = -1.0; 
    if(program_state == STATE_DRAG_CONTROL)
    {
        deploy_speed = Control::Controller_EvalControlLaw();
    }

    // TODO Limit Checks

    // TODO send flap deploy command
}

int main() {

    setup();

    program_state = STATE_PAD_IDLE;
    digitalWrite(LED_BUILTIN,HIGH); // Indicate sucessful startup

    uint32_t recovery_blink = 1;

    while(program_state != STATE_DESCENT)
    {

        if(sampling_timer > SAMPLE_TIME_MS)
        {
            sampling_timer = 0;
            ++sample_n;


            // Consult Sensors if ready
            if(sample_n % IMU_DECIMATION == 0)
            {
                processIMU();
            }
            if(sample_n % ALTIMETER_DECIMATION == 0)
            {
                processAltimeter();
            }

            commandFlaps();


            switch(program_state) // State change logic
            {
            case STATE_PAD_IDLE:
                if(launchFilter.value > LAUNCH_ACCEL_THRESHOLD)
                {
                    Control::Init(groundFilter.value, gravityFilter.value, magFilter.value);
                    program_state = STATE_LAUNCHING;
                }
                break;
            case STATE_LAUNCHING:
                if(sensor_vals.accel[2] < 0) { // Negative longitudinal acceleration (drag)
                    // Check if we are too inclined at burnout to allow deployment
                    if(Control::a_est.cosineTheta < (float)(cos(M_PI*NO_DEPLOY_ANGLE/180.0)))
                    {
                        recovery_blink = 2;
                        program_state = STATE_DESCENT; // Skip control phase
                    } else {
                        program_state = STATE_DRAG_CONTROL;
                    }
                }
                break;
            case STATE_DRAG_CONTROL:
                
                break;
            case STATE_DESCENT: // Will exit loop and fall through to end, runs once
                break;
            }

        }
    }

    loggerFile.flush();

    while(1)
    {
        blinker.blinkDigits(recovery_blink);
        delay(5*LED_DIGIT_SPACE_MS);
    }

}
