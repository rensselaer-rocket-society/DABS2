#include <Arduino.h>
#include <SD.h>
#include "MPL3115A2.h"
#include "matrixmath.hpp"
#include "quaternion.hpp"
#include "pos_est.h"
#include "att_est.h"
#include "drag_control.h"
#include "conf.h"
#include "led_readout.h"
#include "lpf.hpp"
#include "LSM9DS1.h"
#include "Wire.h"   

enum {
    STATE_PAD_IDLE,
    STATE_LAUNCHING,
    STATE_DRAG_CONTROL,
    STATE_DESCENT
} program_state; // State machine state enum

// Auto counting veriables (count every us)
elapsedMicros sampling_timer = 0;
elapsedMicros imu_step = 0;
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
LowPassFilter<Vec<3>> gyroFilter(gravityFilter); // Same parameters as other filters
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

    Wire.begin();

    Serial.begin(115200);
    // if (!SD.begin(BUILTIN_SDCARD)) {
    //     blinkError(2);
    // }
    // else {
    //     loggerFile = SD.open(SD_LOG_FILE, FILE_WRITE);
    //     if(!loggerFile){
    //         blinkError(3);
    //     }
    // }

    if(!LSM::CheckDevicePresent())
    {
        blinkError(4);
    } else {
        LSM::Init();
    }

    if(!MPL::CheckDevicePresent())
    {
        blinkError(5);
    } else {
        MPL::Init();
    }

    MPL::RequestData();
}

#define ESTIMATOR_RUNNING ((program_state==STATE_LAUNCHING) || (program_state==STATE_DRAG_CONTROL))

void processIMU()
{
    if(LSM::IMUNewData())
    {
        LSM::IMUData dat = LSM::ReadIMU();

        // Flip y and x because the LSM9DS1 apparently rolls with left-handed coords by default...
        sensor_vals.accel = LSM::ACCEL_TO_MPSPS*vec3(dat.accel.y, dat.accel.x, dat.accel.z);
        sensor_vals.gyro = LSM::GYRO_TO_RADPS*vec3(dat.gyro.y,dat.gyro.x,dat.gyro.z);

        if(ESTIMATOR_RUNNING){
            Control::Estimator_IMU(imu_step,sensor_vals.accel,sensor_vals.gyro);
        }
        else if(program_state == STATE_PAD_IDLE){
            gravityFilter.ingest(sensor_vals.accel);
            gyroFilter.ingest(sensor_vals.gyro);
            launchFilter.ingest(sensor_vals.accel[2]);
        }
        imu_step = 0;
    }
}

void processMagneto()
{
    if(LSM::MagNewData())
    {
        LSM::vec_i16_t dat = LSM::ReadMag();

        // Different axis shifts because the LSM9DS1 coordinates are fucked
        sensor_vals.magneto = LSM::MAG_TO_UT*vec3(dat.y,-dat.x,dat.z);

        if(ESTIMATOR_RUNNING){
            Control::Estimator_Magnetometer(sensor_vals.magneto);
        }
        else if(program_state == STATE_PAD_IDLE){
            magFilter.ingest(sensor_vals.magneto);
        }
    }
}


void processAltimeter()
{
    MPL::AltTempData data;
    if(MPL::CheckAndRead(&data))
    {
        sensor_vals.alt = MPL::ALT_TO_M*data.alt;

        if(ESTIMATOR_RUNNING){
            Control::Estimator_Altimeter(sensor_vals.alt);
        }
        else if(program_state == STATE_PAD_IDLE){
            groundFilter.ingest(sensor_vals.alt);
        }
    } else {
        Serial.println("FAIL");
    }

    MPL::RequestData(); // Re-request if somehow we really screwed up and weren't requesting
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

        if(sampling_timer > SAMPLE_TIME_US)
        {
            sampling_timer = 0;
            ++sample_n;


            // Consult Sensors if ready
            if(sample_n % IMU_DECIMATION == 0)
            {
                processIMU();
            }
            // if(sample_n % MAGNETOMETER_DECIMATION == 0)
            // {
            //     processMagneto();
            // }
            if(sample_n % ALTIMETER_DECIMATION == 0)
            {
                processAltimeter();
            }

            commandFlaps();


            switch(program_state) // State change logic
            {
            case STATE_PAD_IDLE:
                Serial.println(gravityFilter.value[2]);
                // if(launchFilter.value > LAUNCH_ACCEL_THRESHOLD)
                if(Serial.available())
                {
                    Control::Init(groundFilter.value, gravityFilter.value, magFilter.value, gyroFilter.value);
                    program_state = STATE_LAUNCHING;
                }
                break;
            case STATE_LAUNCHING:
                // Serial.println(Control::p_est.getAltitude());

                if(sample_n%20==0)
                {
                    printMat(Serial,Control::a_est.covar*1e6);
                    printQuat(Serial,Control::a_est.attitude);
                    Serial.println(acosf(Control::a_est.cosineTheta)*180.0f/M_PI);
                    printMat(Serial,Control::p_est.covar*1e6);
                    Serial.println(Control::p_est.getAltitude());
                    Serial.println(sensor_vals.alt-Control::ground_alt);
                    Serial.println();
                }
                
                // Serial.printf("[%f,%f,%f,%f]\r\n",Control::a_est.attitude[0],Control::a_est.attitude[1],Control::a_est.attitude[2],Control::a_est.attitude[3]);

                // Serial.println();
                // if(sensor_vals.accel[2] < 0) { // Negative longitudinal acceleration (drag)
                //     // Check if we are too inclined at burnout to allow deployment
                //     if(Control::a_est.cosineTheta < cosf(DEG_TO_RAD*NO_DEPLOY_ANGLE))
                //     {
                //         recovery_blink = 2;
                //         program_state = STATE_DESCENT; // Skip control phase
                //     } else {
                //         program_state = STATE_DRAG_CONTROL;
                //     }
                // }
                break;
            case STATE_DRAG_CONTROL:
                
                break;
            case STATE_DESCENT: // Will exit loop and fall through to end, runs once
                break;
            }

        }
    }

    // loggerFile.flush();

    while(1)
    {
        blinker.blinkDigits(recovery_blink);
        delay(5*LED_DIGIT_SPACE_MS);
    }

}
