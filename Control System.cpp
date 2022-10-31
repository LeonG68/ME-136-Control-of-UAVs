#include "UserCode.hpp"
#include "UtilityFunctions.hpp"
#include "Vec3f.hpp"
#include <stdio.h>

//An example of a variable that persists beyond the function call.
float exampleVariable_float = 0.0f; //Note the trailing 'f' in the number. This is to force single precision floating point.
Vec3f exampleVariable_Vec3f = Vec3f(0, 0, 0);
int exampleVariable_int = 0;

//We keep the last inputs and outputs around for debugging:
MainLoopInput lastMainLoopInputs;
MainLoopOutput lastMainLoopOutputs;

//Some constants that we may use:
const float mass = 32e-3f;           // mass of the quadcopter [kg]
const float gravity = 9.81f;         // acceleration of gravity [m/s^2]
const float inertia_xx = 16e-6f;     //MMOI about x axis [kg.m^2]
const float inertia_yy = inertia_xx; //MMOI about y axis [kg.m^2]
const float inertia_zz = 29e-6f;     //MMOI about z
const float dt = 1.0f / 500.0f;      //[s] period between successive calls to MainLoop

//Our Code

//Initializes variables and arrays to be used in the estimator
Vec3f estGyroBias = Vec3f(0, 0, 0);
Vec3f rateGyro_corr = Vec3f(0, 0, 0);

float estRoll = 0;
float estPitch = 0;
float estYaw = 0;

float rho = 0.01;
float rollMeas = 0;
float pitchMeas = 0;

float estHeight = 0.0f;
float estDist_1 = 0.0f;
float estDist_2 = 0.0f;
float estVelocity_1 = 0.0f;
float estVelocity_2 = 0.0f;
float estVelocity_3 = 0.0f;

float lastHeightMeas_meas = 0.0f;
float lastHeightMeas_time = 0.0f;
float const mixHeight = 0.3f;

//Time constants
float const timeConstant_rollRate = 0.04f;
float const timeConstant_pitchRate = timeConstant_rollRate;
float const timeConstant_yawRate = 0.1f;

float const timeConstant_rollAngle = 0.12f;
float const timeConstant_pitchAngle = timeConstant_rollAngle;
float const timeConstant_yawAngle = 0.2f;
const float timeConst_horizVel = 2.0f;

//Initialize variable
float l = .033;
float m = .032;
float k = 0.01;
float totalC;
//Input Desired Angles
float desRoll = 0;
float desPitch = 0;
float desYaw = 0;

// Natural frequency and damping ratio
const float natFreq_height = 2.0f;
const float dampingRatio_height = 0.7f;

MainLoopOutput MainLoop(MainLoopInput const &in)
{

    MainLoopOutput outVals;

    //Our Code
    //Collects data in the first second to correct for the bias. Otherwise outputs the corrected gyro data.
    if (in.currentTime < 1.0f)
    {
        estGyroBias = estGyroBias + (in.imuMeasurement.rateGyro / 500.0f);
    }
    else
    {
        rateGyro_corr = in.imuMeasurement.rateGyro - estGyroBias;
    }

    // height estimator :
    estHeight = estHeight + estVelocity_3 * dt;

    // velocity prediction (Velocity is constant)
    estVelocity_1 = estVelocity_1 + 0 * dt;
    estVelocity_2 = estVelocity_2 + 0 * dt;
    estVelocity_3 = estVelocity_3 + 0 * dt;

    // correction step directly after the prediction step
    if (in.heightSensor.updated)
    { //check that the measurment is reasonable
        if (in.heightSensor.value < 5.0f)
        {
            float hMeas = in.heightSensor.value * cosf(estRoll) * cosf(estPitch);
            estHeight = (1.0f - mixHeight) * estHeight + mixHeight * hMeas;
            float v3Meas = (hMeas - lastHeightMeas_meas) / (in.currentTime - lastHeightMeas_time);
            estVelocity_3 = (1 - mixHeight) * estVelocity_3 + mixHeight * v3Meas;
            // store this measurment for the next velocity update
            lastHeightMeas_meas = hMeas;
            lastHeightMeas_time = in.currentTime;
        }
    }
    // correction step:
    float const mixHorizVel = 0.1f;
    if (in.opticalFlowSensor.updated)
    {
        float sigma_1 = in.opticalFlowSensor.value_x;
        float sigma_2 = in.opticalFlowSensor.value_y;
        float div = cosf(estRoll) * cosf(estPitch);

        if (div > 0.5f)
        {
            float deltaPredict = estHeight / div; // this is delta in the equation
            float v1Meas = (-sigma_1 + in.imuMeasurement.rateGyro.y) * deltaPredict;
            float v2Meas = (-sigma_2 - in.imuMeasurement.rateGyro.x) * deltaPredict;
            estVelocity_1 = (1 - mixHorizVel) * estVelocity_1 + mixHorizVel * v1Meas;
            estVelocity_2 = (1 - mixHorizVel) * estVelocity_2 + mixHorizVel * v2Meas;
        }
    }

    // Horizontal Velocity Controller
    // Desired accelerations

    float desAcc1 = -(1 / timeConst_horizVel) * estVelocity_1;
    float desAcc2 = -(1 / timeConst_horizVel) * estVelocity_2;

    // Desired angles
    desRoll = -desAcc2 / gravity;
    if (in.userInput.buttonGreen)
    {
        desPitch = 0.5236f;
    }
    else
    {
        desPitch = desAcc1 / gravity;
    }

    desYaw = 0.0f;

    // Vertical Velocity Controller
    // Desired normalized thrust
    const float desHeight = 0.5f;
    const float desAcc3 = -2 * dampingRatio_height * natFreq_height * estVelocity_3 - natFreq_height * natFreq_height * (estHeight - desHeight);
    float desNormalizedAcceleration = (gravity + desAcc3) / (cosf(estRoll) * cosf(estPitch));
    float cZigma = mass * desNormalizedAcceleration;

    //Integrator using equation 3.1. Uses the gyroscopic measurements to estimate for the attitudes
    estRoll = estRoll + rateGyro_corr.x * float(1) / float(500);
    estPitch = estPitch + rateGyro_corr.y * float(1) / float(500);
    estYaw = estYaw + rateGyro_corr.z * float(1) / float(500);

    //Integrator using equation 3.4. Uses the accelerometers to correct for drift in the estimator
    pitchMeas = -1 * in.imuMeasurement.accelerometer.x / gravity;
    rollMeas = in.imuMeasurement.accelerometer.y / gravity;
    estPitch = (1.0f - rho) * (estPitch) + rho * pitchMeas;
    estRoll = (1.0f - rho) * (estRoll) + rho * rollMeas;

    //Calls PrintStatus to print out desired data
    PrintStatus();

    //Code for Lab 4

    //     // Changes desired pitch to .5236 radians.
    //      if(in.userInput.buttonGreen) {
    //          Pitchdes= .5236;
    //      }
    //      //Input desired acceleration
    //      float acel = 8;
    //      float force = 0;

    //4.3 Calculating dynamics
    float p_cmd = (-1 / timeConstant_rollAngle) * (estRoll - desRoll);
    float q_cmd = (-1 / timeConstant_pitchAngle) * (estPitch - desPitch);
    float r_cmd = (-1 / timeConstant_yawAngle) * (estYaw - desYaw);

    float pd_cmd = (-1 / timeConstant_rollRate) * (rateGyro_corr.x - p_cmd);
    float qd_cmd = (-1 / timeConstant_pitchRate) * (rateGyro_corr.y - q_cmd);
    float rd_cmd = (-1 / timeConstant_yawRate) * (rateGyro_corr.z - r_cmd);

    float Jxx = 16e-6;
    float Jzz = 29e-6;
    float n12 = Jxx * pd_cmd + (Jxx - Jzz) * (-1 * rateGyro_corr.y * rateGyro_corr.z);
    float n22 = Jxx * qd_cmd + (Jxx - Jzz) * (rateGyro_corr.x * rateGyro_corr.z);
    float n32 = Jzz * rd_cmd;

    //Input desired angular acceleration
    float aAcelx = pd_cmd;
    float aAcely = qd_cmd;
    float aAcelz = rd_cmd;

    //Thrust to force
    //         force = acel * mass;

    //Mixer Equation
    //float n1 = 16*10^-6*aAcelx;
    //float n2 = 16*10^-6*aAcely;
    //float n3 = 29*10^-6*aAcelz;

    float cp1 = 1.0f / 4.0f * (cZigma + n12 / l - n22 / l + n32 / k);
    float cp2 = 1.0f / 4.0f * (cZigma - n12 / l - n22 / l - n32 / k);
    float cp3 = 1.0f / 4.0f * (cZigma - n12 / l + n22 / l + n32 / k);
    float cp4 = 1.0f / 4.0f * (cZigma + n12 / l + n22 / l - n32 / k);

    //If statement that checks if the GUI's blue button is pressed.
    if (in.userInput.buttonBlue)
    {
        //If the blue button is pressed, set the motor command to desired value
        outVals.motorCommand1 = pwmCommandFromSpeed(speedFromForce(cp1));
        outVals.motorCommand2 = pwmCommandFromSpeed(speedFromForce(cp2));
        outVals.motorCommand3 = pwmCommandFromSpeed(speedFromForce(cp3));
        outVals.motorCommand4 = pwmCommandFromSpeed(speedFromForce(cp4));
        printf("\n cp1");
        printf("x=%6.3f,", cp1);
    }
    else
    {
        //Else set the motor command to 0
        outVals.motorCommand1 = 0;
        outVals.motorCommand2 = 0;
        outVals.motorCommand3 = 0;
        outVals.motorCommand4 = 0;
    }
    //Our Code

    //copy the inputs and outputs:
    lastMainLoopInputs = in;
    lastMainLoopOutputs = outVals;

    //Our Code
    //Outputs the data we want into the logs where we can analyze the data.
    outVals.telemetryOutputs_plusMinus100[0] = estRoll;
    outVals.telemetryOutputs_plusMinus100[1] = estPitch;
    outVals.telemetryOutputs_plusMinus100[2] = estYaw;
    outVals.telemetryOutputs_plusMinus100[3] = estVelocity_1;
    outVals.telemetryOutputs_plusMinus100[4] = estVelocity_2;
    outVals.telemetryOutputs_plusMinus100[5] = estVelocity_3;
    outVals.telemetryOutputs_plusMinus100[6] = estHeight;
    outVals.telemetryOutputs_plusMinus100[7] = desRoll;
    outVals.telemetryOutputs_plusMinus100[8] = desPitch;
    outVals.telemetryOutputs_plusMinus100[9] = desNormalizedAcceleration;
    outVals.telemetryOutputs_plusMinus100[10] = cZigma;

    //  outVals.telemetryOutputs_plusMinus100[7] = in.heightSensor.value;
    //  outVals.telemetryOutputs_plusMinus100[8] = in.opticalFlowSensor.value_x;
    //  outVals.telemetryOutputs_plusMinus100[9] = in.opticalFlowSensor.value_y;

    //outVals.telemetryOutputs_plusMinus100[3] = estRoll;
    //outVals.telemetryOutputs_plusMinus100[4] = estPitch;
    //outVals.telemetryOutputs_plusMinus100[5] = pd_cmd;
    //outVals.telemetryOutputs_plusMinus100[6] = qd_cmd;
    //outVals.telemetryOutputs_plusMinus100[7] = rd_cmd;
    //outVals.telemetryOutputs_plusMinus100[5] = p_cmd;
    //outVals.telemetryOutputs_plusMinus100[6] = q_cmd;
    //outVals.telemetryOutputs_plusMinus100[7] = r_cmd;

    return outVals;
    //Our Code
}

void PrintStatus()
{

    printf("Last Range = %6.3f m, \n",
           double(lastMainLoopInputs.heightSensor.value));
    printf("Last Flow:\t x = %6.3f \t y = %6.3f \n\n",
           double(lastMainLoopInputs.opticalFlowSensor.value_x),
           double(lastMainLoopInputs.opticalFlowSensor.value_y));
}

