//
// Created by Nikki Pack on 04/05/2021.
//
#include "main.h"
#include <math.h>		// Mathematic Function Library
#include <stdlib.h>		// Standard Function Library

// PID

#define RPM_KP				2.1		// Right Motor P coefficient.
#define RPM_KI				0.001		// Right Motor I coefficient.
#define RPM_KD				10		// Right Motor D coefficient.
#define MIN_RPM  			0	    // Minimum possible RPM.
#define MAX_RPM  			5400	// Maximum possible RPM.
#define SERVO_DEGREES_MAX	45
#define WHEEL_BASE_LENGTH	0.425		// Wheel base length in m.
#define WHEEL_BASE_WIDTH	0.385       // Wheel base width in m.
#define MAX_STEERING_ANGLE	33
#define MIN_STEERING_ANGLE  -33

extern _Bool communicationActive;
extern _Bool differentialActive;

int16_t i_error = 0;
int16_t p_error = 0;

extern int16_t front_l_rpm;
extern int16_t front_r_rpm;
extern int16_t rear_l_rpm;
extern int16_t rear_r_rpm;

uint16_t ESC_MIN_PWM;
uint16_t ESC_MAX_PWM;
uint16_t ESC_NEUTRAL_PWM;
uint16_t THROTTLE_NEUTRAL;

uint32_t F_L_diff = 0, F_R_diff = 0, R_L_diff = 0, R_R_diff = 0;
int16_t	ang = 0;

extern TIM_HandleTypeDef htim1;

// SERVO
extern float TRAN_Y_MAX;
extern float TRAN_Y_MIN;
extern float TRAN_X_MAX;
extern float TRAN_X_MIN;

uint16_t R_SERVO_CENTER	= 1050;
uint16_t L_SERVO_CENTER	= 1050;
uint16_t RX_pulse_width = 0;
uint16_t SERVO_MIN_PWM = 0;
uint16_t SERVO_MAX_PWM = 0;
uint16_t SERVO_NEUTRAL_PWM = 0;
uint16_t THROTTLE_DEADZONE = 10;
uint8_t ONE_DEGREE_STEP = 0;
uint8_t SERVO_ANGLE = 0;
int16_t L_SERVO_ANGLE = 0;
int16_t R_SERVO_ANGLE = 0;

// capture values for comms check.
float old_CH2_Val1 = 0;
float old_CH2_Val2 = 0;
float old_CH3_Val1 = 0;
float old_CH3_Val2 = 0;

void calibrateTransmitter(float ch2_val_2, float ch3_val_2)
{

    if (ch2_val_2 > TRAN_X_MAX)
        TRAN_X_MAX = ch2_val_2;

    if (ch2_val_2 < TRAN_X_MIN && ch2_val_2 != 0)
        TRAN_X_MIN = ch2_val_2;

    if (ch3_val_2 > TRAN_Y_MAX)
        TRAN_Y_MAX = ch3_val_2;

    if (ch3_val_2 < TRAN_Y_MIN && ch3_val_2 != 0)
        TRAN_Y_MIN = ch3_val_2;

    // calculate ESC PWM range
    ESC_MIN_PWM = 1000 + ((TRAN_Y_MIN / TRAN_Y_MAX) * 1000);
    ESC_MAX_PWM = 1000 + ((TRAN_Y_MAX / TRAN_Y_MAX) * 1000);
    ESC_NEUTRAL_PWM = ESC_MIN_PWM + ((ESC_MAX_PWM - ESC_MIN_PWM) / 2);  //1605

    // calculate SERVO PWM range and step size
    SERVO_MIN_PWM = L_SERVO_CENTER + ((TRAN_X_MIN / TRAN_X_MAX) * 1000);
    SERVO_MAX_PWM = L_SERVO_CENTER + ((TRAN_X_MAX / TRAN_X_MAX) * 1000);
    SERVO_NEUTRAL_PWM = SERVO_MIN_PWM + ((SERVO_MAX_PWM - SERVO_MIN_PWM) / 2);
    ONE_DEGREE_STEP = (SERVO_MAX_PWM - SERVO_NEUTRAL_PWM) / SERVO_DEGREES_MAX;
}

_Bool communicationsCheck(float ch2_val_1, float ch2_val_2, float ch3_val_1, float ch3_val_2)
{
    // if reciever not updating
    if(ch2_val_1 == old_CH2_Val1 && ch2_val_2 == old_CH2_Val2 && ch3_val_1 == old_CH3_Val1 && ch3_val_2 == old_CH3_Val2)
    {
        communicationActive = 0;
        differentialActive = !differentialActive;
    }
    else
    {
        communicationActive = 1;
    }

    old_CH2_Val1 = ch2_val_1;
    old_CH2_Val2 = ch2_val_2;
    old_CH3_Val1 = ch3_val_1;
    old_CH3_Val2 = ch3_val_2;

    return communicationActive;
}

void enableMotors()
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

}

void disableMotors()
{
    htim1.Instance->CCR1 = 0;
    htim1.Instance->CCR2 = 0;
    htim1.Instance->CCR3 = 0;
    htim1.Instance->CCR4 = 0;

    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
}

void enableServos(TIM_HandleTypeDef timer, uint32_t pulse_width)
{
    HAL_TIM_PWM_Start(&timer, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&timer, TIM_CHANNEL_2);

    timer.Instance->CCR1 = pulse_width;
    timer.Instance->CCR2 = pulse_width;
}

void activateESCs(TIM_HandleTypeDef timer)
{
    timer.Instance->CCR1 = ESC_MAX_PWM;
    timer.Instance->CCR2 = ESC_MAX_PWM;
    timer.Instance->CCR3 = ESC_MAX_PWM;
    timer.Instance->CCR4 = ESC_MAX_PWM;
    HAL_Delay(100);
    timer.Instance->CCR1 = ESC_MIN_PWM;
    timer.Instance->CCR2 = ESC_MIN_PWM;
    timer.Instance->CCR3 = ESC_MIN_PWM;
    timer.Instance->CCR4 = ESC_MIN_PWM;
}

void applyThrottle(TIM_HandleTypeDef timer, uint32_t F_L_pulse_width, uint32_t F_R_pulse_width, uint32_t R_L_pulse_width, uint32_t R_R_pulse_width)
{
    timer.Instance->CCR1 = F_L_pulse_width;
    timer.Instance->CCR2 = F_R_pulse_width;
    timer.Instance->CCR3 = R_L_pulse_width;
    timer.Instance->CCR4 = R_R_pulse_width;
}

void differential(int16_t angle, uint32_t F_L_pulse_width, uint32_t F_R_pulse_width, uint32_t R_L_pulse_width, uint32_t R_R_pulse_width)
{
    /* Initialise Differential speeds and absolute angle variables, */

    /* Initialise ratio variables for differential equation. */
    float r1 = 0, r2 = 0, r3 = 0;

    ang = abs(angle);  // Calculates absolute angle of Servo.

    r2 = WHEEL_BASE_LENGTH / tan(ang*(3.14159265 / 180));

    r1 = r2 - (WHEEL_BASE_WIDTH / 2);

    r3 = r2 + (WHEEL_BASE_WIDTH / 2);

    THROTTLE_NEUTRAL = 1500;

    // if turning left
    if(angle < 0 && F_L_pulse_width > THROTTLE_NEUTRAL && F_R_pulse_width > THROTTLE_NEUTRAL
       && R_L_pulse_width > THROTTLE_NEUTRAL && R_R_pulse_width > THROTTLE_NEUTRAL) {
        /* Set left motors at max speed and left as lowered speed*/
        F_L_diff = (uint32_t) (THROTTLE_NEUTRAL + ((F_L_pulse_width - THROTTLE_NEUTRAL) * (r1 / r3)));
        R_L_diff = (uint32_t) (THROTTLE_NEUTRAL + ((R_L_pulse_width - THROTTLE_NEUTRAL) * (r1 / r3)));
        F_R_diff = (uint32_t) F_R_pulse_width;
        R_R_diff = (uint32_t) R_R_pulse_width;
    }

    // if turning right
    if(angle > 0 && F_L_pulse_width > THROTTLE_NEUTRAL && F_R_pulse_width > THROTTLE_NEUTRAL
       && R_L_pulse_width > THROTTLE_NEUTRAL && R_R_pulse_width > THROTTLE_NEUTRAL) {
        /* Set right motors at max speed and right as lowered speed*/
        F_L_diff = (uint32_t) F_L_pulse_width;
        R_L_diff = (uint32_t) R_L_pulse_width;
        F_R_diff = (uint32_t) (THROTTLE_NEUTRAL + ((F_R_pulse_width - THROTTLE_NEUTRAL) * (r1 / r3)));
        R_R_diff = (uint32_t) (THROTTLE_NEUTRAL + ((R_R_pulse_width - THROTTLE_NEUTRAL) * (r1 / r3)));
    }

    if (angle == 0) {
        /* No difference in motor speeds. */
        F_L_diff = (uint32_t) F_L_pulse_width;
        R_L_diff = (uint32_t) R_L_pulse_width;
        F_R_diff = (uint32_t) F_R_pulse_width;
        R_R_diff = (uint32_t) R_R_pulse_width;
    }

    if (F_L_diff < (THROTTLE_NEUTRAL + THROTTLE_DEADZONE) && F_L_diff > (THROTTLE_NEUTRAL - THROTTLE_DEADZONE))
        F_L_diff = THROTTLE_NEUTRAL;

    if (R_L_diff < (THROTTLE_NEUTRAL + THROTTLE_DEADZONE) && R_L_diff > (THROTTLE_NEUTRAL - THROTTLE_DEADZONE))
        R_L_diff = THROTTLE_NEUTRAL;

    if (F_R_diff < (THROTTLE_NEUTRAL + THROTTLE_DEADZONE) && F_R_diff > (THROTTLE_NEUTRAL - THROTTLE_DEADZONE))
        F_R_diff = THROTTLE_NEUTRAL;

    if (R_R_diff < (THROTTLE_NEUTRAL + THROTTLE_DEADZONE) && R_R_diff > (THROTTLE_NEUTRAL - THROTTLE_DEADZONE))
        R_R_diff = THROTTLE_NEUTRAL;

    applyThrottle(htim1, F_L_diff, F_R_diff, R_L_diff, R_R_diff);
}

int16_t steerServos(TIM_HandleTypeDef timer, float rx_signal)
{
    RX_pulse_width = L_SERVO_CENTER + ((rx_signal / TRAN_X_MAX) * 1000);
    L_SERVO_ANGLE = ((RX_pulse_width - SERVO_NEUTRAL_PWM) / ONE_DEGREE_STEP);

    // if servo maxes out set maximum
    if(L_SERVO_ANGLE >= MAX_STEERING_ANGLE)
    {
        RX_pulse_width = (MAX_STEERING_ANGLE * ONE_DEGREE_STEP) + SERVO_NEUTRAL_PWM;
        L_SERVO_ANGLE = MAX_STEERING_ANGLE;
    }

    if (L_SERVO_ANGLE <= MIN_STEERING_ANGLE)
    {
        RX_pulse_width = (MIN_STEERING_ANGLE * ONE_DEGREE_STEP) + SERVO_NEUTRAL_PWM;
        L_SERVO_ANGLE = MIN_STEERING_ANGLE;
    }

    timer.Instance->CCR1 = RX_pulse_width;
    timer.Instance->CCR2 = RX_pulse_width;

    return L_SERVO_ANGLE;
}

int16_t RPMtoPWM(int16_t target_pwm, int16_t wheel_rpm)
{
    int16_t pwm_signal = 0;

    // if driving forward
    if(target_pwm >= ESC_NEUTRAL_PWM)
    {
        pwm_signal = ESC_NEUTRAL_PWM + ((wheel_rpm / MAX_RPM) * ESC_NEUTRAL_PWM);
    }

    // if reversing
    if(target_pwm < ESC_NEUTRAL_PWM)
    {
        pwm_signal = ESC_NEUTRAL_PWM - ((wheel_rpm / MAX_RPM) * ESC_NEUTRAL_PWM);
    }

    return pwm_signal;
}

int16_t motorPID(int16_t target_pwm, int16_t calculated_rpm)
{
    int16_t rpm_pwm = RPMtoPWM(target_pwm, calculated_rpm);
    int16_t p = 0, i = 0, d = 0, pid_pwm = 0, error = 0;

    error = target_pwm - rpm_pwm;

    p = RPM_KP * error;
    i = RPM_KI * (i_error + error);
    d = RPM_KD * (error - p_error);

    i_error = i;
    p_error = error;

    pid_pwm = ESC_MIN_PWM + p + i + d;

    /* Set Minimum/Maximum values. */
    if (pid_pwm <= ESC_MIN_PWM)
        pid_pwm = ESC_MIN_PWM;
    if (pid_pwm >= ESC_MAX_PWM)
        pid_pwm = ESC_MAX_PWM;

    return pid_pwm;
}