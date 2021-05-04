//
// Created by Nikki Pack on 04/05/2021.
//
#include "main.h"

float calibrateTransmitter(float ch2_val_2, float ch3_val_2);
_Bool communicationsCheck(float ch2_val_1, float ch2_val_2, float ch3_val_1, float ch3_val_2);
void enableMotors();
void disableMotors();
void enableServos(TIM_HandleTypeDef timer, uint32_t pulse_width);
void activateESCs(TIM_HandleTypeDef timer);
void applyThrottle(TIM_HandleTypeDef timer, uint32_t R_R_pulse_width, uint32_t R_L_pulse_width, uint32_t F_R_pulse_width, uint32_t F_L_pulse_width);
void differential(int16_t angle, uint32_t F_L_pulse_width, uint32_t F_R_pulse_width, uint32_t R_L_pulse_width, uint32_t R_R_pulse_width);
int16_t steerServos(TIM_HandleTypeDef timer, float rx_signal);
int16_t RPMtoPWM(int16_t rpm_value);
int16_t motorPID(int16_t target_pwm, int16_t calculated_rpm);

