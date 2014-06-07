#ifndef HARDWARE_H
#define HARDWARE_H

#include <stdint.h>
#include <stdbool.h>
#include "cvra_param_robot.h"

/** Sets the baudrate of a given UART
 *
 * @author Antoine Albertelli, CVRA
 * @param [in] uart_adress The base adress of the altera UART module.
 * @param [in] baudrate The desired baudrate
 */
void hw_set_uart_speed(int32_t *uart_adress, int baudrate);

#define HW_WHEEL_MOTOR_MAX_PWM DC_PWM_MAX_VALUE
void hw_set_wheel_0_motor_pwm(uint32_t pwm);
void hw_set_wheel_1_motor_pwm(uint32_t pwm);
void hw_set_wheel_2_motor_pwm(uint32_t pwm);
void hw_set_net(bool on);
void hw_cannon_arm_all(void);
void hw_cannon_fire(int index);
void hw_finger_extend(int index);
void hw_finger_retract(int index);

#define HW_WHEEL_ENCODER_STEPS_PER_REVOLUTION ROBOT_ENCODER_RESOLUTION
uint32_t hw_get_wheel_0_encoder(void);
uint32_t hw_get_wheel_1_encoder(void);
uint32_t hw_get_wheel_2_encoder(void);

#endif // HARDWARE_H
