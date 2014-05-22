#include <cvra_dc/cvra_dc.h>
#include <cvra_servo.h>
#include "cvra_param_robot.h"
#include "adresses.h"

#include "hardware.h"


#define UART_FREQ PIO_FREQ

void hw_set_uart_speed(int32_t *uart_adress, int baudrate) {
#ifdef COMPILE_ON_ROBOT
    int32_t divisor;
    /* Formule tiree du Embedded IP User Guide page 7-4 */
    divisor = (int32_t)(((float)UART_FREQ/(float)baudrate) + 0.5);
    IOWR(uart_adress, 0x04, divisor); // ecrit le diviseur dans le bon registre
#endif
}


void hw_set_wheel_0_motor_pwm(uint32_t pwm)
{
    cvra_dc_set_pwm0((void*)HEXMOTORCONTROLLER_BASE, pwm);
}

void hw_set_wheel_1_motor_pwm(uint32_t pwm)
{
    cvra_dc_set_pwm1((void*)HEXMOTORCONTROLLER_BASE, -pwm);
}

void hw_set_wheel_2_motor_pwm(uint32_t pwm)
{
    cvra_dc_set_pwm2((void*)HEXMOTORCONTROLLER_BASE, -pwm);
}

void hw_set_net(bool on)
{
    if (on) {
        cvra_dc_set_pwm3((void*)HEXMOTORCONTROLLER_BASE, DC_PWM_MAX_VALUE);
    } else {
        cvra_dc_set_pwm3((void*)HEXMOTORCONTROLLER_BASE, 0);
    }
}


uint32_t hw_get_wheel_0_encoder(void)
{
    return -cvra_dc_get_encoder0((void*)HEXMOTORCONTROLLER_BASE);
}

uint32_t hw_get_wheel_1_encoder(void)
{
    return -cvra_dc_get_encoder1((void*)HEXMOTORCONTROLLER_BASE);
}

uint32_t hw_get_wheel_2_encoder(void)
{
    return -cvra_dc_get_encoder2((void*)HEXMOTORCONTROLLER_BASE);
}


static const int32_t cannon_arm_pos[6] = {
    14500,
    14000,
    19000,
    16000,
    15000,
    15000
};

static const int32_t cannon_fire_pos[6] = {
    19000,
    12000,
    14000,
    13000,
    12500,
    11000
};

void hw_cannon_arm_all(void)
{
    int i;
    for (i = 0; i < 6; i++) {
        cvra_servo_set(i, cannon_arm_pos[i]);
    }
}

void hw_cannon_fire(int index)
{
    if (index < 1 || index > 6)
        return;
    cvra_servo_set(index-1, cannon_fire_pos[index-1]);
}


