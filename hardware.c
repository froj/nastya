#include <cvra_dc/cvra_dc.h>
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
    cvra_dc_set_pwm0(HEXMOTORCONTROLLER_BASE, pwm);
}

void hw_set_wheel_1_motor_pwm(uint32_t pwm)
{
    cvra_dc_set_pwm1(HEXMOTORCONTROLLER_BASE, -pwm);
}

void hw_set_wheel_2_motor_pwm(uint32_t pwm)
{
    cvra_dc_set_pwm2(HEXMOTORCONTROLLER_BASE, -pwm);
}


uint32_t hw_get_wheel_0_encoder(void)
{
    return -cvra_dc_get_encoder0(HEXMOTORCONTROLLER_BASE);
}

uint32_t hw_get_wheel_1_encoder(void)
{
    return -cvra_dc_get_encoder1(HEXMOTORCONTROLLER_BASE);
}

uint32_t hw_get_wheel_2_encoder(void)
{
    return -cvra_dc_get_encoder2(HEXMOTORCONTROLLER_BASE);
}

