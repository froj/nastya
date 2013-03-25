//      hardware.c
//      
//      Copyright 2009 Antoine Albertelli <a.albertelli@cvra.ch>
//      
//      This program is free software; you can redistribute it and/or modify
//      it under the terms of the GNU General Public License as published by
//      the Free Software Foundation; either version 2 of the License, or
//      (at your option) any later version.
//      
//      This program is distributed in the hope that it will be useful,
//      but WITHOUT ANY WARRANTY; without even the implied warranty of
//      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//      GNU General Public License for more details.
//      
//      You should have received a copy of the GNU General Public License
//      along with this program; if not, write to the Free Software
//      Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
//      MA 02110-1301, USA.

/**
 * \file hardware.c
 * \brief This files contains the hardware functions.
 *
 * This files contains functions to do the hardware init of the board, to set the pwm sign of the VNH2SP30E, and to set the LEDS.
 */

#include <aversive.h>
#include <aversive/error.h>
#include <scheduler.h>
#include <general_errors.h>


#include "error_numbers.h"
#include "hardware.h"
#include "cvra_cs.h"
#include "adresses.h"

#include <stdio.h>


void cvra_set_uart_speed(int32_t *uart_adress, int baudrate) {
#ifdef COMPILE_ON_ROBOT
    int32_t divisor;
    /* Formule tiree du Embedded IP User Guide page 7-4 */
    divisor = (int32_t)(((float)ALT_CPU_FREQ/(float)baudrate) + 0.5);
    IOWR(uart_adress, 0x04, divisor); // ecrit le diviseur dans le bon registre
#endif
    DEBUG(E_UART, "Changed UART speed to %d at adress %p\n", baudrate, uart_adress);
}



/** 
 * \brief Inits the board.
 *
 * This functions configures the pin direction on each port, inits the PWM, inits the ADC. 
 * It also init the UART, and calls fdevopen() to setup UART for printf()/scanf()/CLI.
 */
void cvra_board_init(void) {
    /* Init de l'ADC */
#ifdef COMPILE_ON_ROBOT
    cvra_adc_init(&robot.analog_in, ANALOG_SPI_ADRESS, ANALOGIN_IRQ);
#endif
    /* On manage les capteurs toutes les 5 ms. */
    scheduler_add_periodical_event_priority(cvra_board_manage_sensors,NULL,5000/SCHEDULER_UNIT,130);
    //cvra_board_manage_outputs();
}


void cvra_board_manage_sensors(__attribute__((unused)) void * dummy) {
    //cvra_adc_start_scan(&robot.analog_in);
}

void cvra_get_avoiding_sensors(int *l, int *r) {
    /* C'est branche sur un analog in => seuillage. la val est sur 10
     * bits donc sur 1024. */
    /* Le capteur gauche est sur le port analogique 3. */
    //*l = (cvra_adc_get_value(&robot.analog_in, ADC_OBSTACLE_LEFT) < 500);
    
    ///* Le capteur droit est sur le port analogique 4. */
    //*r = (cvra_adc_get_value(&robot.analog_in, ADC_OBSTACLE_RIGHT) < 500);
}

void cvra_board_manage_outputs(void) {
    //int8_t outval=0;
    //outval |= right_pump_on     << 1;
    //outval |= right_pump_dir    << 0;
    //outval |= left_pump_on    << 2;
    //outval |= left_pump_dir   << 3;
    //IOWR(DIGITAL_OUTPUT0, 0, outval);
}

