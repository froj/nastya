#include <aversive.h>
#include <string.h>
#include <cvra_servo.h>
#include <cvra_dc.h>
#include "adresses.h"
#include "pingpongcannon.h"



void ppc_init(ppc_t *cannon){
    memset(cannon, 0, sizeof(ppc_t));

    cs_init(&cannon->drum_cs);
    pid_init(&cannon->drum_pid);
    pid_set_maximums(&cannon->drum_pid, 0, 0, 0); //TODO
    pid_set_gains(&cannon->drum_pid, 100, 0, 0); //TODO
    pid_set_out_shift(&cannon->drum_pid, 10); //TODO
    cs_set_correct_filter(&cannon->drum_cs, pid_do_filter, &cannon->drum_pid);
    cs_set_process_in(&cannon->drum_cs, cvra_dc_set_pwm3, (void*)HEXMOTORCONTROLLER_BASE);
    cs_set_process_out(&cannon->drum_cs, cvra_dc_get_encoder3, (void*)HEXMOTORCONTROLLER_BASE);
    cs_set_consign(&cannon->drum_cs, 0);

    cs_init(&cannon->cannon_cs);
    pid_init(&cannon->cannon_pid);
    pid_set_maximums(&cannon->cannon_pid, 0, 0, 0); //TODO
    pid_set_gains(&cannon->cannon_pid, 100, 10, 0); //TODO
    pid_set_out_shift(&cannon->cannon_pid, 10); //TODO
    cs_set_correct_filter(&cannon->cannon_cs, pid_do_filter, &cannon->cannon_pid);
    cs_set_process_in(&cannon->cannon_cs, cvra_servo_set0, (void*)SERVOS_BASE);
    //TODO
    //cs_set_process_out(&cannon->drum_cs, TODO, (void*)HEXMOTORCONTROLLER_BASE);
    cs_set_consign(&cannon->cannon_cs, 0);

    cs_enable(&cannon->drum_cs);
    cs_enable(&cannon->cannon_cs);

}

void ppc_manage(ppc_t *cannon){
    
}

void pcc_manage_cs(ppc_t *cannon){
    cs_manage(&cannon->drum_cs);
    cs_manage(&cannon->cannon_cs);
}

void ppc_aspiration_on(ppc_t *cannon){
    cvra_servo_set((void*)SERVOS_BASE, 1, 15000); 
}

void ppc_aspiration_off(ppc_t *cannon){
    cvra_servo_set((void*)SERVOS_BASE, 1, 7000); 
}

void ppc_aspiration_invert(ppc_t *cannon){

}

void ppc_shoot(ppc_t *cannon){

}

void ppc_eject(ppc_t *cannon){

}

void ppc_test_ball(ppc_t *cannon){

}

void ppc_aspirater_down(void)
{
    cvra_servo_set((void*)SERVOS_BASE, 0, 8000); 
}

void ppc_aspirater_up(void)
{
    cvra_servo_set((void*)SERVOS_BASE, 0, 8000); 
}
