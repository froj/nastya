#include <cvra_servo.h>
#include "pingpongcannon.h"



void ppc_init(ppc_t *cannon){
    memset(cannon, 0, sizeof(ppc_t));

    cs_init(&cannon->drum_cs);
    pid_init(&cannon->drum_pid);
    pid_set_maximums(&cannon->drum_pid, 0, 0, 0); //TODO
    pid_set_gains(&cannon->drum_pid, 100, 0, 0); //TODO
    pid_set_out_shift(&cannon->drum_pid, 10); //TODO
    cs_set_correct_filter(&cannon->drum_cs, pid_do_filter, &cannon->drum_pid);

    cs_init(&cannon->cannon_cs);
    pid_init(&cannon->cannon_pid);
    pid_set_maximums(&cannon->cannon_pid, 0, 0, 0); //TODO
    pid_set_gains(&cannon->cannon_pid 100, 10, 0); //TODO
    pid_set_out_shift(&cannon->cannon_pid 10); //TODO
    cs_set_correct_filter(&cannon->cannon_cs, pid_do_filter, &cannon->cannon_pid);
}

void ppc_manage(ppc_t *cannon){
    
}

void pcc_manage_cs(ppc_t *cannon){
    cs_manage(&cannon->drum_cs);
}

void ppc_aspirate(ppc_t *cannon){

}

void ppc_shoot(ppc_t *cannon){

}

void ppc_eject(ppc_t *cannon){

}

void ppc_test_ball(ppc_t *cannon){

}

int32_t get_cannon_turbine_speed(ppc_t *cannon){

}
