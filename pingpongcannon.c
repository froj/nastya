#include <aversive.h>
#include <scheduler.h>
#include <string.h>
#include <cvra_servo.h>
#include <cvra_dc.h>
#include "adresses.h"
#include "pingpongcannon.h"


int32_t get_shooting_speed(ppc_t *cannon);
int32_t get_light_barrier_state(int32_t mask);

void ppc_init(ppc_t *cannon){
    memset(cannon, 0, sizeof(ppc_t));

    cs_init(&cannon->drum_cs);
    pid_init(&cannon->drum_pid);
    pid_set_maximums(&cannon->drum_pid, 0, 0, 450); //TODO
    pid_set_gains(&cannon->drum_pid, 100, 0, 0); //TODO
    pid_set_out_shift(&cannon->drum_pid, 10); //TODO
    cs_set_correct_filter(&cannon->drum_cs, pid_do_filter, &cannon->drum_pid);
    cs_set_process_in(&cannon->drum_cs, cvra_dc_set_pwm3, (void*)HEXMOTORCONTROLLER_BASE);
    cs_set_process_out(&cannon->drum_cs, cvra_dc_get_encoder3, (void*)HEXMOTORCONTROLLER_BASE);
    cs_set_consign(&cannon->drum_cs, 0);

    cs_init(&cannon->cannon_cs);
    pid_init(&cannon->cannon_pid);
    pid_set_maximums(&cannon->cannon_pid, 0, 0, 15000);
    pid_set_gains(&cannon->cannon_pid, 100, 10, 0); //TODO
    pid_set_out_shift(&cannon->cannon_pid, 10); //TODO
    cs_set_correct_filter(&cannon->cannon_cs, pid_do_filter, &cannon->cannon_pid);
    cs_set_process_in(&cannon->cannon_cs, cvra_servo_set0, (void*)SERVOS_BASE);
#ifdef COMPILE_ON_ROBOT
    cs_set_process_out(&cannon->drum_cs, ppc_set_shooting_speed, (void*)FANSPEED_BASE);
#endif
    cs_set_consign(&cannon->cannon_cs, 5000);

    cs_enable(&cannon->drum_cs);
    cs_enable(&cannon->cannon_cs);

    scheduler_add_periodical_event(ppc_manage_cs, (void *)cannon, 10000 / SCHEDULER_UNIT);
    scheduler_add_periodical_event(ppc_manage, (void *)cannon, 10000 / SCHEDULER_UNIT);

    cannon->drum_state = EMPTY;
    cannon->cannon_state = IDLE;
}

void ppc_manage(ppc_t *cannon){
    
    switch(cannon->drum_state){

    default:
    case EMPTY:
        if(cannon->light_barrier_in_state &&
           !get_light_barrier_state(cannon->light_barrier_in_mask)){
            if(get_light_barrier_state(cannon->light_barrier_color_mask)){
                cannon->drum_state = LOADED_SHOOT;
            }else{
                cannon->drum_state = LOADED_EJECT;
            }
        }
        cs_set_consign(cannon->cannon_cs, 5000);
        break;

    case LOADED_SHOOT:
        break;

    case LOADED_EJECT:
        break;

    case UNDERWAY:
        if((!get_light_barrier_state(cannon->light_barrier_shoot_mask) &&
           cannon->light_barrier_shoot_state) ||
           (!get_light_barrier_state(cannon->light_barrier_eject_mask) &&
           cannon->light_barrier_eject_state)){

            if(cannon->light_barrier_in_state &&
               !get_light_barrier_state(cannon->light_barrier_in_mask)){
                if(get_light_barrier_state(cannon->light_barrier_color_mask)){
                    cannon->drum_state = LOADED_SHOOT;
                }else{
                    cannon->drum_state = LOADED_EJECT;
                }
            }else{
                cannon->drum_state = EMPTY;
            }
        }else{
            if(cannon->light_barrier_in_state &&
               !get_light_barrier_state(cannon->light_barrier_in_mask)){
                if(get_light_barrier_state(cannon->light_barrier_color_mask)){
                    cannon->drum_state = UNDERWAY_LOADED_SHOOT;
                }else{
                    cannon->drum_state = UNDERWAY_LOADED_EJECT;
                }
            }
        }
        break;

    case UNDERWAY_LOADED_SHOOT:
        if((!get_light_barrier_state(cannon->light_barrier_shoot_mask) &&
           cannon->light_barrier_shoot_state) ||
           (!get_light_barrier_state(cannon->light_barrier_eject_mask) &&
           cannon->light_barrier_eject_state)){

            cannon->drum_state = LOADED_SHOOT;
        }
        break;

    case UNDERWAY_LOADED_EJECT:
        if((!get_light_barrier_state(cannon->light_barrier_shoot_mask) &&
           cannon->light_barrier_shoot_state) ||
           (!get_light_barrier_state(cannon->light_barrier_eject_mask) &&
           cannon->light_barrier_eject_state)){

            cannon->drum_state = LOADED_EJECT;
        }
        break;

    }



    switch(cannon->cannon_state){

    case SUCK:
        
        break;

    case SHOOT:
        switch(cannon->drum_state){
        default:
        case EMPTY:
        case UNDERWAY:
        case UNDERWAY_LOADED_SHOOT:
        case UNDERWAY_LOADED_EJECT:
            break;

        case LOADED_SHOOT:
            ppc_shoot(cannon);
            cannon->drum_state = UNDERWAY;
            break;

        case LOADED_EJECT:
            ppc_eject(cannon);
            cannon->drum_state = UNDERWAY;
            break;
        }

        break;

    default:
    case IDLE:
    case BLOW:
        break;

    }

    cannon->light_barrier_in_state =
        get_light_barrier_state(cannon->light_barrier_in_mask);
    cannon->light_barrier_shoot_state =
        get_light_barrier_state(cannon->light_barrier_shoot_mask);
    cannon->light_barrier_eject_state =
        get_light_barrier_state(cannon->light_barrier_eject_mask);

}

void ppc_manage_cs(ppc_t *cannon){
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
    const int32_t drum_pos = cannon->drum_encoder_val;

    cs_set_consign(&cannon->cannon_cs, get_shooting_speed(cannon));
    cs_set_consign(&cannon->drum_cs,
                    cannon->drum_encoder_val + cannon->drum_encoder_res / 3); // + or - ?
}

void ppc_eject(ppc_t *cannon){
    cs_set_consign(&cannon->drum_cs,
                    cannon->drum_encoder_val - cannon->drum_encoder_res / 3); // + or - ?
}

int32_t get_shooting_speed(ppc_t *cannon){
    /* TODO
     * should depend on the distance to the cake (which is btw a lie..)
     */
    return 15000;
}

int32_t get_light_barrier_state(int32_t mask){
    return IORD(PIO_BASE, 0) & mask;
}


void ppc_set_shooting_speed(void *base, int32_t value){
    IOWR(base, 0, value);
}

void ppc_aspirater_down(void)
{
    cvra_servo_set((void*)SERVOS_BASE, 0, 8000); 
}

void ppc_aspirater_up(void)
{
    cvra_servo_set((void*)SERVOS_BASE, 0, 8000); 
}
