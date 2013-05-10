#include <aversive.h>
#include <scheduler.h>
#include <string.h>
#include <cvra_servo.h>
#include <cvra_dc.h>
#include "adresses.h"
#include "pingpongcannon.h"


int32_t get_shooting_speed(ppc_t *cannon);

void ppc_init(ppc_t *cannon){
    memset(cannon, 0, sizeof(ppc_t));

    cvra_servo_set((void*)SERVOS_BASE, 0, 20000);
    cvra_servo_set((void*)SERVOS_BASE, 2, 10000);
    cvra_servo_set((void*)SERVOS_BASE, 3, 10000);

    cs_init(&cannon->drum_cs);
    pid_init(&cannon->drum_pid);
    pid_set_maximums(&cannon->drum_pid, 0, 2000, 200); 
    pid_set_gains(&cannon->drum_pid, -600, -125, -400);
    pid_set_out_shift(&cannon->drum_pid, 10); //TODO
    cs_set_correct_filter(&cannon->drum_cs, pid_do_filter, &cannon->drum_pid);
    cs_set_process_in(&cannon->drum_cs, cvra_dc_set_pwm4, (void*)HEXMOTORCONTROLLER_BASE);
    cs_set_process_out(&cannon->drum_cs, cvra_dc_get_encoder4, (void*)HEXMOTORCONTROLLER_BASE);
    cs_set_consign(&cannon->drum_cs, 0);


    cs_enable(&cannon->drum_cs);

    scheduler_add_periodical_event(ppc_manage_cs, (void *)cannon, 10000 / SCHEDULER_UNIT);
    scheduler_add_periodical_event(ppc_manage, (void *)cannon, 10000 / SCHEDULER_UNIT);

    cannon->drum_state = EMPTY;

    cannon->light_barrier_color_mask = 0x0100;
    cannon->light_barrier_in_mask = 0x0200;
    cannon->light_barrier_eject_mask = 0x0400;
    cannon->light_barrier_shoot_mask = 0x0800;

    cannon->drum_encoder_res = 20000;
}

void ppc_manage(ppc_t *cannon){
    //TODO
    if(is_blocked(cannon)){
        deblock_drum(cannon);
    }
    else{
        ppc_manage_shoot(cannon);
    }
}

void ppc_manage_shoot(ppc_t *cannon){
    switch(cannon->drum_state){

    default:
    case EMPTY:
        //printf("EMPTY %u\n", cannon->light_barrier_in_state);
        if(cannon->light_barrier_in_state &&
           !ppc_get_light_barrier_state(cannon->light_barrier_in_mask)){
            printf("change da fuckin state to loaded, biatch\n");
            if(ppc_get_light_barrier_state(cannon->light_barrier_color_mask)){
                cannon->drum_state = LOADED_SHOOT;
            }else{
                cannon->drum_state = LOADED_EJECT;
            }
        }
        break;

    case LOADED_SHOOT:
        //printf("LOADED_SHOOT\n");
        ppc_start_cannon();
        ppc_shoot(cannon);
        cannon->drum_state = UNDERWAY;
        break;

    case LOADED_EJECT:
        //printf("LOADED_EJECT\n");
        ppc_eject(cannon);
        cannon->drum_state = UNDERWAY;
        break;

    case UNDERWAY:
        //printf("UNDERWAY\n");
        if((!ppc_get_light_barrier_state(cannon->light_barrier_shoot_mask) &&
           cannon->light_barrier_shoot_state) ||
           (!ppc_get_light_barrier_state(cannon->light_barrier_eject_mask) &&
           cannon->light_barrier_eject_state)){

            if(cannon->light_barrier_in_state &&
               !ppc_get_light_barrier_state(cannon->light_barrier_in_mask)){
                if(ppc_get_light_barrier_state(cannon->light_barrier_color_mask)){
                    cannon->drum_state = LOADED_SHOOT;
                }else{
                    cannon->drum_state = LOADED_EJECT;
                }
            }else{
                ppc_stop_cannon();
                cannon->drum_state = EMPTY;
            }
        }else{
            if(cannon->light_barrier_in_state &&
               !ppc_get_light_barrier_state(cannon->light_barrier_in_mask)){
                if(ppc_get_light_barrier_state(cannon->light_barrier_color_mask)){
                    cannon->drum_state = UNDERWAY_LOADED_SHOOT;
                }else{
                    cannon->drum_state = UNDERWAY_LOADED_EJECT;
                }
            }
        }
        break;

    case UNDERWAY_LOADED_SHOOT:
        //printf("UNDERWAY_LOADED_SHOOT\n");
        if((!ppc_get_light_barrier_state(cannon->light_barrier_shoot_mask) &&
           cannon->light_barrier_shoot_state) ||
           (!ppc_get_light_barrier_state(cannon->light_barrier_eject_mask) &&
           cannon->light_barrier_eject_state)){

            cannon->drum_state = LOADED_SHOOT;
        }
        break;

    case UNDERWAY_LOADED_EJECT:
        //printf("UNDERWAY_LOADED_EJECT\n");
        if((!ppc_get_light_barrier_state(cannon->light_barrier_shoot_mask) &&
           cannon->light_barrier_shoot_state) ||
           (!ppc_get_light_barrier_state(cannon->light_barrier_eject_mask) &&
           cannon->light_barrier_eject_state)){

            cannon->drum_state = LOADED_EJECT;
        }
        break;

    }



    cannon->light_barrier_in_state =
        ppc_get_light_barrier_state(cannon->light_barrier_in_mask);
    cannon->light_barrier_shoot_state =
        ppc_get_light_barrier_state(cannon->light_barrier_shoot_mask);
    cannon->light_barrier_eject_state =
        ppc_get_light_barrier_state(cannon->light_barrier_eject_mask);

}

void ppc_manage_cs(ppc_t *cannon){
    cs_manage(&cannon->drum_cs);
    cannon->drum_encoder_val = cvra_dc_get_encoder4(HEXMOTORCONTROLLER_BASE);
}

void ppc_aspiration_on(ppc_t *cannon){
    int32_t uptime;
    cvra_servo_set((void*)SERVOS_BASE, 2, 14000); 
    uptime = uptime_get();
    while(uptime_get() < uptime + 500000);
    gpio_set(7, 1);
}

void ppc_aspiration_off(ppc_t *cannon){
    int32_t uptime;
    cvra_servo_set((void*)SERVOS_BASE, 2, 10000);
    uptime = uptime_get();
    while(uptime_get() < uptime + 500000);
    gpio_set(7, 0);
}

void ppc_aspiration_invert(ppc_t *cannon){
    
}

void ppc_shoot(ppc_t *cannon){
    int32_t uptime;
    uptime = uptime_get();
    cs_set_consign(&cannon->drum_cs,
                    cannon->drum_encoder_val - cannon->drum_encoder_res / 3);
    //while(uptime_get() < uptime + 1000000);
}

void ppc_eject(ppc_t *cannon){
    cs_set_consign(&cannon->drum_cs,
                    cannon->drum_encoder_val + cannon->drum_encoder_res / 3);
}

void ppc_start_cannon(void){
    int32_t uptime;
    cvra_servo_set((void*)SERVOS_BASE, 3, 15000);
    uptime = uptime_get();
    while(uptime_get() < uptime + 1000000);
}

void ppc_stop_cannon(void){
    int32_t uptime;
    cvra_servo_set((void*)SERVOS_BASE, 3, 10000);
    uptime = uptime_get();
    while(uptime_get() < uptime + 500000);
}

int32_t get_shooting_speed(ppc_t *cannon){
    /*
       why do we pass a *cannon? (=
    */
    /* TODO
     * should depend on the distance to the cake (which is btw a lie..)
     */
    return 15000;
}

int32_t ppc_get_light_barrier_state(int32_t mask){
    return IORD(PIO_BASE, 0) & mask;
}


uint8_t is_blocked(ppc_t *cannon){ 
    return (pid_get_value_D(&cannon->drum_pid) == 0 && 
            cs_get_error(&cannon->drum_cs) >= cannon->drum_encoder_res/6);
}

void deblock_drum(ppc_t *cannon){
    cs_set_consign(&cannon->drum_cs,
                &cannon->drum_encoder_val + cs_get_error(&cannon->drum_cs));

}


void ppc_set_shooting_speed(void *base, int32_t value){
    IOWR(base, 0, value);
}

void ppc_aspirater_down(void)
{
    cvra_servo_set((void*)SERVOS_BASE, 0, 23000); 
}

void ppc_aspirater_up(void)
{
    cvra_servo_set((void*)SERVOS_BASE, 0, 7000); 
}

void gpio_set(int channel, int value){
    static int32_t output_value;

    if(value) 
        output_value |= (int32_t)(1<<channel);
    else
        output_value &= (int32_t)~(1<<channel); 

    IOWR(PIO_BASE, 0, output_value);
}
