#include <control_system_manager.h>
#include <pid.h>

#ifndef _PINGPONG_H_
#define _PINGPONG_H_


typedef struct {

    struct cs drum_cs;
    struct cs cannon_cs;
    
    struct pid_filter drum_pid;
    struct pid_filter cannon_pid;

    double drum_pos;
    int32_t drum_encoder_val;

    int32_t balls_shot;

    int32_t balls_ejected;

    int32_t shoot_speed;

} ppc_t


void ppc_init(ppc_t *cannon);

void ppc_manage(ppc_t *cannon);

void pcc_manage_cs(ppc_t *cannon);

void ppc_aspirate(ppc_t *cannon);

void ppc_shoot(ppc_t *cannon);

void ppc_eject(ppc_t *cannon);

void ppc_test_ball(ppc_t *cannon);


#endif
