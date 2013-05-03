#include <control_system_manager.h>
#include <holonomic/position_manager.h>
#include <pid.h>

#ifndef _PINGPONG_H_
#define _PINGPONG_H_

typedef ppc_cannon_state {
    SUCK,
    BLOW,
    SHOOT
};

typedef ppc_drum_state {
    EMPTY,
    LOADED_SHOOT,
    LOADED_EJECT,
    UNDERWAY,
    UNDERWAY_LOADED_SHOOT,
    UNDERWAY_LOADED_EJECT
}

typedef struct {

    enum ppc_cannon_state cannon_state;
    enum ppc_drum_state drum_state;

    struct holonomic_robot_position *pos;

    struct cs drum_cs;
    struct cs cannon_cs;
    
    struct pid_filter drum_pid;
    struct pid_filter cannon_pid;

    int32_t drum_encoder_val;
    int32_t drum_encoder_res;

    int32_t balls_shot;

    int32_t balls_ejected;

    int32_t shoot_speed;

    int32_t light_barrier_in_state;
    int32_t light_barrier_in_mask;
    int32_t light_barrier_shoot_state;
    int32_t light_barrier_shoot_mask;
    int32_t light_barrier_eject_state;
    int32_t light_barrier_eject_mask;
    int32_t light_barrier_color_mask;


} ppc_t;


void ppc_init(ppc_t *cannon);

void ppc_manage(ppc_t *cannon);

void pcc_manage_cs(ppc_t *cannon);

void ppc_aspiration_on(ppc_t *cannon);
void ppc_aspiration_off(ppc_t *cannon);
void ppc_aspiration_invert(ppc_t *cannon);

void ppc_shoot(ppc_t *cannon);

void ppc_eject(ppc_t *cannon);

void ppc_set_shooting_speed(void *base, int32_t value);


#endif
