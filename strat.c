#include "cvra_cs.h"
#include "strat.h"
#include <string.h>
#include <holonomic/trajectory_manager.h>
#include <holonomic/position_manager.h>
#include <scheduler.h>
#include <aversive/error.h>

struct strat_info strat;

//void strat_open_servo(enum servo_e servo) {
    //if(servo == RIGHT)
        //cvra_servo_set(SERVOS_BASE, 1, 21000);
    //else
        //cvra_servo_set(SERVOS_BASE, 0, 9000); 
//}


//void strat_close_servo(enum servo_e servo) {
    //if(servo == RIGHT)
        //cvra_servo_set(SERVOS_BASE, 1, 17500);
    //else
        //cvra_servo_set(SERVOS_BASE, 0, 11500); 
//}

//void strat_release_servo(enum servo_e servo) {
    //if(servo == RIGHT)
        //cvra_servo_set(SERVOS_BASE, 1, 15000);
    //else
        //cvra_servo_set(SERVOS_BASE, 0, 15000); 
//}

/** Increments the match timer, called every second. */
static void increment_timer(__attribute__((unused))void *data) {
    strat.time++;
}

/** @brief Do the gift
 */
static void strat_do_gift(void) {
    holonomic_trajectory_moving_straight_goto_xy_abs(&robot.traj,500, COLOR_Y(500));

}


/** @todo : passe to double */
void strat_start_position(void) {
    //distance centre/ coté : 88.5 mm
    //distance centre /calibre : 112.87 mm
    //épaisseur bord blanc : 100 mm
    holonomic_position_set_x_s16(&robot.pos, 88.5);
    holonomic_position_set_y_s16(&robot.pos,COLOR_Y(2000 - 213));
    holonomic_position_set_a_s16(&robot.pos, COLOR_A(90));
}

void strat_begin(strat_color_t color) {
    /* Starts the game timer. */
    strat.time = 0;
    strat.color = color;
    scheduler_add_periodical_event(increment_timer, NULL, 1000000/SCHEDULER_UNIT);
    
    strat_start_position();

    /* Do the two central glasses. */
    strat_do_gift();

}


