#include "cvra_cs.h"
#include "strat.h"
#include <string.h>
#include <2wheels/trajectory_manager_utils.h>
#include <scheduler.h>
#include <aversive/error.h>

struct strat_info strat;

///** Increments the match timer, called every second. */
//static void increment_timer(__attribute__((unused))void *data) {
    //strat.time++;
//}

void strat_set_objects(void) {
}

void strat_begin(void) {

}

void strat_autopos(int16_t x, int16_t y, int16_t a, int16_t epaisseurRobot) {
    (void)x;
    (void)y;
    (void)a;
    (void)epaisseurRobot;
}


int test_traj_end(int why) {
    (void)why;
    return 0;
}

int wait_traj_end(int why) {
    (void)why;  
    return 0;
}
