#include "cvra_cs.h"
#include "strat.h"
#include <string.h>
#include <2wheels/trajectory_manager_utils.h>
#include <scheduler.h>
#include <aversive/error.h>

struct strat_info strat;

/** Increments the match timer, called every second. */
static void increment_timer(__attribute__((unused))void *data) {
    strat.time++;
}

/** @brief Take the first two glasses.
 *
 * This function takes the first two glasses on the correct side.
 * @todo Test the starting coordinates.
 */
static void strat_do_first_glasses(void) {
    DEBUG(E_STRAT, "Doing first glasses."); 
}

void strat_set_objects(void) {
}

void strat_begin(void) {

}

void strat_autopos(int16_t x, int16_t y, int16_t a, int16_t epaisseurRobot) {

}


int test_traj_end(int why) {

}

int wait_traj_end(int why) {

}
