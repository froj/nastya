#include "cvra_cs.h"
#include "strat.h"
#include <string.h>
#include <holonomic/trajectory_manager.h>
#include <holonomic/position_manager.h>
#include <scheduler.h>
#include <aversive/error.h>
#include <cvra_servo.h>
#include <uptime.h>
#include <fast_math.h>
#include "adresses.h"

#define BASE_RAD 200    /**< [mm] */
#define BACK_X 80
#define BACK_Y 190
/*
#define BASE_SIN 76.537
#define BASE_COS 184.776 */
#define CANDLE_ANG 0.2617993878     /**< angle to the next candle [rad] */

struct strat_info strat;

void strat_long_arm_up(void){
        cvra_servo_set((void*)SERVOS_BASE, 1, 15000); 
}

void strat_long_arm_down(void){
        cvra_servo_set((void*)SERVOS_BASE, 1, 7000); 
}

void strat_short_arm_up(void){
        cvra_servo_set((void*)SERVOS_BASE, 0, 17000); 
}

void strat_short_arm_down(void){
        cvra_servo_set((void*)SERVOS_BASE, 0, 8000); 
}


/** Increments the match timer, called every second. */
static void increment_timer(__attribute__((unused))void *data) {
    strat.time++;
}

void strat_wait_90_seconds(void)
{
    printf("Stoppping at end of 90 sec \n");
    //while (strat.time < 90);
    strat_short_arm_down();
    cs_disable(&robot.wheel0_cs);
    cs_disable(&robot.wheel1_cs);
    cs_disable(&robot.wheel2_cs);
    strat_short_arm_down();
}


void strat_set_objects(void) {
    memset(&strat.glasses, 0, sizeof(glass_t)*12);
    memset(&strat.gifts, 0, sizeof(gift_t)*4);

    /* Init gifts position. */ 
    strat.gifts[0].x = 525; /* middle of the gift. */
    strat.gifts[1].x = 1125;
    strat.gifts[2].x = 1725;
    strat.gifts[3].x = 2325;

    /* Init glasses positions. */
    strat.glasses[0].pos.x = 900; strat.glasses[0].pos.y = (1550);
    strat.glasses[1].pos.x = 900; strat.glasses[1].pos.y = (1050);
    strat.glasses[2].pos.x = 1050; strat.glasses[2].pos.y = (1200);

    /*XXX Not sure about coordinates of 3 and 4. */
    strat.glasses[3].pos.x = 1200; strat.glasses[3].pos.y = (1550);
    strat.glasses[4].pos.x = 1200; strat.glasses[4].pos.y = (1050);
    strat.glasses[5].pos.x = 1350; strat.glasses[5].pos.y = (1200);
    strat.glasses[6].pos.x = 1650; strat.glasses[6].pos.y = (1300);
    strat.glasses[7].pos.x = 1800; strat.glasses[7].pos.y = (1550);
    strat.glasses[8].pos.x = 1800; strat.glasses[8].pos.y = (1050);
    strat.glasses[9].pos.x = 1950; strat.glasses[9].pos.y = (1300);
    strat.glasses[10].pos.x = 2100; strat.glasses[10].pos.y = (1550);
    strat.glasses[11].pos.x = 2100; strat.glasses[11].pos.y = (1050);

    /* Init white candles. */
    strat.candles[0].pos.x = 1672.208; strat.candles[0].pos.y = 415.746;
    strat.candles[1].pos.x = 1558.737; strat.candles[1].pos.y = 446.150;
    strat.candles[2].pos.x = 1441.263; strat.candles[2].pos.y = 446.150;
    strat.candles[3].pos.x = 1327.793; strat.candles[3].pos.y = 415.746;
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
#ifdef COMPILE_ON_ROBOT
    cvra_beacon_init(&robot.beacon, AVOIDING_BASE, AVOIDING_IRQ);
#endif
    /* Starts the game timer. */
    strat.time = 0;
    strat.state = 0;
    strat.sub_state = 0;
    strat.color = color;
    
    strat_set_objects();
    strat_start_position();

    strat_long_arm_down();
    strat_short_arm_down();

 //   holonomic_trajectory_moving_straight_goto_xy_abs(&robot.traj, 200, COLOR_Y(2000-300));
 //   while(!holonomic_end_of_traj(&robot.traj));

    while((IORD(PIO_BASE, 0) & 0x1000) == 0);
    scheduler_add_periodical_event(increment_timer, NULL, 1000000/SCHEDULER_UNIT);
    holonomic_trajectory_moving_straight_goto_xy_abs(&robot.traj, 500, COLOR_Y(1500));
    while(!holonomic_end_of_traj(&robot.traj));

    strat_do_gift(strat.state);
    strat_wait_90_seconds();
}

/** 
 * @brief Do the gift
 */
void strat_do_gift(int number) {
    if (!strat.avoiding)
    {
        if (strat.sub_state == 0 )
        {
            strat_short_arm_down();
            holonomic_trajectory_moving_straight_goto_xy_abs(&robot.traj,
                                                        strat.gifts[number].x + COLOR_C,
                                                         COLOR_Y(2000-140));
            while(!holonomic_end_of_traj(&robot.traj));
            strat.sub_state++;
        }
        
        if (strat.sub_state == 1)
        {
            holonomic_trajectory_turning_cap(&robot.traj, COLOR_A(TO_RAD(-90)));
            while(!holonomic_end_of_traj(&robot.traj));
        
            strat.sub_state++;
        }
        
        if (strat.sub_state == 2)
        { 
            if (strat.state < 3)
            {
            holonomic_trajectory_moving_straight_goto_xy_abs(&robot.traj,
                                                         strat.gifts[number].x + COLOR_C,
                                                         COLOR_Y(2000-140));
            }
            else{
                        holonomic_trajectory_moving_straight_goto_xy_abs(&robot.traj,
                                                         strat.gifts[number].x + COLOR_C,
                                                         COLOR_Y(2000-120));
            }
            
            while(!holonomic_end_of_traj(&robot.traj));
            strat_short_arm_up();
            
            int32_t time = uptime_get();
            while(time + 500000 > uptime_get());
        }
        strat.sub_state = 0;
        strat.state++;
        if (strat.state < 4 && strat.state > -1)
        {
            
                strat_do_gift(strat.state);
        }
        else
            strat_wait_90_seconds();
    }
    strat_do_gift(strat.state);
}


void strat_avoiding(void)
{
    strat.avoiding = 1;
    
    /** stop current traj */
    holonomic_delete_event(&robot.traj);
    
    /** @strat do gift */
    //strat_do_gift(strat.state);
    /** to be sure stop current move */
    rsh_set_speed(&robot.rs, 0);
    rsh_set_rotation_speed(&robot.rs, 0);
    cs_disable(&robot.wheel0_cs);
cs_disable(&robot.wheel1_cs);
        cs_disable(&robot.wheel2_cs);
    cvra_dc_set_pwm0(HEXMOTORCONTROLLER_BASE,0);
    cvra_dc_set_pwm1(HEXMOTORCONTROLLER_BASE,0);
    cvra_dc_set_pwm2(HEXMOTORCONTROLLER_BASE,0);
    while(1);
    
}

void strat_restart_after_avoiding(void)
{
    strat.avoiding = 0;
    /** Si on etait en train de faire des cadeaux */
    if (strat.state < 4)
        strat_do_gift(strat.state);
    else
        strat_wait_90_seconds();
}

/** @brief Get to the first candle and get into a good angle with it. */
void strat_init_candle(int number)
{
    if (strat.sub_state == 3) {
        holonomic_trajectory_moving_straight_goto_xy_abs(&robot.traj,
            strat.candles[number].x - BACK_X - COLOR_C, COLOR_Y(strat.candles[number].y + BACK_Y));
        while(!holonomic_end_of_traj(&robot.traj));
    }

    if (strat.sub_state == 4) {
        holonomic_trajectory_turning_cap(&robot.traj, 
            (holonomic_position_get_a_rad_float(&robot.pos) 
            - COLOR_A(fast_atanf(holonomic_position_get_y_float(&robot.pos) 
                / (holonomic_position_get_x_float(&robot.pos) - 1500))));
        while(!holonomic_end_of_traj(&robot.traj));
    }

    /** @todo Descend arm. Angle is needed to be determined. */
}

/** @brief Move to next candle and descend it. */
void strat_do_candle(int number)
{
    if (strat.sub_state == 5) {
        holonomic_trajectory_moving_circle(&robot.traj, 1500, COLOR_Y(0), CANDLE_ANG);
        while(!holonomic_end_of_traj(&robot.traj));
    }

    /** @todo Descend arm. */
}
