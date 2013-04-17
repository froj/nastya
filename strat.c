#include "cvra_cs.h"
#include "strat.h"
#include <string.h>
#include <holonomic/trajectory_manager.h>
#include <holonomic/position_manager.h>
#include <scheduler.h>
#include <aversive/error.h>
#include <cvra_servo.h>
#include <uptime.h>
#include "adresses.h"

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

/** Rigid calibrating for the holonomic_robot 
 *  Must be called near the calibration stop, and 
 * no trajectory must be running */
void strat_do_calibration(void)
{
    printf("Start calibration\n");
    /** Go to the right position */
    holonomic_trajectory_moving_straight_goto_xy_abs(&robot.traj, 700, COLOR_Y(200));
    while(!holonomic_end_of_traj(&robot.traj));
    holonomic_trajectory_turning_cap(&robot.traj,0);
    while(!holonomic_end_of_traj(&robot.traj));
    holonomic_trajectory_moving_straight_goto_xy_abs(&robot.traj, 700, COLOR_Y(200));
    while(!holonomic_end_of_traj(&robot.traj));
    
    /** Calibration */
    pid_set_gains(&robot.wheel0_pid, 5, 0,0);
    pid_set_gains(&robot.wheel1_pid, 5, 0,0);
    pid_set_gains(&robot.wheel2_pid, 5, 0,0);
    
    rsh_set_speed(&robot.rs,50);
    rsh_set_direction(&robot.rs,-M_PI_2);
    
    int normal_x_2 = 15*cvra_dc_get_current(HEXMOTORCONTROLLER_BASE, 3);
    
    /** Wheel  is the most affected since in the right direction */
    while(cvra_dc_get_current(HEXMOTORCONTROLLER_BASE, 3) < normal_x_2); //TODO : timeout
    
    
    holonomic_position_set(&robot.pos,holonomic_position_get_x_double(&robot.pos),
    88.5,0);
    rsh_set_speed(&robot.rs,0);
    
    holonomic_trajectory_moving_straight_goto_xy_abs(&robot.traj, 700, COLOR_Y(200));
    while(!holonomic_end_of_traj(&robot.traj));
    holonomic_trajectory_turning_cap(&robot.traj,0);
    while(!holonomic_end_of_traj(&robot.traj));
    
    int i;
    int32_t time;
    for (i=3;i>=1;i--)
    {
    pid_set_gains(&robot.wheel0_pid, ROBOT_PID_WHEEL0_P/i, ROBOT_PID_WHEEL0_I/i,ROBOT_PID_WHEEL0_D/i);
    pid_set_gains(&robot.wheel1_pid, ROBOT_PID_WHEEL1_P/i, ROBOT_PID_WHEEL1_I/i,ROBOT_PID_WHEEL1_D/i);
    pid_set_gains(&robot.wheel2_pid, ROBOT_PID_WHEEL2_P/i, ROBOT_PID_WHEEL2_I/i,ROBOT_PID_WHEEL2_D/i);
    time = uptime_get();
    while(time + 50000 > uptime_get());
    }
    pid_set_gains(&robot.wheel0_pid, ROBOT_PID_WHEEL0_P, ROBOT_PID_WHEEL0_I,ROBOT_PID_WHEEL0_D);
    pid_set_gains(&robot.wheel1_pid, ROBOT_PID_WHEEL1_P, ROBOT_PID_WHEEL1_I,ROBOT_PID_WHEEL1_D);
    pid_set_gains(&robot.wheel2_pid, ROBOT_PID_WHEEL2_P, ROBOT_PID_WHEEL2_I,ROBOT_PID_WHEEL2_D);
    
        holonomic_trajectory_moving_straight_goto_xy_abs(&robot.traj, 700, COLOR_Y(200));
    while(!holonomic_end_of_traj(&robot.traj));
    holonomic_trajectory_turning_cap(&robot.traj,0);
    while(!holonomic_end_of_traj(&robot.traj));
    printf("End of Calibration\n");
}
