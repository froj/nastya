/**
 @file cvra_cs.c
 @author Antoine Albertelli
 @date 19th September 2009
 @brief This files implements the control system loop wrappers.
 
 The file provides all the implementation of the Control System Management (CSM).
 A CSM is made of 3 parts : acceleration / deceleration ramp generator 
 (Quadramp), a position regulator (PID), and I/O interfaces (PWM & quadratures
 encoders).
 The quadramp tells the PID what position the wheel should be, depending on the 
 acceleration and time. This value is then fed as the consign value to the PID 
 regulator, with the encoder value as the measured position. The output of the 
 PID is then applied to the motor via the PWM.
 The others functions computed here are the position manager, the trajectory
 manager and the blocking detection system.
 */

#include <aversive.h>

#include <holonomic/trajectory_manager.h>
#include <holonomic/robot_system.h>
#include <holonomic/position_manager.h>
#include <control_system_manager.h>
#include <pid.h>
#include <quadramp.h>
#include <scheduler.h>

#include <aversive/error.h>
#include "error_numbers.h"
#include "adresses.h"

#include <string.h>
#include <stdio.h>

#include "cvra_cs.h"
#include "hardware.h"
#include "cvra_param_robot.h"


struct _rob robot;

/** Limite PWM = + ou - 475 */

void cvra_cs_init(void) {
    /****************************************************************************/
    /*                                Motor                                     */
    /****************************************************************************/

#ifdef COMPILE_ON_ROBOT
    int i;

    for(i=0;i<6;i++) {
        cvra_dc_set_encoder((void*)HEXMOTORCONTROLLER_BASE, i, 0);
        cvra_dc_set_pwm((void*)HEXMOTORCONTROLLER_BASE, i, 0);
    }
#endif

    /****************************************************************************/
    /*                             Robot system                                 */
    /****************************************************************************/
    
    rsh_init(&robot.rs);
    rsh_set_position_manager(&robot.rs, &robot.pos);


    /****************************************************************************/
    /*                         Regulation Wheel-by-Wheel                        */
    /****************************************************************************/

    pid_init(&robot.wheel0_pid);
    pid_init(&robot.wheel1_pid);
    pid_init(&robot.wheel2_pid);
    
    // CALIBRATION : Mettre les gains < 0 si le moteur compense dans le mauvais sens
    pid_set_gains(&robot.wheel0_pid, ROBOT_PID_WHEEL0_P, ROBOT_PID_WHEEL0_I,ROBOT_PID_WHEEL0_D);
    pid_set_gains(&robot.wheel1_pid, ROBOT_PID_WHEEL1_P, ROBOT_PID_WHEEL1_I,ROBOT_PID_WHEEL1_D);
    pid_set_gains(&robot.wheel2_pid, ROBOT_PID_WHEEL2_P, ROBOT_PID_WHEEL2_I,ROBOT_PID_WHEEL2_D);
    
    //pid_set_maximums(&robot.angle_pid, 0, 5000, 30000);
    
    pid_set_out_shift(&robot.wheel0_pid, 10);
    pid_set_out_shift(&robot.wheel1_pid, 10);
    pid_set_out_shift(&robot.wheel2_pid, 10);
    
    cs_init(&robot.wheel0_cs); 
    cs_init(&robot.wheel1_cs); 
    cs_init(&robot.wheel2_cs);
    
    cs_set_correct_filter(&robot.wheel0_cs, pid_do_filter, &robot.wheel0_pid);
    cs_set_correct_filter(&robot.wheel1_cs, pid_do_filter, &robot.wheel1_pid);
    cs_set_correct_filter(&robot.wheel2_cs, pid_do_filter, &robot.wheel2_pid);

    ramp_init(&robot.wheel0_ramp);
    ramp_init(&robot.wheel1_ramp);
    ramp_init(&robot.wheel2_ramp);

    ramp_set_vars(&robot.wheel0_ramp, 1000, 1000);
    ramp_set_vars(&robot.wheel1_ramp, 1000, 1000);
    ramp_set_vars(&robot.wheel2_ramp, 1000, 1000);

    cs_set_consign_filter(&robot.wheel0_cs, ramp_do_filter, &robot.wheel0_ramp);
    cs_set_consign_filter(&robot.wheel1_cs, ramp_do_filter, &robot.wheel1_ramp);
    cs_set_consign_filter(&robot.wheel2_cs, ramp_do_filter, &robot.wheel2_ramp);

    

#ifdef COMPILE_ON_ROBOT

    cs_set_process_in(&robot.wheel0_cs, cvra_dc_set_pwm0, (void*)HEXMOTORCONTROLLER_BASE);
    cs_set_process_in(&robot.wheel1_cs, cvra_dc_set_pwm1, (void*)HEXMOTORCONTROLLER_BASE);
    cs_set_process_in(&robot.wheel2_cs, cvra_dc_set_pwm2, (void*)HEXMOTORCONTROLLER_BASE);
    
    cs_set_process_out(&robot.wheel0_cs, cvra_dc_get_encoder0, (void*)HEXMOTORCONTROLLER_BASE);
    cs_set_process_out(&robot.wheel1_cs, cvra_dc_get_encoder1, (void*)HEXMOTORCONTROLLER_BASE);
    cs_set_process_out(&robot.wheel2_cs, cvra_dc_get_encoder2, (void*)HEXMOTORCONTROLLER_BASE);
#endif

    
    cs_set_consign(&robot.wheel0_cs, 0);
    cs_set_consign(&robot.wheel1_cs, 0);
    cs_set_consign(&robot.wheel2_cs, 0);
    
    
    
    rsh_set_cs(&robot.rs, 0 , &robot.wheel0_cs);
    rsh_set_cs(&robot.rs, 1 , &robot.wheel1_cs);
    rsh_set_cs(&robot.rs, 2 , &robot.wheel2_cs);
    
    ///****************************************************************************/
    ///*                          Position manager                                */
    ///****************************************************************************/

    holonomic_position_init(&robot.pos);

    double beta[] = {ROBOT_BETA_WHEEL0_RAD,
                    ROBOT_BETA_WHEEL1_RAD,
                    ROBOT_BETA_WHEEL2_RAD};

    double wheel_radius[] = {ROBOT_RADIUS_WHEEL0_MM,
                            ROBOT_RADIUS_WHEEL1_MM,
                            ROBOT_RADIUS_WHEEL2_MM};

    double wheel_distance[] = {ROBOT_DISTANCE_WHEEL0_MM,
                              ROBOT_DISTANCE_WHEEL1_MM,
                              ROBOT_DISTANCE_WHEEL2_MM};

    holonomic_position_set_physical_params(
            &robot.pos,
            beta,
            wheel_radius,
            wheel_distance,
            ROBOT_ENCODER_RESOLUTION);

    holonomic_position_set_update_frequency(&robot.pos, (float)ASSERV_FREQUENCY);

    int32_t (*motor_encoder[])(void *) = {cvra_dc_get_encoder0,
                                          cvra_dc_get_encoder1,
                                          cvra_dc_get_encoder2};

    void* motor_encoder_param[] = { (void*)HEXMOTORCONTROLLER_BASE,
    								(void*)HEXMOTORCONTROLLER_BASE,
    								(void*)HEXMOTORCONTROLLER_BASE};

    holonomic_position_set_mot_encoder(&robot.pos, motor_encoder, motor_encoder_param);


    /****************************************************************************/
    /**      CS pour les macros-variables (seulement les rampes, pas de PID)    */
    /****************************************************************************/
    
    /******************************** ANGLE *************************************/
    quadramp_init(&robot.angle_qr);
    quadramp_set_2nd_order_vars(&robot.angle_qr,10000,10000);
    quadramp_set_1st_order_vars(&robot.angle_qr,10000,10000);
    
    
    ///******************************** OMEGA ************************************/
    ramp_init(&robot.omega_r);
    ramp_set_vars(&robot.omega_r,10000,10000);
    
    
    ///******************************** SPEED *************************************/
    ramp_init(&robot.speed_r);
    ramp_set_vars(&robot.speed_r,10000,10000);
    

    ///****************************************************************************/
    ///*                           Trajectory Manager (Trivial)                   */
    ///****************************************************************************/
    holonomic_trajectory_init(&robot.traj, ASSERV_FREQUENCY);
    holonomic_trajectory_set_ramps(&robot.traj, &robot.speed_r, &robot.angle_qr, &robot.omega_r);
    
    holonomic_trajectory_set_robot_params(&robot.traj, &robot.rs, &robot.pos);
    holonomic_trajectory_set_windows(&robot.traj, 10, 0.1);
    
    /* ajoute la regulation au multitache. ASSERV_FREQUENCY est dans cvra_cs.h */
    scheduler_add_periodical_event_priority(cvra_cs_manage, NULL, (1000000
            / ASSERV_FREQUENCY) / SCHEDULER_UNIT, 130);
}

///** Logge l'erreur sur les differents regulateurs et l'affiche avec le temps. */
//static void dump_error(void) {
    //static int time = 0;
    //if (robot.error_dump_enabled) {
       //// if (time % 10)
            /////@todo : Afficher des trucs utiles
            ////fprintf(stderr, "%d;%d;%d\n", time,
                    ////(int)cs_get_error(&robot.angle_cs),
                    ////(int)cs_get_error(&robot.omega_cs),
                    ////(int)cs_get_error(&robot.speed_cs));
        //time++;
    //} else {
        //time = 0;
    //}
//}

void cvra_cs_manage(__attribute__((unused)) void * dummy) {
    
    //NOTICE(ERROR_CS, __FUNCTION__);
    //DEBUG(E_ROBOT_SYSTEM, "LOL");
    /* Gestion de la position. */
    
    rsh_update(&robot.rs);
    holonomic_position_manage(&robot.pos);

    cs_manage(&robot.wheel0_cs);
    cs_manage(&robot.wheel1_cs);
    cs_manage(&robot.wheel2_cs);
    
    /* Affichage des courbes d'asservissement. */
    //dump_error();
}
