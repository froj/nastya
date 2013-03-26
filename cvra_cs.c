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


void cvra_cs_init(void) {
    /****************************************************************************/
    /*                                Motor                                     */
    /****************************************************************************/

#ifdef COMPILE_ON_ROBOT
    int i;

    for(i=0;i<6;i++) {
        cvra_dc_set_encoder(HEXMOTORCONTROLLER_BASE, i, 0);
        cvra_dc_set_pwm(HEXMOTORCONTROLLER_BASE, i, 0);
    }
#endif

    /****************************************************************************/
    /*                             Robot system                                 */
    /****************************************************************************/
    //TODO
    rsh_init(&robot.rs);


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
    
    /** @todo : demander à Antoine*/
    //pid_set_maximums(&robot.angle_pid, 0, 5000, 30000);
    //pid_set_out_shift(&robot.angle_pid, 10);
    
    cs_init(&robot.wheel0_cs); 
    cs_init(&robot.wheel1_cs); 
    cs_init(&robot.wheel2_cs);
    
    cs_set_correct_filter(&robot.wheel0_cs, pid_do_filter, &robot.wheel0_pid);
    cs_set_correct_filter(&robot.wheel1_cs, pid_do_filter, &robot.wheel1_pid);
    cs_set_correct_filter(&robot.wheel2_cs, pid_do_filter, &robot.wheel2_pid);
    
    /** @todo : quel pwm fait quoi ? */
    cs_set_process_in(&robot.wheel0_cs, cvra_dc_set_pwm4, HEXMOTORCONTROLLER_BASE);
    cs_set_process_in(&robot.wheel1_cs, cvra_dc_set_pwm3, HEXMOTORCONTROLLER_BASE);
    cs_set_process_in(&robot.wheel2_cs, cvra_dc_set_pwm5, HEXMOTORCONTROLLER_BASE);
    
    cs_set_process_out(&robot.wheel0_cs, cvra_dc_get_encoder4, HEXMOTORCONTROLLER_BASE);
    cs_set_process_out(&robot.wheel1_cs, cvra_dc_get_encoder3, HEXMOTORCONTROLLER_BASE);
    cs_set_process_out(&robot.wheel2_cs, cvra_dc_get_encoder5, HEXMOTORCONTROLLER_BASE);
    
    //cs_set_consign(&robot.wheel0_cs, 0);
    //cs_set_consign(&robot.wheel1_cs, 0);
    //cs_set_consign(&robot.wheel2_cs, 0);
    
    rsh_set_cs(&robot.rs, 0 , &robot.wheel0_cs);
    rsh_set_cs(&robot.rs, 1 , &robot.wheel1_cs);
    rsh_set_cs(&robot.rs, 2 , &robot.wheel2_cs);
    
    /****************************************************************************/
    /**      CS pour les macros-variables (seulement les rampes, pas de PID)    */
    /****************************************************************************/
    
    /******************************** ANGLE *************************************/
    quadramp_init(&robot.angle_qr);
    cs_init(&robot.angle_cs);
    
    cs_set_consign_filter(&robot.angle_cs, quadramp_do_filter, &robot.angle_qr);
    //cs_set_process_in(&robot.angle_cs, rsh_set_direction, &robot.rs);
    //cs_set_process_out(&robot.angle_cs, holonomic_position_get_a_rad_double, &robot.pos);
    cs_set_consign(&robot.angle_cs, 0);
    
    /******************************** OMEGA ************************************/
    ramp_init(&robot.omega_r);
    cs_init(&robot.omega_cs);
    
    cs_set_consign_filter(&robot.omega_cs, ramp_do_filter, &robot.omega_r);
    //cs_set_process_in(&robot.omega_cs, rsh_set_rotation_speed, &robot.rs);
    ///@todo : GETER LA VITEESSE ANGULAIRE IL FAUT UNE FONCTION 
    //cs_set_process_out(&robot.omega_cs, holonomic_position, &robot.pos);
    cs_set_consign(&robot.omega_cs, 0);
    
    /******************************** SPEED *************************************/
    ramp_init(&robot.speed_r);
    cs_init(&robot.omega_cs);
    
    //cs_set_consign_filter(&robot.speed_cs, ramp_do_filter, &robot.speed_r);
    //cs_set_process_in(&robot.speed_cs, rsh_set_speed, &robot.rs);
    ///@todo : GETER LA VITEESSE ANGULAIRE IL FAUT UNE FONCTION 
    //cs_set_process_out(&robot.angle_cs, rs_get_ext_angle, &robot.rs);
    cs_set_consign(&robot.speed_cs, 0);
    
    /****************************************************************************/
    /*                          Position manager                                */
    /****************************************************************************/

    holonomic_position_init(&robot.pos);

    float beta[] = {ROBOT_BETA_WHEEL0_RAD,
                    ROBOT_BETA_WHEEL1_RAD,
                    ROBOT_BETA_WHEEL2_RAD};

    float wheel_radius[] = {ROBOT_RADIUS_WHEEL0_MM,
                            ROBOT_RADIUS_WHEEL1_MM,
                            ROBOT_RADIUS_WHEEL2_MM};

    float wheel_distance[] = {ROBOT_DISTANCE_WHEEL0_MM,
                              ROBOT_DISTANCE_WHEEL1_MM,
                              ROBOT_DISTANCE_WHEEL2_MM};

    holonomic_position_set_physical_params(
            &robot.pos,
            beta,
            wheel_radius,
            wheel_distance,
            ROBOT_ENCODER_RESOLUTION);

    holonomic_position_set_update_frequency(&robot.pos, ASSERV_FREQUENCY);

    int32_t (*motor_encoder[])(void *) = {cvra_dc_get_encoder4,
                                          cvra_dc_get_encoder3,
                                          cvra_dc_get_encoder5};

    void* motor_encoder_param[] = {HEXMOTORCONTROLLER_BASE,
                                     HEXMOTORCONTROLLER_BASE,
                                     HEXMOTORCONTROLLER_BASE};

    holonomic_position_set_mot_encoder(&robot.pos, motor_encoder, motor_encoder_param);


    /****************************************************************************/
    /*                           Trajectory Manager (Trivial)                   */
    /****************************************************************************/
    //trajectory_init(&robot.traj, ASSERV_FREQUENCY);
    //trajectory_set_cs(&robot.traj, &robot.distance_cs, &robot.angle_cs);
    //trajectory_set_robot_params(&robot.traj, &robot.rs, &robot.pos);
    //trajectory_set_speed(&robot.traj, 2400, 1200); /* distance, angle */
    //trajectory_set_acc(&robot.traj, 40., 30.);
    ///* distance window, angle window, angle start */
    //trajectory_set_windows(&robot.traj, 30., 1.0, 20.); // Prod

    //// Angle BDM
    //bd_init(&robot.angle_bd, &robot.angle_cs);
    //bd_set_thresholds(&robot.angle_bd, ROBOT_ANGLE_BD, 5);

    //// Distance BDM
    //bd_init(&robot.distance_bd, &robot.distance_cs);
    //bd_set_thresholds(&robot.distance_bd, ROBOT_DIST_BD, 5);

    //robot.is_aligning = 0;

    //// Initialisation déplacement:
    //position_set(&robot.pos, 0, 0, 0);

    ///* ajoute la regulation au multitache. ASSERV_FREQUENCY est dans cvra_cs.h */
    scheduler_add_periodical_event_priority(cvra_cs_manage, NULL, (1000000
            / ASSERV_FREQUENCY) / SCHEDULER_UNIT, 130);
}

/** Logge l'erreur sur les differents regulateurs et l'affiche avec le temps. */
static void dump_error(void) {
    static int time = 0;
    if (robot.error_dump_enabled) {
        if (time % 10)
            ///@todo : Afficher des trucs utiles
            //fprintf(stderr, "%d;%d;%d\n", time,
                    //(int)cs_get_error(&robot.angle_cs),
                    //(int)cs_get_error(&robot.omega_cs),
                    //(int)cs_get_error(&robot.speed_cs));
        time++;
    } else {
        time = 0;
    }
}

void cvra_cs_manage(__attribute__((unused)) void * dummy) {
    NOTICE(ERROR_CS, __FUNCTION__);

    /* Gestion de la position. */
    rsh_update(&robot.rs);
    holonomic_position_manage(&robot.pos);

    /* Gestion de l'asservissement. */
    cs_manage(&robot.angle_cs); /// @todo : wich one ?
    
    /* Affichage des courbes d'asservissement. */
    dump_error();
}
