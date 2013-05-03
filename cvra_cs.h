/**
 * @file cvra_cs.c
 * @author Antoine Albertelli
 * @date 19th September 2009
*/

#ifndef CVRA_CS_H
#define CVRA_CS_H

#include <aversive.h>
#include <aversive/queue.h>
#include <holonomic/trajectory_manager.h>
#include <holonomic/robot_system.h>
#include <holonomic/position_manager.h>
#include "pingpongcannon.h"
#include <control_system_manager.h>
#include <pid.h>
#include <quadramp.h>
#include <ramp.h>
//#include <blocking_detection_manager.h>

#include <cvra_adc.h>
#include <cvra_dc.h>

#ifdef COMPILE_ON_ROBOT
#include <cvra_beacon.h>
#endif

#include <obstacle_avoidance.h>

#include "strat.h"
#include "cvra_param_robot.h"

/** Frequency of the regulation loop (in Hz) */
#define ASSERV_FREQUENCY 100.0

/**
 @brief contains all global vars.
 
 This structure contains all vars that should be global. This is a clean way to
 group all vars in one place. It also serve as a namespace.
 */
struct _rob {
    uint8_t verbosity_level;                ///< @deprecated Contient le niveau de debug du robot.
    
    struct robot_system_holonomic rs;       ///< Holonomic robot system
    struct holonomic_robot_position pos;      ///< Position manager
    
    /** Control system associé à chaque roue (juste les PID) */
    struct cs wheel0_cs;
    struct cs wheel1_cs;
    struct cs wheel2_cs;
    
    /** PID associés à chque roues */
    struct pid_filter wheel0_pid;
    struct pid_filter wheel1_pid;
    struct pid_filter wheel2_pid;
    
#ifdef COMPILE_ON_ROBOT
    volatile cvra_beacon_t beacon;
#endif

    struct ramp_filter wheel0_ramp;
    struct ramp_filter wheel1_ramp;
    struct ramp_filter wheel2_ramp;
    
    /** Filtres */
    struct quadramp_filter angle_qr;
    struct ramp_filter omega_r;
    struct ramp_filter speed_r; /* antoine: dafuq ? */
    
    /** Control system des macro-variables (juste les filtres) */
    struct cs angle_cs;
    struct cs omega_cs;
    struct cs speed_cs;
    
    struct h_trajectory traj;                 ///< Trivial trajectory manager.
    
    int avoiding;
    
    /** waiting for this to be implemented */
    int robot_in_sight;
    // Sans balises on n'en a pas besoin
    //struct blocking_detection angle_bd;     ///< Angle blocking detection manager.
    //struct blocking_detection distance_bd;  ///< Distance blocking detection manager.

    ppc_t cannon;

    uint8_t is_aligning:1;                  ///< =1 if the robot is aligning on border
    
    uint8_t error_dump_enabled:1;           ///< =1 if infos should be dumped
    uint8_t avoiding_enabled:1;
    uint8_t askLog;
};


/**
 @brief Contient toutes les variables globales.
 
 Cette structure sert en quelque sorte de namespace pour les variables globales,
 qui peuvent ensuite etre accedees en faisant robot.color par exemple.
 */
extern struct _rob robot;

/**
 @brief Inits all the regulation related modules.
 
 This function inits and setups the following modules :
 - robot_system
 - encoders_cvra
 - position_manager
 - control_system_manager
 - pid
 - quadramp (acceleration and speed ramps)
 - trajectory_manager
 - blocking_detection_manager
 - couple_limiter
 */
void cvra_cs_init(void);


/**
 @brief Manages regulation related modules
 
 This functions starts by reading the coders value (encoders_cvra), then it
 converts it to angle / distance (robot_system), manages the control systems if
 needed (depends of the robot.mode value). It manages a few event depending on
 the blocking_detection module. Finally it computes position and updates the
 x,y consign if we reached the destination.
 
 @note This function needs to be called often and is compatible with the
 base/scheduler module. The trajectory_manager runs its own task at 10 Hz.
 */        
void cvra_cs_manage(void * dummy);


 
#endif /* CVRA_CS_H */
