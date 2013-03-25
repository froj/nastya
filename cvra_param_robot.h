/*
 * cvra_param_robot.h
 *
 *  Created on: 10 avr. 2012
 *      Author: Flopy
 */

#ifndef CVRA_PARAM_ROBOT_H_
#define CVRA_PARAM_ROBOT_H_

//*******************************************************************************************
//                                             ROBOT
//*******************************************************************************************
//                       *********************************************
//                                             PID
//                       *********************************************

#define ROBOT_PID_WHEEL0_P 200
#define ROBOT_PID_WHEEL0_I 0
#define ROBOT_PID_WHEEL0_D 400

#define ROBOT_PID_WHEEL1_P 200
#define ROBOT_PID_WHEEL1_I 0
#define ROBOT_PID_WHEEL1_D 400

#define ROBOT_PID_WHEEL2_P 200
#define ROBOT_PID_WHEEL2_I 0
#define ROBOT_PID_WHEEL2_D 400


//                       *********************************************
//                                         Dimensions
//                       *********************************************
//#define ROBOT_ECART_ROUE 275.7880216836 //275.1716841994 // 276.8397370540  276
//#define ROBOT_INC_MM 162.73750397497828327709 //162.487138584247  //Calculé: 162.9746

//#define ROBOT_WHEEL_L_CORR 1.00107255365769 //1.00172702807009
//#define ROBOT_WHEEL_R_CORR -0.998927446342313 //-0.99827297192991

//                       *********************************************
//                                             Blocage
//                       *********************************************
//#define ROBOT_ANGLE_BD 10000
//#define ROBOT_DIST_BD 5000

//                       *********************************************
//                                    Vitesse/Accélérations
//                       *********************************************
//#define ROBOT_DIST_SPEED_FAST 500 // MM PAR S
//#define ROBOT_DIST_SPEED_SLOW 300
//#define ROBOT_ANGL_SPEED_FAST 300 // DEG PAR S
//#define ROBOT_ANGL_SPEED_SLOW 100

//#define ROBOT_DIST_ACC 8 // MM PAR S^2
//#define ROBOT_ANGL_ACC 1 // DEG PAR S^2  // 4 normalement

//#define NB_COINS 38
//#define NB_LINGOT 7
//#define NB_POS_PRISE 4

//#define TABLE_X_MM 3000
//#define TABLE_Y_MM 2000
#endif /* CVRA_PARAM_ROBOT_H_ */
