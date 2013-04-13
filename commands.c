#include <commandline.h>
#include <string.h>
#include <stdio.h>
#include <cvra_servo.h>
#include "adresses.h"
#include "cvra_cs.h"
#include "strat.h"

/** Prints all args, then exits. */
void test_func(int argc, char **argv) {
    int i;
    for(i=0;i<argc;i++)
        printf("argv[%d] = \"%s\"\n", i, argv[i]);
}

/** resets the robot. */
void cmd_reset(void) {
    reset();
}


/** Move to a point */
void cmd_move(int argc, char **argv) {
    if (argc == 3) {
        holonomic_trajectory_moving_straight_goto_xy_abs(&robot.traj, atoi(argv[1]), atoi(argv[2]));
        while(!holonomic_robot_in_xy_window(&robot.traj, 30));
        }
    else {
         printf("Usage: move x_mm y_mm\n");
     }
}

void cmd_turn(int argc, char **argv) {
    if (argc == 2) {
        holonomic_trajectory_turning_cap(&robot.traj, atof(argv[1]));
        //while(!holonomic_robot_in_xy_window(&robot.traj, 30));
        }
    else {
         printf("Usage: turn angle [rad] \n");
     }
}


/** Writes to a specific PWM. */
void cmd_pwm(int argc, char **argv) {
    if(argc == 3) {
        printf("Putting channel %d = %d\n", atoi(argv[1]), atoi(argv[2]));
#ifdef COMPILE_ON_ROBOT
        cvra_dc_set_pwm((void*)HEXMOTORCONTROLLER_BASE, atoi(argv[1]), atoi(argv[2]));
#endif
    }else{
        printf("Usage: pwm channel value\n");
    }
}

/** Gets the encoder values. */
void cmd_encoders(int argc, char **argv) {
#ifdef COMPILE_ON_ROBOT
    int i;
    for(i=0;i<6;i++){
        if(argc > 1){
            cvra_dc_set_encoder((void*)HEXMOTORCONTROLLER_BASE, i, 0);
        }
        printf("%d;", (int)cvra_dc_get_encoder((void*)HEXMOTORCONTROLLER_BASE, i));
    }
#else
    (void)argc;
    (void)argv;
    printf("Not on robot, bitch.\n");
#endif
    printf("\n");
}

/** Setups PID. */
void cmd_pid(int argc, char **argv) {
    if(argc < 2) {
        /* Show current gains. */
        printf("Wheel 0 : \tKp=%d\tGi=%d\tGd=%d\n",
                pid_get_gain_P(&robot.wheel0_pid), 
                pid_get_gain_I(&robot.wheel0_pid),
                pid_get_gain_D(&robot.wheel0_pid));

        printf("Wheel 1 : \tKp=%d\tGi=%d\tGd=%d\n",
                pid_get_gain_P(&robot.wheel1_pid), 
                pid_get_gain_I(&robot.wheel1_pid),
                pid_get_gain_D(&robot.wheel1_pid));

        printf("Wheel 2 : \tKp=%d\tGi=%d\tGd=%d\n",
                pid_get_gain_P(&robot.wheel2_pid), 
                pid_get_gain_I(&robot.wheel2_pid),
                pid_get_gain_D(&robot.wheel2_pid));

    }
    else if(argc < 5) {
            printf("usage: %s pid_name P I D\n", argv[0]);
    } 
    else {
        struct pid_filter *pid;

        if(!strcmp(argv[1], "w0")) pid =  &robot.wheel0_pid;
        else if(!strcmp(argv[1], "w1")) pid =  &robot.wheel1_pid;
        else if(!strcmp(argv[1], "w2")) pid =  &robot.wheel2_pid;
        else {
            printf("Unknown PID name : %s\n", argv[1]);
            return;
        }

        /** @todo We should be more cautious when handling user input. */
        /** @workaround : to set all the pid */
        pid_set_gains(&robot.wheel0_pid, atoi(argv[2]), atoi(argv[3]), atoi(argv[4])); 
        pid_set_gains(&robot.wheel1_pid, atoi(argv[2]), atoi(argv[3]), atoi(argv[4]));
        pid_set_gains(&robot.wheel2_pid, atoi(argv[2]), atoi(argv[3]), atoi(argv[4]));
        //pid_set_gains(pid, atoi(argv[2]), atoi(argv[3]), atoi(argv[4])); 
    }
}

/** Set or get the position */
void cmd_position(int argc, char **argv){
    if(argc == 1){
        printf("x: %lf; y: %lf; a: %lf\n", holonomic_position_get_x_double(&robot.pos), 
                                          holonomic_position_get_y_double(&robot.pos),
                                          holonomic_position_get_a_rad_double(&robot.pos));
    }else{
        holonomic_position_set_x_s16(&robot.pos, (int16_t)atoi(argv[1]));
        holonomic_position_set_y_s16(&robot.pos, (int16_t)atoi(argv[2]));
        holonomic_position_set_a_s16(&robot.pos, (int16_t)atoi(argv[3]));
    }
}

/** Set the macro-variable (speed. direction, omega) via trajectory */
void cmd_set_macro_var(int argc, char **argv) {
    if(argc < 3)
       printf("Usage: macro_var SPEED DIRECTION ROT_SPEED\n");
   else
      holonomic_trajectory_set_var(&robot.traj, (int32_t)atoi(argv[1]), (int32_t)atoi(argv[2]), (int32_t)atoi(argv[3]));
}

/** Lists all available commands. */
void cmd_help(void) {
    int i;
    extern command_t commands_list[];
    for(i=0;commands_list[i].f!= NULL;i++) {
        printf("%s\t", commands_list[i].name);
        if(i > 0 && i%4 == 0)
            printf("\n");
    }
    printf("\n");
}

void cmd_speed(int argc, char **argv) {
    if(argc < 2){
        printf("Translation Speed: %f\nDirection:         %f\nRotation Speed:    %f\n",
                robot.rs.speed, robot.rs.direction, robot.rs.rotation_speed);
    }else if(argc < 3){
        printf("Usage: speed SPEED DIRECTION (DEG) ROT_SPEED\n");
    }else{
        rsh_set_speed(&robot.rs, (int32_t)atoi(argv[1]));
        rsh_set_direction_int(&robot.rs, (int32_t)atoi(argv[2]));
        if(argc > 3){
            rsh_set_rotation_speed(&robot.rs, (int32_t)atoi(argv[3]));
        }
    }
}

void cmd_circle(int argc, char **argv) {
    if(argc < 3){
        printf("Usage: circle center_x[mm] center_y[mm] section[rad]\n");
    }else{
        holonomic_trajectory_moving_circle(&robot.traj,(int32_t)atoi(argv[1]) ,(int32_t)atoi(argv[2]), (double)atof(argv[3]));
        }
}

void cmd_get_speed(void){
    printf("Translation Speed: %f\nDirection: %d\nRotations Speed: %lf\n",
            holonomic_position_get_instant_translation_speed(&robot.pos),
            (int)holonomic_position_get_theta_v_int(&robot.pos),
            holonomic_position_get_instant_rotation_speed(&robot.pos));
}

void cmd_delta_enc(void){
    printf("%d; %d; %d;\n", (int)robot.pos.delta_enc[0], (int)robot.pos.delta_enc[1], (int)robot.pos.delta_enc[2]);
}

void cmd_cs_enable(int argc, char **argv) {
    (void)argv;
    if(argc > 1){
        cs_disable(&robot.wheel0_cs);
        cs_disable(&robot.wheel1_cs);
        cs_disable(&robot.wheel2_cs);
    }else{
        cs_enable(&robot.wheel0_cs);
        cs_enable(&robot.wheel1_cs);
        cs_enable(&robot.wheel2_cs);
    }
}

void cmd_exit(void) {
    exit(0);
}
void cmd_start(int argc) {
    printf("Press a key to start the robot.\n");
    getchar();
    while((IORD(PIO_BASE, 0) & 0x1000) == 0);
    if (argc == 1)
        strat_begin(RED);
    else
        strat_begin(BLUE);
    printf("Match done. Hope you enjoyed it !\n");
}

void cmd_do_gift(int argc, char** argv){
    strat_do_gift(atoi(argv[1]));
}

void cmd_servo(int argc, char** argv){
    cvra_servo_set((void*)SERVOS_BASE, (int)atoi(argv[1]), (uint32_t)atoi(argv[2]));
}

void cmd_get_io(int argc, char** argv){
    printf("%d\n", (uint32_t)IORD(PIO_BASE, 0));
}


/** An array of all the commands. */
command_t commands_list[] = {
    COMMAND("test_argv",test_func),
    COMMAND("reset", cmd_reset),
    COMMAND("start",cmd_start),
    COMMAND("pid", cmd_pid), 
    COMMAND("pwm", cmd_pwm),
    COMMAND("encoders", cmd_encoders),
    COMMAND("pos", cmd_position),
    COMMAND("help", cmd_help),
    COMMAND("speed", cmd_speed),
    COMMAND("cs_enable", cmd_cs_enable),
    COMMAND("get_speed", cmd_get_speed),
    COMMAND("delta_enc", cmd_delta_enc),
    COMMAND("move", cmd_move),
    COMMAND("macro_var", cmd_set_macro_var),
    COMMAND("exit", cmd_exit),
    COMMAND("circle", cmd_circle),
    COMMAND("start", cmd_start),
    COMMAND("do_gift", cmd_do_gift),
	COMMAND("turn", cmd_turn),
    COMMAND("servo", cmd_servo),
    COMMAND("io", cmd_get_io),
    COMMAND("none",NULL), /* must be last. */
};




