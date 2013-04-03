#include <commandline.h>
#include <string.h>
#include "cvra_cs.h"

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


/** starts the strategy. */
void cmd_start() {
    printf("Press a key to start the robot.\n");
    getchar();
    printf("Match done. Hope you enjoyed it !\n");
}

/** Writes to a specific PWM. */
void cmd_pwm(int argc, char **argv) {
    if(argc == 3) {
        printf("Putting channel %d = %d\n", atoi(argv[1]), atoi(argv[2]));
#ifdef COMPILE_ON_ROBOT
        cvra_dc_set_pwm(HEXMOTORCONTROLLER_BASE, atoi(argv[1]), atoi(argv[2]));
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
            cvra_dc_set_encoder(HEXMOTORCONTROLLER_BASE, i, 0);
        }
        printf("%d;", cvra_dc_get_encoder(HEXMOTORCONTROLLER_BASE, i));
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
        pid_set_gains(pid, atoi(argv[2]), atoi(argv[3]), atoi(argv[4])); 
    }
}

/** Set or get the position */
void cmd_position(int argc, char **argv){
    if(argc == 1){
        printf("x: %lf; y: %lf; a: %d\n", holonomic_position_get_x_double(&robot.pos), 
                                          holonomic_position_get_y_double(&robot.pos),
                                          holonomic_position_get_a_deg_s16(&robot.pos));
    }else{
        holonomic_position_set_x_s16(&robot.pos, (int16_t)atoi(argv[1]));
        holonomic_position_set_y_s16(&robot.pos, (int16_t)atoi(argv[2]));
        holonomic_position_set_a_s16(&robot.pos, (int16_t)atoi(argv[3]));
    }
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
        printf("Usage: speed SPEED DIRECTION ROT_SPEED\n");
    }else{
        rsh_set_speed(&robot.rs, (int32_t)atoi(argv[1]));
        rsh_set_direction_int(&robot.rs, (int32_t)atoi(argv[2]));
        if(argc > 3){
            rsh_set_rotation_speed(&robot.rs, (int32_t)atoi(argv[3]));
        }
    }
}

void cmd_cs_enable(int argc, char **argv) {
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

/** An array of all the commands. */
command_t commands_list[] = {
    COMMAND("test_argv",test_func),
//    COMMAND("reset", cmd_reset),
    COMMAND("start",cmd_start),
    COMMAND("pid", cmd_pid), 
    COMMAND("pwm", cmd_pwm),
    COMMAND("encoders", cmd_encoders),
    COMMAND("pos", cmd_position),
    COMMAND("help", cmd_help),
    COMMAND("speed", cmd_speed),
    COMMAND("cs_enable", cmd_cs_enable),
    COMMAND("none",NULL), /* must be last. */
};




