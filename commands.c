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
void cmd_encoders(void) {
#ifdef COMPILE_ON_ROBOT
    int i;
    for(i=0;i<6;i++)
        printf("%d;", cvra_dc_get_encoder(HEXMOTORCONTROLLER_BASE, i));
#else
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

/** An array of all the commands. */
command_t commands_list[] = {
    COMMAND("test_argv",test_func),
//    COMMAND("reset", cmd_reset),
    COMMAND("start",cmd_start),
    COMMAND("pid", cmd_pid), 
    COMMAND("pwm", cmd_pwm),
    COMMAND("encoders", cmd_encoders),
    COMMAND("help", cmd_help),
    COMMAND("none",NULL), /* must be last. */
};




