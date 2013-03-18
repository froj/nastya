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



/** An array of all the commands. */
command_t commands_list[] = {
    COMMAND("test_argv",test_func),
    COMMAND("reset", cmd_reset),
    COMMAND("start",cmd_start),
    COMMAND("none",NULL), /* must be last. */
};




