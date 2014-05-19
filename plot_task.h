enum plot_types {
    PLOT_INT8,
    PLOT_INT16,
    PLOT_INT32,
    PLOT_UINT8,
    PLOT_UINT16,
    PLOT_UINT32,
    PLOT_FLOAT,
    PLOT_DOUBLE
};


void plot_task(void *arg);
void plot_add_variable(char description[8], void* variable, enum plot_types type);
void plot_init(void);
