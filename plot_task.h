enum plot_types {
    INT8,
    INT16,
    INT32,
    UINT8,
    UINT16,
    UINT32,
    FLOAT,
    DOUBLE
};


void plot_task(void);
void plot_add_variable(char description[8], void* variable, enum plot_types type);
void plot_init(void);
