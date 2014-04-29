#ifndef POSITION_INTEGRATION_H
#define POSITION_INTEGRATION_H


void get_position(float *x, float *y);
float get_heading(void);

void get_velocity(float *x, float *y);
float get_omega(void);

void position_reset(void);
void position_reset_to(float x, float y, float theta);

void start_position_integration(void);

#endif // POSITION_INTEGRATION_H
