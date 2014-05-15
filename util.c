#include <math.h>
#include "util.h"

float limit_sym(float val, float max)
{
    if (val > max)
        return max;
    if (val < -max)
        return -max;
    return val;
}

float circular_range(float ang)
{
    while (ang > M_PI)
        ang -= 2*M_PI;
    while (ang <= -M_PI)
        ang += 2*M_PI;
    return ang;
}
