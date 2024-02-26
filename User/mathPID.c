#include "mathPID.h"
#include "main.h"
void Math_Constrain(float *x, float Min, float Max)
{
    if (*x < Min)
    {
        *x = Min;
    }
    else if (*x > Max)
    {
        *x = Max;
    }
}

float Math_Abs(float x)
{
    return ((x > 0) ? x : -x);
}
