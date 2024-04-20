//define a IIR SOS CMSIS-DSP coefficient array

#include <stdint.h>

#ifndef STAGES
#define STAGES 2
#endif
/*********************************************************/
/*                     IIR SOS Filter Coefficients       */
float32_t ba_coeff[10] = { //b0,b1,b2,a1,a2,... by stage
    +1.295857e-01, +1.082315e-01, +1.295857e-01,
    +4.447339e-01, -1.301534e-01,
    +1.000000e+00, -8.216198e-01, +1.000000e+00,
    +1.036323e+00, -6.679663e-01
};
/*********************************************************/
