#include <math.h>
#include "encoder.h"
#include "motor.h"
#define m_per_step 0.0314159f*0.005f //RAGGIO PER ANGOLO DISPLACEMENT PER STEP (ANGOLO DISPLACEMENT= 0.0314159 ||RAGGIO DELLA PULEGGIA=0.005 )
void lqr_controller(Encoder_HandleTypeDef *encoder, motor_drv_t *motor_drv, float *acc);
