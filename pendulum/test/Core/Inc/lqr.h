#include <math.h>
#include "encoder.h"
#include "motor.h"
#define m_per_step 0.0003141592653589793f //RAGGIO PER ANGOLO DISPLACEMENT PER STEP (ANGOLO DISPLACEMENT=||RAGGIO DELLA PULEGGIA=)
void lqr_controller(Encoder_HandleTypeDef *encoder, motor_drv_t *motor_drv, float *acc);