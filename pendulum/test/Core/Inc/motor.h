/*
 * motor.h
 *
 *  Created on: 13 set 2026
 *      Author: vito
 */
typedef enum {
NEMA_CW,
NEMA_CCW
} nema_direction_t;

typedef struct {
uint16_t steps_per_rev; /* Steps per revolution: ex. 200 */
GPIO_TypeDef *dir_port; /* DIR gpio port */
uint16_t dir_pin; /* DIR gpio pin */
TIM_HandleTypeDef *step_tim; /* STEP timer */
uint32_t step_tim_clk; /* Timer clock frequency in Hz*/
uint32_t step_tim_channel; /* Timer channel */
nema_direction_t dir;
} nema_t;


typedef enum {
DRV8825_FULL_STEP=1,
DRV8825_HALF_STEP=2,
DRV8825_4_STEPS=4,
DRV8825_8_STEPS=8,
DRV8825_16_STEPS=16,
DRV8825_32_STEPS=32
} drv8825_microstep_t;

typedef struct {
drv8825_microstep_t microsteps;
GPIO_TypeDef *m0_port;
uint16_t m0_pin;
GPIO_TypeDef *m1_port;
uint16_t m1_pin;
GPIO_TypeDef *m2_port;
uint16_t m2_pin;

GPIO_TypeDef *rst_slp_port;
uint16_t rst_slp_pin;
} drv8825_t;


typedef enum {
NEMA_DRV8825_STOPPED,
NEMA_DRV8825_ACCEL,
NEMA_DRV8825_RUN,
NEMA_DRV8825_DECEL
} nema_drv8825_motion_state_t;

typedef struct {
drv8825_t *drv; /* Pointer to the driver */
nema_t *nema; /* Pointer to nema */
float current_rpm; /* Current rot. speed [rpm] */
float target_rpm; /* Desired rot. speed [rpm] */
float accel_rpm_s; /* Acceleration [rpm/s] */
float decel_rpm_s; /* Deceleration [rpm/s] */
nema_drv8825_motion_state_t state; /* Motion state */
uint8_t pwm_on; /* PWM is running [boolean] */
} nema_drv8825_t;

