#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#define RES 4u
#define CPR 600u
#define ENCODER_DEFAULT_CUTOFF_HZ 20.0f
#define ENCODER_DEFAULT_MAX_SPEED_RAD_S 50.0f

/* TIM3 wraps at cpr*res counts (ARR=cpr*res-1). Angle is measured from
 * calibrated DOWN, in [0, 2*pi). Choose direction so a small positive angle
 * from down moves the COM toward positive cart x. An incremental encoder
 * cannot discover the physical down reference itself. */
typedef struct {
    float ts;
    uint32_t cpr;
    uint8_t res;
    volatile float rad_angle;  /* [rad], calibrated down=0, upright=pi */
    volatile float angle;      /* [degree], same orientation */
    volatile float prev_angle; /* [degree], previous accepted sample */
    volatile float w;          /* [rad/s], filtered angular velocity */
    float velocity_cutoff_hz;  /* first-order derivative low-pass cutoff */
    float max_angular_speed_rad_s; /* reject implausible raw count jumps */
    uint16_t down_count;
    uint16_t previous_count;
    volatile uint16_t raw_count; /* latest input, including rejected samples */
    int8_t direction;          /* +1 or -1; verify on the actual plant */
    volatile uint8_t valid;    /* latest sample accepted */
    uint8_t calibrated;
    uint8_t primed;
} Encoder_HandleTypeDef;

void init_Encoder(Encoder_HandleTypeDef *encoder, uint32_t cpr,
                  uint8_t res, float sample_time_s);
void reset_Encoder(Encoder_HandleTypeDef *encoder, uint32_t cpr,
                   uint8_t res, float sample_time_s);
/* Changing direction invalidates the reference: calibrate again afterward. */
void encoder_set_direction(Encoder_HandleTypeDef *encoder, int8_t direction);
/* Call only with the pendulum physically stationary at its DOWN reference.
 * The first subsequent sample seeds history with zero velocity, avoiding a
 * false startup derivative after a gap in sampling. */
void encoder_calibrate_down(Encoder_HandleTypeDef *encoder, uint16_t raw);
/* Call exactly once per ts. Returns 1 for an accepted calibrated sample.
 * Rejected data never changes the last good position/history. The application
 * must latch a sensor fault on rejection while armed. A stationary/disconnected
 * encoder cannot be distinguished using counts alone. */
uint8_t encoder_process_count(Encoder_HandleTypeDef *encoder, uint16_t raw);
#ifndef PENDULUM_HOST_TEST
void update_Encoder(Encoder_HandleTypeDef *encoder);
#endif

#endif
