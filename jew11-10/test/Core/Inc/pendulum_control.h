#ifndef PENDULUM_CONTROL_H
#define PENDULUM_CONTROL_H

/* Hardware-independent acceleration controller, shared by firmware and SIL.
 * x=0 is the homed rail midpoint; x and velocity are in metres and m/s.
 * theta=0 is DOWN; theta>0 displaces the bob toward positive cart x.
 * theta_ddot = -(g*sin(theta) + cart_accel*cos(theta))/effective_length.
 * effective_length = J_pivot/(mass * COM_distance), NOT COM_distance.
 */
typedef enum {
    PENDULUM_SWING_UP = 0,
    PENDULUM_BALANCE,
    PENDULUM_FAULT
} pendulum_mode_t;

typedef enum {
    PENDULUM_FAULT_NONE = 0,
    PENDULUM_FAULT_CONFIG,
    PENDULUM_FAULT_SENSOR,
    PENDULUM_FAULT_TRAVEL,
    PENDULUM_FAULT_TIMEOUT
} pendulum_fault_t;

typedef struct {
    float dt_s;
    float effective_length_m;
    float max_accel_m_s2;
    float max_speed_m_s;
    float track_half_width_m;
    float travel_margin_m;
    float swing_timeout_s;
    float energy_gain;
    float swing_position_gain;
    float swing_velocity_gain;
} pendulum_config_t;

typedef struct {
    float position_m;
    float velocity_m_s;
    float angle_rad;
    float angular_velocity_rad_s;
} pendulum_input_t;

typedef struct {
    float acceleration_m_s2;
    float upright_error_rad;
    float normalized_energy;  /* E/(m*g*COM_distance), upright reference=2 */
    pendulum_mode_t mode;
    pendulum_fault_t fault;
} pendulum_output_t;

typedef struct {
    pendulum_config_t config;
    pendulum_output_t output;
    float elapsed_s;
    float swing_elapsed_s;
    float capture_elapsed_s;
    float kickoff_sign;
    unsigned int captures;
} pendulum_control_t;

/* Conservative legacy hardware envelope; commissioning must confirm limits.
 * Width is a placeholder: replace it with the measured homed half stroke.
 */
pendulum_config_t pendulum_default_config(void);
int pendulum_control_init(pendulum_control_t *controller, const pendulum_config_t *config);
pendulum_output_t pendulum_control_step(pendulum_control_t *controller, const pendulum_input_t *input);
float pendulum_wrap_angle(float angle_rad);
/* Continuous-time pole placement at -2,-2,-3,-3 rad/s. */
float pendulum_balance_acceleration(float effective_length_m, const pendulum_input_t *input);

#endif
