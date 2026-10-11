#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* Commissioning values: see docs/COMMISSIONING.md before enabling motion.
 * The real plant is unavailable. These are explicit, UNVERIFIED assumptions.
 * Flashing the default build permits GPIO/encoder inspection but no movement.
 */
#define APP_HARDWARE_CONFIRMED 0u
#define APP_RUN_MODE 0u /* 0=swing-up+balance, 1=homing only, 2=manual upright capture */
#define APP_LIMIT_A_ACTIVE GPIO_PIN_SET
#define APP_LIMIT_B_ACTIVE GPIO_PIN_SET
#define APP_LIMIT_CW_MASK 2u /* 1=A, 2=B; fictional/example wiring until measured */
#define APP_LIMIT_A_PULL GPIO_NOPULL /* use external bias, or set the measured pull */
#define APP_LIMIT_B_PULL GPIO_NOPULL
#define APP_ENCODER_DIRECTION 1 /* +1/-1: theta>0 at bottom moves bob toward +cart x */
#define APP_START_BUTTON_ACTIVE GPIO_PIN_RESET

#define APP_CONTROL_HZ 1000u
#define APP_TIMER_HZ 84000000u
#define APP_STEPS_PER_REV 200u
#define APP_MICROSTEPS DRV8825_FULL_STEP
#define APP_PULLEY_RADIUS_M 0.005f
#define APP_METRES_PER_STEP \
    (6.283185307179586f * APP_PULLEY_RADIUS_M / (APP_STEPS_PER_REV * APP_MICROSTEPS))

/* Preserve the original speed and swing acceleration envelope; these values
 * are NOT a demonstrated swing-up envelope. Raise only after motor tests.
 * The nonlinear simulator has a separately labelled, more capable profile.
 */
#define APP_MAX_STEP_RATE_HZ 1000.0f
#define APP_MAX_STEP_ACCEL 2000.0f
#define APP_HOMING_SPEED_HZ 200.0f
#define APP_HOMING_ACCEL 1000.0f
#define APP_HOMING_MAX_STEPS 20000u
#define APP_HOMING_MIN_STEPS 500u
#define APP_HOMING_TIMEOUT_MS 120000u
#define APP_HOMING_RELEASE_STEPS 100u
#define APP_HOMING_RELEASE_TIMEOUT_MS 2000u
#define APP_HOMING_DEBOUNCE_MS 10u
#define APP_HOMING_BACKOFF_STEPS 20u

/* Uniform-rod interpretation of cartpole.m: J/(m*d), not COM distance. */
#define APP_EFFECTIVE_LENGTH_M 0.70555556f
#define APP_TRAVEL_MARGIN_M 0.03f
#define APP_SWING_TIMEOUT_S 90.0f

#endif
