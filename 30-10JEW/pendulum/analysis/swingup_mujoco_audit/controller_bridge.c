/* Diagnostic bridge: execute the SIM copy's actual swingup.c on the host.
 * motor_acc is replaced by ideal command integration and speed clamping.
 * TIM5 and physical STEP/DIR timing are not emulated. Source firmware is unchanged.
 */
#include "swingup.h"
#include <math.h>
#include <string.h>

static float cfg_eps = SWINGUP_EPS;
static float cfg_k = SWINGUP_K;
static float cfg_ref = SWINGUP_E_RIF / SWINGUP_MGL;
static float cfg_amax = SWINGUP_ACC_MAX;
static float cfg_vmax = AUDIT_FMAX_STEPS * M_PER_STEP;
static float cfg_kx = SWINGUP_KX, cfg_kv = SWINGUP_KV;
static float cfg_J = SWINGUP_J, cfg_mgl = SWINGUP_MGL;

/* Allow controlled experiments without editing the original headers. */
#undef SWINGUP_EPS
#undef SWINGUP_K
#undef SWINGUP_E_RIF
#undef SWINGUP_ACC_MAX
#undef SWINGUP_KX
#undef SWINGUP_KV
#undef SWINGUP_J
#undef SWINGUP_MGL
#define SWINGUP_EPS cfg_eps
#define SWINGUP_K cfg_k
#define SWINGUP_E_RIF (cfg_ref * cfg_mgl)
#define SWINGUP_ACC_MAX cfg_amax
#define SWINGUP_KX cfg_kx
#define SWINGUP_KV cfg_kv
#define SWINGUP_J cfg_J
#define SWINGUP_MGL cfg_mgl
#include "swingup.c"

static Encoder_HandleTypeDef enc;
static motor_drv_t md;
static swingup_t su;
static double position_steps;
static const float ts = 0.001f;

/* Read once, before any experiment overrides, to obtain the source defaults. */
void audit_defaults(float *out)
{
    const float values[] = {cfg_eps, cfg_k, cfg_ref, cfg_amax, cfg_vmax,
                           cfg_kx, cfg_kv, cfg_J, cfg_mgl};
    memcpy(out, values, sizeof values);
}

void motor_acc(float a, motor_drv_t *motor)
{
    float v = motor->fcurrent + a * ts;
    const float fmax = cfg_vmax / M_PER_STEP;
    if (v > fmax) v = fmax;
    if (v < -fmax) v = -fmax;
    motor->fcurrent = v;
}

void audit_config(float eps, float k, float ref, float amax, float vmax,
                  float kx, float kv, float J, float mgl)
{
    cfg_eps=eps; cfg_k=k; cfg_ref=ref; cfg_amax=amax; cfg_vmax=vmax;
    cfg_kx=kx; cfg_kv=kv; cfg_J=J; cfg_mgl=mgl;
}

void audit_reset(unsigned short count)
{
    memset(&enc, 0, sizeof enc);
    memset(&md, 0, sizeof md);
    enc.ts=ts; enc.cpr=600; enc.res=4;
    enc.angle=(float)count * (360.0f/2400.0f);
    enc.rad_angle=enc.angle * (float)M_PI/180.0f;
    enc.prev_angle=enc.angle;
    position_steps=0.0;
    swingup_init(&su);
}

/* out: velocity, requested acceleration, commanded position, filtered omega,
 * estimated energy, remembered sign, acceleration after velocity clamping. */
int audit_tick(unsigned short count, float *out)
{
    enc.prev_angle=enc.angle;
    enc.angle=(float)count * (360.0f/2400.0f);
    enc.rad_angle=enc.angle * (float)M_PI/180.0f;
    float delta=(enc.angle-enc.prev_angle)*(float)M_PI/180.0f;
    enc.w=atan2f(sinf(delta),cosf(delta))/ts;
    md.position=(int32_t)llround(position_steps);
    const float previous_v=md.fcurrent*M_PER_STEP;
    int result=swingup_update(&su,&enc,&md);
    out[0]=md.fcurrent*M_PER_STEP;
    out[1]=su.acc_ms2;
    out[2]=(float)(position_steps*M_PER_STEP);
    out[3]=su.w_f;
    out[4]=su.energia;
    out[5]=su.segno;
    out[6]=(out[0]-previous_v)/ts;
    position_steps += (double)md.fcurrent*ts;
    return result;
}
