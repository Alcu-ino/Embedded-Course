#include "pendulum_control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#define PI 3.14159265358979323846f
static pendulum_output_t samples(pendulum_control_t *c, pendulum_input_t *in, unsigned n)
{
    pendulum_output_t out={0};
    while(n--) out=pendulum_control_step(c,in);
    return out;
}
int main(void)
{
    pendulum_control_t c;
    pendulum_config_t cfg=pendulum_default_config();
    pendulum_input_t in={0};
    assert(pendulum_control_init(&c,&cfg));
    assert(pendulum_control_step(&c,&in).acceleration_m_s2>0); /* deterministic launch */
    in.angle_rad=PI;
    in.angular_velocity_rad_s=2; /* high-speed crossing cannot capture */
    assert(samples(&c,&in,100).mode==PENDULUM_SWING_UP);
    in.angular_velocity_rad_s=0;
    assert(samples(&c,&in,10).mode==PENDULUM_SWING_UP);
    assert(samples(&c,&in,20).mode==PENDULUM_BALANCE);
    in.position_m=.05f;
    pendulum_output_t out=pendulum_control_step(&c,&in);
    assert(out.acceleration_m_s2>0); /* position reference survives capture */
    in.angle_rad=PI+.3f;
    assert(pendulum_control_step(&c,&in).mode==PENDULUM_BALANCE); /* hysteresis */
    in.angle_rad=PI+.7f;
    assert(pendulum_control_step(&c,&in).mode==PENDULUM_SWING_UP);
    in.position_m=.42f;
    in.velocity_m_s=.15f;
    assert(pendulum_control_step(&c,&in).acceleration_m_s2<0); /* endpoint braking */
    in.position_m=.41f;
    in.velocity_m_s=1.0f; /* infeasible incoming speed: brake, never exceed acceleration */
    out=pendulum_control_step(&c,&in);
    assert(out.acceleration_m_s2<0 && fabsf(out.acceleration_m_s2)<=cfg.max_accel_m_s2);
    in.position_m=.45f;
    out=pendulum_control_step(&c,&in);
    assert(out.mode==PENDULUM_FAULT && out.fault==PENDULUM_FAULT_TRAVEL);
    in.position_m=0;
    assert(pendulum_control_step(&c,&in).fault==PENDULUM_FAULT_TRAVEL); /* latched */
    assert(pendulum_control_init(&c,&cfg));
    in.angle_rad=NAN;
    assert(pendulum_control_step(&c,&in).fault==PENDULUM_FAULT_SENSOR);
    cfg.dt_s=0;
    assert(!pendulum_control_init(&c,&cfg));
    assert(pendulum_control_step(&c,&in).fault==PENDULUM_FAULT_CONFIG);
    cfg=pendulum_default_config();
    cfg.swing_timeout_s=.05f;
    assert(pendulum_control_init(&c,&cfg));
    in=(pendulum_input_t){0};
    assert(samples(&c,&in,100).fault==PENDULUM_FAULT_TIMEOUT);

    /* Independent energy balance from the nonlinear equation. The pump's
     * command must increase pendulum energy below the upright energy. */
    cfg.swing_position_gain=0;
    cfg.swing_velocity_gain=0;
    cfg.swing_timeout_s=1;
    assert(pendulum_control_init(&c,&cfg));
    c.elapsed_s=.3f;
    in.angle_rad=.6f; in.angular_velocity_rad_s=1.0f;
    out=pendulum_control_step(&c,&in);
    const float acceleration_theta=-(9.81f*sinf(in.angle_rad)+out.acceleration_m_s2*cosf(in.angle_rad))/cfg.effective_length_m;
    const float energy_rate=cfg.effective_length_m/9.81f*in.angular_velocity_rad_s*acceleration_theta+sinf(in.angle_rad)*in.angular_velocity_rad_s;
    assert(energy_rate>0);
    assert(fabsf(out.acceleration_m_s2)<=cfg.max_accel_m_s2);
    puts("PASS controller: launch, angle+rate capture, hysteresis, homed reference, braking, latched faults, timeout, physical energy injection.");
    return 0;
}
