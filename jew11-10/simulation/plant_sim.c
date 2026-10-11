/* Software-in-the-loop plant. The controller below is the actual firmware C;
 * only the plant and raw encoder counts are simulated. */
#include "pendulum_control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "encoder.h"

#define PI 3.14159265358979323846
#define DT 0.001
#define COUNT_RAD (2.0 * PI / 2400.0)
#define STEP_M (2.0 * PI * .005 / 200.0)

typedef struct { double x, v, theta, omega; } plant_t;
typedef struct { double length, damping, friction_nm, inertia, lag_s; } physics_t;

static plant_t derivative(plant_t y, double a, const physics_t *p)
{
    plant_t d = {y.v, a, y.omega,
        -(9.81*sin(y.theta) + a*cos(y.theta))/p->length
        - p->damping*y.omega - p->friction_nm/p->inertia*tanh(y.omega/.03)};
    return d;
}
static plant_t add(plant_t y, plant_t d, double h)
{
    plant_t result = { y.x+h*d.x, y.v+h*d.v, y.theta+h*d.theta, y.omega+h*d.omega };
    return result;
}
static plant_t integrate(plant_t y, double a, const physics_t *p)
{
    plant_t k1=derivative(y,a,p), k2=derivative(add(y,k1,DT/2),a,p),
        k3=derivative(add(y,k2,DT/2),a,p), k4=derivative(add(y,k3,DT),a,p);
    plant_t result = {y.x+DT/6*(k1.x+2*k2.x+2*k3.x+k4.x),
        y.v+DT/6*(k1.v+2*k2.v+2*k3.v+k4.v),
        y.theta+DT/6*(k1.theta+2*k2.theta+2*k3.theta+k4.theta),
        y.omega+DT/6*(k1.omega+2*k2.omega+2*k3.omega+k4.omega)};
    return result;
}
int main(int argc, char **argv)
{
    const char *scenario=argc>1?argv[1]:"swingup";
    pendulum_config_t config=pendulum_default_config();
    config.max_accel_m_s2=3.0f;
    config.max_speed_m_s=1.0f;
    plant_t y={0};
    physics_t physics={.70555556,.02,0,.010583333,0};
    double duration=80, perturb_at=-1, capture=-1, maxx=0, maxv=0, maxa=0;
    int expect_timeout=0;
    if (!strcmp(scenario,"offset")) { y.x=.05; y.theta=.05; }
    else if (!strcmp(scenario,"mirror")) { y.x=-.05; y.theta=-.05; }
    else if (!strcmp(scenario,"model_error")) { physics.length*=1.1; physics.damping=.04; }
    else if (!strcmp(scenario,"friction")) { physics.friction_nm=.001; }
    else if (!strcmp(scenario,"balance")) { y.theta=PI-.03; duration=25; perturb_at=8; }
    else if (!strcmp(scenario,"legacy_limits")) {
        config=pendulum_default_config(); duration=95; expect_timeout=1;
    } else if (strcmp(scenario,"swingup")) { fprintf(stderr,"unknown scenario\n"); return 2; }
    if (argc>3) config.energy_gain=(float)atof(argv[3]);
    if (argc>4) config.swing_position_gain=(float)atof(argv[4]);
    if (argc>5) config.swing_velocity_gain=(float)atof(argv[5]);
    pendulum_control_t controller;
    assert(pendulum_control_init(&controller,&config));
    Encoder_HandleTypeDef encoder;
    init_Encoder(&encoder,600,4,(float)DT);
    encoder_set_direction(&encoder,1);
    encoder_calibrate_down(&encoder,0);
    /* Sensor was continuously sampled before the test begins. Warm its
     * previous count to the initial state without claiming this is down. */
    encoder_process_count(&encoder,(uint16_t)(((long)llround(y.theta/COUNT_RAD)%2400+2400)%2400));
    for(int j=0;j<1000;j++) encoder_process_count(&encoder,(uint16_t)(((long)llround(y.theta/COUNT_RAD)%2400+2400)%2400));

    FILE *csv=argc>2 && strcmp(argv[2],"-") ? fopen(argv[2],"w") : NULL;
    if(csv) fprintf(csv,"time_s,x_m,v_m_s,theta_rad,omega_rad_s,accel_m_s2,measured_omega,mode\n");
    double actual_a=0, sum_error=0, sumx=0, final_peak=0; int final_n=0;
    pendulum_output_t out={0};
    for(int i=0;i<(int)(duration/DT);i++) {
        double t=i*DT;
        if(perturb_at>=0 && i==(int)(perturb_at/DT)) y.omega+=.2;
        long count=llround(y.theta/COUNT_RAD);
        if(!encoder_process_count(&encoder,(uint16_t)((count%2400+2400)%2400))) { fprintf(stderr,"encoder rejected count\n"); return 4; }
        double measured_theta=encoder.rad_angle, measured_omega=encoder.w;

        pendulum_input_t in={(float)(llround(y.x/STEP_M)*STEP_M),(float)y.v,(float)measured_theta,(float)measured_omega};
        out=pendulum_control_step(&controller,&in);
        if(out.mode==PENDULUM_FAULT) {
            if(csv) fclose(csv);
            if(expect_timeout && out.fault==PENDULUM_FAULT_TIMEOUT) {
                printf("PASS %s: bounded timeout at %.3fs; no swing-up claimed; max |x|=%.4fm |v|=%.4fm/s\n",scenario,t,maxx,maxv); return 0;
            }
            fprintf(stderr,"FAIL %s: controller fault %d at %.2fs x%.3f\n",scenario,out.fault,t,y.x); return 1;
        }
        if(out.mode==PENDULUM_BALANCE && capture<0) capture=t;
        actual_a+= physics.lag_s>0 ? DT/(physics.lag_s+DT)*(out.acceleration_m_s2-actual_a) : out.acceleration_m_s2-actual_a;
        y=integrate(y,actual_a,&physics);
        maxx=fmax(maxx,fabs(y.x)); maxv=fmax(maxv,fabs(y.v)); maxa=fmax(maxa,fabs(actual_a));
        if(fabs(y.x)>config.track_half_width_m || fabs(y.v)>config.max_speed_m_s+.002 || fabs(actual_a)>config.max_accel_m_s2+.001) {
            fprintf(stderr,"FAIL %s: exceeded envelope t%.2f x%.4f v%.4f a%.4f\n",scenario,t,y.x,y.v,actual_a);if(csv)fclose(csv);return 1;
        }
        if(t>duration-10) {
            double e=remainder(y.theta-PI,2*PI);
            sum_error+=e*e; sumx+=y.x*y.x; final_peak=fmax(final_peak,fabs(e));final_n++;
            if(out.mode!=PENDULUM_BALANCE) final_peak=10;
        }
        if(csv && i%10==0) fprintf(csv,"%.3f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%d\n",t,y.x,y.v,y.theta,y.omega,actual_a,measured_omega,out.mode);
    }
    if(csv) fclose(csv);
    const double rms=sqrt(sum_error/fmax(final_n,1)),xrms=sqrt(sumx/fmax(final_n,1));
    if(capture<0 || final_peak>.05 || xrms>.025) {
        fprintf(stderr,"FAIL %s: capture %.3fs final angle RMS %.4frad peak%.4frad cart RMS%.4fm max |x|%.4fm\n",scenario,capture,rms,final_peak,xrms,maxx); return 1;
    }
    printf("PASS %s: capture %.3fs; final 10s angle RMS %.5frad, cart RMS %.5fm; max |x| %.4fm |v| %.4fm/s |a| %.4fm/s^2; captures %u\n",scenario,capture,rms,xrms,maxx,maxv,maxa,controller.captures);
    return 0;
}
