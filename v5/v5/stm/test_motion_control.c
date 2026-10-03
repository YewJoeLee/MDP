#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "v5_motion_control.h"
int main(void) {
    /* Controller bounds and anti-windup; not a model of the actual chassis. */
    V5TurnControl c={0};
    float pwm=0;
    for(int i=0;i<1000;i++) {
        pwm=v5_turn_pwm(&c,90,0,.02f,3075,57);
        assert(isfinite(pwm) && pwm>=0 && pwm<=3075);
        assert(c.rate_target<=V5_TURN_RATE_MAX_DPS);
        assert(fabsf(c.integral)<=3075);
    }
    for(int i=0;i<100;i++) pwm=v5_turn_pwm(&c,1,120,.02f,3075,57);
    assert(pwm<1); /* Do not keep powering a turn already spinning too fast. */
    assert(c.rate_target<=V5_TURN_RATE_MIN_DPS+.01f);
    float centre=1516;
    float f=v5_heading_pulse(centre,5,1,centre,.01f);
    float b=v5_heading_pulse(centre,5,-1,centre,.01f);
    assert(f>centre && b<centre && f-centre<=4.01f);
    float pulse=centre;
    for(int i=0;i<1000;i++) pulse=v5_heading_pulse(centre,100,1,pulse,.01f);
    assert(fabsf(pulse-centre)<=V5_HEADING_MAX_US+.01f);
    assert(v5_heading_pulse(centre,0,1,centre,.01f)==centre);
    /* Simple first-order mock plant: feedback handles different response gains.
       This verifies controller logic only, not wheel slip or stopping accuracy. */
    for(int k=0;k<3;k++) {
        float gain=.7f+.3f*k,rate=0; V5TurnControl d={0};
        for(int i=0;i<1000;i++) {
            float out=v5_turn_pwm(&d,90,rate,.02f,3075,57);
            rate += .2f*(gain*57*out/3075-rate);
        }
        assert(fabsf(rate-fminf(45,gain*57))<2);
    }
    puts("V5 controller logic checks passed (not hardware validation)");
    return 0;
}
