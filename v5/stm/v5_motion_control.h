#ifndef V5_MOTION_CONTROL_H
#define V5_MOTION_CONTROL_H
#include <math.h>
/* NEW starting gains, NOT hardware-validated calibration. Existing v4 limits
 * and per-turn wheel ratios remain in latest_stm.c. Measure before arena use. */
#define V5_TURN_CONTROL_ENABLED 1
#define V5_HEADING_HOLD_ENABLED 1
#define V5_TURN_HOLD_LOCK 1
#define V5_TURN_RATE_MAX_DPS 45.0f
#define V5_TURN_RATE_MIN_DPS 18.0f
#define V5_TURN_ACCEL_DPS2 90.0f
#define V5_TURN_DECEL_DPS2 90.0f
#define V5_TURN_RATE_KP 12.0f
#define V5_TURN_RATE_KI 20.0f
#define V5_TURN_FINAL_TOL_DEG 3.0f
#define V5_HEADING_KP_US 3.0f
#define V5_HEADING_MAX_US 80.0f
#define V5_HEADING_SLEW_US_PER_S 400.0f
#define V5_CENTRE_APPROACH_US 40

typedef struct { float rate_target, integral; } V5TurnControl;
static inline float v5_clamp(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}
static inline float v5_turn_pwm(V5TurnControl *c, float remaining_deg,
        float measured_dps, float dt, float max_pwm, float reference_dps) {
    /* Brake prediction remains outside this controller. Slow before braking. */
    float limit = sqrtf(2.0f * V5_TURN_DECEL_DPS2 * fmaxf(remaining_deg, 0.0f));
    limit = v5_clamp(limit, V5_TURN_RATE_MIN_DPS, V5_TURN_RATE_MAX_DPS);
    c->rate_target = fminf(limit, c->rate_target + V5_TURN_ACCEL_DPS2 * dt);
    float error = c->rate_target - measured_dps;
    float feedforward = max_pwm * c->rate_target / reference_dps;
    float proposed = v5_clamp(c->integral + V5_TURN_RATE_KI * error * dt,
                             -max_pwm, max_pwm);
    float output = feedforward + V5_TURN_RATE_KP * error + proposed;
    /* Conditional integration prevents saturation winding up the controller. */
    if ((output >= 0.0f && output <= max_pwm) ||
        (output > max_pwm && error < 0.0f) || (output < 0.0f && error > 0.0f))
        c->integral = proposed;
    return v5_clamp(feedforward + V5_TURN_RATE_KP * error + c->integral, 0.0f, max_pwm);
}
static inline float v5_heading_pulse(float centre, float error_deg, int dir,
                                     float previous, float dt) {
    float offset = v5_clamp(V5_HEADING_KP_US * error_deg * dir,
                           -V5_HEADING_MAX_US, V5_HEADING_MAX_US);
    float step = V5_HEADING_SLEW_US_PER_S * dt;
    return previous + v5_clamp(centre + offset - previous, -step, step);
}
#endif
