#include "Navigation.h"

// Clamp helper
static inline float clampF(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

// Convert distance (cm) → closeness [0–1]
float computeCloseness(float d, float range, float crit) {
    if (d <= 0 || d >= range) return 0.0f;
    if (d <= crit) return 1.0f;
    return (range - d) / (range - crit);
}

// === Main Steering Function ===
void computeSteering(
    SteeringConfig &cfg,
    SteeringState &state,
    float dF, float dL, float dR
) 
{
    // 1. Normalize distances
    float C_F = computeCloseness(dF, cfg.rangeFront, cfg.critFront);
    float C_L = computeCloseness(dL, cfg.rangeSide,  cfg.critSide);
    float C_R = computeCloseness(dR, cfg.rangeSide,  cfg.critSide);

    // 2. Forward Speed
    float forward = cfg.baseSpeed * powf((1 - C_F), cfg.speedExponent);
    forward = clampF(forward, cfg.minSpeed, cfg.maxSpeed);

    float corridorFactor = 1.0f - 0.30f * ((C_L + C_R) * 0.5f);
    if (corridorFactor < 0.4f) corridorFactor = 0.4f;
    forward *= corridorFactor;

    // 3. Repulsive Forces
    float F_F = cfg.kFrontRepulse * C_F;
    float F_L = cfg.kSideRepulse  * C_L;
    float F_R = cfg.kSideRepulse  * C_R;

    float Rx = -F_F;          // front pushes back
    float Ry = F_L - F_R;     // left/right differential

    // 4. Combine with forward drive
    float Mx = forward + Rx;
    float My = Ry;

    float theta = atan2f(My, Mx);
    float Vmag  = sqrtf(Mx*Mx + My*My);
    Vmag = clampF(Vmag, -cfg.maxSpeed, cfg.maxSpeed);

    // 5. Compute angular velocity
    float omega = cfg.kTheta * theta;
    omega *= (1 + cfg.frontCurvGain * C_F * C_F);
    omega += cfg.sideBiasGain * (C_R - C_L);

    omega = clampF(omega, -cfg.maxOmega, cfg.maxOmega);

    // 6. Convert to wheel speeds
    float vL_cmd = Vmag - 0.5f * omega * cfg.wheelBase;
    float vR_cmd = Vmag + 0.5f * omega * cfg.wheelBase;

    vL_cmd = clampF(vL_cmd, -cfg.maxSpeed, cfg.maxSpeed);
    vR_cmd = clampF(vR_cmd, -cfg.maxSpeed, cfg.maxSpeed);

    // 7. Smooth ramping
    state.vL += cfg.rampAlpha * (vL_cmd - state.vL);
    state.vR += cfg.rampAlpha * (vR_cmd - state.vR);
}
