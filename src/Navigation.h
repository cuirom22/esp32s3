#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <Arduino.h>

// === Steering State ===
struct SteeringState {
    float vL = 0;
    float vR = 0;
};

// === Config Parameters ===
struct SteeringConfig {
    float rangeFront = 60.0f;
    float critFront  = 15.0f;

    float rangeSide  = 45.0f;
    float critSide   = 12.0f;

    float baseSpeed  = 140.0f;
    float minSpeed   = 50.0f;
    float maxSpeed   = 200.0f;

    float speedExponent = 1.4f;

    float kFrontRepulse = 140.0f;
    float kSideRepulse  = 80.0f;

    float kTheta       = 1.2f;
    float frontCurvGain= 1.3f;
    float sideBiasGain = 0.6f;
    float maxOmega     = 3.5f;
    float wheelBase    = 0.16f;

    float rampAlpha    = 0.25f;
};

// === FUNCTION PROTOTYPES ===
float computeCloseness(float d, float range, float crit);

void computeSteering(
    SteeringConfig &cfg,
    SteeringState &state,
    float dF, float dL, float dR
);

#endif