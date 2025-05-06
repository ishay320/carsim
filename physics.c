#include "physics.h"

#include <math.h>

float radian(float degree) { return DEG2RAD * degree; }
float degree(float radian) { return RAD2DEG * radian; }

Vector2 Vector2FromPolar(float angle_deg, float magnitude)
{
    Vector2 out;
    out.x = cosf(radian(angle_deg)) * magnitude;
    out.y = sinf(radian(angle_deg)) * magnitude;
    return out;
}

float ms_to_kmh(float speed) { return speed * 3.6f; }

float force_to_acc(float force, float mass) { return force / mass; }

float acc_to_velocity(float acceleration, double delta_time)
{
    return acceleration * delta_time;
}

float max_steering_angle(float wheelbase, float wheelspace,
                         float turning_circle_m)
{
    return atanf(wheelbase / (turning_circle_m - wheelspace));
}
