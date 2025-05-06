#ifndef SMATH_H
#define SMATH_H
#include "raylib.h"

float radian(float degree);
float degree(float radian);

Vector2 Vector2FromPolar(float angle_deg, float magnitude);

float ms_to_kmh(float speed);

float force_to_acc(float force, float mass);

float acc_to_velocity(float acceleration, double delta_time);

float max_steering_angle(float wheelbase, float wheelspace,
                         float turning_circle_m);

#endif  // y SMATH_H
