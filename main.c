#include <math.h>
#include <stdio.h>

#include "raylib.h"
#include "raymath.h"

struct car {
    Color color;
    float width;
    float length;
    Vector2 position;  // middle of the car
    float rotation;    // in degrees

    // forces
    float mass;
    float force;
    float acceleration;
    float velocity;

    // pedal params
    float max_force;
};

float radian(float degree) { return (PI / 180.f) * degree; }
float degree(float radian) { return (180.f / PI) * radian; }

Vector2 Vector2Create(float direction, float power)
{
    Vector2 out;
    out.x = cosf(radian(direction)) * power;
    out.y = sinf(radian(direction)) * power;
    return out;
}

float force_to_acc(float force, float mass) { return force / mass; }
float acc_to_velocity(float acceleration, double time_diff)
{
    return acceleration * time_diff;
}

int main(int argc, char const **argv)
{
    InitWindow(800, 450, "cars sim");
    SetTargetFPS(60);
    double time = GetTime();

    struct car car = {.color        = {255, 0, 0, 255},
                      .width        = 30,
                      .length       = 50,
                      .position     = {50, 50},
                      .rotation     = 0,
                      .mass         = 20,
                      .force        = 0,
                      .acceleration = 0,
                      .velocity     = 0,

                      .max_force = 10};

    Vector2 origin = {0, 0};

    Vector2 last_force = {};
    while (!WindowShouldClose()) {
        double now       = GetTime();
        double time_diff = now - time;
        time             = now;

        // draw
        BeginDrawing();
        ClearBackground(RAYWHITE);
        DrawFPS(10, 10);
        {
            float length = sqrtf(powf(car.length, 2) + powf(car.width, 2)) / 2;
            float angle  = degree(tanhf(car.width / car.length));
            Vector2 pos_fix = Vector2Create(car.rotation + angle, length);
            DrawRectanglePro(
                (Rectangle){car.position.x - pos_fix.x,
                            car.position.y - pos_fix.y, car.length, car.width},
                origin, car.rotation, car.color);
            DrawCircle(car.position.x, car.position.y, 4,
                       (Color){0, 0, 0, 255});
        }
        char text[255];
        sprintf(text, "rot: %f, x: %f, y: %f", car.rotation, last_force.x,
                last_force.y);
        DrawText(text, 100, 10, 25, (Color){0, 255, 0, 255});
        EndDrawing();

        // logic
        if (IsKeyDown(KEY_UP)) {
            car.force  = 100;
            last_force = Vector2Create(car.rotation, 1);
        }
        else if (IsKeyDown(KEY_DOWN)) {
            // TODO: make a better system
            if (car.velocity > 0) {  // breaks
                car.force = -250;
            }
            else if (car.velocity > -100) {  // max speed in reverse
                car.force = -50;
            }
            else {
                car.force = 0;
            }
        }
        else {
            car.force = 0;
        }

        if (IsKeyDown(KEY_RIGHT)) {
            car.rotation += 1;
        }
        if (IsKeyDown(KEY_LEFT)) {
            car.rotation -= 1;
        }

        // TODO: add friction system

        car.acceleration = force_to_acc(car.force, car.mass);
        car.velocity += acc_to_velocity(car.acceleration, time_diff);
        car.position =
            Vector2Add(car.position, Vector2Create(car.rotation, car.velocity));
    }

    CloseWindow();
    return 0;
}
