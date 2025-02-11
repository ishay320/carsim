#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "raylib.h"
#include "raymath.h"

struct car {
    Color color;
    float width;
    float length;
    Vector2 position;  // middle of the car
    float rotation;    // in degrees

    // state
    bool breaks;

    // forces
    float mass;
    float force;
    float acceleration;
    float velocity;  // m/s (multiply by 3.6 to get km/h)

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

void calculate_forces(struct car* car, double time_diff)
{
    // FIXME: the speed is heavily dependent on the fps
    car->acceleration = force_to_acc(car->force, car->mass);
    car->velocity += acc_to_velocity(car->acceleration, time_diff);
    car->position =
        Vector2Add(car->position, Vector2Create(car->rotation, car->velocity));
}

float mh_to_kmh(float speed) { return speed * 3.6f; }

void DrawSpeedometer(Rectangle pos, float speed_kmh, float max_speed)
{
    const Vector2 middle = {pos.x + pos.width / 2, pos.y + pos.height / 2};
    DrawCircle(middle.x, middle.y, 5, RED);

    const float needle_offset = 90 + 20;
    const float needle_length = pos.width * 4.f / 5.f;
    float needle_angle = needle_offset + (fabsf(speed_kmh) / max_speed) * 280;
    Vector2 needle_end_point = Vector2Create(needle_angle, needle_length);
    DrawLineEx(middle, Vector2Add(middle, needle_end_point), 3, RED);

    // draw the labels
    const uint8_t labels_number = 10;
    for (uint8_t i = 1; i <= labels_number; i++) {
        char speed_text[32];
        sprintf(speed_text, "%d", (int)(i / (float)labels_number * max_speed));
        Vector2 speed_text_pos = Vector2Add(
            Vector2Create(i / (float)labels_number * 280.f + needle_offset, 60),
            middle);
        int speed_text_size = MeasureText(speed_text, 5);
        DrawText(speed_text, speed_text_pos.x - speed_text_size - 5,
                 speed_text_pos.y, 5, BLACK);
        DrawCircleV(speed_text_pos, 2, RED);
    }
    char kmh[]   = "km/h";
    int kmh_size = MeasureText(kmh, 5);
    DrawText(kmh, pos.x + pos.width - kmh_size, pos.y + pos.height, 5, BLACK);
}

void draw_car(const struct car car, const Vector2 origin)
{
    float length    = sqrtf(powf(car.length, 2) + powf(car.width, 2)) / 2;
    float angle     = degree(tanhf(car.width / car.length));
    Vector2 pos_fix = Vector2Create(car.rotation + angle, length);
    DrawRectanglePro(
        (Rectangle){car.position.x - pos_fix.x, car.position.y - pos_fix.y,
                    car.length, car.width},
        origin, car.rotation, car.color);
    if (car.breaks) {
        Vector2 pos_fix2 = Vector2Create(car.rotation + 90, car.width / 3 * 2);
        DrawRectanglePro((Rectangle){car.position.x - pos_fix.x + pos_fix2.x,
                                     car.position.y - pos_fix.y + pos_fix2.y,
                                     car.length / 15, car.width / 3},
                         origin, car.rotation, RED);
        DrawRectanglePro(
            (Rectangle){car.position.x - pos_fix.x, car.position.y - pos_fix.y,
                        car.length / 15, car.width / 3},
            origin, car.rotation, RED);
    }
    DrawCircle(car.position.x, car.position.y, 4, (Color){0, 0, 0, 255});

    // speedometer
    Rectangle speedometer_pos = {.x      = 50,
                                 .y      = GetRenderHeight() * 7 / 10.f,
                                 .width  = 100,
                                 .height = 100};
    DrawSpeedometer(speedometer_pos, mh_to_kmh(car.velocity), 100);
}

int main(int argc, char const** argv)
{
    InitWindow(800, 450, "cars sim");
    SetTargetFPS(60);
    double time = GetTime();

    struct car car = {.color        = GREEN,
                      .width        = 30,
                      .length       = 50,
                      .position     = {50, 50},
                      .rotation     = 0,
                      .breaks       = false,
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
            draw_car(car, origin);
        }
        char text[255];
        sprintf(text, "rot: %f, x: %f, y: %f", car.rotation, last_force.x,
                last_force.y);
        DrawText(text, 100, 10, 25, (Color){0, 255, 0, 255});
        EndDrawing();

        // logic
        car.breaks = false;
        if (IsKeyDown(KEY_UP)) {
            car.force  = 100;
            last_force = Vector2Create(car.rotation, 1);
            if (car.velocity < 0) {
                car.breaks = true;
            }
        }
        else if (IsKeyDown(KEY_DOWN)) {
            // TODO: make a better system
            if (car.velocity > 0) {  // breaks
                car.force  = -250;
                car.breaks = true;
            }
            else if (car.velocity > -100) {  // max speed in reverse
                car.force = -50;
            }
            else {
                car.force = 0;
            }
        }
        else {
            car.force  = 0;
            car.breaks = false;
        }

        if (IsKeyDown(KEY_RIGHT)) {
            car.rotation += 1;
        }
        if (IsKeyDown(KEY_LEFT)) {
            car.rotation -= 1;
        }

        // TODO: add friction system

        calculate_forces(&car, time_diff);
    }

    CloseWindow();
    return 0;
}
