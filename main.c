#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "raylib.h"
#include "raymath.h"

typedef enum {
    COLOR_ROAD,
    COLOR_GRASS,
    COLOR_WHEELS,
    COLOR_CAR,
    COLOR_YELOW_LINE,
    COLOR_COUNT
} ColorType;

const Color COLORS[COLOR_COUNT] = {
    [COLOR_ROAD]       = {0xA2, 0xA2, 0xA2, 0xff},
    [COLOR_GRASS]      = {0x84, 0x93, 0x24, 0xff},
    [COLOR_WHEELS]     = {0x2F, 0x2F, 0x2F, 0xff},
    [COLOR_CAR]        = {0x40, 0x62, 0xBB, 0xff},
    [COLOR_YELOW_LINE] = YELLOW,
};

struct car {
    Color color;
    float width_m;
    float length_m;
    Vector2 position;    // middle of the car
    float rotation_deg;  // in degrees

    struct {
        float steering_angle_deg;
        float max_steering_angle_deg;
        float wheelspace_m;  // left and right
        float wheelbase_m;   // front back

        float width_m;
        float length_m;
    } wheels;

    // state
    bool breaks;
    float motor_force;

    // forces
    float c_mass;
    float c_drag;
    float c_rolling;

    // physics calculation
    float current_force;
    float velocity;  // m/s (multiply by 3.6 to get km/h)

    // pedal params
    float max_force;
};

float radian(float degree) { return DEG2RAD * degree; }
float degree(float radian) { return RAD2DEG * radian; }

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

float resistance_forces(const struct car* car, double time_diff)
{
    Vector2 v = Vector2Create(car->rotation_deg, car->velocity * time_diff);

    Vector2 drag;
    drag.x = -car->c_drag * v.x * car->velocity;
    drag.y = -car->c_drag * v.y * car->velocity;

    Vector2 rolling;
    rolling.x = -car->c_rolling * v.x;
    rolling.y = -car->c_rolling * v.y;

    // TODO: add traction - engine force
    Vector2 resistance = Vector2Add(drag, rolling);
    float force =
        sqrtf(resistance.x * resistance.x + resistance.y * resistance.y);
    if (car->velocity < 0) {
        return -force;
    }
    else {
        return force;
    }
}

void calculate_forces(struct car* car, double time_diff)
{
    float rf           = resistance_forces(car, time_diff);
    car->current_force = car->motor_force - rf;
    float acceleration = force_to_acc(car->current_force, car->c_mass);
    car->velocity += acc_to_velocity(acceleration, time_diff);

    if (fabs(car->wheels.steering_angle_deg) < FLT_EPSILON) {  // Going straight
        car->position = Vector2Add(
            car->position,
            Vector2Create(car->rotation_deg, car->velocity * time_diff));
    }
    else {
        float steering_angle_rad = radian(car->wheels.steering_angle_deg);
        float wheelbase          = car->wheels.wheelbase_m;
        Vector2 bm               = Vector2Subtract(
            car->position, Vector2Create(car->rotation_deg, wheelbase / 2));

        float radius           = wheelbase / tan(steering_angle_rad);
        float angular_velocity = car->velocity / radius;
        float delta_theta = angular_velocity * time_diff;  // Change in angle

        Vector2 next_pos = {sin(delta_theta) * radius,
                            radius - cos(delta_theta) * radius};

        next_pos = Vector2Rotate(next_pos, radian(car->rotation_deg));
        bm       = Vector2Add(bm, next_pos);

        car->rotation_deg += degree(delta_theta);
        car->position =
            Vector2Add(bm, Vector2Create(car->rotation_deg, wheelbase / 2));
    }
}

float ms_to_kmh(float speed) { return speed * 3.6f; }

void DrawSpeedometer(Rectangle pos, float speed_kmh, float max_speed_kmh)
{
    const float short_line = pos.height < pos.width ? pos.height : pos.width;
    const Vector2 middle   = {pos.x + short_line / 2, pos.y + short_line / 2};
    const int font_size    = 5;
#ifdef SHOW_DRAW_OVERLAY
    DrawRectangleLines(pos.x, pos.y, short_line, short_line, BLACK);
#endif

    DrawCircleV(middle, short_line / 2, (Color){210, 210, 210, 255});

    const float needle_start_angle = 110;  // the start rotation of the needle
    const float empty_space        = 80;

    // draw the labels
    const uint8_t labels_number = 10;
    for (uint8_t i = 1; i <= labels_number; i++) {
        char speed_text[32];
        sprintf(speed_text, "%d",
                (int)(i / (float)labels_number * max_speed_kmh));
        int speed_text_size = MeasureText(speed_text, font_size);

        const Vector2 direction =
            Vector2Create((i / (float)labels_number) * (360 - empty_space) +
                              needle_start_angle,
                          1);
        Vector2 speed_point_pos =
            Vector2Add(Vector2Scale(direction, short_line / 2), middle);
        Vector2 speed_text_pos = Vector2Add(
            Vector2Scale(direction, short_line / (11 / 4.f)), middle);
        Vector2 speed_point_line =
            Vector2Add(Vector2Scale(direction, -font_size), speed_point_pos);

        // draw the lines and the speed
        DrawLineEx(speed_point_pos, speed_point_line, 2, RED);
        DrawText(speed_text, speed_text_pos.x - (speed_text_size / 2.f),
                 speed_text_pos.y - font_size, font_size, BLACK);
    }
    const char* kmh = "km/h";
    int kmh_size    = MeasureText(kmh, font_size);
    DrawText(kmh, pos.x + short_line - kmh_size,
             pos.y + short_line - (font_size * 2), font_size, BLACK);

    const float needle_length = short_line / 2;
    float needle_angle =
        needle_start_angle +
        (fabsf(speed_kmh) / max_speed_kmh) * (360 - empty_space);
    Vector2 needle_end_point = Vector2Create(needle_angle, needle_length);
    DrawLineEx(middle, Vector2Add(middle, needle_end_point), 3, RED);

    // point on the knob
    DrawCircle(middle.x, middle.y, 5, GRAY);
}

void DrawRectangleProMiddle(Rectangle rec, Vector2 origin, float rotation,
                            Color color)
{
    float length    = sqrtf(powf(rec.width, 2) + powf(rec.height, 2)) / 2;
    float angle     = degree(tanhf(rec.width / rec.height));
    Vector2 pos_fix = Vector2Create(rotation + angle, length);
    DrawRectanglePro((Rectangle){rec.x - pos_fix.x, rec.y - pos_fix.y,
                                 rec.height, rec.width},
                     origin, rotation, color);
}

void draw_wheels(const struct car car, const Vector2 origin)
{
    const Color wheels_color = COLORS[COLOR_WHEELS];
    Vector2 fl_wheel         = {+(car.wheels.wheelbase_m / 2.f),
                                -(car.wheels.wheelspace_m / 2.f)};
    fl_wheel = Vector2Rotate(fl_wheel, radian(car.rotation_deg));
    fl_wheel = Vector2Add(fl_wheel, car.position);
    DrawRectangleProMiddle(
        (Rectangle){fl_wheel.x, fl_wheel.y, car.wheels.width_m,
                    car.wheels.length_m},
        origin, car.rotation_deg + car.wheels.steering_angle_deg, wheels_color);

    Vector2 fr_wheel = {+(car.wheels.wheelbase_m / 2.f),
                        +(car.wheels.wheelspace_m / 2.f)};
    fr_wheel         = Vector2Rotate(fr_wheel, radian(car.rotation_deg));
    fr_wheel         = Vector2Add(fr_wheel, car.position);
    DrawRectangleProMiddle(
        (Rectangle){fr_wheel.x, fr_wheel.y, car.wheels.width_m,
                    car.wheels.length_m},
        origin, car.rotation_deg + car.wheels.steering_angle_deg, wheels_color);

    Vector2 bl_wheel = {-(car.wheels.wheelbase_m / 2.f),
                        -(car.wheels.wheelspace_m / 2.f)};
    bl_wheel         = Vector2Rotate(bl_wheel, radian(car.rotation_deg));
    bl_wheel         = Vector2Add(bl_wheel, car.position);
    DrawRectangleProMiddle((Rectangle){bl_wheel.x, bl_wheel.y,
                                       car.wheels.width_m, car.wheels.length_m},
                           origin, car.rotation_deg, wheels_color);

    Vector2 br_wheel = {-(car.wheels.wheelbase_m / 2.f),
                        +(car.wheels.wheelspace_m / 2.f)};
    br_wheel         = Vector2Rotate(br_wheel, radian(car.rotation_deg));
    br_wheel         = Vector2Add(br_wheel, car.position);
    DrawRectangleProMiddle((Rectangle){br_wheel.x, br_wheel.y,
                                       car.wheels.width_m, car.wheels.length_m},
                           origin, car.rotation_deg, wheels_color);
#if DEBUG_ACKERMANN
    Vector2 bm_wheel = {-(car.wheels.wheelbase_m / 2.f), 0};
    bm_wheel         = Vector2Rotate(bm_wheel, radian(car.rotation_deg));
    bm_wheel         = Vector2Add(bm_wheel, car.position);
    DrawRectangleProMiddle((Rectangle){bm_wheel.x, bm_wheel.y,
                                       car.wheels.width_m, car.wheels.length_m},
                           origin, car.rotation_deg, (Color){0, 0, 0, 255});
    Vector2 fm_wheel = {+(car.wheels.wheelbase_m / 2.f), 0};
    fm_wheel         = Vector2Rotate(fm_wheel, radian(car.rotation_deg));
    fm_wheel         = Vector2Add(fm_wheel, car.position);
    DrawRectangleProMiddle((Rectangle){fm_wheel.x, fm_wheel.y,
                                       car.wheels.width_m, car.wheels.length_m},
                           origin,
                           car.rotation_deg + car.wheels.steering_angle_deg,
                           (Color){0, 0, 0, 255});

    float radius = tanf(radian(90 - car.wheels.steering_angle_deg)) *
                   car.wheels.wheelbase_m;

    DrawLineEx(
        bm_wheel,
        Vector2Add(Vector2Create(car.rotation_deg + 90, radius), bm_wheel), 0.3,
        (Color){255, 0, 0, 255});
    DrawLineEx(fm_wheel,
               Vector2Add(Vector2Create(car.rotation_deg +
                                            car.wheels.steering_angle_deg + 90,
                                        radius),
                          fm_wheel),
               0.3, (Color){255, 0, 0, 255});

    Vector2 p = Vector2Create(car.rotation_deg + 90, radius);
    // from the middle of the car
    p = Vector2Add(car.position, p);
    // from the middle of the car
    // p = Vector2Add(bm_wheel, p);
    DrawPixelV(p, (Color){0, 0, 0, 255});
    DrawCircleLinesV(p, radius, (Color){0, 0, 0, 255});
#endif
}

void draw_car(const struct car car, const Vector2 origin)
{
    draw_wheels(car, origin);

    DrawRectangleProMiddle(
        (Rectangle){car.position.x, car.position.y, car.width_m, car.length_m},
        origin, car.rotation_deg, car.color);

    if (car.breaks) {
        float length = sqrtf(powf(car.length_m, 2) + powf(car.width_m, 2)) / 2;
        float angle  = degree(tanhf(car.width_m / car.length_m));
        Vector2 back_left =
            Vector2Subtract((Vector2){car.position.x, car.position.y},
                            Vector2Create(car.rotation_deg + angle, length));
        DrawRectanglePro((Rectangle){back_left.x, back_left.y,
                                     car.length_m / 15, car.width_m / 3},
                         origin, car.rotation_deg, RED);

        Vector2 back_right_light = Vector2Add(
            Vector2Create(car.rotation_deg + 90, car.width_m / 3 * 2),
            back_left);
        DrawRectanglePro((Rectangle){back_right_light.x, back_right_light.y,
                                     car.length_m / 15, car.width_m / 3},
                         origin, car.rotation_deg, RED);
    }

    if (car.velocity < -0.001) {
        float length = sqrtf(powf(car.length_m, 2) + powf(car.width_m, 2)) / 2;
        float angle  = degree(tanhf(car.width_m / car.length_m));
        Vector2 back_left =
            Vector2Subtract((Vector2){car.position.x, car.position.y},
                            Vector2Create(car.rotation_deg + angle, length));
        Vector2 back_left_light = Vector2Add(
            Vector2Create(car.rotation_deg + 90, car.width_m / 5 * 1),
            back_left);
        DrawRectanglePro((Rectangle){back_left_light.x, back_left_light.y,
                                     car.length_m / 15, car.width_m / 5},
                         origin, car.rotation_deg, WHITE);

        Vector2 back_right_light = Vector2Add(
            Vector2Create(car.rotation_deg + 90, car.width_m / 5 * 3),
            back_left);
        DrawRectanglePro((Rectangle){back_right_light.x, back_right_light.y,
                                     car.length_m / 15, car.width_m / 5},
                         origin, car.rotation_deg, WHITE);
    }
}
float max_steering_angle(float wheelbase, float wheelspace,
                         float turning_circle_m)
{
    return atanf(wheelbase / (turning_circle_m - wheelspace));
}

#define MAX_WIDTH 10
#define MAX_HEIGHT 10
// TODO: move to file outide the exe
const bool level_road[MAX_HEIGHT][MAX_WIDTH] = {
    {1, 1, 1, 1, 1, 1, 1, 1}, {0, 0, 1, 0, 0, 0, 0, 1},
    {0, 1, 1, 1, 0, 0, 0, 1}, {0, 1, 1, 1, 0, 0, 0, 1},
    {0, 0, 0, 0, 0, 0, 0, 1},
};

void drawBackground()
{
    ClearBackground(COLORS[COLOR_GRASS]);
    float jump_size = 10;
    float width     = GetRenderWidth();
    float height    = GetRenderHeight();

    // TODO: render less because of the zoom
    for (size_t x = 0; x < MAX_WIDTH; x++) {
        for (size_t y = 0; y < MAX_HEIGHT; y++) {
            size_t x_pos = x * jump_size;
            size_t y_pos = y * jump_size;
            if (level_road[y][x]) {
                DrawRectanglePro(
                    (Rectangle){x_pos, y_pos, jump_size, jump_size},
                    (Vector2){0, 0}, 0, COLORS[COLOR_ROAD]);
            }
        }
    }

    return;
}

int main(int argc, char const** argv)
{
    const int screen_width  = 800;
    const int screen_height = 450;
    InitWindow(screen_width, screen_height, "cars sim");
    SetTargetFPS(60);
    double time = GetTime();

    const float wheelbase_m      = 5;
    const float wheelspace_m     = 3;
    const float turning_circle_m = 10;
    struct car car               = {
                      .color        = COLORS[COLOR_CAR],
                      .width_m      = 3,
                      .length_m     = 5,
                      .position     = {5, 5},
                      .rotation_deg = 0,
                      .wheels       = {.steering_angle_deg     = 0,
                                       .max_steering_angle_deg = degree(max_steering_angle(
                       wheelbase_m, wheelspace_m, turning_circle_m)),
                                       .wheelspace_m           = wheelspace_m,
                                       .wheelbase_m            = wheelbase_m,
                                       .width_m                = 0.6,
                                       .length_m               = 1},
                      .breaks       = false,
                      .motor_force  = 0,
                      .c_mass       = 20,
                      .c_drag       = 0.25,
                      .c_rolling    = 35.,

                      .current_force = 0,
                      .velocity      = 0,

                      .max_force = 10};

    Vector2 origin = {0, 0};

    Camera2D camera = {0};
    camera.zoom     = 10.0f;

    while (!WindowShouldClose()) {
        double now       = GetTime();
        double time_diff = now - time;
        time             = now;

        // draw
        BeginMode2D(camera);
        {
            drawBackground();
            draw_car(car, origin);
        }
        EndMode2D();
        {
            DrawFPS(10, 10);
#ifdef DEBUG
            char text[255];
            sprintf(text, "x: %f, y: %f, s: %f", car.position.x, car.position.y,
                    car.velocity);
            DrawText(text, 100, 10, 25, (Color){0, 255, 0, 255});
#endif  // DEBUG

            // speedometer
            Rectangle speedometer_pos = {.x      = 0,
                                         .y      = GetRenderHeight() - 125,
                                         .width  = 125,
                                         .height = 125};
            DrawSpeedometer(speedometer_pos, ms_to_kmh(car.velocity), 100);
        }
        EndDrawing();

        // logic
        car.breaks = false;
        if (IsKeyDown(KEY_UP)) {
            if (car.velocity < 0) {
                car.breaks      = true;
                car.motor_force = 250;
            }
            else {
                car.motor_force = 100;
            }
        }
        else if (IsKeyDown(KEY_DOWN)) {
            // TODO: make a better system
            if (car.velocity > 0) {  // breaks
                car.motor_force = -250;
                car.breaks      = true;
            }
            else if (car.velocity > -100) {  // max speed in reverse
                car.motor_force = -50;
            }
            else {
                car.motor_force = 0;
            }
        }
        else {
            car.motor_force = 0;
            car.breaks      = false;
        }

        if (IsKeyDown(KEY_RIGHT) &&
            car.wheels.steering_angle_deg < car.wheels.max_steering_angle_deg) {
            car.wheels.steering_angle_deg += 1;
        }
        if (IsKeyDown(KEY_LEFT) && car.wheels.steering_angle_deg >
                                       -car.wheels.max_steering_angle_deg) {
            car.wheels.steering_angle_deg -= 1;
        }

        camera.zoom += ((float)GetMouseWheelMove() * 0.05f);

        if (camera.zoom > 10.0f) {
            camera.zoom = 10.0f;
        }
        else if (camera.zoom < 0.1f) {
            camera.zoom = 0.1f;
        }

        // Camera reset (zoom and rotation)
        if (IsKeyPressed(KEY_R)) {
            camera.zoom = 10.0f;
        }

        calculate_forces(&car, time_diff);
    }

    CloseWindow();
    return 0;
}
