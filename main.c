#include <float.h>
#include <libgen.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>

#include "config.h"
#ifdef GRAPH
#include "graph.h"
#endif
#include "physics.h"
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

enum signal {
    SIG_NONE,
    SIG_RIGHT,
    SIG_LEFT,
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
    float points;
    enum signal signal;
    double signal_update;

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

float resistance_forces(const struct car* car, double delta_time)
{
    Vector2 v = Vector2FromPolar(car->rotation_deg, car->velocity * delta_time);

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

/**
 * @brief Calculate the forces and returns copy of the car with the updated
 * parameters
 *
 * @param car
 * @param delta_time
 * @return The updated car with the new position
 */
struct car calculate_forces(const struct car* car, double delta_time)
{
    struct car car_new    = *car;
    float rf              = resistance_forces(&car_new, delta_time);
    car_new.current_force = car_new.motor_force - rf;
    float acceleration    = force_to_acc(car_new.current_force, car_new.c_mass);
    car_new.velocity += acc_to_velocity(acceleration, delta_time);

    if (fabs(car_new.wheels.steering_angle_deg) <
        FLT_EPSILON) {  // Going straight
        car_new.position = Vector2Add(
            car_new.position, Vector2FromPolar(car_new.rotation_deg,
                                               car_new.velocity * delta_time));
    }
    else {
        float steering_angle_rad = radian(car_new.wheels.steering_angle_deg);
        float wheelbase          = car_new.wheels.wheelbase_m;
        Vector2 bm               = Vector2Subtract(
            car_new.position,
            Vector2FromPolar(car_new.rotation_deg, wheelbase / 2));

        float radius           = wheelbase / tan(steering_angle_rad);
        float angular_velocity = car_new.velocity / radius;
        float delta_theta = angular_velocity * delta_time;  // Change in angle

        Vector2 next_pos = {sin(delta_theta) * radius,
                            radius - cos(delta_theta) * radius};

        next_pos = Vector2Rotate(next_pos, radian(car_new.rotation_deg));
        bm       = Vector2Add(bm, next_pos);

        car_new.rotation_deg += degree(delta_theta);
        car_new.position = Vector2Add(
            bm, Vector2FromPolar(car_new.rotation_deg, wheelbase / 2));
    }
    return car_new;
}

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
            Vector2FromPolar((i / (float)labels_number) * (360 - empty_space) +
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
    Vector2 needle_end_point = Vector2FromPolar(needle_angle, needle_length);
    DrawLineEx(middle, Vector2Add(middle, needle_end_point), 3, RED);

    // point on the knob
    DrawCircle(middle.x, middle.y, 5, GRAY);
}

void DrawRectangleProMiddle(Rectangle rec, Vector2 origin, float rotation,
                            Color color)
{
    float length    = sqrtf(powf(rec.width, 2) + powf(rec.height, 2)) / 2;
    float angle     = degree(tanhf(rec.width / rec.height));
    Vector2 pos_fix = Vector2FromPolar(rotation + angle, length);
    DrawRectanglePro((Rectangle){rec.x - pos_fix.x, rec.y - pos_fix.y,
                                 rec.height, rec.width},
                     origin, rotation, color);
}

struct wheels {
    Vector2 br;
    Vector2 bl;
    Vector2 fr;
    Vector2 fl;
};

struct wheels get_wheels(const struct car car)
{
    Vector2 fl_wheel = {+(car.wheels.wheelbase_m / 2.f),
                        -(car.wheels.wheelspace_m / 2.f)};
    fl_wheel         = Vector2Rotate(fl_wheel, radian(car.rotation_deg));
    fl_wheel         = Vector2Add(fl_wheel, car.position);

    Vector2 fr_wheel = {+(car.wheels.wheelbase_m / 2.f),
                        +(car.wheels.wheelspace_m / 2.f)};
    fr_wheel         = Vector2Rotate(fr_wheel, radian(car.rotation_deg));
    fr_wheel         = Vector2Add(fr_wheel, car.position);

    Vector2 bl_wheel = {-(car.wheels.wheelbase_m / 2.f),
                        -(car.wheels.wheelspace_m / 2.f)};
    bl_wheel         = Vector2Rotate(bl_wheel, radian(car.rotation_deg));
    bl_wheel         = Vector2Add(bl_wheel, car.position);

    Vector2 br_wheel = {-(car.wheels.wheelbase_m / 2.f),
                        +(car.wheels.wheelspace_m / 2.f)};
    br_wheel         = Vector2Rotate(br_wheel, radian(car.rotation_deg));
    br_wheel         = Vector2Add(br_wheel, car.position);

    return (struct wheels){br_wheel, bl_wheel, fr_wheel, fl_wheel};
}

void draw_wheels(const struct car car, const Vector2 origin)
{
    const Color wheels_color = COLORS[COLOR_WHEELS];
    struct wheels wheels     = get_wheels(car);
    DrawRectangleProMiddle(
        (Rectangle){wheels.fl.x, wheels.fl.y, car.wheels.width_m,
                    car.wheels.length_m},
        origin, car.rotation_deg + car.wheels.steering_angle_deg, wheels_color);

    DrawRectangleProMiddle(
        (Rectangle){wheels.fr.x, wheels.fr.y, car.wheels.width_m,
                    car.wheels.length_m},
        origin, car.rotation_deg + car.wheels.steering_angle_deg, wheels_color);

    DrawRectangleProMiddle((Rectangle){wheels.bl.x, wheels.bl.y,
                                       car.wheels.width_m, car.wheels.length_m},
                           origin, car.rotation_deg, wheels_color);

    DrawRectangleProMiddle((Rectangle){wheels.br.x, wheels.br.y,
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

void draw_signals(struct car* car, const Vector2 origin, double delta_time)
{
    if (car->signal == SIG_NONE) {
        car->signal_update = 0;
        return;
    }

    float length = sqrtf(powf(car->length_m, 2) + powf(car->width_m, 2)) / 2;
    float angle  = degree(tanhf(car->width_m / car->length_m));

    Vector2 back_left =
        Vector2Subtract((Vector2){car->position.x, car->position.y},
                        Vector2FromPolar(car->rotation_deg + angle, length));

    car->signal_update += delta_time;
    float update_speed = 0.5;
    if (car->signal_update < update_speed) {
        return;
    }
    else if (car->signal_update > update_speed * 2) {
        car->signal_update = 0;
    }

    if (car->signal == SIG_LEFT) {
        DrawRectanglePro((Rectangle){back_left.x, back_left.y,
                                     car->length_m / 15, car->width_m / 6},
                         origin, car->rotation_deg, ORANGE);
        return;
    }
    else if (car->signal == SIG_RIGHT) {
        Vector2 back_right_light =
            Vector2Add(Vector2FromPolar(car->rotation_deg + 90,
                                        car->width_m - (car->width_m / 6)),
                       back_left);
        DrawRectanglePro((Rectangle){back_right_light.x, back_right_light.y,
                                     car->length_m / 15, car->width_m / 6},
                         origin, car->rotation_deg, ORANGE);
        return;
    }
    printf("unreachable code reached\n");
}

void draw_car(struct car* car, const Vector2 origin, double delta_time)
{
    draw_wheels(*car, origin);

    DrawRectangleProMiddle((Rectangle){car->position.x, car->position.y,
                                       car->width_m, car->length_m},
                           origin, car->rotation_deg, car->color);

    if (car->breaks) {
        float length =
            sqrtf(powf(car->length_m, 2) + powf(car->width_m, 2)) / 2;
        float angle       = degree(tanhf(car->width_m / car->length_m));
        Vector2 back_left = Vector2Subtract(
            (Vector2){car->position.x, car->position.y},
            Vector2FromPolar(car->rotation_deg + angle, length));
        DrawRectanglePro((Rectangle){back_left.x, back_left.y,
                                     car->length_m / 15, car->width_m / 3},
                         origin, car->rotation_deg, RED);

        Vector2 back_right_light = Vector2Add(
            Vector2FromPolar(car->rotation_deg + 90, car->width_m / 3 * 2),
            back_left);
        DrawRectanglePro((Rectangle){back_right_light.x, back_right_light.y,
                                     car->length_m / 15, car->width_m / 3},
                         origin, car->rotation_deg, RED);
    }

    if (car->velocity < -0.001) {
        float length =
            sqrtf(powf(car->length_m, 2) + powf(car->width_m, 2)) / 2;
        float angle       = degree(tanhf(car->width_m / car->length_m));
        Vector2 back_left = Vector2Subtract(
            (Vector2){car->position.x, car->position.y},
            Vector2FromPolar(car->rotation_deg + angle, length));
        Vector2 back_left_light = Vector2Add(
            Vector2FromPolar(car->rotation_deg + 90, car->width_m / 5 * 1),
            back_left);
        DrawRectanglePro((Rectangle){back_left_light.x, back_left_light.y,
                                     car->length_m / 15, car->width_m / 5},
                         origin, car->rotation_deg, WHITE);

        Vector2 back_right_light = Vector2Add(
            Vector2FromPolar(car->rotation_deg + 90, car->width_m / 5 * 3),
            back_left);
        DrawRectanglePro((Rectangle){back_right_light.x, back_right_light.y,
                                     car->length_m / 15, car->width_m / 5},
                         origin, car->rotation_deg, WHITE);
    }

    draw_signals(car, origin, delta_time);
}

enum TYPE {
    GRASS = 0,
    ROAD,
    PARKING,
    BUILDING,
    PAVEMENT,
    OUTSIDE_MAP,
    ROAD_TYPE_COUNT,
};

struct stage {
    float scale;
    Texture2D texture;
    Image map_seg;
};

void draw_background(const struct stage stage)
{
    ClearBackground(COLORS[COLOR_GRASS]);

    float width  = GetRenderWidth();
    float height = GetRenderHeight();

    DrawTextureEx(stage.texture, (Vector2){0, 0}, 0, stage.scale, WHITE);

    return;
}

char* join_basedir_with(const char* config_path, const char* relative_path)
{
    char* config_path_copy = strdup(config_path);
    char* dir              = dirname(config_path_copy);

    size_t len      = strlen(dir) + 1 + strlen(relative_path) + 1;
    char* full_path = malloc(len);
    snprintf(full_path, len, "%s/%s", dir, relative_path);

    free(config_path_copy);
    return full_path;
}

bool color_cmp(Color a, Color b) { return *(int32_t*)(&a) == *(int32_t*)(&b); }

Color get_color(const Image image, size_t x, size_t y)
{
    size_t position = x + y * image.width;
    return GetPixelColor(((Color*)image.data) + position, image.format);
}

uint32_t compact_color(Color color)
{
    return color.r << 24 | color.g << 16 | color.b << 8 | color.a;
}

bool check_in_image(float x, float y, Image image, float scale)
{
    return x > 0 && y > 0 && x / scale < image.width &&
           y / scale < image.height;
}

enum TYPE position_to_type(const struct stage stage, float x, float y)
{
    if (!check_in_image(x, y, stage.map_seg, stage.scale)) {
        return OUTSIDE_MAP;
    }

    const uint32_t grass    = compact_color((Color){0x00, 0xff, 0x00, 0xff});
    const uint32_t pavement = compact_color((Color){0x5b, 0x5b, 0x5b, 0xff});
    const uint32_t road     = compact_color((Color){0x00, 0x00, 0x00, 0xff});
    const uint32_t parking  = compact_color((Color){0x00, 0x00, 0xff, 0xff});
    const uint32_t building = compact_color((Color){0xfc, 0xff, 0x00, 0xff});

    Color color = get_color(stage.map_seg, x / stage.scale, y / stage.scale);
    uint32_t color_simple = compact_color(color);
    if (color_simple == grass) {
        return GRASS;
    }
    else if (color_simple == pavement) {
        return PAVEMENT;
    }
    else if (color_simple == road) {
        return ROAD;
    }
    else if (color_simple == parking) {
        return PARKING;
    }
    else if (color_simple == building) {
        return BUILDING;
    }
    return ROAD_TYPE_COUNT;
}

float count_points(struct stage stage, float x, float y)
{
    float sum = 0;

    enum TYPE type = position_to_type(stage, x, y);
    switch (type) {
        case GRASS:
            sum -= 1;
            break;
        case PAVEMENT:
            sum -= 1;
            break;
        case BUILDING:
            sum -= 10;
            break;
        case OUTSIDE_MAP:
            sum -= 10;
            break;
        case ROAD:
        case PARKING:
        case ROAD_TYPE_COUNT:
            break;
    }

    return sum;
}

struct game_configs {
    char* map_relative_path;
    float pixels_per_meter;
};

bool process_config(struct game_configs* game_configs, const char* config_path)
{
    // TODO: free the configurations
    struct configs configs = read_config(config_path);

    const char* map_path_key  = "map_path";
    struct config* c_map_path = find_config(configs, map_path_key);
    if (!c_map_path) {
        printf("ERROR: %s is not found\n", map_path_key);
        goto error;
    }
    if (c_map_path->type != VALUE_STR) {
        printf("ERROR: %s is not a string\n", map_path_key);
        goto error;
    }
    game_configs->map_relative_path = c_map_path->value.s;

    const char* pixels_per_meter_key = "pixels_per_meter";
    struct config* c_pixels_per_meter =
        find_config(configs, pixels_per_meter_key);
    if (!c_pixels_per_meter) {
        printf("ERROR: %s is not found\n", pixels_per_meter_key);
        goto error;
    }
    if (c_pixels_per_meter->type != VALUE_FLOAT) {
        printf("ERROR: %s is not a float\n", pixels_per_meter_key);
        goto error;
    }
    game_configs->pixels_per_meter = c_pixels_per_meter->value.f;
    if (game_configs->pixels_per_meter == 0) {
        printf("ERROR: %s param cannot be 0\n", pixels_per_meter_key);
        goto error;
    }
    return true;

error:
    free_config(&configs);
    return false;
}

struct wheel_line {
    struct wheels* items;
    size_t length;
    size_t curser;
    float history_sec;
    float time_pass;
    // TODO:
    // - add color to each history
};

bool wheel_line_create(struct wheel_line* wheel_line, float history_sec,
                       size_t resolution, const struct car car)
{
    wheel_line->items = calloc(resolution, sizeof(*wheel_line->items));
    if (!wheel_line->items) {
        return false;
    }
    struct wheels wheels = get_wheels(car);
    for (size_t i = 0; i < resolution; i++) {
        wheel_line->items[i] = wheels;
    }
    wheel_line->length      = resolution;
    wheel_line->history_sec = history_sec;
    wheel_line->curser      = 0;
    wheel_line->time_pass   = 0;
    return true;
}

void wheel_line_push(struct wheel_line* wheel_line, const struct car car,
                     float delta_time)
{
    wheel_line->time_pass += delta_time;
    if (wheel_line->time_pass < wheel_line->history_sec) {
        return;
    }

    struct wheels wheels                  = get_wheels(car);
    wheel_line->items[wheel_line->curser] = wheels;
    wheel_line->curser = (wheel_line->curser + 1) % wheel_line->length;
}

void wheel_line_draw(const struct wheel_line wheel_line, const struct car car)
{
    size_t power       = 0;
    struct wheels last = wheel_line.items[wheel_line.curser];
    for (size_t i = (wheel_line.curser + 1) % wheel_line.length;
         i != ((int)wheel_line.curser) % wheel_line.length;
         i = (i + 1) % wheel_line.length) {
        struct wheels current = wheel_line.items[i];
        Color color = {0, 0, 0, ((float)power / wheel_line.length) * 255};
        DrawLineEx(last.bl, current.bl, car.wheels.width_m, color);
        DrawLineEx(last.br, current.br, car.wheels.width_m, color);
        DrawLineEx(last.fl, current.fl, car.wheels.width_m, color);
        DrawLineEx(last.fr, current.fr, car.wheels.width_m, color);
        last = current;
        power++;
    }
}

int main(int argc, char const** argv)
{
    if (argc < 2) {
        printf("usage: %s <map config>\n", argv[0]);
        return 1;
    }
    const char* config_path = argv[1];
    struct game_configs game_configs;
    if (!process_config(&game_configs, config_path)) {
        return 1;
    }

    char* full_map_path =
        join_basedir_with(config_path, game_configs.map_relative_path);
    Image map_seg = LoadImage(full_map_path);
    free(full_map_path);

    const int screen_width  = 800;
    const int screen_height = 450;
    InitWindow(screen_width, screen_height, "cars sim");
    SetTargetFPS(60);

    struct stage stage;
    stage.map_seg = map_seg;
    stage.texture = LoadTextureFromImage(map_seg);
    stage.scale   = 1 / game_configs.pixels_per_meter;

    double time = GetTime();

    const float wheelbase_m      = 3.5;
    const float wheelspace_m     = 1.8;
    const float turning_circle_m = 7;
    struct car car               = {
                      .color        = COLORS[COLOR_CAR],
                      .width_m      = 1.8,
                      .length_m     = 4.4,
                      .position     = {20, 13},
                      .rotation_deg = 20,
                      .wheels       = {.steering_angle_deg     = 0,
                                       .max_steering_angle_deg = degree(max_steering_angle(
                       wheelbase_m, wheelspace_m, turning_circle_m)),
                                       .wheelspace_m           = wheelspace_m,
                                       .wheelbase_m            = wheelbase_m,
                                       .width_m                = 0.6,
                                       .length_m               = 1},
                      .breaks       = false,
                      .motor_force  = 0,
                      .points       = 0,
                      .signal       = SIG_NONE,
                      .c_mass       = 1457,
                      .c_drag       = 10,
                      .c_rolling    = 1000,

                      .current_force = 0,
                      .velocity      = 0,

                      .max_force = 10};

    Vector2 origin = {0, 0};

    Camera2D camera = {0};
    camera.zoom     = 10.0f;

#ifdef GRAPH
    struct graph graph;
    if (!graph_create(&graph, 1000, RED, BLANK)) {
        printf("could not create graph\n");
        return 1;
    }
#endif

    struct wheel_line wheel_line;
    if (!wheel_line_create(&wheel_line, 0.5, 1000, car)) {
        printf("could not create wheel line\n");
        return 1;
    }

    Rectangle speedometer_pos = {
        .x = 0, .y = GetRenderHeight() - 125, .width = 125, .height = 125};

    while (!WindowShouldClose()) {
        double now        = GetTime();
        double delta_time = now - time;
        time              = now;

#ifdef GRAPH
        graph_push(&graph, car.velocity);
#endif
        wheel_line_push(&wheel_line, car, delta_time);
        // draw
        BeginMode2D(camera);
        {
            draw_background(stage);
            wheel_line_draw(wheel_line, car);
            draw_car(&car, origin, delta_time);
        }
        EndMode2D();
        {
            DrawFPS(10, 10);
            char text[255];
            sprintf(text, "points: %.2f", car.points);
            DrawText(text, 10, 30, 25, (Color){100, 100, 0, 255});
#ifdef DEBUG
            sprintf(text, "x: %f, y: %f, s: %f", car.position.x, car.position.y,
                    car.velocity);
            DrawText(text, 100, 10, 25, WHITE);
#endif  // DEBUG
#ifdef GRAPH
            graph_draw(graph, (Rectangle){10, 70, 200, 100});
#endif

            // speedometer
            DrawSpeedometer(speedometer_pos, ms_to_kmh(car.velocity), 100);
        }
        EndDrawing();

        // logic
        car.breaks = false;
        if (IsKeyDown(KEY_UP)) {
            if (car.velocity < 0) {
                car.breaks      = true;
                car.motor_force = 14300;
            }
            else {
                car.motor_force = 7500;
            }
        }
        else if (IsKeyDown(KEY_DOWN)) {
            // TODO: make a better system
            if (car.velocity > 0) {  // breaks
                car.motor_force = -14300;
                car.breaks      = true;
            }
            else if (car.velocity > -100) {  // max speed in reverse
                car.motor_force = -4500;
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
            car.wheels.steering_angle_deg += 100 * delta_time;
        }
        else if (IsKeyDown(KEY_LEFT) &&
                 car.wheels.steering_angle_deg >
                     -car.wheels.max_steering_angle_deg) {
            car.wheels.steering_angle_deg -= 100 * delta_time;
        }
        else if (!IsKeyDown(KEY_RIGHT) && !IsKeyDown(KEY_LEFT)) {
            car.wheels.steering_angle_deg *= 0.99;
        }

        if (IsKeyDown(KEY_Q)) {
            car.signal = SIG_LEFT;
        }
        else if (IsKeyDown(KEY_W)) {
            car.signal = SIG_RIGHT;
        }
        else {
            car.signal = SIG_NONE;
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

        Vector2 target_abs = Vector2Add(
            car.position, (Vector2){-screen_width / 2.f * 1 / camera.zoom,
                                    -screen_height / 2.f * 1 / camera.zoom});
        Vector2 target_diff = Vector2Subtract(camera.target, target_abs);
        camera.target.x -= target_diff.x * delta_time * 3;
        camera.target.y -= target_diff.y * delta_time * 3;

        struct car car_new       = calculate_forces(&car, delta_time);
        struct wheels wheels_new = get_wheels(car_new);
        bool touch_solid         = false;

        touch_solid |= position_to_type(stage, wheels_new.bl.x,
                                        wheels_new.bl.y) == BUILDING;
        touch_solid |= position_to_type(stage, wheels_new.fr.x,
                                        wheels_new.fr.y) == BUILDING;
        touch_solid |= position_to_type(stage, wheels_new.fl.x,
                                        wheels_new.fl.y) == BUILDING;
        touch_solid |= position_to_type(stage, wheels_new.br.x,
                                        wheels_new.br.y) == BUILDING;

        if (touch_solid) {
            car.current_force = 0;
            car.velocity      = 0;
        }
        else {
            car = car_new;
        }

        // points calculations
        struct wheels wheels = get_wheels(car);
        car.points +=
            count_points(stage, wheels.bl.x, wheels.bl.y) * delta_time;
        car.points +=
            count_points(stage, wheels.fr.x, wheels.fr.y) * delta_time;
        car.points +=
            count_points(stage, wheels.fl.x, wheels.fl.y) * delta_time;
        car.points +=
            count_points(stage, wheels.br.x, wheels.br.y) * delta_time;
    }

    UnloadTexture(stage.texture);
    CloseWindow();
    UnloadImage(map_seg);
    return 0;
}
