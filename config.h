#ifndef CONFIG_H
#define CONFIG_H

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum value_type {
    VALUE_STR,
    VALUE_INT,
    VALUE_BOOL,
    VALUE_FLOAT,
    VALUE_TYPE_MAX,
};

struct config {
    enum value_type type;
    char* key;
    union {
        char* s;
        int i;
        bool b;
        float f;
    } value;
};

struct configs {
    size_t len;
    size_t capacity;
    struct config* array;
};

void free_config(struct configs* config);

/**
 * @brief Reads config file and creates key value array
 * @note need free
 *
 * @param config_path is the path to the config file
 */
struct configs read_config(const char* config_path);

struct config* find_config(const struct configs config, const char* key);

#endif  // CONFIG_H
