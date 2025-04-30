#include "config.h"

#include <ctype.h>

/**
 * @brief Trim spaces from the left of the str in place
 *
 * @param str to trim
 * @return the trimmed start position
 */
char* trim(char* str)
{
    while (isspace((unsigned char)*str)) str++;
    if (*str == 0) return str;
    char* end = str + strlen(str) - 1;
    while (end > str && isspace((unsigned char)*end)) end--;
    *(end + 1) = '\0';
    return str;
}

bool parse_line(const char* line, struct config* out)
{
    char* copy = strdup(line);
    char* eq   = strchr(copy, '=');
    if (!eq) {
        free(copy);
        return false;
    }

    *eq       = '\0';
    char* key = trim(copy);
    char* val = trim(eq + 1);

    out->key = strdup(key);

    // Infer type from value
    if (val[0] == '"' && val[strlen(val) - 1] == '"') {
        val[strlen(val) - 1] = '\0';
        out->type            = VALUE_STR;
        out->value.s         = strdup(val + 1);
    }
    else if (strcasecmp(val, "true") == 0 || strcasecmp(val, "false") == 0) {
        out->type    = VALUE_BOOL;
        out->value.b = strcasecmp(val, "true") == 0;
    }
    else if (strchr(val, '.') != NULL) {
        // TODO: make better checks for float
        out->type    = VALUE_FLOAT;
        out->value.f = strtof(val, NULL);
    }
    else if (isdigit((unsigned char)*val) || *val == '-') {
        out->type    = VALUE_INT;
        out->value.i = strtol(val, NULL, 10);
    }

    free(copy);
    return true;
}

void add_config(struct configs* configs, struct config new_config)
{
    if (configs->len == configs->capacity) {
        configs->capacity = configs->capacity == 0 ? 4 : configs->capacity * 2;
        configs->array =
            realloc(configs->array, configs->capacity * sizeof(struct config));
    }
    configs->array[configs->len++] = new_config;
}

struct configs read_config(const char* config_path)
{
    FILE* fp = fopen(config_path, "r");
    if (!fp) {
        perror("could not open config file");
        exit(EXIT_FAILURE);
    }

    struct configs configs = {0};

    char* line   = NULL;
    size_t n     = 0;
    size_t count = 0;
    while (getline(&line, &n, fp) != -1) {
        count++;
        // skip empty lines or comments
        char* trimmed = trim(line);
        if (trimmed[0] == '\0' || trimmed[0] == '#') {
            continue;
        }

        struct config cfg;
        if (!parse_line(trimmed, &cfg)) {
            fprintf(stderr, "Invalid config line %ld: '%s'\n", count, line);
            continue;
        }

        add_config(&configs, cfg);
    }

    free(line);
    fclose(fp);
    return configs;
}

void free_config(struct configs* config)
{
    if (config->capacity > 0) {
        free(config->array);
        config->capacity = 0;
    }
}

struct config* find_config(const struct configs config, const char* key)
{
    for (size_t i = 0; i < config.len; i++) {
        if (!strcmp(config.array[i].key, key)) {
            return &config.array[i];
        }
    }
    return NULL;
}
