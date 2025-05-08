#ifndef GRAPH_H
#define GRAPH_H

#include <stddef.h>
#include <stdlib.h>

#include "raylib.h"

struct graph {
    float* items;
    size_t length;
    size_t curser;
    Color line;
    Color background;
};

bool graph_create(struct graph* graph, size_t items, Color line,
                  Color background);

void graph_free(struct graph* graph);

void graph_push(struct graph* graph, float item);

void graph_draw(const struct graph graph, Rectangle pos);
#endif  // GRAPH_H
