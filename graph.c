#include "graph.h"

bool graph_create(struct graph* graph, size_t items, Color line,
                  Color background)
{
    graph->items = calloc(items, sizeof(*graph->items));
    if (!graph->items) {
        return false;
    }
    graph->curser     = 0;
    graph->length     = items;
    graph->line       = line;
    graph->background = background;
    return true;
}

void graph_free(struct graph* graph)
{
    if (!graph->items) {
        return;
    }
    free(graph->items);
}

void graph_push(struct graph* graph, float item)
{
    graph->items[graph->curser] = item;
    graph->curser               = (graph->curser + 1) % graph->length;
}

void graph_draw(const struct graph graph, Rectangle pos)
{
    DrawRectanglePro(pos, (Vector2){0, 0}, 0, graph.background);
    float x_start = pos.x;
    float x_end   = pos.x + pos.width;
    float y_start = pos.y;
    float y_end   = pos.y + pos.height;

    size_t curser = 0;
    for (size_t i = graph.curser; i != ((int)graph.curser - 1) % graph.length;
         i        = (i + 1) % graph.length) {
        float distance_to_start = (float)curser / graph.length * pos.width;
        float distance_to_end = (float)(curser + 1) / graph.length * pos.width;
        Vector2 start = {x_start + distance_to_start, y_end - graph.items[i]};
        Vector2 end   = {x_start + distance_to_end,
                         y_end - graph.items[(i + 1) % graph.length]};
        DrawLineEx(start, end, 1, graph.line);
        curser++;
    }
}
