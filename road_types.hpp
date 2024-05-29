#ifndef ROAD_TYPES_HPP
#define ROAD_TYPES_HPP

#include <raylib.h>
#include <vector>

enum Mode {
    MODE_DEFAULT,
    MODE_LINE
};


struct Point {
    int x;
    int y;
    bool is_hovering;
};


struct RoadLine {
    std::vector<Point> points;
};


struct Square {
    int x;
    int y;
    int size;
    bool is_selected;
    bool is_hovering;
};

#endif
