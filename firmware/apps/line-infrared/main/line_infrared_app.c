/*
 * 红外循线应用的功能实现。
 *
 * 该文件是本应用中唯一启用下列模块实现的位置；各头文件同时提供
 * 对外接口和实现，以便在一个小型 ESP-IDF 应用中集中查看、调参和维护。
 */
#define LINE_TRACKER_IMPLEMENTATION
#include "line_tracker.h"

#define OBSTACLE_AVOID_IMPLEMENTATION
#include "obstacle_avoid.h"

#define OLED_DISPLAY_IMPLEMENTATION
#include "oled_display.h"
