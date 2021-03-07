/* Copyright 2020 Christopher Courtney, aka Drashna Jael're  (@drashna) <drashna@live.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "quantum.h"
#include "pointing_device.h"

#ifndef TRACKBALL_ADDRESS
    #define TRACKBALL_ADDRESS 0x0A
#endif

#ifndef TRACKBALL_ANGLE_OFFSET
    #define TRACKBALL_ANGLE_OFFSET 0
#endif

#define TRACKBALL_WRITE ((TRACKBALL_ADDRESS << 1) | I2C_WRITE)
#define TRACKBALL_READ ((TRACKBALL_ADDRESS << 1) | I2C_READ)

#define TB_I2C_TIMEOUT 100

#define SIGN(x) ((x > 0) - (x < 0))

#define REG_RED 0x00
#define REG_GREEN 0x01
#define REG_BLUE 0x02
#define REG_WHITE 0x03

#define REG_LEFT 0x04

void trackball_set_rgbw(uint8_t red, uint8_t green, uint8_t blue, uint8_t white);
void trackball_check_click(bool pressed, report_mouse_t *mouse);
void trackball_register_button(bool pressed, uint8_t button);

float trackball_get_precision(void);
void  trackball_set_precision(bool precision);
bool  trackball_is_scrolling(void);
void  trackball_set_scrolling(bool scroll);

void trackball_set_brightness(uint8_t brightness);

void trackball_set_mouse_layer(uint8_t layer);
void trackball_process_matrix_scan(void);

typedef struct {
    int16_t x;
    int16_t y;
    bool button_down;
    bool button_triggered;
#ifndef TRACKBALL_NO_MATH
    double vector_length;
    double angle_rad;
    int8_t raw_x;
    int8_t raw_y;
#endif
} trackball_state_t;
