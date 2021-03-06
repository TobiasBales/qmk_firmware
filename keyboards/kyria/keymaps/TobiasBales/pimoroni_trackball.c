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

#include "pimoroni_trackball.h"
#include "i2c_master.h"
#include "print.h"


static uint8_t scrolling      = 0;
static int16_t x_offset       = 0;
static int16_t y_offset       = 0;
static int16_t h_offset       = 0;
static int16_t v_offset       = 0;
static bool precisionMode  = false;


#ifdef TRACKBALL_MOUSE_LAYER
    #define MOUSE_LAYER_TIMEOUT 600
    static int16_t mouse_auto_layer_timer = 0;
    static uint8_t mouse_layer    = 0;

    void trackball_set_mouse_layer(uint8_t layer) {
        mouse_layer = layer;
    }
#endif

#ifndef I2C_TIMEOUT
     #define I2C_TIMEOUT 100
#endif

void trackball_process_matrix_scan(void) {
#ifdef TRACKBALL_MOUSE_LAYER
    if (mouse_auto_layer_timer && timer_elapsed(mouse_auto_layer_timer) > MOUSE_LAYER_TIMEOUT) {
        report_mouse_t rep = pointing_device_get_report();
        if (rep.buttons) {
            return;
        }
        layer_off(mouse_layer);
        mouse_auto_layer_timer = 0;
    }
#endif
}

void trackball_set_brightness(uint8_t brightness) {
    uint8_t data[4] = {};
    i2c_readReg(TRACKBALL_WRITE, REG_RED, data, 4, TB_I2C_TIMEOUT);
    for (int i=0; i<4; i++) {
        if (data[i]) {
            data[i] = brightness;
        }
    }
    i2c_writeReg(TRACKBALL_WRITE, REG_RED, data, 4, TB_I2C_TIMEOUT);
}


void trackball_read_state(uint8_t* data, uint16_t size_of_data) {
    i2c_readReg(TRACKBALL_WRITE, REG_LEFT, data, size_of_data, TB_I2C_TIMEOUT);
}

trackball_state_t trackball_get_state(void) {
    // up down left right button
    uint8_t s[5] = {};
    trackball_read_state(s, 5);

    trackball_state_t state = {
#if TRACKBALL_ORIENTATION == 0
        // Pimoroni text is up
        .y = s[0] - s[1],
        .x = s[3] - s[2],
#elif TRACKBALL_ORIENTATION == 1
        // Pimoroni text is right
        .y = s[3] - s[2],
        .x = s[1] - s[0],
#elif TRACKBALL_ORIENTATION == 2
        // Pimoroni text is down
        .y = s[1] - s[0],
        .x = s[2] - s[3],
#else
        // Pimoroni text is left
        .y = s[2] - s[3],
        .x = s[0] - s[1],
#endif
        .button_down = s[4] & 0x80,
        .button_triggered = s[4] & 0x01,
};

#ifndef TRACKBALL_NO_MATH
    state.angle_rad = atan2(state.y, state.x) + TRACKBALL_ANGLE_OFFSET;
    state.vector_length = sqrt(pow(state.x, 2) + pow(state.y, 2));
    state.raw_x = state.x;
    state.raw_y = state.y;
    state.x = (int16_t)(state.vector_length * cos(state.angle_rad));
    state.y = (int16_t)(state.vector_length * sin(state.angle_rad));
#endif

    return state;
}


void trackball_set_rgbw(uint8_t red, uint8_t green, uint8_t blue, uint8_t white) {
    uint8_t data[] = {0x00, red, green, blue, white};
    i2c_transmit(TRACKBALL_WRITE, data, sizeof(data), I2C_TIMEOUT);
}

void update_member(int8_t* member, int16_t* offset) {
    if (*offset > 127) {
        *member = 127;
        *offset -= 127;
    } else if (*offset < -127) {
        *member = -127;
        *offset += 127;
    } else {
        *member = *offset;
        *offset = 0;
    }
}

__attribute__((weak)) void trackball_check_click(bool pressed, report_mouse_t* mouse) {
    if (pressed) {
        mouse->buttons |= MOUSE_BTN1;
    } else {
        mouse->buttons &= ~MOUSE_BTN1;
    }
}

void trackball_register_button(bool pressed, uint8_t button) {
    report_mouse_t currentReport = pointing_device_get_report();
    if (pressed) {
        currentReport.buttons |= button;
    } else {
        currentReport.buttons &= ~button;
    }
    pointing_device_set_report(currentReport);
}

float trackball_get_precision(void) { return precisionMode; }
void  trackball_set_precision(bool precision) { precisionMode = precision; }
bool  trackball_is_scrolling(void) { return scrolling; }
void  trackball_set_scrolling(bool scroll) { scrolling = scroll; }

bool has_report_changed (report_mouse_t first, report_mouse_t second) {
    return !(
        (!first.buttons && first.buttons == second.buttons) &&
        (!first.x && first.x == second.x) &&
        (!first.y && first.y == second.y) &&
        (!first.h && first.h == second.h) &&
        (!first.v && first.v == second.v) );
}


__attribute__((weak)) void pointing_device_init(void) { trackball_set_rgbw(0x00, 0x00, 0x00, 0x4F); }

void pointing_device_task(void) {
    trackball_state_t state = trackball_get_state();
    if (state.x || state.y) {
        if (scrolling) {
            h_offset += state.x;
            v_offset -= state.y;
        } else {
#ifdef TRACKBALL_MOUSE_LAYER
            if (!mouse_auto_layer_timer) {
                layer_on(mouse_layer);
            }
            mouse_auto_layer_timer = timer_read() | 1;
#endif

            uint8_t scale = 8;
            if (precisionMode) {
                scale = 4;
            }
            x_offset += state.x * state.x * SIGN(state.x) * scale;
            y_offset += state.y * state.y * SIGN(state.y) * scale;
        }
    }

    report_mouse_t mouse = pointing_device_get_report();
    if (state.button_triggered) {
        trackball_check_click(state.button_down, &mouse);
     }

    update_member(&mouse.x, &x_offset);
    update_member(&mouse.y, &y_offset);
    update_member(&mouse.h, &h_offset);
    update_member(&mouse.v, &v_offset);


    pointing_device_set_report(mouse);
    if (has_report_changed(mouse, pointing_device_get_report())) {
        pointing_device_send();
    }
}
