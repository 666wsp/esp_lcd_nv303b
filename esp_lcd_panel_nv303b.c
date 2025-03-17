/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <sys/cdefs.h>
#include "sdkconfig.h"

#if CONFIG_LCD_ENABLE_DEBUG_LOG
// The local log level must be defined before including esp_log.h
// Set the maximum log level for this source file
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_commands.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_compiler.h"
#include "esp_lcd_panel_nv303b.h"

#define NV303B_CMD_RAMCTRL 0xb0
#define NV303B_DATA_LITTLE_ENDIAN_BIT (1 << 3)

static const char *TAG = "lcd_panel.nv303b";

static esp_err_t panel_nv303b_del(esp_lcd_panel_t *panel);
static esp_err_t panel_nv303b_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_nv303b_init(esp_lcd_panel_t *panel);
static esp_err_t panel_nv303b_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data);
static esp_err_t panel_nv303b_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t panel_nv303b_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_nv303b_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t panel_nv303b_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t panel_nv303b_disp_on_off(esp_lcd_panel_t *panel, bool off);
static esp_err_t panel_nv303b_sleep(esp_lcd_panel_t *panel, bool sleep);

typedef struct
{
    esp_lcd_panel_t base;
    esp_lcd_panel_io_handle_t io;
    int reset_gpio_num;
    bool reset_level;
    int x_gap;
    int y_gap;
    uint8_t fb_bits_per_pixel;
    uint8_t madctl_val; // save current value of LCD_CMD_MADCTL register
    uint8_t colmod_val; // save current value of LCD_CMD_COLMOD register
    uint8_t ramctl_val_1;
    uint8_t ramctl_val_2;
} nv303b_panel_t;

esp_err_t esp_lcd_new_panel_nv303b(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config,
                                   esp_lcd_panel_handle_t *ret_panel)
{
#if CONFIG_LCD_ENABLE_DEBUG_LOG
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
#endif
    esp_err_t ret = ESP_OK;
    nv303b_panel_t *nv303b = NULL;
    ESP_GOTO_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    // leak detection of nv303b because saving nv303b->base address
    nv303b = calloc(1, sizeof(nv303b_panel_t));
    ESP_GOTO_ON_FALSE(nv303b, ESP_ERR_NO_MEM, err, TAG, "no mem for nv303b panel");

    if (panel_dev_config->reset_gpio_num >= 0)
    {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST line failed");
    }

    switch (panel_dev_config->rgb_ele_order)
    {
    case LCD_RGB_ELEMENT_ORDER_RGB:
        nv303b->madctl_val = 0;
        break;
    case LCD_RGB_ELEMENT_ORDER_BGR:
        nv303b->madctl_val |= LCD_CMD_BGR_BIT;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported RGB element order");
        break;
    }

    uint8_t fb_bits_per_pixel = 0;
    switch (panel_dev_config->bits_per_pixel)
    {
    case 16: // RGB565
        nv303b->colmod_val = 0x55;
        fb_bits_per_pixel = 16;
        break;
    case 18: // RGB666
        nv303b->colmod_val = 0x66;
        // each color component (R/G/B) should occupy the 6 high bits of a byte, which means 3 full bytes are required for a pixel
        fb_bits_per_pixel = 24;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported pixel width");
        break;
    }

    nv303b->ramctl_val_1 = 0x00;
    nv303b->ramctl_val_2 = 0xf0; // Use big endian by default
    if ((panel_dev_config->data_endian) == LCD_RGB_DATA_ENDIAN_LITTLE)
    {
        // Use little endian
        // nv303b->ramctl_val_2 |= ST7789_DATA_LITTLE_ENDIAN_BIT;
    }

    nv303b->io = io;
    nv303b->fb_bits_per_pixel = fb_bits_per_pixel;
    nv303b->reset_gpio_num = panel_dev_config->reset_gpio_num;
    nv303b->reset_level = panel_dev_config->flags.reset_active_high;
    nv303b->base.del = panel_nv303b_del;
    nv303b->base.reset = panel_nv303b_reset;
    nv303b->base.init = panel_nv303b_init;
    nv303b->base.draw_bitmap = panel_nv303b_draw_bitmap;
    nv303b->base.invert_color = panel_nv303b_invert_color;
    nv303b->base.set_gap = panel_nv303b_set_gap;
    nv303b->base.mirror = panel_nv303b_mirror;
    nv303b->base.swap_xy = panel_nv303b_swap_xy;
    nv303b->base.disp_on_off = panel_nv303b_disp_on_off;
    nv303b->base.disp_sleep = panel_nv303b_sleep;
    *ret_panel = &(nv303b->base);
    ESP_LOGD(TAG, "new nv303b panel @%p", nv303b);

    return ESP_OK;

err:
    if (nv303b)
    {
        if (panel_dev_config->reset_gpio_num >= 0)
        {
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        }
        free(nv303b);
    }
    return ret;
}

static esp_err_t panel_nv303b_del(esp_lcd_panel_t *panel)
{
    nv303b_panel_t *nv303b = __containerof(panel, nv303b_panel_t, base);

    if (nv303b->reset_gpio_num >= 0)
    {
        gpio_reset_pin(nv303b->reset_gpio_num);
    }
    ESP_LOGD(TAG, "del nv303b panel @%p", nv303b);
    free(nv303b);
    return ESP_OK;
}

static esp_err_t panel_nv303b_reset(esp_lcd_panel_t *panel)
{
    nv303b_panel_t *nv303b = __containerof(panel, nv303b_panel_t, base);
    esp_lcd_panel_io_handle_t io = nv303b->io;

    // perform hardware reset
    if (nv303b->reset_gpio_num >= 0)
    {
        gpio_set_level(nv303b->reset_gpio_num, nv303b->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(nv303b->reset_gpio_num, !nv303b->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    else
    { // perform software reset
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_SWRESET, NULL, 0), TAG,
                            "io tx param failed");
        vTaskDelay(pdMS_TO_TICKS(20)); // spec, wait at least 5m before sending new command
    }

    return ESP_OK;
}

static esp_err_t panel_nv303b_init(esp_lcd_panel_t *panel)
{
    nv303b_panel_t *nv303b = __containerof(panel, nv303b_panel_t, base);
    esp_lcd_panel_io_handle_t io = nv303b->io;
    // LCD goes into sleep mode and display will be turned off after power on reset, exit sleep mode first
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_SLPOUT, NULL, 0), TAG,
                        "io tx param failed");
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_lcd_panel_io_tx_param(io, 0xfd, (uint8_t[]){0x06, 0x08}, 2);

    esp_lcd_panel_io_tx_param(io, 0x61, (uint8_t[]){0x07, 0x07}, 2);

    esp_lcd_panel_io_tx_param(io, 0x73, (uint8_t[]){0x70}, 1);
    esp_lcd_panel_io_tx_param(io, 0x73, (uint8_t[]){0x00}, 1);

    // bias
    esp_lcd_panel_io_tx_param(io, 0x62, (uint8_t[]){0x00, 0x44, 0x40}, 3);

    esp_lcd_panel_io_tx_param(io, 0x63, (uint8_t[]){0x41, 0x07, 0x12, 0x12}, 4);
    esp_lcd_panel_io_tx_param(io, 0x64, (uint8_t[]){0x37}, 1);

    // VSP
    esp_lcd_panel_io_tx_param(io, 0x65, (uint8_t[]){0x09, 0x10, 0x21}, 3);
    // VSN
    esp_lcd_panel_io_tx_param(io, 0x66, (uint8_t[]){0x09, 0x10, 0x21}, 3);
    // add source_neg_time
    esp_lcd_panel_io_tx_param(io, 0x67, (uint8_t[]){0x20, 0x40}, 2);
    // gamma vap/van
    esp_lcd_panel_io_tx_param(io, 0x68, (uint8_t[]){0x90, 0x4c, 0x7c, 0x66}, 4);

    esp_lcd_panel_io_tx_param(io, 0xb1, (uint8_t[]){0x0F, 0x02, 0x01}, 3);

    esp_lcd_panel_io_tx_param(io, 0xB4, (uint8_t[]){0x01}, 1); // 01:1dot 00:column
    ////porch
    esp_lcd_panel_io_tx_param(io, 0xB5, (uint8_t[]){0x02, 0x02, 0x0a, 0x14}, 4);

    esp_lcd_panel_io_tx_param(io, 0xB6, (uint8_t[]){0x04, 0x01, 0x9f, 0x00, 0x02}, 5);
    ////gamme sel
    esp_lcd_panel_io_tx_param(io, 0xdf, (uint8_t[]){0x11}, 1);

    ////gamma_test1 A1#_wangly
    // 3030b_gamma_new_
    // GAMMA---------------------------------/////////////

    // GAMMA---------------------------------/////////////
    esp_lcd_panel_io_tx_param(io, 0xE2, (uint8_t[]){0x13, 0x00, 0x00, 0x26, 0x27, 0x3f}, 6);

    esp_lcd_panel_io_tx_param(io, 0xE5, (uint8_t[]){0x3f, 0x27, 0x26, 0x00, 0x00, 0x13}, 6);

    esp_lcd_panel_io_tx_param(io, 0xE1, (uint8_t[]){0x00, 0x57}, 2);

    esp_lcd_panel_io_tx_param(io, 0xE4, (uint8_t[]){0x58, 0x00}, 2);

    esp_lcd_panel_io_tx_param(io, 0xE0, (uint8_t[]){0x01, 0x03, 0x0d, 0x0e, 0x0e, 0x0c, 0x15, 0x19}, 8);

    esp_lcd_panel_io_tx_param(io, 0xE3, (uint8_t[]){0x1a, 0x16, 0x0c, 0x0f, 0x0e, 0x0d, 0x02, 0x01}, 8);
    // GAMMA---------------------------------/////////////
    // source
    esp_lcd_panel_io_tx_param(io, 0xE6, (uint8_t[]){0x00, 0xff}, 2);

    esp_lcd_panel_io_tx_param(io, 0xE7, (uint8_t[]){0x01, 0x04, 0x03, 0x03, 0x00, 0x12}, 6);

    esp_lcd_panel_io_tx_param(io, 0xE8, (uint8_t[]){0x00, 0x70, 0x00}, 3);
    ////gate
    esp_lcd_panel_io_tx_param(io, 0xEc, (uint8_t[]){0x52}, 1);

    esp_lcd_panel_io_tx_param(io, 0xF1, (uint8_t[]){0x01, 0x01, 0x02}, 3);

    esp_lcd_panel_io_tx_param(io, 0xF6, (uint8_t[]){0x01, 0x30, 0x00, 0x00}, 4);

    esp_lcd_panel_io_tx_param(io, 0xfd, (uint8_t[]){0xfa, 0xfc}, 2);

    esp_lcd_panel_io_tx_param(io, 0x3a, (uint8_t[]){0x05}, 1);

    esp_lcd_panel_io_tx_param(io, 0x35, (uint8_t[]){0x00}, 1);

    esp_lcd_panel_io_tx_param(io, 0x36, (uint8_t[]){0x00}, 1);

    esp_lcd_panel_io_tx_param(io, 0x21, NULL, 0);
    esp_lcd_panel_io_tx_param(io, 0x11, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_lcd_panel_io_tx_param(io, 0x29, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    return ESP_OK;
}

static esp_err_t panel_nv303b_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end,
                                          const void *color_data)
{
    nv303b_panel_t *nv303b = __containerof(panel, nv303b_panel_t, base);
    esp_lcd_panel_io_handle_t io = nv303b->io;

    x_start += nv303b->x_gap;
    x_end += nv303b->x_gap;
    y_start += nv303b->y_gap;
    y_end += nv303b->y_gap;

    // define an area of frame memory where MCU can access
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_CASET, (uint8_t[]){
                                                                         (x_start >> 8) & 0xFF,
                                                                         x_start & 0xFF,
                                                                         ((x_end - 1) >> 8) & 0xFF,
                                                                         (x_end - 1) & 0xFF,
                                                                     },
                                                  4),
                        TAG, "io tx param failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_RASET, (uint8_t[]){
                                                                         (y_start >> 8) & 0xFF,
                                                                         y_start & 0xFF,
                                                                         ((y_end - 1) >> 8) & 0xFF,
                                                                         (y_end - 1) & 0xFF,
                                                                     },
                                                  4),
                        TAG, "io tx param failed");
    // transfer frame buffer
    size_t len = (x_end - x_start) * (y_end - y_start) * nv303b->fb_bits_per_pixel / 8;
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_color(io, LCD_CMD_RAMWR, color_data, len), TAG, "io tx color failed");

    return ESP_OK;
}

static esp_err_t panel_nv303b_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    nv303b_panel_t *nv303b = __containerof(panel, nv303b_panel_t, base);
    esp_lcd_panel_io_handle_t io = nv303b->io;
    int command = 0;
    if (invert_color_data)
    {
        command = LCD_CMD_INVON;
    }
    else
    {
        command = LCD_CMD_INVOFF;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG,
                        "io tx param failed");
    return ESP_OK;
}

static esp_err_t panel_nv303b_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    nv303b_panel_t *nv303b = __containerof(panel, nv303b_panel_t, base);
    esp_lcd_panel_io_handle_t io = nv303b->io;
    if (mirror_x)
    {
        nv303b->madctl_val |= LCD_CMD_MX_BIT;
    }
    else
    {
        nv303b->madctl_val &= ~LCD_CMD_MX_BIT;
    }
    if (mirror_y)
    {
        nv303b->madctl_val |= LCD_CMD_MY_BIT;
    }
    else
    {
        nv303b->madctl_val &= ~LCD_CMD_MY_BIT;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]){nv303b->madctl_val}, 1), TAG, "io tx param failed");
    return ESP_OK;
}

static esp_err_t panel_nv303b_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
    nv303b_panel_t *nv303b = __containerof(panel, nv303b_panel_t, base);
    esp_lcd_panel_io_handle_t io = nv303b->io;
    if (swap_axes)
    {
        nv303b->madctl_val |= LCD_CMD_MV_BIT;
    }
    else
    {
        nv303b->madctl_val &= ~LCD_CMD_MV_BIT;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]){nv303b->madctl_val}, 1), TAG, "io tx param failed");
    return ESP_OK;
}

static esp_err_t panel_nv303b_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    nv303b_panel_t *nv303b = __containerof(panel, nv303b_panel_t, base);
    nv303b->x_gap = x_gap;
    nv303b->y_gap = y_gap;
    return ESP_OK;
}

static esp_err_t panel_nv303b_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
    nv303b_panel_t *nv303b = __containerof(panel, nv303b_panel_t, base);
    esp_lcd_panel_io_handle_t io = nv303b->io;
    int command = 0;
    if (on_off)
    {
        command = LCD_CMD_DISPON;
    }
    else
    {
        command = LCD_CMD_DISPOFF;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG,
                        "io tx param failed");
    return ESP_OK;
}

static esp_err_t panel_nv303b_sleep(esp_lcd_panel_t *panel, bool sleep)
{
    nv303b_panel_t *nv303b = __containerof(panel, nv303b_panel_t, base);
    esp_lcd_panel_io_handle_t io = nv303b->io;
    int command = 0;
    if (sleep)
    {
        command = LCD_CMD_SLPIN;
    }
    else
    {
        command = LCD_CMD_SLPOUT;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG,
                        "io tx param failed");
    vTaskDelay(pdMS_TO_TICKS(100));

    return ESP_OK;
}
