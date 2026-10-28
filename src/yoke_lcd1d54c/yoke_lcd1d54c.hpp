#pragma once

#include <LovyanGFX.hpp>
#include <Wire.h>

#include "../ryzobee_yoke.hpp"
#include "Touch_CST816T.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define YOKE_LCD_WRITE_FREQ 40000000
#define YOKE_LCD_READ_FREQ  16000000
#define YOKE_LCD_BL_FREQ    44100
#define YOKE_LCD_BL_CHANNEL 7

static constexpr uint16_t YOKE_LCD1D54C_SIZE = 240;

class Yoke_lcd1d54c : public ryzobee_yoke, public lgfx::LGFX_Device {
public:
    Yoke_lcd1d54c(TwoWire* wire = nullptr, SemaphoreHandle_t* mutex = nullptr)
        : ryzobee_yoke(kRYZOBEE_YOKE_F_BUS),
          _dev_i2c(wire == nullptr ? &Wire : wire), _i2c_mutex(mutex) {
        _iodata = nullptr;
        configure();
    }

    /**
     * @brief 占用 F_BUS 端口并初始化屏幕和触摸。
     *
     * @param iodata 开发板 useport() 返回的端口数据，不能为空。
     * @return YOKE_PORT_ERROR_NONE 表示成功，其余为端口错误码。
     */
    int begin(std::vector<uint8_t>* iodata) {
        if (_initialized) {
            return YOKE_PORT_ERROR_ALREADY_INITIALIZED;
        }
        if (portInvalid(iodata)) {
            return YOKE_PORT_ERROR_INVALID;
        }
        int result = portBegin(iodata);
        if (result == YOKE_PORT_ERROR_NONE) {
            _iodata = iodata;
        }
        return result;
    }

    /**
     * @brief 关闭屏幕并释放已占用的端口。
     *
     * @return YOKE_PORT_ERROR_NONE 表示成功，其余为端口错误码。
     */
    int end() {
        int result = portEnd();
        if (result == YOKE_PORT_ERROR_NONE && _iodata != nullptr) {
            _iodata->at(YOKE_PORT_USED) = 0;
            _iodata = nullptr;
        }
        return result;
    }

    /**
     * @brief 初始化默认接线的屏幕；共享 I2C 时须先初始化传入的 TwoWire。
     *
     * @return 屏幕是否初始化成功。
     */
    bool init() {
        if (_initialized) {
            return true;
        }
        if (_i2c_mutex != nullptr && *_i2c_mutex == nullptr) {
            return false;
        }
        if (_i2c_mutex == nullptr && !_dev_i2c->begin(_pin_sda, _pin_scl, TP_FREQ)) {
            return false;
        }
        if (_i2c_mutex != nullptr) {
            xSemaphoreTake(*_i2c_mutex, portMAX_DELAY);
        }
        bool ready = lgfx::LGFX_Device::init();
        if (_i2c_mutex != nullptr) {
            xSemaphoreGive(*_i2c_mutex);
        }
        _initialized = ready;
        return ready;
    }

    std::string getYokeName() override { return "Yoke LCD1D54C"; }
    std::string getYokeInfo() override { return "240x240 ST7789 LCD with CST816T touch"; }

    /**
     * @brief 清屏为黑色。
     */
    void clear() {
        fillScreen(TFT_BLACK);
    }

    /**
     * @brief 读取随屏幕旋转变换后的单点触摸坐标。
     *
     * @param x 输出横坐标，不能为空。
     * @param y 输出纵坐标，不能为空。
     * @return 是否检测到触摸。
     */
    bool getTouch(uint16_t* x, uint16_t* y) {
        if (x == nullptr || y == nullptr || !_initialized ||
            (_i2c_mutex != nullptr && *_i2c_mutex == nullptr)) {
            return false;
        }

        bool touched = false;
        if (_i2c_mutex == nullptr || xSemaphoreTake(*_i2c_mutex, portMAX_DELAY) == pdTRUE) {
            lgfx::touch_point_t point;
            int count = lgfx::LGFX_Device::getTouch(&point, 1);
            if (count > 0) {
                *x = point.x;
                *y = point.y;
                touched = true;
            }
            if (_i2c_mutex != nullptr) {
                xSemaphoreGive(*_i2c_mutex);
            }
        }
        return touched;
    }

protected:
    /**
     * @brief 按 F_BUS9～F_BUS24 的引脚向量初始化屏幕和触摸。
     *
     * @param iodata 已由基类校验的 F_BUS 端口数据。
     * @return YOKE_PORT_ERROR_NONE 表示成功，其余为端口错误码。
     */
    int portBegin(std::vector<uint8_t>* iodata) override {
        if (_initialized) {
            return YOKE_PORT_ERROR_ALREADY_INITIALIZED;
        }
        if (iodata->size() < YOKE_PORT_PIN_START + 16 ||
            iodata->at(YOKE_PORT_PIN_NUM) < 16) {
            return YOKE_PORT_ERROR_NOT_CONFIGURED;
        }
        _pin_sclk = iodata->at(YOKE_PORT_PIN_START + 11 - 9);
        _pin_rst = iodata->at(YOKE_PORT_PIN_START + 13 - 9);
        _pin_mosi = iodata->at(YOKE_PORT_PIN_START + 14 - 9);
        _pin_bl = iodata->at(YOKE_PORT_PIN_START + 16 - 9);
        _pin_cs = iodata->at(YOKE_PORT_PIN_START + 18 - 9);
        _pin_dc = iodata->at(YOKE_PORT_PIN_START + 20 - 9);
        _pin_tp_rst = iodata->at(YOKE_PORT_PIN_START + 21 - 9);
        _pin_tp_int = iodata->at(YOKE_PORT_PIN_START + 22 - 9);
        _pin_sda = iodata->at(YOKE_PORT_PIN_START + 23 - 9);
        _pin_scl = iodata->at(YOKE_PORT_PIN_START + 24 - 9);
        configure();
        if (!init()) {
            releaseBus();
            return YOKE_PORT_ERROR_NOT_AVAILABLE;
        }
        iodata->at(YOKE_PORT_USED) = 1;
        return YOKE_PORT_ERROR_NONE;
    }

    /**
     * @brief 关闭背光并释放屏幕 SPI 总线，保留共享 I2C 总线。
     *
     * @return YOKE_PORT_ERROR_NONE 表示成功，未初始化时返回对应错误码。
     */
    int portEnd() override {
        if (!_initialized) {
            return YOKE_PORT_ERROR_NOT_INITIALIZED;
        }
        setBrightness(0);
        releaseBus();
        _initialized = false;
        return YOKE_PORT_ERROR_NONE;
    }

private:
    /**
     * @brief 配置 ST7789、背光 PWM 和 CST816T 触摸。
     */
    void configure() {
        {
            auto cfg = _bus_instance.config();

            cfg.spi_mode = 0;
            cfg.freq_write = YOKE_LCD_WRITE_FREQ;
            cfg.freq_read = YOKE_LCD_READ_FREQ;
            cfg.spi_3wire = true;
            cfg.use_lock = true;
            cfg.dma_channel = SPI_DMA_CH_AUTO;
            cfg.pin_sclk = _pin_sclk;
            cfg.pin_mosi = _pin_mosi;
            cfg.pin_miso = -1;
            cfg.pin_dc = _pin_dc;

            _bus_instance.config(cfg);
            _panel_instance.setBus(&_bus_instance);
        }

        {
            auto cfg = _panel_instance.config();

            cfg.pin_cs = _pin_cs;
            cfg.pin_rst = _pin_rst;
            cfg.pin_busy = -1;
            cfg.panel_width = YOKE_LCD1D54C_SIZE;
            cfg.panel_height = YOKE_LCD1D54C_SIZE;
            cfg.offset_x = 0;
            cfg.offset_y = 0;
            cfg.offset_rotation = 0;
            cfg.dummy_read_pixel = 8;
            cfg.dummy_read_bits = 1;
            cfg.readable = true;
            cfg.invert = true;
            cfg.rgb_order = false;
            cfg.dlen_16bit = false;
            cfg.bus_shared = true;

            _panel_instance.config(cfg);
        }

        {
            auto cfg = _light_instance.config();

            cfg.pin_bl = _pin_bl;
            cfg.invert = false;
            cfg.freq = YOKE_LCD_BL_FREQ;
            cfg.pwm_channel = YOKE_LCD_BL_CHANNEL;

            _light_instance.config(cfg);
            _panel_instance.setLight(&_light_instance);
        }

        {
            auto cfg = _touch_instance.config();

            cfg.x_min = 0;
            cfg.x_max = YOKE_LCD1D54C_SIZE;
            cfg.y_min = 0;
            cfg.y_max = YOKE_LCD1D54C_SIZE;
            cfg.pin_int = _pin_tp_int;
            cfg.pin_rst = _pin_tp_rst;
            cfg.bus_shared = true;
            cfg.offset_rotation = 0;
            cfg.i2c_port = 0;
            cfg.i2c_addr = TP_ADDR;
            cfg.pin_sda = _pin_sda;
            cfg.pin_scl = _pin_scl;
            cfg.freq = TP_FREQ;

            _touch_instance.config(cfg);
            _touch_instance.setWire(_dev_i2c);
            _panel_instance.setTouch(&_touch_instance);
        }

        setPanel(&_panel_instance);
    }

    lgfx::Panel_ST7789 _panel_instance;
    lgfx::Bus_SPI _bus_instance;
    lgfx::Light_PWM _light_instance;
    lgfx::Touch_CST816T _touch_instance;
    TwoWire* _dev_i2c = nullptr;
    SemaphoreHandle_t* _i2c_mutex = nullptr;
    // 默认接线与 RootMaker 的 F_BUS 相同。
    int _pin_sclk = 12, _pin_mosi = 9, _pin_dc = 3;
    int _pin_cs = 5, _pin_rst = 10, _pin_bl = 7;
    int _pin_tp_int = 1, _pin_tp_rst = 2;
    int _pin_sda = 41, _pin_scl = 40;
};
