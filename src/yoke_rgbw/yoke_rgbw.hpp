#pragma once 

#include "../ryzobee_yoke.hpp"

#if defined(ARDUINO_ARCH_ESP32)
#include "driver/rmt.h"
#endif

class Yoke_rgbw : public ryzobee_yoke {

public:
    Yoke_rgbw(void) : ryzobee_yoke(kRYZOBEE_YOKE_RS) {

    }
    void setColor(uint8_t r, uint8_t g, uint8_t b, uint8_t w = 0){
        if(!_initialized){
            YOKE_I_PRINTF("Port is not initialized.\n");
            return;
        }
        _writeLEDData(r, g, b, w);
    };
    void setColor(uint32_t color){
        if(!_initialized){
            YOKE_I_PRINTF("Port is not initialized.\n");
            return;
        }
        uint8_t w = (color >> 24) & 0xFF;
        uint8_t r = (color >> 16) & 0xFF;
        uint8_t g = (color >> 8) & 0xFF;
        uint8_t b = color & 0xFF;
        _writeLEDData(r, g, b, w);
    };
protected:
    int portBegin(std::vector<uint8_t>* iodata) override{
        if(_initialized){
            YOKE_I_PRINTF("Port is already initialized.\n");
            return YOKE_PORT_ERROR_ALREADY_INITIALIZED;
        }
#if defined(ARDUINO_ARCH_ESP32)
        YOKE_I_PRINTF("ESP32 detected.\n");
        _rmt_tx.rmt_mode = RMT_MODE_TX;
        _rmt_tx.channel = _getFirstFreeRMTChannel();
        YOKE_I_PRINTF("Free RMT channel: %d\n", _rmt_tx.channel);
        _rmt_tx.gpio_num = (gpio_num_t)iodata->at(YOKE_PORT_PIN_START);
        _rmt_tx.mem_block_num = 1;
        _rmt_tx.clk_div = 8;   // 80MHz/8 = 10MHz
    
        _rmt_tx.tx_config.loop_en = false;
        _rmt_tx.tx_config.carrier_en = false;
        _rmt_tx.tx_config.idle_output_en = true;
        _rmt_tx.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
        
        if (_rmt_tx.channel == RMT_CHANNEL_MAX) {
            ESP_LOGE("ryzobee_led", "No free RMT channel found, please check the RMT channel status");
            return YOKE_PORT_ERROR_NOT_AVAILABLE;
        }
        rmt_driver_uninstall(_rmt_tx.channel);
        rmt_config(&_rmt_tx);
        rmt_set_gpio(_rmt_tx.channel, RMT_MODE_TX, (gpio_num_t)iodata->at(YOKE_PORT_PIN_START), false);
        rmt_driver_install(_rmt_tx.channel, 0, 0);
        rmt_set_source_clk(_rmt_tx.channel, RMT_BASECLK_APB);
        rmt_tx_start(_rmt_tx.channel, true);
#endif
        _initialized = true;
        return YOKE_PORT_ERROR_NONE;
    };
    int portEnd() override{
        _initialized = false;
#if defined(ARDUINO_ARCH_ESP32)
        rmt_driver_uninstall(_rmt_tx.channel);
#endif
        return YOKE_PORT_ERROR_NONE;
    };
    std::string getYokeName() override { return "Yoke RGBW"; }
    std::string getYokeInfo() override { return "Yoke RGBW using RS PORT"; }

private:
#if defined(ARDUINO_ARCH_ESP32)
    rmt_config_t _rmt_tx;
    rmt_item32_t _rmt_items[32];
    /*  G-R-B-W 32bits */
    void _setRMTItem(uint8_t r, uint8_t g, uint8_t b, uint8_t w){
        uint32_t grbw = ((uint32_t)g << 24) | ((uint32_t)r << 16) | ((uint32_t)b << 8) | w;
        for(int i=0;i<32;i++){
            bool bit = grbw & (1 << (31-i));
            if(bit){ _rmt_items[i].level0 = 1; _rmt_items[i].duration0 = 7; _rmt_items[i].level1 = 0; _rmt_items[i].duration1 = 3;}
            else{ _rmt_items[i].level0 = 1; _rmt_items[i].duration0 = 3; _rmt_items[i].level1 = 0; _rmt_items[i].duration1 = 7; }
        }
    }
    rmt_channel_t _getFirstFreeRMTChannel(){
        rmt_channel_status_result_t status;
        rmt_get_channel_status(&status);
        for(int i=0;i<RMT_CHANNEL_MAX;i++){
            if(status.status[i] == RMT_CHANNEL_UNINIT) {
                return (rmt_channel_t)i;
            }
        }
        return RMT_CHANNEL_MAX;
    }
#endif
    void _writeLEDData(uint8_t r, uint8_t g, uint8_t b, uint8_t w){
#if defined(ARDUINO_ARCH_ESP32)
        _setRMTItem(r, g, b, w);
        rmt_write_items(_rmt_tx.channel, _rmt_items, sizeof(_rmt_items)/sizeof(_rmt_items[0]), true);
#endif
    }
};