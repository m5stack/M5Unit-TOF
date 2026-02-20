/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example of using M5UnitUnified to connect both UnitToF and HatToF
*/
#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedTOF.h>
#include <M5HAL.hpp>
#include <Wire.h>
#include <cassert>

namespace {
auto& lcd = M5.Display;
m5::unit::UnitUnified Units;
// ------
// Please change it to the unit you use
m5::unit::UnitToF unit;
// m5::unit::UnitToF4M unit;
// m5::unit::UnitToF90 unit;
//  ------
m5::unit::HatToF hat;

struct HatPins {
    int sda, scl;
};

HatPins get_hat_pins(const m5::board_t board)
{
    switch (board) {
        case m5::board_t::board_M5StickC:
        case m5::board_t::board_M5StickCPlus:
        case m5::board_t::board_M5StickCPlus2:
            return {0, 26};
        case m5::board_t::board_M5StickS3:
            return {8, 0};
        case m5::board_t::board_M5StackCoreInk:
            return {25, 26};
        case m5::board_t::board_ArduinoNessoN1:
            return {6, 7};
        default:
            return {-1, -1};
    }
}

class View {
public:
    View(LovyanGFX& dst, const int32_t maxRange, const bool hat, const int32_t frames = 8)
        : _hat{hat}, _max_value{maxRange}, _frames{frames}
    {
        _sprite.setPsram(false);
        _sprite.setColorDepth(8);
        auto p = _sprite.createSprite(dst.width(), dst.height() >> 1);
        assert(p && "Failed to createSprite");
        _sprite.setFont(&fonts::FreeSansBold9pt7b);
        _sprite.setTextColor(TFT_WHITE);
        _sprite.setTextDatum(_hat ? textdatum_t::top_right : textdatum_t::top_left);
    }

    void push_back(const int32_t range)
    {
        int32_t val = std::max(std::min(_max_value, range), (int32_t)0);
        if (_value == val) {
            return;
        }
        _value   = val;
        _counter = _frames;
        _inc     = (_value - _now) / _counter;
    }

    bool update()
    {
        if (!_counter) {
            return false;
        }

        _sprite.clear();

        if (!--_counter) {
            _now = _value;
        } else {
            _now += _inc;
        }

        uint32_t hgt = _sprite.height() * (_now / _max_value);
        if (_hat) {
            _sprite.fillTriangle(0, _sprite.height() - hgt, 0, _sprite.height(), _sprite.width() >> 1, _sprite.height(),
                                 TFT_BLUE);

            _sprite.drawString("Hat", _sprite.width(), 8);
            _sprite.drawString(m5::utility::formatString("%4umm", (int32_t)_now).c_str(), _sprite.width(), 32);
            _sprite.drawString("<<<<    ", _sprite.width(), _sprite.height() - 16);

        } else {
            _sprite.fillTriangle(_sprite.width(), _sprite.height() - hgt, _sprite.width() >> 1, _sprite.height(),
                                 _sprite.width(), _sprite.height(), TFT_RED);

            _sprite.drawString("Unit", 0, 8);
            _sprite.drawString(m5::utility::formatString("%4umm", (int32_t)_now).c_str(), 0, 32);
            _sprite.drawString("    >>>>", 0, _sprite.height() - 16);
        }
        _dirty = true;
        return _dirty;
    }

    void push(LovyanGFX* dst, const uint32_t x, const uint32_t y)
    {
        if (_dirty) {
            _sprite.pushSprite(dst, x, y);
            _dirty = false;
        }
    }

private:
    bool _hat{}, _dirty{};
    LGFX_Sprite _sprite{};
    int32_t _max_value{}, _frames{}, _value{}, _counter{};
    float _now{}, _inc{};
};
View* view[2]{};

}  // namespace

void setup()
{
    // Configuration for HatToF
    auto m5cfg         = M5.config();
    m5cfg.pmic_button  = false;  // Disable BtnPWR
    m5cfg.internal_imu = false;  // Disable internal IMU
    m5cfg.internal_rtc = false;  // Disable internal RTC
    M5.begin(m5cfg);
    M5.setTouchButtonHeightByRatio(100);

    const auto board_type = M5.getBoard();

    const auto pins = get_hat_pins(board_type);
    M5_LOGI("getHatPin: SDA:%d SCL:%d", pins.sda, pins.scl);
    if (pins.sda < 0 || pins.scl < 0) {
        M5_LOGE("Unsupported board for DualSensor");
        lcd.fillScreen(TFT_RED);
        while (true) {
            m5::utility::delay(10000);
        }
    }

    // The screen shall be in landscape mode
    if (lcd.height() > lcd.width()) {
        lcd.setRotation(1);
    }

    // HatToF settings
    bool hat_ready{};
    if (board_type == m5::board_t::board_ArduinoNessoN1) {
        // Using Wire1
        Wire1.end();
        Wire1.begin(pins.sda, pins.scl, 400 * 1000U);
        hat_ready = Units.add(hat, Wire1);
    } else {
        // SoftwareI2C for Hat
        m5::hal::bus::I2CBusConfig hat_i2c_cfg;
        hat_i2c_cfg.pin_sda = m5::hal::gpio::getPin(pins.sda);
        hat_i2c_cfg.pin_scl = m5::hal::gpio::getPin(pins.scl);
        auto hat_i2c_bus    = m5::hal::bus::i2c::getBus(hat_i2c_cfg);
        hat_ready           = Units.add(hat, hat_i2c_bus ? hat_i2c_bus.value() : nullptr);
    }

    // UnitToF settings
    auto pin_num_sda = M5.getPin(m5::pin_name_t::port_a_sda);
    auto pin_num_scl = M5.getPin(m5::pin_name_t::port_a_scl);
    bool unit_ready{};
    if (board_type == m5::board_t::board_ArduinoNessoN1) {
        // For NessoN1 GROVE
        // Wire is used internally, so SoftwareI2C handles the unit
        pin_num_sda = M5.getPin(m5::pin_name_t::port_b_out);
        pin_num_scl = M5.getPin(m5::pin_name_t::port_b_in);
        M5_LOGI("getPin(NessoN1): SDA:%u SCL:%u", pin_num_sda, pin_num_scl);
        m5::hal::bus::I2CBusConfig i2c_cfg;
        i2c_cfg.pin_sda = m5::hal::gpio::getPin(pin_num_sda);
        i2c_cfg.pin_scl = m5::hal::gpio::getPin(pin_num_scl);
        auto i2c_bus    = m5::hal::bus::i2c::getBus(i2c_cfg);
        M5_LOGI("Bus:%d", i2c_bus.has_value());
        unit_ready = Units.add(unit, i2c_bus ? i2c_bus.value() : nullptr);
    } else {
        // UnitToF on Wire
        M5_LOGI("getPin: SDA:%u SCL:%u", pin_num_sda, pin_num_scl);
        Wire.end();
        Wire.begin(pin_num_sda, pin_num_scl, 400 * 1000U);
        unit_ready = Units.add(unit, Wire);
    }

    if (!hat_ready || !unit_ready || !Units.begin()) {
        M5_LOGE("Failed to begin %u/%u", hat_ready, unit_ready);
        M5_LOGE("%s", Units.debugInfo().c_str());
        lcd.fillScreen(TFT_RED);
        while (true) {
            m5::utility::delay(10000);
        }
    }

    M5_LOGI("M5UnitUnified has been begun");
    M5_LOGI("%s", Units.debugInfo().c_str());

    // e-ink displays cannot keep up with multi-frame animation; use 1 frame
    const int32_t frames = lcd.isEPD() ? 1 : 8;
    view[0]              = new View(lcd, 2000, true, frames);
    view[1]              = new View(lcd, 2000, false, frames);
}

void loop()
{
    M5.update();
    Units.update();

    if (hat.updated()) {
        auto range = hat.range();
        if (range >= 0) {
            view[0]->push_back(range);
        }
    }
    if (unit.updated()) {
        auto range = unit.range();
        if (range >= 0) {
            view[1]->push_back(range);
        }
    }

    uint32_t idx{};
    for (auto&& v : view) {
        lcd.startWrite();
        if (v->update()) {
            v->push(&lcd, 0, idx == 0 ? 0 : lcd.height() >> 1);
        }
        lcd.endWrite();
        ++idx;
    }
}
