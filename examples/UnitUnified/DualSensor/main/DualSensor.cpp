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
#include <Wire.h>
#include <cassert>

namespace {
auto& lcd = M5.Display;
m5::unit::UnitUnified Units;
m5::unit::UnitToF unit;
m5::unit::HatToF hat;

class View {
public:
    View(LovyanGFX& dst, const int32_t maxRange, const bool hat) : _hat{hat}, _max_value{maxRange}
    {
        _sprite.setPsram(false);
        auto p = _sprite.createSprite(dst.width(), dst.height() >> 1);
        assert(p && "Failed to createSprite");
        _sprite.setFont(&fonts::FreeSansBold9pt7b);
        _sprite.setTextColor(TFT_WHITE);
        _sprite.setTextDatum(_hat ? textdatum_t::top_right : textdatum_t::top_left);
    }

    void push_back(const int32_t range)
    {
        int32_t val = std::max(std::min(_max_value, range), 0);
        if (_value == val) {
            return;
        }
        _value   = val;
        _counter = 8;
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
    int32_t _max_value{}, _value{}, _counter{};
    float _now{}, _inc{};
};
View* view[2]{};

}  // namespace

void setup()
{
    // Configuration for using Wire1
    auto m5cfg         = M5.config();
    m5cfg.pmic_button  = false;  // Disable BtnPWR
    m5cfg.internal_imu = false;  // Disable internal IMU
    m5cfg.internal_rtc = false;  // Disable internal RTC
    M5.begin(m5cfg);

    auto board = M5.getBoard();
    if (board != m5::board_t::board_M5StickCPlus && board != m5::board_t::board_M5StickCPlus2) {
        M5_LOGE("Example for StickCPlus/CPlus2");
        lcd.clear(TFT_RED);
        while (true) {
            m5::utility::delay(10000);
        }
    }

    // The screen shall be in portrait mode
    if (lcd.height() < lcd.width()) {
        lcd.setRotation(0);
    }

    // Wire settings
    auto pin_num_sda = M5.getPin(m5::pin_name_t::port_a_sda);
    auto pin_num_scl = M5.getPin(m5::pin_name_t::port_a_scl);
    Wire.end();
    Wire.begin(pin_num_sda, pin_num_scl, 400 * 1000U);

    Wire1.end();
    Wire1.begin(0, 26, 400 * 1000U);

    // UnitToF connected to GROOVE with Wire
    // HatToF connected to PIN sockect with Wire1
    if (!Units.add(unit, Wire) || !Units.add(hat, Wire1) || !Units.begin()) {
        M5_LOGE("Failed to begin");
        lcd.clear(TFT_RED);
        while (true) {
            m5::utility::delay(10000);
        }
    }

    M5_LOGI("M5UnitUnified has been begun");
    M5_LOGI("%s", Units.debugInfo().c_str());

    //
    view[0] = new View(lcd, 2000, true);
    view[1] = new View(lcd, 2000, false);

    lcd.startWrite();
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
        if (v->update()) {
            v->push(&lcd, 0, idx == 0 ? 0 : lcd.height() >> 1);
        }
        ++idx;
    }
}
