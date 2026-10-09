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
#include <M5Utility.h>
#include <wiring/m5_unit_unified_wiring.hpp>  // Board-aware connection helpers (include last)

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

constexpr uint32_t hat_color{0xCC66FFU};  // Lavender
constexpr uint32_t unit_color{TFT_ORANGE};

bool has_lcd{};

class View {
public:
    View(LovyanGFX& dst, const int32_t maxRange, const char* label, uint32_t color, const int32_t frames = 8)
        : _label{label}, _max_value{(float)maxRange}, _frames{frames}
    {
        _sprite.setPsram(false);
        _sprite.setColorDepth(1);
        _w      = dst.width();
        _h      = dst.height() >> 1;
        has_lcd = _sprite.createSprite(_w, _h);
        _sprite.setPaletteColor(0, TFT_BLACK);
        _sprite.setPaletteColor(1, color);
        _sprite.setTextColor(1, 0);
        _sprite.fillSprite(0);
    }

    void push_back(const int32_t range)
    {
        int32_t val = std::max(std::min((int32_t)_max_value, range), (int32_t)0);
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

        if (!--_counter) {
            _now = _value;
        } else {
            _now += _inc;
        }

        _sprite.fillSprite(0);

        const int32_t m     = 3;
        const int32_t bar_h = std::max(_h / 8, (int32_t)4);
        const int32_t bar_y = _h - bar_h - m;
        const int32_t bar_w = _w - m * 2;

        // Bar gauge: outline + proportional fill
        _sprite.drawRect(m, bar_y, bar_w, bar_h, 1);
        int32_t fill = (int32_t)((bar_w - 2) * (_now / _max_value));
        if (fill > 0) {
            _sprite.fillRect(m + 1, bar_y + 1, fill, bar_h - 2, 1);
        }

        // Small labels: name (top-left), unit (top-right)
        _sprite.setFont(&fonts::Font0);
        _sprite.setTextSize(1);
        _sprite.setTextDatum(top_left);
        _sprite.drawString(_label, m + 1, m);
        _sprite.setTextDatum(top_right);
        _sprite.drawString("mm", _w - m - 1, m);

        // Large distance number (Orbitron, centered above bar)
        _sprite.setFont(&fonts::Orbitron_Light_32);
        const int32_t text_top  = 10;
        const int32_t text_area = bar_y - text_top;
        float sx                = _w / (32.0f * 4);
        float sy                = text_area / 36.0f;
        float s                 = std::min(sx, sy);
        if (s < 0.3f) {
            s = 0.3f;
        }
        _sprite.setTextSize(s);
        _sprite.setTextDatum(middle_center);
        _sprite.drawString(m5::utility::formatString("%d", (int32_t)_now).c_str(), _w / 2, text_top + text_area / 2);

        _dirty = true;
        return true;
    }

    void push(LovyanGFX* dst, const uint32_t x, const uint32_t y)
    {
        if (_dirty) {
            _sprite.pushSprite(dst, x, y);
            _dirty = false;
        }
    }

private:
    bool _dirty{};
    LGFX_Sprite _sprite{};
    const char* _label;
    float _max_value{}, _now{}, _inc{};
    int32_t _w{}, _h{}, _frames{}, _value{}, _counter{};
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

    const auto hat_pins = m5::unit::wiring::hatI2CPins();
    M5_LOGI("getHatPin: SDA:%d SCL:%d", hat_pins.sda, hat_pins.scl);
    if (hat_pins.sda < 0 || hat_pins.scl < 0) {
        M5_LOGE("Unsupported board for DualSensor");
        m5::unit::wiring::failStop();
    }

    // The screen shall be in landscape mode
    if (lcd.height() > lcd.width()) {
        lcd.setRotation(1);
    }

    // HatToF: NessoN1 uses Wire1 (Wire is used internally, GROVE is on SoftwareI2C)
    //         Other boards use SoftwareI2C (Wire is used by the unit on GROVE)
    bool hat_ready{};
    if (hat_pins.useWire1) {
        hat_ready = m5::unit::wiring::addHatI2C(Units, hat);
    } else {
        hat_ready = m5::unit::wiring::i2cSoftware(Units, hat, hat_pins.sda, hat_pins.scl);
    }

    // UnitToF/ToF4M/ToF90 on GROVE (board-aware: NessoN1 -> SoftwareI2C, others -> Wire)
    const bool unit_ready = m5::unit::wiring::addI2C(Units, unit);

    if (!hat_ready || !unit_ready || !Units.begin()) {
        M5_LOGE("Failed to begin %u/%u", hat_ready, unit_ready);
        M5_LOGE("%s", Units.debugInfo().c_str());
        m5::unit::wiring::failStop();
    }

    M5_LOGI("M5UnitUnified initialized");
    M5_LOGI("%s", Units.debugInfo().c_str());

    // Display setup (skip for no-LCD or EPD devices)
    has_lcd = lcd.width() > 0 && lcd.height() > 0 && !lcd.isEPD();
    if (has_lcd) {
        const int32_t frames = 8;
        view[0]              = new View(lcd, 2000, "HAT", hat_color, frames);
        view[1]              = new View(lcd, 2000, "UNIT", unit_color, frames);
    }
}

void loop()
{
    M5.update();
    Units.update();

    if (hat.updated()) {
        auto range = hat.range();
        if (range >= 0) {
            M5.Log.printf(">HatRange:%d\n", range);
            if (has_lcd) {
                view[0]->push_back(range);
            }
        }
    }
    if (unit.updated()) {
        auto range = unit.range();
        if (range >= 0) {
            M5.Log.printf(">UnitRange:%d\n", range);
            if (has_lcd) {
                view[1]->push_back(range);
            }
        }
    }

    if (has_lcd) {
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
}
