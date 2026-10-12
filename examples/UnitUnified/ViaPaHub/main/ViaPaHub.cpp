/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example of using UnitToF and UnitToF4M via UnitPaHub (PaHub2)

  Device ---> PaHub2 (0x70)
                |- ch:0 UnitToF   (0x29)
                `- ch:1 UnitToF4M (0x29)

  UnitToF / UnitToF4M / UnitToF90 all use the same address (0x29).
  Each PaHub channel is an independent I2C bus, so they can be used together without changing the address.
  UnitToF90 can be connected in the same way (m5::unit::UnitToF90 on a free channel).
*/
#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedHUB.h>
#include <M5UnitUnifiedTOF.h>
#include <M5Utility.h>
#include <wiring/m5_unit_unified_wiring.hpp>  // Board-aware connection helpers (include last)

namespace {
auto& lcd = M5.Display;
m5::unit::UnitUnified Units;

// The default address of PaHub2 is 0x70 (changeable with the A0-A2 pins (v2.0) or the DIP switch (v2.1))
// https://docs.m5stack.com/en/unit/pahub2
m5::unit::UnitPaHub2 hub{};

m5::unit::UnitToF tof{};
m5::unit::UnitToF4M tof4m{};

// PaHub channels used by each unit
constexpr uint8_t CH_TOF{0};
constexpr uint8_t CH_TOF4M{1};

constexpr uint32_t tof_color{0xFFB400U};    // Orange (RGB888)
constexpr uint32_t tof4m_color{0x00CC99U};  // Mint

bool has_lcd{};

class View {
public:
    View(LovyanGFX& dst, const int32_t maxRange, const char* label, const uint32_t color, const int32_t frames = 8)
        : _label{label}, _max_value{(float)maxRange}, _frames{frames}
    {
        _sprite.setPsram(false);
        _sprite.setColorDepth(1);
        _w     = dst.width();
        _h     = dst.height() >> 1;
        _valid = _sprite.createSprite(_w, _h);
        if (_valid) {
            _sprite.setPaletteColor(0, TFT_BLACK);
            _sprite.setPaletteColor(1, color);
            _sprite.setTextColor(1, 0);
            _sprite.fillSprite(0);
        }
    }

    bool valid() const
    {
        return _valid;
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
    bool _valid{};
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
    M5.begin();
    M5.setTouchButtonHeightByRatio(100);
    // The screen shall be in landscape mode
    if (lcd.height() > lcd.width()) {
        lcd.setRotation(1);
    }

    if (!hub.add(tof, CH_TOF) || !hub.add(tof4m, CH_TOF4M)) {
        M5_LOGE("Failed to add");
        m5::unit::wiring::failStop();
    }

    // The hub is the I2C device on the port
    // NessoN1 -> SoftwareI2C (M5HAL), NanoC6 / NanoH2 -> M5.Ex_I2C, others -> Wire
    if (!m5::unit::wiring::addI2C(Units, hub) || !Units.begin()) {
        M5_LOGE("Failed to begin");
        M5_LOGW("%s", Units.debugInfo().c_str());
        m5::unit::wiring::failStop();
    }

    M5_LOGI("M5UnitUnified initialized");
    M5_LOGI("%s", Units.debugInfo().c_str());

    // Display setup (skip for no-LCD or EPD devices)
    has_lcd = lcd.width() > 0 && lcd.height() > 0 && !lcd.isEPD();
    if (has_lcd) {
        const int32_t frames = 8;
        view[0]              = new View(lcd, 2000, "ToF", tof_color, frames);
        view[1]              = new View(lcd, 4000, "ToF4M", tof4m_color, frames);
        has_lcd              = view[0]->valid() && view[1]->valid();
    }
}

void loop()
{
    M5.update();
    Units.update();

    // Can be checked e.g. by serial plotters
    if (tof.updated()) {
        const auto range = tof.range();
        if (range >= 0) {
            M5.Log.printf(">ToF:%d\n", range);
            if (has_lcd) {
                view[0]->push_back(range);
            }
        }
    }
    if (tof4m.updated()) {
        const auto range = tof4m.range();
        if (range >= 0) {
            M5.Log.printf(">ToF4M:%d\n", range);
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

#if !defined(ARDUINO)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>

#if CONFIG_FREERTOS_UNICORE
// Single-core SoCs: run loop() back-to-back, but every ~2 s yield a 5 ms slice so the IDLE task
// runs and feeds the task watchdog (default 5 s).
static inline void feedIdleTaskPeriodically(void)
{
    constexpr uint32_t FEED_INTERVAL_MS{2000};
    constexpr TickType_t FEED_SLEEP_TICKS{pdMS_TO_TICKS(5)};
    static uint32_t s_last_feed_ms{};
    const uint32_t now_ms{static_cast<uint32_t>(esp_timer_get_time() / 1000)};
    if (now_ms - s_last_feed_ms >= FEED_INTERVAL_MS) {
        s_last_feed_ms = now_ms;
        vTaskDelay(FEED_SLEEP_TICKS);
    }
}
#endif

extern "C" void app_main(void)
{
    setup();
    for (;;) {
#if CONFIG_FREERTOS_UNICORE
        feedIdleTaskPeriodically();
#endif
        loop();
    }
}
#endif
