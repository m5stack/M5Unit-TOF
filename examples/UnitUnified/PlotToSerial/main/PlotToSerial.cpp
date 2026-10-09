/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example using M5UnitUnified for UnitToF/UnitToF4M/HatToF/UnitToF90
*/
#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedTOF.h>
#include <M5Utility.h>
#include <wiring/m5_unit_unified_wiring.hpp>  // Board-aware connection helpers (include last)

// *************************************************************
// Choose one define symbol to match the unit you are using
// *************************************************************
#if !defined(USING_UNIT_TOF) && !defined(USING_UNIT_TOF4M) && !defined(USING_HAT_TOF) && !defined(USING_UNIT_TOF90)
// For UnitToF (U010)
// #define USING_UNIT_TOF
// For UnitToF4M (U172)
// #define USING_UNIT_TOF4M
// For HatToF (U072)
// #define USING_HAT_TOF
// For UnitToF90 (U196)
// #define USING_UNIT_TOF90
#endif

namespace {
auto& lcd = M5.Display;
m5::unit::UnitUnified Units;
#if defined(USING_UNIT_TOF)
m5::unit::UnitToF unit;
#elif defined(USING_UNIT_TOF4M)
m5::unit::UnitToF4M unit;
#elif defined(USING_HAT_TOF)
m5::unit::HatToF unit;
#elif defined(USING_UNIT_TOF90)
m5::unit::UnitToF90 unit;
#else
#error Choose unit please!
#endif

#if defined(USING_UNIT_TOF)
constexpr uint32_t display_color{0xFFB400U};  // Orange (RGB888)
#elif defined(USING_UNIT_TOF4M)
constexpr uint32_t display_color{0x00CC99U};  // Mint
#elif defined(USING_HAT_TOF)
constexpr uint32_t display_color{0xCC66FFU};  // Lavender
#elif defined(USING_UNIT_TOF90)
constexpr uint32_t display_color{0x66CCFFU};  // Sky blue
#endif

LGFX_Sprite sprite;
bool has_lcd{};
int16_t last_range{-1};

}  // namespace

void setup()
{
    auto m5cfg = M5.config();
#if defined(USING_HAT_TOF)
    m5cfg.pmic_button  = false;  // Disable BtnPWR
    m5cfg.internal_imu = false;  // Disable internal IMU
    m5cfg.internal_rtc = false;  // Disable internal RTC
#endif
    M5.begin(m5cfg);
    M5.setTouchButtonHeightByRatio(100);

    // The screen shall be in landscape mode
    if (lcd.height() > lcd.width()) {
        lcd.setRotation(1);
    }

#if defined(USING_HAT_TOF)
    // Hat header: NessoN1 uses Wire1, other boards use Wire
    const auto hat_pins = m5::unit::wiring::hatI2CPins();
    M5_LOGI("getHatPin: SDA:%d SCL:%d", hat_pins.sda, hat_pins.scl);
    if (hat_pins.sda < 0 || hat_pins.scl < 0) {
        M5_LOGE("Unsupported board for HatToF");
        m5::unit::wiring::failStop();
    }
    if (!m5::unit::wiring::addHatI2C(Units, unit) || !Units.begin()) {
        M5_LOGE("Failed to begin");
        m5::unit::wiring::failStop();
    }
#else
    // Board-aware: NessoN1 -> SoftwareI2C (GROVE on PortB), NanoC6/NanoH2 -> M5.Ex_I2C, others -> Wire
    if (!m5::unit::wiring::addI2C(Units, unit) || !Units.begin()) {
        M5_LOGE("Failed to begin");
        m5::unit::wiring::failStop();
    }
#endif

    M5_LOGI("M5UnitUnified initialized");
    M5_LOGI("%s", Units.debugInfo().c_str());

    // Display setup (skip for no-LCD or EPD devices)
    has_lcd = lcd.width() > 0 && lcd.height() > 0 && !lcd.isEPD();
    if (has_lcd) {
        sprite.setPsram(false);
        sprite.setColorDepth(1);
        has_lcd = sprite.createSprite(lcd.width(), lcd.height());
    }
    if (has_lcd) {
        sprite.setPaletteColor(0, TFT_BLACK);
        sprite.setPaletteColor(1, display_color);
        sprite.setFont(&fonts::Orbitron_Light_32);
        sprite.setTextDatum(middle_center);
        float scale = lcd.width() / (32 * 4.0f);
        sprite.setTextSize(scale, scale);
        sprite.setTextColor(1, 0);
        sprite.fillSprite(0);
        sprite.pushSprite(&lcd, 0, 0);
    }
}

namespace {
// Behavior when BtnA is clicked changes depending on the value
constexpr int32_t BTN_A_FUNCTION{0};

#if defined(USING_UNIT_TOF4M)
using namespace m5::unit::vl53l1x;
uint32_t idx{};
constexpr Timing tb_table[] = {
    // Timing::Budget15ms // only Short
    Timing::Budget20ms,  Timing::Budget33ms,  Timing::Budget50ms,
    Timing::Budget100ms, Timing::Budget200ms, Timing::Budget500ms,
};
constexpr Window w_table[] = {Window::Below, Window::Beyond, Window::Out, Window::In};

void button_function()
{
    switch (BTN_A_FUNCTION) {
        case 0: {  // Singleshot
            static uint32_t sscnt{};
            unit.stopPeriodicMeasurement();
            m5::unit::vl53l1x::Data d{};
            if (unit.measureSingleshot(d)) {
                M5.Log.printf("Singleshot[%u]: >Range:%d\nStatus:%u\n", sscnt, d.range(),
                              static_cast<unsigned>(m5::stl::to_underlying(d.range_status())));
            } else {
                M5_LOGE("Failed to measure");
            }
            // Return to periodic measurement after 8 measurements
            if (++sscnt >= 8) {
                sscnt = 0;
                unit.startPeriodicMeasurement(unit.config().distance);
            }
        } break;
        case 1: {  // Change window mode
            M5.Log.printf("Change Window:%u\n", static_cast<unsigned>(m5::stl::to_underlying(w_table[idx])));
            unit.writeDistanceThreshold(w_table[idx], 200 /*20cm*/, 400 /*40cm*/);
            if (++idx >= m5::stl::size(w_table)) {
                idx = 0;
            }
        } break;
        case 2: {  // Change interval
            M5.Log.printf("Change interval %d\n", static_cast<int>(m5::stl::to_underlying(tb_table[idx])));
            unit.stopPeriodicMeasurement();
            unit.startPeriodicMeasurement(Distance::Short, tb_table[idx]);
            if (++idx >= m5::stl::size(tb_table)) {
                idx = 0;
            }
        } break;
        case 3: {  // Change ROI
            unit.stopPeriodicMeasurement();
            if (++idx & 1) {
                unit.writeROI(5, 5);
                unit.writeROICenter(18);
                M5.Log.printf("Change ROI 5:5\n");

            } else {
                unit.writeROI(16, 16);  // default
                M5.Log.printf("Change ROI 16:16\n");
            }
            unit.startPeriodicMeasurement(unit.config().distance);

        } break;
        default:
            break;
    }
}

#else

void button_function()
{
    switch (BTN_A_FUNCTION) {
        case 0: {  // Singleshot
            static uint32_t sscnt{};
            unit.stopPeriodicMeasurement();
            m5::unit::vl53l0x::Data d{};
            if (unit.measureSingleshot(d)) {
                M5.Log.printf("Singleshot[%u]: >Range:%d\nStatus:%u\n", sscnt, d.range(),
                              static_cast<unsigned>(m5::stl::to_underlying(d.range_status())));
            } else {
                M5_LOGE("Failed to measure");
            }
            // Return to periodic measurement after 8 measurements
            if (++sscnt >= 8) {
                sscnt = 0;
                unit.startPeriodicMeasurement(unit.config().mode);
            }
        } break;
        default:
            break;
    }
}
#endif

}  // namespace

void loop()
{
    M5.update();
    Units.update();

    if (unit.updated()) {
        auto range = unit.range();
        if (range >= 0) {
            // Can be checked e.g. by serial plotters
            M5.Log.printf(">Range:%d\n", range);

            if (has_lcd && range != last_range) {
                last_range = range;
                sprite.fillSprite(0);
                sprite.drawString(m5::utility::formatString("%d", range).c_str(), sprite.width() / 2,
                                  sprite.height() / 2);
                sprite.pushSprite(&lcd, 0, 0);
            }
        }
    }

    if (M5.BtnA.wasClicked()) {
        button_function();
    } else if (M5.BtnA.wasHold()) {
        M5.Log.printf("Reset!\n");
        if (unit.softReset()) {
#if defined(USING_UNIT_TOF4M)
            unit.startPeriodicMeasurement(unit.config().distance);
#else
            unit.startPeriodicMeasurement(unit.config().mode);
#endif
        } else {
            M5_LOGE("Failed to reset");
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
