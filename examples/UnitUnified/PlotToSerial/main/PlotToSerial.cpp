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
float number_scale{1.0f};  // Text size of the range
float label_scale{1.0f};   // Text size of the function state
int16_t last_range{-1};
bool redraw{true};
const char* notice{};                     // Shown instead of the range for a while (e.g. after reset)
m5::utility::elapsed_time_t notice_at{};  // When the notice was shown (ms)

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
    M5_LOGI("BtnA: click to run, double click to select the function, hold to reset");

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
        number_scale = lcd.width() / (32 * 4.0f);
        label_scale  = (lcd.width() >= 320) ? 2.0f : 1.0f;
        sprite.setTextColor(1, 0);
        sprite.fillSprite(0);
        sprite.pushSprite(&lcd, 0, 0);
    }
}

namespace {
// BtnA: single click runs the selected function, double click selects the next one, hold resets the unit
#if defined(USING_UNIT_TOF4M)
using namespace m5::unit::vl53l1x;

enum class Function : uint8_t { Periodic, Singleshot, Window, Interval, ROI };
constexpr const char* function_name[] = {"Periodic", "Singleshot", "Window", "Interval", "ROI"};

constexpr Timing tb_table[] = {
    // Timing::Budget15ms // only Short
    Timing::Budget20ms,  Timing::Budget33ms,  Timing::Budget50ms,
    Timing::Budget100ms, Timing::Budget200ms, Timing::Budget500ms,
};
constexpr const char* tb_name[] = {"20ms", "33ms", "50ms", "100ms", "200ms", "500ms"};
constexpr Window w_table[]      = {Window::Below, Window::Beyond, Window::Out, Window::In};
constexpr const char* w_name[]  = {"Below", "Beyond", "Out", "In"};
uint32_t w_idx{};        // Next window to apply
uint32_t tb_idx{};       // Next timing budget to apply
int32_t w_applied{-1};   // Applied window (-1: not changed)
int32_t tb_applied{-1};  // Applied timing budget (-1: not changed)
bool roi_small{};
#else
using namespace m5::unit::vl53l0x;

enum class Function : uint8_t { Periodic, Singleshot };
constexpr const char* function_name[] = {"Periodic", "Singleshot"};
#endif

Function function{Function::Periodic};
uint32_t ss_count{};

bool start_periodic()
{
#if defined(USING_UNIT_TOF4M)
    return unit.startPeriodicMeasurement(unit.config().distance, unit.config().timing_budget);
#else
    return unit.startPeriodicMeasurement(unit.config().mode);
#endif
}

// Selected function and its current setting
std::string function_state()
{
    switch (function) {
        case Function::Singleshot:
            return m5::utility::formatString("Singleshot #%u", ss_count);
#if defined(USING_UNIT_TOF4M)
        case Function::Window:
            return m5::utility::formatString("Window:%s", w_applied >= 0 ? w_name[w_applied] : "-");
        case Function::Interval:
            return m5::utility::formatString("Interval:%s", tb_applied >= 0 ? tb_name[tb_applied] : "-");
        case Function::ROI:
            return m5::utility::formatString("ROI:%s", roi_small ? "5x5" : "16x16");
#endif
        default:
            return function_name[m5::stl::to_underlying(function)];
    }
}

void run_function()
{
    switch (function) {
        case Function::Singleshot: {
            Data d{};
            if (unit.measureSingleshot(d)) {
                ++ss_count;
                M5.Log.printf("Singleshot[%u]: >Range:%d\nStatus:%u\n", ss_count, d.range(),
                              static_cast<unsigned>(m5::stl::to_underlying(d.range_status())));
                last_range = d.range();
            } else {
                M5_LOGE("Failed to measure");
            }
        } break;
#if defined(USING_UNIT_TOF4M)
        case Function::Window: {
            M5.Log.printf("Change Window:%u\n", static_cast<unsigned>(m5::stl::to_underlying(w_table[w_idx])));
            unit.writeDistanceThreshold(w_table[w_idx], 200 /*20cm*/, 400 /*40cm*/);
            w_applied = static_cast<int32_t>(w_idx);
            if (++w_idx >= m5::stl::size(w_table)) {
                w_idx = 0;
            }
        } break;
        case Function::Interval: {
            M5.Log.printf("Change interval %s\n", tb_name[tb_idx]);
            unit.stopPeriodicMeasurement();
            unit.startPeriodicMeasurement(Distance::Short, tb_table[tb_idx]);
            tb_applied = static_cast<int32_t>(tb_idx);
            if (++tb_idx >= m5::stl::size(tb_table)) {
                tb_idx = 0;
            }
        } break;
        case Function::ROI: {
            unit.stopPeriodicMeasurement();
            roi_small = !roi_small;
            if (roi_small) {
                unit.writeROI(5, 5);
                unit.writeROICenter(18);
                M5.Log.printf("Change ROI 5:5\n");
            } else {
                unit.writeROI(16, 16);  // default
                M5.Log.printf("Change ROI 16:16\n");
            }
            start_periodic();
        } break;
#endif
        default:  // Periodic: nothing to do
            break;
    }
    redraw = true;
}

void select_next_function()
{
    auto next = m5::stl::to_underlying(function) + 1;
    if (next >= m5::stl::size(function_name)) {
        next = 0;
    }
    const auto prev = function;
    function        = static_cast<Function>(next);

#if defined(USING_UNIT_TOF4M)
    // Settings of a function apply only while it is selected; restore the defaults when leaving it
    switch (prev) {
        case Function::Window:
            if (w_applied >= 0) {
                unit.clearDistanceThreshold();
                M5.Log.printf("Clear Window\n");
            }
            w_applied = -1;
            w_idx     = 0;
            break;
        case Function::Interval:
            if (tb_applied >= 0) {
                unit.stopPeriodicMeasurement();
                start_periodic();
                M5.Log.printf("Restore interval\n");
            }
            tb_applied = -1;
            tb_idx     = 0;
            break;
        case Function::ROI:
            if (roi_small) {
                unit.stopPeriodicMeasurement();
                unit.writeROI(16, 16);  // Also restores the center
                start_periodic();
                M5.Log.printf("Restore ROI 16:16\n");
            }
            roi_small = false;
            break;
        default:
            break;
    }
#endif
    // Singleshot measures only while periodic measurement is stopped
    if (function == Function::Singleshot) {
        unit.stopPeriodicMeasurement();
        ss_count = 0;
    } else if (prev == Function::Singleshot) {
        start_periodic();
    }
    M5.Log.printf("Function: %s\n", function_state().c_str());
    redraw = true;
}

void reset_unit()
{
    M5.Log.printf("Reset!\n");
    const bool success = unit.softReset();
    if (success) {
        ss_count = 0;
#if defined(USING_UNIT_TOF4M)
        w_applied  = -1;
        tb_applied = -1;
        roi_small  = false;
#endif
        // softReset() stops periodic measurement; keep it stopped in Singleshot
        if (function != Function::Singleshot) {
            start_periodic();
        }
        M5.Log.printf("Function: %s\n", function_state().c_str());
    } else {
        M5_LOGE("Failed to reset");
    }
    notice    = success ? "RESET" : "FAILED";
    notice_at = m5::utility::millis();
    redraw    = true;
}

void handle_button()
{
    if (M5.BtnA.wasSingleClicked()) {
        run_function();
    } else if (M5.BtnA.wasDoubleClicked()) {
        select_next_function();
    } else if (M5.BtnA.wasHold()) {
        reset_unit();
    }
}

void draw_lcd()
{
    sprite.fillSprite(0);
    sprite.setFont(&fonts::Orbitron_Light_32);
    sprite.setTextDatum(middle_center);
    sprite.setTextSize(number_scale, number_scale);
    if (notice) {
        sprite.drawString(notice, sprite.width() / 2, sprite.height() / 2);
    } else if (last_range >= 0) {
        sprite.drawString(m5::utility::formatString("%d", last_range).c_str(), sprite.width() / 2, sprite.height() / 2);
    }
    sprite.setFont(&fonts::Font0);
    sprite.setTextDatum(top_left);
    sprite.setTextSize(label_scale, label_scale);
    sprite.drawString(function_state().c_str(), 2, 2);
    sprite.pushSprite(&lcd, 0, 0);
}

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
            if (range != last_range) {
                last_range = range;
                redraw     = true;
            }
        }
    }

    handle_button();

    if (notice && m5::utility::hasElapsed(notice_at, 1000)) {
        notice = nullptr;
        redraw = true;
    }
    if (has_lcd && redraw) {
        redraw = false;
        draw_lcd();
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
