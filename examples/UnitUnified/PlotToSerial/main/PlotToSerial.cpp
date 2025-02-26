/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example using M5UnitUnified for UnitToF/Unit/ToF4M/HatToF
*/
#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedTOF.h>
#include <M5Utility.h>

namespace {
auto& lcd = M5.Display;
m5::unit::UnitUnified Units;
#if defined(USING_UNIT_TOF)
m5::unit::UnitToF unit;
#elif defined(USING_UNIT_TOF4M)
m5::unit::UnitToF4M unit;
#elif defined(USING_HAT_TOF)
m5::unit::HatToF unit;
#else
#error Choose unit please!
#endif
}  // namespace

void setup()
{
    M5.begin();

#if defined(USING_HAT_TOF)
    Wire.begin(0, 26, 400 * 1000U);
#else
    auto pin_num_sda = M5.getPin(m5::pin_name_t::port_a_sda);
    auto pin_num_scl = M5.getPin(m5::pin_name_t::port_a_scl);
    M5_LOGI("getPin: SDA:%u SCL:%u", pin_num_sda, pin_num_scl);
    Wire.begin(pin_num_sda, pin_num_scl, 400 * 1000U);
#endif

    if (!Units.add(unit, Wire) || !Units.begin()) {
        M5_LOGE("Failed to begin");
        lcd.clear(TFT_RED);
        while (true) {
            m5::utility::delay(10000);
        }
    }

    M5_LOGI("M5UnitUnified has been begun");
    M5_LOGI("%s", Units.debugInfo().c_str());

    lcd.clear(TFT_DARKGREEN);
}

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
                M5.Log.printf("Singleshort[%d]: >Range:%d\nStatus:%u\n", sscnt, d.range(), d.range_status());
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
            M5.Log.printf("Change Window:%u\n", w_table[idx]);
            unit.writeDistanceThreshold(w_table[idx], 200 /*20cm*/, 400 /*40cm*/);
            if (++idx >= m5::stl::size(w_table)) {
                idx = 0;
            }
        } break;
        case 2: {  // Change interval
            M5.Log.printf("Change interval %u", tb_table[idx]);
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
                M5.Log.printf("Singleshort[%d]: >Range:%d\nStatus:%u\n", sscnt, d.range(), d.range_status());
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

void loop()
{
    M5.update();
    auto touch = M5.Touch.getDetail();

    Units.update();

    if (unit.updated() && unit.range() >= 0) {
        // Can be checked e.g. by serial plotters
        M5.Log.printf(">Range:%d\n", unit.range());
    }

    if (M5.BtnA.wasClicked() || touch.wasClicked()) {
        button_function();
    } else if (M5.BtnA.wasHold()) {
        M5.Log.printf("Reset!\n");
        unit.softReset();
    }
}
