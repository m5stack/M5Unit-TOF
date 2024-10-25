/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example using M5UnitUnified for UnitToF4M
*/
#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedTOF.h>
#include <M5Utility.h>

namespace {
auto& lcd = M5.Display;
m5::unit::UnitUnified Units;
m5::unit::UnitToF4M unit;

using m5::unit::vl53l1x::Distance;
using m5::unit::vl53l1x::Timing;
using m5::unit::vl53l1x::Window;

uint32_t idx{};
constexpr Timing tb_table[] = {
    // Timing::Budget15ms // only Short
    Timing::Budget20ms,  Timing::Budget33ms,  Timing::Budget50ms,
    Timing::Budget100ms, Timing::Budget200ms, Timing::Budget500ms,
};
constexpr Window w_table[] = {Window::Below, Window::Beyond, Window::Out, Window::In};
}  // namespace

void setup()
{
    M5.begin();

    auto pin_num_sda = M5.getPin(m5::pin_name_t::port_a_sda);
    auto pin_num_scl = M5.getPin(m5::pin_name_t::port_a_scl);
    M5_LOGI("getPin: SDA:%u SCL:%u", pin_num_sda, pin_num_scl);
    Wire.begin(pin_num_sda, pin_num_scl, 400 * 1000U);

    if (!Units.add(unit, Wire) || !Units.begin()) {
        M5_LOGE("Failed to begin");
        lcd.clear(TFT_RED);
        while (true) {
            m5::utility::delay(10000);
        }
    }

    M5_LOGI("M5UnitUnified has been begun");
    M5_LOGI("%s", Units.debugInfo().c_str());

    uint8_t w, h;
    unit.readROI(w, h);
    M5_LOGI("ROI %u,%u", w, h);

    lcd.clear(TFT_DARKGREEN);
}

void loop()
{
    M5.update();
    Units.update();
    if (unit.updated()) {
        // Can be checked e.g. by serial plotters
        M5_LOGI("\n>Range:%d\nStatus:%u", unit.range(), unit.range_status());
    }

    // Behavior when BtnA is clicked changes depending on the value.
    constexpr int32_t BTN_A_FUNCTION{-1};

    if (M5.BtnA.wasClicked()) {
        switch (BTN_A_FUNCTION) {
            case 0: {  // Singleshot
                static uint32_t sscnt{};
                unit.stopPeriodicMeasurement();
                m5::unit::vl53l1x::Data d{};
                if (unit.measureSingleshot(d)) {
                    M5_LOGI("\nSingleshort[%d]: >Range:%d\nStatus:%u", sscnt, d.range(), d.range_status());
                } else {
                    M5_LOGE("Failed to measure");
                }
                // Return to periodic measurement after 8 measurements
                if (++sscnt >= 8) {
                    sscnt = 0;
                    unit.startPeriodicMeasurement();
                }
            } break;
            case 1: {  // Change window mode
                M5_LOGW("Window:%u", w_table[idx]);
                unit.writeDistanceThreshold(w_table[idx], 200 /*20cm*/, 400 /*40cm*/);
                if (++idx >= m5::stl::size(w_table)) {
                    idx = 0;
                }
            } break;
            case 2: {  // Change interval
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
                    M5_LOGI("ROI 5:5");

                } else {
                    unit.writeROI(16, 16);  // default
                    M5_LOGI("ROI 16:16");
                }
                unit.startPeriodicMeasurement();

            } break;
            default:
                break;
        }
    }
}
