/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example using M5UnitUnified for UnitToF
*/
#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedToF.h>
#include <M5Utility.h>

namespace {
auto& lcd = M5.Display;
m5::unit::UnitUnified Units;
m5::unit::UnitToF unit;
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
    constexpr int32_t BTN_A_FUNCTION{0};

    if (M5.BtnA.wasClicked()) {
        switch (BTN_A_FUNCTION) {
            case 2: {
                m5::unit::vl53l0x::Data d{};
                if (unit.measureSingleshot(d)) {
                    M5_LOGI("\nSingleshort: >Range:%d\nStatus:%u", d.range(), d.range_status());
                } else {
                    M5_LOGE("Failed to measure");
                }

            } break;

            case 1: {
                static bool p{};
                p        = !p;
                auto ret = p ? unit.startPeriodicMeasurement() : unit.stopPeriodicMeasurement();
                M5_LOGI("%s:%s", p ? "start" : "stop", ret ? "OK" : "NG");
                if (!p) {
                    m5::unit::vl53l0x::Data d{};
                    if (unit.measureSingleshot(d)) {
                        M5_LOGI("\nSingleshort: >Range:%d\nStatus:%u", d.range(), d.range_status());
                    } else {
                        M5_LOGE("Failed to measure");
                    }
                }
            } break;

            case 0: {  // Singleshot
                static uint32_t sscnt{};
                unit.stopPeriodicMeasurement();
                m5::unit::vl53l0x::Data d{};
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
            default:
                break;
        }
    } else if (M5.BtnA.wasHold()) {
        auto ret = unit.softReset();
        M5_LOGI("Reset:%s", ret ? "OK" : "NG");
    }
}
