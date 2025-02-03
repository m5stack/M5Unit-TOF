/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
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
int16_t lastRange{};
}  // namespace

void setup()
{
    M5.begin();
    // The screen shall be in landscape mode if exists
    if (lcd.height() > lcd.width()) {
        lcd.setRotation(1);
    }

    auto pin_num_sda = M5.getPin(m5::pin_name_t::port_a_sda);
    auto pin_num_scl = M5.getPin(m5::pin_name_t::port_a_scl);
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

    lcd.setTextColor(TFT_ORANGE);
    lcd.setFont(&fonts::Orbitron_Light_32);
    lcd.setTextDatum(middle_center);
    float scale = lcd.width() / (32 * 4.0f);
    lcd.setTextSize(scale, scale);

    lcd.startWrite();
    lcd.clear();
}

void loop()
{
    M5.update();
    Units.update();

    if (unit.updated()) {
        auto range = unit.range();
        if (range > 0 && range != lastRange) {
            lcd.fillRect(0, lcd.height() / 2 - lcd.fontHeight() / 2, lcd.width(), lcd.fontHeight(), TFT_BLACK);
            lcd.drawString(m5::utility::formatString("%d", range).c_str(), lcd.width() / 2, lcd.height() / 2);
            lastRange = range;
        }
    }
}
