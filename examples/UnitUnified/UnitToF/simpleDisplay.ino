/*
  Example using M5UnitUnified for UnitToF (this works on ESP32-S3 based devices)
*/
#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedTOF.h>
#include <M5Utility.h>

namespace {
auto& lcd = M5.Display;
m5::unit::UnitUnified Units;
m5::unit::UnitToF unit;
}  // namespace

String lastDisplayedText = "";

void setup()
{
    M5.begin();

    auto pin_num_sda = M5.getPin(m5::pin_name_t::port_a_sda);
    auto pin_num_scl = M5.getPin(m5::pin_name_t::port_a_scl);
    
    Wire.begin(pin_num_sda, pin_num_scl, 400 * 1000U);

    Serial.begin(115200);
    Serial.println("Starting...");

    M5.Display.setTextColor(TFT_ORANGE);
    M5.Display.setTextDatum(middle_center);
    M5.Display.setTextFont(&fonts::Orbitron_Light_32);
    M5.Display.setTextSize(2);

    if (!Units.add(unit, Wire) || !Units.begin()) {
        M5_LOGE("Failed to begin");
        lcd.clear(TFT_RED);
        while (true) {
            m5::utility::delay(10000);
        }
    }

    Serial.println("M5UnitUnified has begun");
    Serial.println(Units.debugInfo().c_str());

    lcd.clear();
}

void loop()
{
    M5.update();
    Units.update();

    String range = lastDisplayedText;

    if (unit.updated()) {
        range = String(unit.range());
    }

    if (lastDisplayedText != range) {
        M5.Display.clear();
        M5.Display.drawString(range, M5.Display.width() / 2, M5.Display.height() / 2);

        lastDisplayedText = range;
    }
}
