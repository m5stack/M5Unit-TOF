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
#include <M5HAL.hpp>
#include <M5Utility.h>
#include <Wire.h>

// *************************************************************
// Choose one define symbol to match the unit you are using
// *************************************************************
#if !defined(USING_UNIT_TOF) && !defined(USING_UNIT_TOF4M) && !defined(USING_HAT_TOF) && !defined(USING_UNIT_TOF90)
// For UnitToF
// #define USING_UNIT_TOF
// For UnitToF4M
// #define USING_UNIT_TOF4M
// For HatToF
// #define USING_HAT_TOF
// For UnitToF90
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
constexpr uint32_t display_color{TFT_ORANGE};
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

#if defined(USING_HAT_TOF)
struct HatPins {
    int sda, scl;
};
HatPins get_hat_pins(const m5::board_t board)
{
    switch (board) {
        case m5::board_t::board_M5StickC:
        case m5::board_t::board_M5StickCPlus:
        case m5::board_t::board_M5StickCPlus2:
            return {0, 26};
        case m5::board_t::board_M5StickS3:
            return {8, 0};
        case m5::board_t::board_M5StackCoreInk:
            return {25, 26};
        case m5::board_t::board_ArduinoNessoN1:
            return {6, 7};
        default:
            return {-1, -1};
    }
}
#endif

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

    auto board = M5.getBoard();

#if defined(USING_HAT_TOF)
    const auto pins = get_hat_pins(board);
    M5_LOGI("getHatPin: SDA:%d SCL:%d", pins.sda, pins.scl);
    if (pins.sda < 0 || pins.scl < 0) {
        M5_LOGE("Unsupported board for HatToF");
        lcd.fillScreen(TFT_RED);
        while (true) {
            m5::utility::delay(10000);
        }
    }
    auto& wire = (board == m5::board_t::board_ArduinoNessoN1) ? Wire1 : Wire;
    wire.end();
    wire.begin(pins.sda, pins.scl, 400 * 1000U);
    if (!Units.add(unit, wire) || !Units.begin()) {
        M5_LOGE("Failed to begin");
        lcd.fillScreen(TFT_RED);
        while (true) {
            m5::utility::delay(10000);
        }
    }
#else
    // NessoN1: Arduino Wire (I2C_NUM_0) cannot be used for GROVE port.
    //   Wire is used by M5Unified In_I2C for internal devices (IOExpander etc.).
    //   Wire1 exists but is reserved for HatPort — cannot be used for GROVE.
    //   Reconfiguring Wire to GROVE pins breaks In_I2C, causing ESP_ERR_INVALID_STATE in M5.update().
    //   Solution: Use SoftwareI2C via M5HAL (bit-banging) for the GROVE port.
    // NanoC6: Wire.begin() on GROVE pins conflicts with m5::I2C_Class registered by Ex_I2C.setPort()
    //   on the same I2C_NUM_0, causing sporadic NACK errors.
    //   Solution: Use M5.Ex_I2C (m5::I2C_Class) directly instead of Arduino Wire.
    bool unit_ready{};
    if (board == m5::board_t::board_ArduinoNessoN1) {
        // NessoN1: GROVE is on port_b (GPIO 5/4), not port_a (which maps to Wire pins 8/10)
        auto pin_num_sda = M5.getPin(m5::pin_name_t::port_b_out);
        auto pin_num_scl = M5.getPin(m5::pin_name_t::port_b_in);
        M5_LOGI("getPin(M5HAL): SDA:%u SCL:%u", pin_num_sda, pin_num_scl);
        m5::hal::bus::I2CBusConfig i2c_cfg;
        i2c_cfg.pin_sda = m5::hal::gpio::getPin(pin_num_sda);
        i2c_cfg.pin_scl = m5::hal::gpio::getPin(pin_num_scl);
        auto i2c_bus    = m5::hal::bus::i2c::getBus(i2c_cfg);
        M5_LOGI("Bus:%d", i2c_bus.has_value());
        unit_ready = Units.add(unit, i2c_bus ? i2c_bus.value() : nullptr) && Units.begin();
    } else if (board == m5::board_t::board_M5NanoC6) {
        // NanoC6: Use M5.Ex_I2C (m5::I2C_Class, not Arduino Wire)
        M5_LOGI("Using M5.Ex_I2C");
        unit_ready = Units.add(unit, M5.Ex_I2C) && Units.begin();
    } else {
        auto pin_num_sda = M5.getPin(m5::pin_name_t::port_a_sda);
        auto pin_num_scl = M5.getPin(m5::pin_name_t::port_a_scl);
        M5_LOGI("getPin: SDA:%u SCL:%u", pin_num_sda, pin_num_scl);
        Wire.end();
        Wire.begin(pin_num_sda, pin_num_scl, 400 * 1000U);
        unit_ready = Units.add(unit, Wire) && Units.begin();
    }
    if (!unit_ready) {
        M5_LOGE("Failed to begin");
        lcd.fillScreen(TFT_RED);
        while (true) {
            m5::utility::delay(10000);
        }
    }
#endif

    M5_LOGI("M5UnitUnified has been begun");
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
                M5.Log.printf("Singleshot[%d]: >Range:%d\nStatus:%u\n", sscnt, d.range(), d.range_status());
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
                M5.Log.printf("Singleshot[%d]: >Range:%d\nStatus:%u\n", sscnt, d.range(), d.range_status());
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
        unit.softReset();
    }
}
