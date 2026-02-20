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
    auto pin_num_sda = M5.getPin(m5::pin_name_t::port_a_sda);
    auto pin_num_scl = M5.getPin(m5::pin_name_t::port_a_scl);
    // For NessoN1 GROVE
    if (board == m5::board_t::board_ArduinoNessoN1) {
        // Port A of the NessoN1 is QWIIC, then use portB (GROVE)
        pin_num_sda = M5.getPin(m5::pin_name_t::port_b_out);
        pin_num_scl = M5.getPin(m5::pin_name_t::port_b_in);
        M5_LOGI("getPin(NessoN1): SDA:%u SCL:%u", pin_num_sda, pin_num_scl);
        // Wire is used internally, so SoftwareI2C handles the unit
        m5::hal::bus::I2CBusConfig i2c_cfg;
        i2c_cfg.pin_sda = m5::hal::gpio::getPin(pin_num_sda);
        i2c_cfg.pin_scl = m5::hal::gpio::getPin(pin_num_scl);
        auto i2c_bus    = m5::hal::bus::i2c::getBus(i2c_cfg);
        M5_LOGI("Bus:%d", i2c_bus.has_value());
        if (!Units.add(unit, i2c_bus ? i2c_bus.value() : nullptr) || !Units.begin()) {
            M5_LOGE("Failed to begin");
            lcd.fillScreen(TFT_RED);
            while (true) {
                m5::utility::delay(10000);
            }
        }
    } else {
        // Using TwoWire
        M5_LOGI("getPin: SDA:%u SCL:%u", pin_num_sda, pin_num_scl);
        Wire.end();
        Wire.begin(pin_num_sda, pin_num_scl, 400 * 1000U);
        if (!Units.add(unit, Wire) || !Units.begin()) {
            M5_LOGE("Failed to begin");
            lcd.fillScreen(TFT_RED);
            while (true) {
                m5::utility::delay(10000);
            }
        }
    }
#endif

    M5_LOGI("M5UnitUnified has been begun");
    M5_LOGI("%s", Units.debugInfo().c_str());

    lcd.fillScreen(TFT_DARKGREEN);
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

    if (unit.updated() && unit.range() >= 0) {
        // Can be checked e.g. by serial plotters
        M5.Log.printf(">Range:%d\n", unit.range());
    }

    if (M5.BtnA.wasClicked()) {
        button_function();
    } else if (M5.BtnA.wasHold()) {
        M5.Log.printf("Reset!\n");
        unit.softReset();
    }
}
