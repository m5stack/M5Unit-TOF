/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  UnitTest for UnitVL53L0X
*/
#include <gtest/gtest.h>
#include <Wire.h>
#include <M5Unified.h>
#include <M5UnitUnified.hpp>
#include <googletest/test_template.hpp>
#include <googletest/test_helper.hpp>
#include <unit/unit_VL53L0X.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <esp_random.h>

using namespace m5::unit::googletest;
using namespace m5::unit;
using namespace m5::unit::vl53l0x;
using namespace m5::unit::vl53l0x::command;

#if defined(USING_HAT_TOF)
namespace hat {
struct I2cPins {
    int sda, scl;
};

I2cPins get_hat_pins(const m5::board_t board)
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
}  // namespace hat
#endif

namespace {
#if 0  // For debugging
void i2c_scan(TwoWire& wire)
{
    M5_LOGI("I2C scan start");
    for (uint8_t addr = 0x08; addr < 0x78; ++addr) {
        wire.beginTransmission(addr);
        if (wire.endTransmission() == 0) {
            M5_LOGI("  Found device at 0x%02X", addr);
        }
    }
    M5_LOGI("I2C scan end");
}
#endif
}  // namespace

class TestVL53L0X : public I2CComponentTestBase<UnitVL53L0X> {
protected:
    virtual UnitVL53L0X* get_instance() override
    {
        auto ptr = new m5::unit::UnitVL53L0X();
        if (ptr) {
            auto ccfg        = ptr->component_config();
            ccfg.stored_size = 4;
            ptr->component_config(ccfg);
        }
        return ptr;
    }
    virtual bool begin() override
    {
#if defined(USING_HAT_TOF)
        auto board      = M5.getBoard();
        const auto pins = hat::get_hat_pins(board);
        if (pins.sda < 0 || pins.scl < 0) {
            M5_LOGE("Unsupported board for HatToF");
            return false;
        }
        auto& wire = (board == m5::board_t::board_ArduinoNessoN1) ? Wire1 : Wire;
        wire.end();
        wire.begin(pins.sda, pins.scl, unit->component_config().clock);
        // i2c_scan(wire);
        return Units.add(*unit, wire) && Units.begin();
#else

#if 0
        // I2C scan for debugging (uses temporary Wire, then delegates to base)
        auto board = M5.getBoard();
        if (board != m5::board_t::board_ArduinoNessoN1 && board != m5::board_t::board_M5NanoC6) {
            auto pin_num_sda = M5.getPin(m5::pin_name_t::port_a_sda);
            auto pin_num_scl = M5.getPin(m5::pin_name_t::port_a_scl);
            Wire.end();
            Wire.begin(pin_num_sda, pin_num_scl, 100000);
            i2c_scan(Wire);
            Wire.end();
        }
#endif
        return I2CComponentTestBase<UnitVL53L0X>::begin();
#endif
    }
};

namespace {

constexpr Mode mode_table[]         = {Mode::Default, Mode::HighAccuracy, Mode::LongRange, Mode::HighSpeed};
constexpr const char* mode_string[] = {"Default", "Acc", "Long", "Speed"};

// Number of single-shot measurements per mode
constexpr size_t SINGLESHOT_COUNT{32};
// Tests named *WithTarget need a matte object placed at this distance in front of the sensor.
// They FAIL (not skip) without it, so that a forgotten object is noticed.
constexpr uint16_t TARGET_DISTANCE_MM{140};

// "status:range" of each measurement, for failure messages
std::string describe(const std::vector<vl53l0x::Data>& results)
{
    std::string s;
    for (auto&& r : results) {
        char buf[16]{};
        snprintf(buf, sizeof(buf), "%u:%d ", static_cast<unsigned>(m5::stl::to_underlying(r.range_status())),
                 r.range());
        s += buf;
    }
    return s;
}

std::vector<vl53l0x::Data> measure_singleshots(UnitVL53L0X* unit)
{
    std::vector<vl53l0x::Data> results;
    vl53l0x::Data data{};
    size_t cnt{SINGLESHOT_COUNT};
    while (cnt--) {
        EXPECT_TRUE(unit->measureSingleshot(data));
        results.push_back(data);
    }
    return results;
}

// Without a target: every measurement completes and the sensor reports no hardware failure
void check_singleshot_results(const std::vector<vl53l0x::Data>& results)
{
    EXPECT_EQ(results.size(), SINGLESHOT_COUNT);
    EXPECT_TRUE(std::none_of(results.begin(), results.end(), [](const vl53l0x::Data& r) {
        return r.range_status() == vl53l0x::RangeStatus::HardwareFailure;
    })) << describe(results);
}

// With a target: judged by the status, not by the value changing (a stable target can give the same value)
// - At least half of them must be valid
// - A valid measurement must not report 0 mm (it does when the object is too close or glossy; use a matte object)
void check_singleshot_results_with_target(const std::vector<vl53l0x::Data>& results)
{
    EXPECT_EQ(results.size(), SINGLESHOT_COUNT);
    const auto valid_count =
        std::count_if(results.begin(), results.end(), [](const vl53l0x::Data& r) { return r.valid(); });
    EXPECT_GE(static_cast<size_t>(valid_count) * 2, results.size())
        << "No target? Place a matte object " << TARGET_DISTANCE_MM << " mm away. " << describe(results);
    EXPECT_TRUE(
        std::all_of(results.begin(), results.end(), [](const vl53l0x::Data& r) { return !r.valid() || r.range() > 0; }))
        << "0 mm while valid: object too close or glossy? " << describe(results);
}

}  // namespace

TEST_F(TestVL53L0X, BeginAppliesConfig)
{
    SCOPED_TRACE(ustr);

    const auto cfg = unit->config();
    EXPECT_EQ(unit->inPeriodic(), cfg.start_periodic);
    EXPECT_EQ(unit->mode(), cfg.mode);
}

TEST_F(TestVL53L0X, SignalRateLimitAndReset)
{
    SCOPED_TRACE(ustr);

    EXPECT_FALSE(unit->writeSignalRateLimit(512.f));
    EXPECT_FALSE(unit->writeSignalRateLimit(-0.000001f));
    EXPECT_FALSE(unit->writeSignalRateLimit(NAN));

    uint32_t cnt{32};
    float mcps{}, tmp{};
    while (cnt--) {
        mcps   = (float)(esp_random() % 511999 + 1) / 1000.0f;
        auto s = m5::utility::formatString("MCPS:%.3f", mcps);
        SCOPED_TRACE(s);
        EXPECT_TRUE(unit->writeSignalRateLimit(mcps));
        EXPECT_TRUE(unit->readSignalRateLimit(tmp));
        EXPECT_NEAR(mcps, tmp, 0.008f);
    }

    EXPECT_TRUE(unit->softReset());
    EXPECT_TRUE(unit->readSignalRateLimit(tmp));
    // EXPECT_FLOAT_EQ(tmp, 0.0f);
    EXPECT_FLOAT_EQ(tmp, 0.25f);
}

// The reset waits for MODEL_ID (as ST VL53L0X_ResetDevice), so repeated resets must not fail
TEST_F(TestVL53L0X, RepeatedReset)
{
    SCOPED_TRACE(ustr);

    constexpr uint32_t REPEAT{20};
    for (uint32_t i = 0; i < REPEAT; ++i) {
        SCOPED_TRACE(i);
        EXPECT_TRUE(unit->stopPeriodicMeasurement());
        EXPECT_TRUE(unit->begin());
    }
    for (uint32_t i = 0; i < REPEAT; ++i) {
        SCOPED_TRACE(i);
        EXPECT_TRUE(unit->softReset());
        float mcps{};
        EXPECT_TRUE(unit->readSignalRateLimit(mcps));
        EXPECT_FLOAT_EQ(mcps, 0.25f);
    }

    Data d{};
    EXPECT_FALSE(unit->inPeriodic());
    EXPECT_TRUE(unit->measureSingleshot(d));
}

TEST_F(TestVL53L0X, Singleshot)
{
    SCOPED_TRACE(ustr);

    Data d{};
    EXPECT_FALSE(unit->measureSingleshot(d));
    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_FALSE(unit->inPeriodic());

    for (auto&& m : mode_table) {
        SCOPED_TRACE(mode_string[m5::stl::to_underlying(m)]);
        EXPECT_TRUE(unit->writeMode(m));
        EXPECT_EQ(unit->mode(), m);
        check_singleshot_results(measure_singleshots(unit.get()));
    }
}

// NOTICE: Needs a matte object placed TARGET_DISTANCE_MM in front of the sensor
TEST_F(TestVL53L0X, SingleshotWithTarget)
{
    SCOPED_TRACE(ustr);

    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    for (auto&& m : mode_table) {
        SCOPED_TRACE(mode_string[m5::stl::to_underlying(m)]);
        EXPECT_TRUE(unit->writeMode(m));
        check_singleshot_results_with_target(measure_singleshots(unit.get()));
    }
}

TEST_F(TestVL53L0X, Periodic)
{
    SCOPED_TRACE(ustr);

    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_FALSE(unit->inPeriodic());

    for (auto&& m : mode_table) {
        SCOPED_TRACE(mode_string[m5::stl::to_underlying(m)]);

        EXPECT_FALSE(unit->inPeriodic());
        EXPECT_TRUE(unit->startPeriodicMeasurement(m));
        EXPECT_TRUE(unit->inPeriodic());

        {
            auto r = collect_periodic_measurements(unit.get(), 4);
            EXPECT_FALSE(r.timed_out);
            EXPECT_EQ(r.update_count, 4U);
            EXPECT_LE(r.median(), r.expected_interval + 11);
        }

        EXPECT_TRUE(unit->stopPeriodicMeasurement());
        EXPECT_FALSE(unit->inPeriodic());

        EXPECT_TRUE(unit->full());
        EXPECT_FALSE(unit->empty());
        EXPECT_EQ(unit->available(), 4U);

        uint32_t cnt{2};
        while (unit->available() && cnt--) {
            if (unit->valid()) {
                EXPECT_GE(unit->range(), 0);
            } else {
                EXPECT_LT(unit->range(), 0);
            }
            unit->discard();
            EXPECT_FALSE(unit->empty());
            EXPECT_FALSE(unit->full());
        }

        EXPECT_EQ(unit->available(), 2U);
        unit->flush();

        EXPECT_EQ(unit->available(), 0U);
        EXPECT_FALSE(unit->full());
        EXPECT_TRUE(unit->empty());

        EXPECT_LT(unit->range(), 0);
        EXPECT_EQ(unit->range_status(), RangeStatus::Unknown);
    }
}

TEST_F(TestVL53L0X, ChangeI2CAddress)
{
    SCOPED_TRACE(ustr);

    uint8_t addr{};

    EXPECT_FALSE(unit->changeI2CAddress(0x07));  // Invalid
    EXPECT_FALSE(unit->changeI2CAddress(0x78));  // Invalid

    // Change to 0x0A (addresses unused by M5 products: docs.m5stack.com/en/product_i2c_addr)
    EXPECT_TRUE(unit->changeI2CAddress(0x0A));
    EXPECT_TRUE(unit->readI2CAddress(addr));
    EXPECT_EQ(addr, 0x0A);
    EXPECT_EQ(unit->address(), 0x0A);

    // Change to 0x74
    EXPECT_TRUE(unit->changeI2CAddress(0x74));
    EXPECT_TRUE(unit->readI2CAddress(addr));
    EXPECT_EQ(addr, 0x74);
    EXPECT_EQ(unit->address(), 0x74);

    // Change to 0x5B
    EXPECT_TRUE(unit->changeI2CAddress(0x5B));
    EXPECT_TRUE(unit->readI2CAddress(addr));
    EXPECT_EQ(addr, 0x5B);
    EXPECT_EQ(unit->address(), 0x5B);

    // Change to default
    EXPECT_TRUE(unit->changeI2CAddress(UnitVL53L0X::DEFAULT_ADDRESS));
    EXPECT_TRUE(unit->readI2CAddress(addr));
    EXPECT_EQ(addr, +UnitVL53L0X::DEFAULT_ADDRESS);
    EXPECT_EQ(unit->address(), +UnitVL53L0X::DEFAULT_ADDRESS);

    // Verify softReset restores default address
    EXPECT_TRUE(unit->changeI2CAddress(0x0A));
    EXPECT_EQ(unit->address(), 0x0A);
    EXPECT_TRUE(unit->softReset());
    EXPECT_EQ(unit->address(), +UnitVL53L0X::DEFAULT_ADDRESS);
    EXPECT_TRUE(unit->readI2CAddress(addr));
    EXPECT_EQ(addr, +UnitVL53L0X::DEFAULT_ADDRESS);
}

// begin() resets the sensor, which reverts its I2C address to the default; begin() must keep the address in use
TEST_F(TestVL53L0X, BeginAfterChangeI2CAddress)
{
    SCOPED_TRACE(ustr);

    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_TRUE(unit->changeI2CAddress(0x2A));
    EXPECT_TRUE(unit->begin());
    EXPECT_EQ(unit->address(), 0x2A);

    uint8_t addr{};
    EXPECT_TRUE(unit->readI2CAddress(addr));
    EXPECT_EQ(addr, 0x2A);

    Data d{};
    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_TRUE(unit->measureSingleshot(d));

    EXPECT_TRUE(unit->changeI2CAddress(UnitVL53L0X::DEFAULT_ADDRESS));
}
