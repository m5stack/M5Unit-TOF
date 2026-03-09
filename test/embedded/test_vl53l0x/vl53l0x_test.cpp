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
#include <cmath>
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
#if defined(USING_HAT_TOF)
    virtual bool begin() override
    {
        auto board      = M5.getBoard();
        const auto pins = hat::get_hat_pins(board);
        auto& wire      = (board == m5::board_t::board_ArduinoNessoN1) ? Wire1 : Wire;
        wire.end();
        wire.begin(pins.sda, pins.scl, unit->component_config().clock);
        return Units.add(*unit, wire) && Units.begin();
    }
#endif
};

namespace {

constexpr Mode mode_table[]         = {Mode::Default, Mode::HighAccuracy, Mode::LongRange, Mode::HighSpeed};
constexpr const char* mode_string[] = {"Default", "Acc", "Long", "Speed"};

}  // namespace

TEST_F(TestVL53L0X, CheckConfig)
{
    SCOPED_TRACE(ustr);
}

TEST_F(TestVL53L0X, SignalRateLimitAndReset)
{
    SCOPED_TRACE(ustr);

    EXPECT_FALSE(unit->writeSignalRateLimit(512.f));
    EXPECT_FALSE(unit->writeSignalRateLimit(-0.000001f));

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

TEST_F(TestVL53L0X, Singleshot)
{
    SCOPED_TRACE(ustr);

    Data d{};
    EXPECT_FALSE(unit->measureSingleshot(d));
    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_FALSE(unit->inPeriodic());

    for (auto&& m : mode_table) {
        std::vector<uint16_t> results;

        SCOPED_TRACE(mode_string[m5::stl::to_underlying(m)]);

        EXPECT_TRUE(unit->writeMode(m));

        uint32_t cnt{32};
        while (cnt--) {
            EXPECT_TRUE(unit->measureSingleshot(d));
            results.push_back(d.range());
        }

        EXPECT_FALSE(results.empty());
        EXPECT_EQ(results.size(), 32U);
        if (!results.empty()) {
            uint16_t v = results.front();
            EXPECT_FALSE(std::all_of(results.begin(), results.end(), [&v](const uint16_t x) { return x == v; })) << v;
        }
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

TEST_F(TestVL53L0X, ChageI2CAddress)
{
    SCOPED_TRACE(ustr);

    uint8_t addr{};

    EXPECT_FALSE(unit->changeI2CAddress(0x07));  // Invalid
    EXPECT_FALSE(unit->changeI2CAddress(0x78));  // Invalid

    // Change to 0x10
    EXPECT_TRUE(unit->changeI2CAddress(0x10));
    EXPECT_TRUE(unit->readI2CAddress(addr));
    EXPECT_EQ(addr, 0x10);
    EXPECT_EQ(unit->address(), 0x10);

    // Change to 0x77
    EXPECT_TRUE(unit->changeI2CAddress(0x77));
    EXPECT_TRUE(unit->readI2CAddress(addr));
    EXPECT_EQ(addr, 0x77);
    EXPECT_EQ(unit->address(), 0x77);

    // Change to 0x52
    EXPECT_TRUE(unit->changeI2CAddress(0x52));
    EXPECT_TRUE(unit->readI2CAddress(addr));
    EXPECT_EQ(addr, 0x52);
    EXPECT_EQ(unit->address(), 0x52);

    // Change to default
    EXPECT_TRUE(unit->changeI2CAddress(UnitVL53L0X::DEFAULT_ADDRESS));
    EXPECT_TRUE(unit->readI2CAddress(addr));
    EXPECT_EQ(addr, +UnitVL53L0X::DEFAULT_ADDRESS);
    EXPECT_EQ(unit->address(), +UnitVL53L0X::DEFAULT_ADDRESS);
}
