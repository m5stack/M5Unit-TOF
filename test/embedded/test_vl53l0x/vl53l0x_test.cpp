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
#include <random>

using namespace m5::unit::googletest;
using namespace m5::unit;
using namespace m5::unit::vl53l0x;
using namespace m5::unit::vl53l0x::command;

const ::testing::Environment* global_fixture = ::testing::AddGlobalTestEnvironment(new GlobalFixture<100000U>());

class TestVL53L0X : public ComponentTestBase<UnitVL53L0X, bool> {
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
    virtual bool is_using_hal() const override
    {
        return GetParam();
    };
};

// INSTANTIATE_TEST_SUITE_P(ParamValues, TestVL53L0X, ::testing::Values(false, true));
// INSTANTIATE_TEST_SUITE_P(ParamValues, TestVL53L0X, ::testing::Values(true));
INSTANTIATE_TEST_SUITE_P(ParamValues, TestVL53L0X, ::testing::Values(false));

namespace {
auto rng = std::default_random_engine{};

constexpr Mode mode_table[]         = {Mode::Default, Mode::HighAccuracy, Mode::LongRange, Mode::HighSpeed};
constexpr const char* mode_string[] = {"Default", "Acc", "Long", "Speed"};

}  // namespace

TEST_P(TestVL53L0X, CheckConfig)
{
    SCOPED_TRACE(ustr);
}

TEST_P(TestVL53L0X, SignalRateLimitAndReset)
{
    SCOPED_TRACE(ustr);

    std::uniform_real_distribution<> dist(0.0f, 512.0f);

    EXPECT_FALSE(unit->writeSignalRateLimit(512.f));
    EXPECT_FALSE(unit->writeSignalRateLimit(-0.000001f));

    uint32_t cnt{32};
    float mcps{}, tmp{};
    while (cnt--) {
        mcps = dist(rng);
        EXPECT_TRUE(unit->writeSignalRateLimit(mcps));
        EXPECT_TRUE(unit->readSignalRateLimit(tmp));
        EXPECT_NEAR(mcps, tmp, 0.0075f);
    }

    EXPECT_TRUE(unit->softReset());
    EXPECT_TRUE(unit->readSignalRateLimit(tmp));
    EXPECT_FLOAT_EQ(tmp, 0.0f);
}

TEST_P(TestVL53L0X, Singleshot)
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

TEST_P(TestVL53L0X, Periodic)
{
    SCOPED_TRACE(ustr);

    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_FALSE(unit->inPeriodic());

    for (auto&& m : mode_table) {
        SCOPED_TRACE(mode_string[m5::stl::to_underlying(m)]);

        EXPECT_FALSE(unit->inPeriodic());
        EXPECT_TRUE(unit->startPeriodicMeasurement(m));
        EXPECT_TRUE(unit->inPeriodic());

        test_periodic_measurement(unit.get(), 4, 11);

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

TEST_P(TestVL53L0X, ChageI2CAddress)
{
    SCOPED_TRACE(ustr);

    uint8_t addr{};

    EXPECT_FALSE(unit->changeI2CAddress(0x07));  // Invalid
    EXPECT_FALSE(unit->changeI2CAddress(0x78));  // Invalid

    // Change to 0x08
    EXPECT_TRUE(unit->changeI2CAddress(0x08));
    EXPECT_TRUE(unit->readI2CAddress(addr));
    EXPECT_EQ(addr, 0x08);
    EXPECT_EQ(unit->address(), 0x08);

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
