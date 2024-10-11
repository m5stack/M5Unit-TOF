/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  UnitTest for UnitVL53L1X
*/
#include <gtest/gtest.h>
#include <Wire.h>
#include <M5Unified.h>
#include <M5UnitUnified.hpp>
#include <googletest/test_template.hpp>
#include <googletest/test_helper.hpp>
#include <unit/unit_VL53L1X.hpp>
#include <cmath>
#include <random>

using namespace m5::unit::googletest;
using namespace m5::unit;
using namespace m5::unit::vl53l1x;
using namespace m5::unit::vl53l1x::command;

const ::testing::Environment* global_fixture = ::testing::AddGlobalTestEnvironment(new GlobalFixture<400000U>());

struct TestParams {
    bool hal;
    bool store_on_change;
};

class TestVL53L1X : public ComponentTestBase<UnitVL53L1X, bool> {
protected:
    virtual UnitVL53L1X* get_instance() override
    {
        auto ptr = new m5::unit::UnitVL53L1X();
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

// INSTANTIATE_TEST_SUITE_P(ParamValues, TestMAX30100,
//                         ::testing::Values(false, true));
// INSTANTIATE_TEST_SUITE_P(ParamValues, TestMAX30100,
// ::testing::Values(true));
INSTANTIATE_TEST_SUITE_P(ParamValues, TestVL53L1X, ::testing::Values(false));

namespace {
auto rng                            = std::default_random_engine{};
constexpr Distance distance_table[] = {
    Distance::Short,
    Distance::Long,
};
constexpr Timing tb_table[] = {
    Timing::Budget15ms,  Timing::Budget20ms,  Timing::Budget33ms,  Timing::Budget50ms,
    Timing::Budget100ms, Timing::Budget200ms, Timing::Budget500ms,
};

constexpr Window window_table[] = {Window::Below, Window::Beyond, Window::Out, Window::In, Window::Regular};

}  // namespace

TEST_P(TestVL53L1X, CheckConfig)
{
}

TEST_P(TestVL53L1X, Offset)
{
    SCOPED_TRACE(ustr);

    int16_t offset{}, offset2{};
    // NOTICE: The presence of detectable objects in the vicinity of the specified distance!
    EXPECT_TRUE(unit->calibrateOffset(offset, 140));
    EXPECT_TRUE(unit->readOffset(offset2));
    EXPECT_EQ(offset, offset2);

    EXPECT_FALSE(unit->writeOffset(-1025));
    EXPECT_TRUE(unit->writeOffset(-1024));
    EXPECT_TRUE(unit->readOffset(offset2));
    EXPECT_EQ(-1024, offset2);

    EXPECT_TRUE(unit->writeOffset(0));
    EXPECT_TRUE(unit->readOffset(offset2));
    EXPECT_EQ(0, offset2);

    EXPECT_TRUE(unit->writeOffset(-1));
    EXPECT_TRUE(unit->readOffset(offset2));
    EXPECT_EQ(-1, offset2);

    EXPECT_TRUE(unit->writeOffset(1));
    EXPECT_TRUE(unit->readOffset(offset2));
    EXPECT_EQ(1, offset2);

    EXPECT_FALSE(unit->writeOffset(1024));
    EXPECT_TRUE(unit->writeOffset(1023));
    EXPECT_TRUE(unit->readOffset(offset2));
    EXPECT_EQ(1023, offset2);
}

TEST_P(TestVL53L1X, Xtalk)
{
    SCOPED_TRACE(ustr);

    uint16_t xtalk{}, xtalk2{};
    // NOTICE: The presence of detectable objects in the vicinity of the specified distance!
    EXPECT_TRUE(unit->calibrateXtalk(xtalk, 140));
    EXPECT_TRUE(unit->readXtalk(xtalk2));
    EXPECT_EQ((xtalk << 9) / 1000, (xtalk2 << 9) / 1000) << xtalk << "/" << xtalk2;

    uint32_t cnt{32};
    while (cnt--) {
        uint16_t x = rng() & 0xFFFF;
        EXPECT_TRUE(unit->writeXtalk(x));
        EXPECT_TRUE(unit->readXtalk(xtalk2));
        EXPECT_EQ((x << 9) / 1000, (xtalk2 << 9) / 1000) << x;
    }
    EXPECT_TRUE(unit->writeXtalk(0));
    EXPECT_TRUE(unit->readXtalk(xtalk2));
    EXPECT_EQ(0, xtalk2);
}

TEST_P(TestVL53L1X, DistanceAndTimingBudget)
{
    SCOPED_TRACE(ustr);

    // distance -> tb loop
    for (auto&& d : distance_table) {
        Distance dd{};
        EXPECT_TRUE(unit->writeDistanceMode(d));
        EXPECT_TRUE(unit->readDistanceMode(dd));
        EXPECT_EQ(d, dd);
        EXPECT_EQ(d, unit->distanceMode());
        for (auto&& tb : tb_table) {
            Timing tb2{};
            if (d == Distance::Long && tb == Timing::Budget15ms) {
                EXPECT_FALSE(unit->writeTimingBudget(tb));
                EXPECT_TRUE(unit->readTimingBudget(tb2));
                EXPECT_NE(tb, tb2);
                EXPECT_NE(tb, unit->timingBudget());
            } else {
                EXPECT_TRUE(unit->writeTimingBudget(tb));
                EXPECT_TRUE(unit->readTimingBudget(tb2));
                EXPECT_EQ(tb, tb2);
                EXPECT_EQ(tb, unit->timingBudget());
            }
        }
    }

    // tb -> distance loop;
    for (auto&& tb : tb_table) {
        Timing tb2{};
        // to short
        EXPECT_TRUE(unit->writeDistanceMode(Distance::Short));
        // write TB
        EXPECT_TRUE(unit->writeTimingBudget(tb));
        EXPECT_TRUE(unit->readTimingBudget(tb2));
        EXPECT_EQ(tb, tb2);
        EXPECT_EQ(tb, unit->timingBudget());

        // change distance
        for (auto&& d : distance_table) {
            Distance dd{};
            if (d == Distance::Long && tb == Timing::Budget15ms) {
                EXPECT_FALSE(unit->writeDistanceMode(d));
                EXPECT_NE(unit->distanceMode(), d);
            } else {
                EXPECT_TRUE(unit->writeDistanceMode(d));
                EXPECT_TRUE(unit->readDistanceMode(dd));
                EXPECT_EQ(d, dd);
                EXPECT_EQ(d, unit->distanceMode());
            }
        }
    }

    // IMP
    {
        uint32_t cnt{32};
        while (cnt--) {
            uint16_t ms = rng() % 0xFFFF;
            EXPECT_TRUE(unit->writeInterMeasurementPeriod(ms)) << ms;
            uint16_t v{};
            EXPECT_TRUE(unit->readInterMeasurementPeriod(v)) << ms;
            EXPECT_EQ(ms, v);
        }

        uint16_t v{};
        EXPECT_TRUE(unit->writeInterMeasurementPeriod(0));
        EXPECT_TRUE(unit->readInterMeasurementPeriod(v));
        EXPECT_EQ(v, 0);
    }
}

TEST_P(TestVL53L1X, Reset)
{
    SCOPED_TRACE(ustr);

    //
    EXPECT_TRUE(unit->inPeriodic());
    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_FALSE(unit->inPeriodic());

    EXPECT_TRUE(unit->writeDistanceMode(Distance::Short));
    EXPECT_TRUE(unit->writeTimingBudget(Timing::Budget15ms));
    EXPECT_TRUE(unit->writeInterMeasurementPeriod(987));
    EXPECT_TRUE(unit->writeOffset(123));
    EXPECT_TRUE(unit->writeXtalk(456));

    EXPECT_TRUE(unit->softReset());  // DO!

    Distance d{};
    Timing tb{};
    int16_t offset{};
    uint16_t xtalk{}, ms{};
    EXPECT_TRUE(unit->readDistanceMode(d));
    EXPECT_TRUE(unit->readTimingBudget(tb));
    EXPECT_TRUE(unit->readInterMeasurementPeriod(ms));
    EXPECT_TRUE(unit->readOffset(offset));
    EXPECT_TRUE(unit->readXtalk(xtalk));

    EXPECT_NE(d, Distance::Short);
    EXPECT_NE(tb, Timing::Budget15ms);
    EXPECT_NE(987, ms);
    EXPECT_NE(123, offset);
    EXPECT_NE(456U, xtalk);

    // reset in periodic
    EXPECT_TRUE(unit->writeDistanceMode(Distance::Short));
    EXPECT_TRUE(unit->writeTimingBudget(Timing::Budget15ms));
    EXPECT_TRUE(unit->writeInterMeasurementPeriod(987));
    EXPECT_TRUE(unit->writeOffset(123));
    EXPECT_TRUE(unit->writeXtalk(456));
    EXPECT_TRUE(unit->startPeriodicMeasurement());
    EXPECT_TRUE(unit->inPeriodic());

    EXPECT_TRUE(unit->softReset());  // DO!

    EXPECT_FALSE(unit->inPeriodic());
    EXPECT_TRUE(unit->readDistanceMode(d));
    EXPECT_TRUE(unit->readTimingBudget(tb));
    EXPECT_TRUE(unit->readInterMeasurementPeriod(ms));
    EXPECT_TRUE(unit->readOffset(offset));
    EXPECT_TRUE(unit->readXtalk(xtalk));

    EXPECT_NE(d, Distance::Short);
    EXPECT_NE(tb, Timing::Budget15ms);
    EXPECT_NE(987, ms);
    EXPECT_NE(123, offset);
    EXPECT_NE(456U, xtalk);
}

TEST_P(TestVL53L1X, SingleShot)
{
    SCOPED_TRACE(ustr);

    Data d{};

    EXPECT_FALSE(unit->measureSingleshot(d));
    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_FALSE(unit->inPeriodic());

    for (auto&& d ::distance_table) {
        std::vector<uint16_t> results;
        SCOPED_TRACE(d == Distance::Long ? "Long" : "Short");

        EXPECT_TRUE(unit->writeDistanceMode(d));

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

TEST_P(TestVL53L1X, ROI)
{
    SCOPED_TRACE(ustr);

    for (uint8_t h = 3; h <= 17; ++h) {
        for (uint8_t w = 3; w <= 17; ++w) {
            if (h < 4 || h > 16 || w < 4 || w > 16) {
                EXPECT_FALSE(unit->writeROI(w, h));
                continue;
            }
            uint8_t c = rng() & 0xFF;
            EXPECT_TRUE(unit->writeROICenter(c));
            uint8_t c2{};
            EXPECT_TRUE(unit->readROICenter(c2));
            EXPECT_EQ(c, c2);

            EXPECT_TRUE(unit->writeROI(w, h));
            uint8_t w2{}, h2{};
            EXPECT_TRUE(unit->readROI(w2, h2));
            EXPECT_EQ(w, w2);
            EXPECT_EQ(h, h2);

            EXPECT_TRUE(unit->readROICenter(c2));
            if (w > 10 || h > 10) {
                EXPECT_EQ(c2, 199);
            } else {
                EXPECT_EQ(c, c2);
            }
        }
    }
}

TEST_P(TestVL53L1X, Window)
{
    SCOPED_TRACE(ustr);

    for (auto&& w : window_table) {
        uint16_t low{}, high{};
        do {
            low  = rng() & 0xFFFF;
            high = rng() & 0xFFFF;
        } while (!(high > low && low > 0 && high > 0));

        EXPECT_TRUE(unit->writeDistanceThreshold(w, low, high));
        Window w2{};
        uint16_t low2{}, high2{};
        EXPECT_TRUE(unit->readDistanceThresholdWindow(w2));
        EXPECT_TRUE(unit->readDistanceThresholdLow(low2));
        EXPECT_TRUE(unit->readDistanceThresholdHigh(high2));

        EXPECT_EQ(w, w2);
        EXPECT_EQ(low, low2);
        EXPECT_EQ(high, high2);

        std::swap(high, low);
        EXPECT_FALSE(unit->writeDistanceThreshold(w, low, high));

        EXPECT_TRUE(unit->clearDistanceThreshold());
        EXPECT_TRUE(unit->readDistanceThresholdWindow(w2));
        EXPECT_TRUE(unit->readDistanceThresholdLow(low2));
        EXPECT_TRUE(unit->readDistanceThresholdHigh(high2));
        EXPECT_EQ(w2, Window::Regular);
        EXPECT_EQ(low2, 0U);
        EXPECT_EQ(high2, 0U);
    }
}

TEST_P(TestVL53L1X, Periodic)
{
    SCOPED_TRACE(ustr);

    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_FALSE(unit->inPeriodic());

    for (auto&& d : distance_table) {
        for (auto&& tb : tb_table) {
            auto s = m5::utility::formatString("Dis:%u TB:%u", d, tb);
            SCOPED_TRACE(s.c_str());
            if (d == Distance::Long && tb == Timing::Budget15ms) {
                continue;
            }

            EXPECT_FALSE(unit->inPeriodic());
            EXPECT_TRUE(unit->startPeriodicMeasurement(d, tb));
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
            EXPECT_EQ(unit->range_status(), RangeStatus::Unknown255);
        }
    }
}

TEST_P(TestVL53L1X, ChageI2CAddress)
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
    EXPECT_TRUE(unit->changeI2CAddress(UnitVL53L1X::DEFAULT_ADDRESS));
    EXPECT_TRUE(unit->readI2CAddress(addr));
    EXPECT_EQ(addr, +UnitVL53L1X::DEFAULT_ADDRESS);
    EXPECT_EQ(unit->address(), +UnitVL53L1X::DEFAULT_ADDRESS);
}
