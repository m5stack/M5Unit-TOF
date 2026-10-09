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
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <esp_random.h>

using namespace m5::unit::googletest;
using namespace m5::unit;
using namespace m5::unit::vl53l1x;
using namespace m5::unit::vl53l1x::command;

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

class TestVL53L1X : public I2CComponentTestBase<UnitVL53L1X> {
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
    virtual bool begin() override
    {
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
        return I2CComponentTestBase<UnitVL53L1X>::begin();
    }
};

namespace {
constexpr Distance distance_table[] = {
    Distance::Short,
    Distance::Long,
};
constexpr Timing tb_table[] = {
    Timing::Budget15ms,  Timing::Budget20ms,  Timing::Budget33ms,  Timing::Budget50ms,
    Timing::Budget100ms, Timing::Budget200ms, Timing::Budget500ms,
};

constexpr Window window_table[] = {Window::Below, Window::Beyond, Window::Out, Window::In, Window::Regular};

// Number of single-shot measurements per mode
constexpr size_t SINGLESHOT_COUNT{32};
// Tests named *WithTarget need a matte object placed at this distance in front of the sensor.
// They FAIL (not skip) without it, so that a forgotten object is noticed.
// Distance from the sensor face. E.g. with the unit lying on the desk, put the object on two M5 Cores stacked on
// each side (2 x 54 mm = 108 mm above the desk, about 100 mm from the sensor face)
constexpr uint16_t TARGET_DISTANCE_MM{100};

// "status:range" of each measurement, for failure messages
std::string describe(const std::vector<vl53l1x::Data>& results)
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

std::vector<vl53l1x::Data> measure_singleshots(UnitVL53L1X* unit)
{
    std::vector<vl53l1x::Data> results;
    vl53l1x::Data data{};
    size_t cnt{SINGLESHOT_COUNT};
    while (cnt--) {
        EXPECT_TRUE(unit->measureSingleshot(data));
        results.push_back(data);
    }
    return results;
}

// Without a target: every measurement completes and the sensor reports no hardware failure
void check_singleshot_results(const std::vector<vl53l1x::Data>& results)
{
    EXPECT_EQ(results.size(), SINGLESHOT_COUNT);
    EXPECT_TRUE(std::none_of(results.begin(), results.end(), [](const vl53l1x::Data& r) {
        return r.range_status() == vl53l1x::RangeStatus::HardwareFailure;
    })) << describe(results);
}

// With a target: judged by the status, not by the value changing (a stable target can give the same value)
// - At least half of them must be valid
// - A valid measurement must not report 0 mm (it does when the object is too close or glossy; use a matte object)
void check_singleshot_results_with_target(const std::vector<vl53l1x::Data>& results)
{
    EXPECT_EQ(results.size(), SINGLESHOT_COUNT);
    const auto valid_count =
        std::count_if(results.begin(), results.end(), [](const vl53l1x::Data& r) { return r.valid(); });
    EXPECT_GE(static_cast<size_t>(valid_count) * 2, results.size())
        << "No target? Place a matte object " << TARGET_DISTANCE_MM << " mm away. " << describe(results);
    EXPECT_TRUE(
        std::all_of(results.begin(), results.end(), [](const vl53l1x::Data& r) { return !r.valid() || r.range() > 0; }))
        << "0 mm while valid: object too close or glossy? " << describe(results);
}

}  // namespace

TEST_F(TestVL53L1X, BeginAppliesConfig)
{
    SCOPED_TRACE(ustr);

    const auto cfg = unit->config();
    EXPECT_EQ(unit->inPeriodic(), cfg.start_periodic);

    Distance d{};
    EXPECT_TRUE(unit->readDistanceMode(d));
    EXPECT_EQ(d, cfg.distance);
    EXPECT_EQ(unit->distanceMode(), cfg.distance);

    Timing tb{};
    EXPECT_TRUE(unit->readTimingBudget(tb));
    EXPECT_EQ(tb, cfg.timing_budget);
    EXPECT_EQ(unit->timingBudget(), cfg.timing_budget);
}

TEST_F(TestVL53L1X, Offset)
{
    SCOPED_TRACE(ustr);

    int16_t offset2{};
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

TEST_F(TestVL53L1X, Xtalk)
{
    SCOPED_TRACE(ustr);

    uint16_t xtalk2{};
    uint32_t cnt{32};
    while (cnt--) {
        uint16_t x = esp_random() & 0xFFFF;
        auto s     = m5::utility::formatString("Xtalk:0x%04X", x);
        SCOPED_TRACE(s);
        EXPECT_TRUE(unit->writeXtalk(x));
        EXPECT_TRUE(unit->readXtalk(xtalk2));
        EXPECT_EQ((x << 9) / 1000, (xtalk2 << 9) / 1000) << x;
    }
    EXPECT_TRUE(unit->writeXtalk(0));
    EXPECT_TRUE(unit->readXtalk(xtalk2));
    EXPECT_EQ(0, xtalk2);
}

// NOTICE: Needs a matte object placed TARGET_DISTANCE_MM in front of the sensor
TEST_F(TestVL53L1X, CalibrationWithTarget)
{
    SCOPED_TRACE(ustr);

    // calibrateOffset() / calibrateXtalk() do not check the range status, so they also "succeed" without a target.
    // Check the target first so that a missing object fails here.
    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    check_singleshot_results_with_target(measure_singleshots(unit.get()));

    int16_t offset{}, offset2{};
    EXPECT_TRUE(unit->calibrateOffset(offset, TARGET_DISTANCE_MM))
        << "No target? Place a matte object " << TARGET_DISTANCE_MM << " mm away";
    EXPECT_TRUE(unit->readOffset(offset2));
    EXPECT_EQ(offset, offset2);

    // UM2510: calibrate the crosstalk with the offset applied
    uint16_t xtalk{}, xtalk2{};
    EXPECT_TRUE(unit->calibrateXtalk(xtalk, TARGET_DISTANCE_MM))
        << "No target? Place a matte object " << TARGET_DISTANCE_MM << " mm away";
    EXPECT_TRUE(unit->readXtalk(xtalk2));
    EXPECT_EQ((xtalk << 9) / 1000, (xtalk2 << 9) / 1000) << xtalk << "/" << xtalk2;
    EXPECT_TRUE(unit->writeXtalk(0));
    EXPECT_TRUE(unit->writeOffset(0));
}

// Long mode cannot use the 15 ms budget: the change must be rejected before any register is written
TEST_F(TestVL53L1X, DistanceModeLongRejectsBudget15ms)
{
    SCOPED_TRACE(ustr);

    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_TRUE(unit->writeDistanceMode(Distance::Short));
    EXPECT_TRUE(unit->writeTimingBudget(Timing::Budget15ms));

    EXPECT_FALSE(unit->writeDistanceMode(Distance::Long));

    Distance d{};
    Timing tb{};
    EXPECT_TRUE(unit->readDistanceMode(d));
    EXPECT_TRUE(unit->readTimingBudget(tb));
    EXPECT_EQ(d, Distance::Short);
    EXPECT_EQ(unit->distanceMode(), Distance::Short);
    EXPECT_EQ(tb, Timing::Budget15ms);
    EXPECT_EQ(unit->timingBudget(), Timing::Budget15ms);
}

TEST_F(TestVL53L1X, DistanceAndTimingBudget)
{
    SCOPED_TRACE(ustr);

    // Values out of the enum are rejected
    EXPECT_FALSE(unit->writeTimingBudget(static_cast<Timing>(7)));
    EXPECT_FALSE(unit->writeTimingBudget(Timing::BudgetUnknown));

    // For each distance mode, write every timing budget
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

    // For each timing budget, change the distance mode and check the budget is kept
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
                // Verify TB is preserved after distance mode change
                Timing tb_after{};
                EXPECT_TRUE(unit->readTimingBudget(tb_after));
                EXPECT_EQ(tb, tb_after) << "TB changed after writeDistanceMode";
            }
        }
    }

    // IMP (register read/write roundtrip only)
    // Datasheet requires IMP >= timing_budget + 4ms, but writeInterMeasurementPeriod()
    // does not enforce this constraint. The normal API path (begin() and
    // startPeriodicMeasurement(dist, tb)) always sets IMP from interval_table[tb],
    // which satisfies the constraint by design.
    {
        uint32_t cnt{32};
        while (cnt--) {
            uint16_t ms = esp_random() & 0xFFFF;
            auto s      = m5::utility::formatString("IMP:%u", ms);
            SCOPED_TRACE(s);
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

TEST_F(TestVL53L1X, Reset)
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
    // Cached settings are cleared too (the sensor is back to its defaults)
    EXPECT_EQ(unit->distanceMode(), Distance::Unknown);
    EXPECT_EQ(unit->timingBudget(), Timing::BudgetUnknown);

    Distance d{};
    Timing tb{};
    int16_t offset{};
    uint16_t xtalk{}, ms{};
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

TEST_F(TestVL53L1X, SingleShot)
{
    SCOPED_TRACE(ustr);

    Data data{};

    EXPECT_FALSE(unit->measureSingleshot(data));
    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_FALSE(unit->inPeriodic());

    for (auto&& d : distance_table) {
        SCOPED_TRACE(d == Distance::Long ? "Long" : "Short");
        EXPECT_TRUE(unit->writeDistanceMode(d));
        check_singleshot_results(measure_singleshots(unit.get()));
    }
}

// NOTICE: Needs a matte object placed TARGET_DISTANCE_MM in front of the sensor
TEST_F(TestVL53L1X, SingleShotWithTarget)
{
    SCOPED_TRACE(ustr);

    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    for (auto&& d : distance_table) {
        SCOPED_TRACE(d == Distance::Long ? "Long" : "Short");
        EXPECT_TRUE(unit->writeDistanceMode(d));
        check_singleshot_results_with_target(measure_singleshots(unit.get()));
    }
}

TEST_F(TestVL53L1X, ROI)
{
    SCOPED_TRACE(ustr);

    for (uint8_t h = 3; h <= 17; ++h) {
        for (uint8_t w = 3; w <= 17; ++w) {
            if (h < 4 || h > 16 || w < 4 || w > 16) {
                EXPECT_FALSE(unit->writeROI(w, h));
                continue;
            }
            uint8_t c = esp_random() & 0xFF;
            auto sc   = m5::utility::formatString("ROI Center:0x%02X W:%u H:%u", c, w, h);
            SCOPED_TRACE(sc);
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

    // A smaller ROI written after a large one keeps the center set by the large one (199)
    EXPECT_TRUE(unit->writeROI(16, 16));
    EXPECT_TRUE(unit->writeROI(4, 4));
    uint8_t center{};
    EXPECT_TRUE(unit->readROICenter(center));
    EXPECT_EQ(center, 199);
}

TEST_F(TestVL53L1X, Window)
{
    SCOPED_TRACE(ustr);

    for (auto&& w : window_table) {
        uint16_t low{}, high{};
        do {
            low  = esp_random() & 0xFFFF;
            high = esp_random() & 0xFFFF;
        } while (!(high > low && low > 0 && high > 0));

        auto sw = m5::utility::formatString("Window:%u Low:%u High:%u", m5::stl::to_underlying(w), low, high);
        SCOPED_TRACE(sw);

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

TEST_F(TestVL53L1X, Periodic)
{
    SCOPED_TRACE(ustr);

    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_FALSE(unit->inPeriodic());

    for (auto&& d : distance_table) {
        for (auto&& tb : tb_table) {
            auto s = m5::utility::formatString("Dis:%d TB:%d", static_cast<int>(m5::stl::to_underlying(d)),
                                               static_cast<int>(m5::stl::to_underlying(tb)));
            SCOPED_TRACE(s.c_str());
            if (d == Distance::Long && tb == Timing::Budget15ms) {
                continue;
            }

            EXPECT_FALSE(unit->inPeriodic());
            EXPECT_TRUE(unit->startPeriodicMeasurement(d, tb));
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
            EXPECT_EQ(unit->range_status(), RangeStatus::Unknown255);
        }
    }
}

TEST_F(TestVL53L1X, ChangeI2CAddress)
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
    EXPECT_TRUE(unit->changeI2CAddress(UnitVL53L1X::DEFAULT_ADDRESS));
    EXPECT_TRUE(unit->readI2CAddress(addr));
    EXPECT_EQ(addr, +UnitVL53L1X::DEFAULT_ADDRESS);
    EXPECT_EQ(unit->address(), +UnitVL53L1X::DEFAULT_ADDRESS);

    // Verify softReset restores default address
    EXPECT_TRUE(unit->changeI2CAddress(0x0A));
    EXPECT_EQ(unit->address(), 0x0A);
    EXPECT_TRUE(unit->softReset());
    EXPECT_EQ(unit->address(), +UnitVL53L1X::DEFAULT_ADDRESS);
    EXPECT_TRUE(unit->readI2CAddress(addr));
    EXPECT_EQ(addr, +UnitVL53L1X::DEFAULT_ADDRESS);
}

// begin() resets the sensor, which reverts its I2C address to the default; begin() must keep the address in use
TEST_F(TestVL53L1X, BeginAfterChangeI2CAddress)
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

    EXPECT_TRUE(unit->changeI2CAddress(UnitVL53L1X::DEFAULT_ADDRESS));
}

// begin() resumes a sensor held in reset (a reset whose resume was not written, e.g. by an interrupted program)
TEST_F(TestVL53L1X, BeginResumesHeldReset)
{
    SCOPED_TRACE(ustr);

    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_TRUE(unit->writeRegister8(m5::unit::vl53l1x::command::SOFT_RESET, 0x00));
    EXPECT_TRUE(unit->begin());

    Data d{};
    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_TRUE(unit->measureSingleshot(d));
}
