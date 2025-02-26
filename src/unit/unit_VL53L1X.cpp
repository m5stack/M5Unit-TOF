/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_VL53L1X.cpp
  @brief VL53L1X Unit for M5UnitUnified
 */
#include "unit_VL53L1X.hpp"
#include <M5Utility.hpp>

using namespace m5::utility::mmh3;
using namespace m5::unit::types;
using namespace m5::unit::vl53l1x;
using namespace m5::unit::vl53l1x::command;

namespace {
constexpr uint8_t VALID_MODEL_ID{0xEA};
constexpr uint8_t VALID_MODULE_TYPE{0xCC};

enum class SystemMode : uint8_t {
    RangeSingleShort = 0x10,
    RangeBackToBack  = 0x20,
    RangeTimed       = 0x40,
    RangeAbort       = 0x80,
};

constexpr uint16_t default_values_start{0x2D};
// Between 0x2D and  0x87
constexpr uint8_t default_values[] = {
    0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00,
    0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0b, 0x00, 0x00, 0x02, 0x0a, 0x21, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00,
    0x00, 0xc8, 0x00, 0x00, 0x38, 0xff, 0x01, 0x00, 0x08, 0x00, 0x00, 0x01, 0xcc, 0x0f, 0x01, 0xf1, 0x0d, 0x01, 0x68,
    0x00, 0x80, 0x08, 0xb8, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0f,
    0x0d, 0x0e, 0x0e, 0x00, 0x00, 0x02, 0xc7, 0xff, 0x9B, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};

constexpr uint16_t tb_table[2][7][2] = {
    // Short
    {{0x001D, 0x0027},
     {0x0051, 0x006E},
     {0x00D6, 0x006E},
     {0x01AE, 0x01E8},
     {0x02E1, 0x0388},
     {0x03E1, 0x0496},
     {0x0591, 0x05C1}},
    // Long
    {{0x0000, 0x0000},
     {0x001E, 0x0022},
     {0x0060, 0x006E},
     {0x00AD, 0x00C6},
     {0x01CC, 0x01EA},
     {0x02D9, 0x02F8},
     {0x048F, 0x04A4}},
};

constexpr uint8_t status_table[32] = {255, 255, 255, 5, 2,   4,   1,  7,  3,   0,   255, 255, 9,   13,  255, 255,
                                      255, 255, 10,  6, 255, 255, 11, 12, 255, 255, 255, 255, 255, 255, 255, 255};

constexpr uint32_t interval_table[] = {15, 20, 33, 50, 100, 200, 500};

#if 0
// clang-format off
constexpr uint8_t roi_region[16*16] = {
    128,136,144,152,160,168,176,184,   192,200,208,216,224,232,240,248,
    129,137,145,153,161,169,177,185,   193,201,209,217,225,233,241,249,
    130,138,146,154,162,170,178,186,   194,202,210,218,226,234,242,250,
    131,139,147,155,163,171,179,187,   195,203,211,219,227,235,243,251,
    132,140,148,156,164,172,180,188,   196,204,212,220,228,236,244,252,
    133,141,149,157,165,173,181,189,   197,205,213,221,229,237,245,253,
    134,142,150,158,166,174,182,190,   198,206,214,222,230,238,246,254,
    135,143,151,159,167,175,183,191,   199,207,215,223,231,239,247,255,

    127,119,111,103, 95, 87, 79, 71,    63, 55, 47, 39, 31, 23, 15,  7,
    126,118,110,102, 94, 86, 78, 70,    62, 54, 46, 38, 30, 22, 14,  6,
    125,117,109,101, 93, 85, 77, 69,    61, 53, 45, 37, 29, 21, 13,  5,
    124,116,108,100, 92, 84, 76, 68,    60, 52, 44, 36, 28, 20, 12,  4,
    123,115,107, 99, 91, 83, 75, 67,    59, 51, 43, 35, 27, 19, 11,  3,
    122,114,106, 98, 90, 82, 74, 66,    58, 50, 42, 34, 26, 18, 10,  2,
    121,113,105, 97, 89, 81, 73, 65,    57, 49, 41, 33, 25, 17,  9,  1,
    120,112,104, 96, 88, 80, 72, 64,    56, 48, 40, 32, 24, 16,  8,  0,
};
// clang-format on
#endif

}  // namespace

namespace m5 {
namespace unit {

namespace vl53l1x {
RangeStatus Data::range_status() const
{
    return static_cast<RangeStatus>(status_table[raw[0] & 0x1F]);
}

}  // namespace vl53l1x

const char UnitVL53L1X::name[] = "UnitVL53L1X";
const types::uid_t UnitVL53L1X::uid{"UnitVL53L1X"_mmh3};
const types::uid_t UnitVL53L1X::attr{0};

bool UnitVL53L1X::write_default_values()
{
    uint16_t reg{default_values_start};
    for (auto&& v : default_values) {
        uint8_t val{v};
        if (reg == EXTSUP_CONFIG) {
            val = m5::stl::to_underlying(_cfg.operating);
        }
        M5_LIB_LOGV("reg:%02x val:%02x", reg, val);
        if (!writeRegister8(reg, val)) {
            return false;
        }
        ++reg;
    }
    return true;
}

bool UnitVL53L1X::begin()
{
    auto ssize = stored_size();
    assert(ssize && "stored_size must be greater than zero");
    if (ssize != _data->capacity()) {
        _data.reset(new m5::container::CircularBuffer<Data>(ssize));
        if (!_data) {
            M5_LIB_LOGE("Failed to allocate");
            return false;
        }
    }

    // Check unit
    uint8_t model_id{}, module_type{};
    readRegister8(MODEL_ID, model_id, 0);
    readRegister8(MODULE_TYPE, module_type, 0);
    if (model_id != VALID_MODEL_ID || module_type != VALID_MODULE_TYPE) {
        M5_LIB_LOGE("This unit is NOT VL53L1X %x/%x", model_id, module_type);
        return false;
    }

    // Reset
    if (!soft_reset()) {
        M5_LIB_LOGE("Failed to reset");
        return false;
    }

    // Booted?
    if (!wait_booted()) {
        M5_LIB_LOGE("Not booted");
        return false;
    }

    // Set default values
    if (!write_default_values()) {
        M5_LIB_LOGE("Failed to write default");
        return false;
    }

    // distance mode
    if (!readDistanceMode(_distance)) {
        M5_LIB_LOGE("Failed to readDistanceMode");
        return false;
    }

    // Operating condition
    if (!write_operating_condition(_cfg.operating)) {
        M5_LIB_LOGE("Failed to write_oOperation_condition");
        return false;
    }

    // Check
    if (writeRegister8(SYSTEM_MODE_START, m5::stl::to_underlying(SystemMode::RangeTimed))) {
        auto timeout_at = m5::utility::millis() + 1000;
        bool ready{};
        do {
            ready = read_data_ready_status();
            m5::utility::delay(1);
        } while (!ready && m5::utility::millis() <= timeout_at);
        if (!ready) {
            M5_LIB_LOGE("Timeout");
            return false;
        }
        if (!writeRegister8(SYSTEM_INTERRUPT_CLEAR, 0x01) ||
            !writeRegister8(SYSTEM_MODE_START, m5::stl::to_underlying(SystemMode::RangeAbort)) ||
            !writeRegister8(VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, 0x09) || !writeRegister8(VHV_CONFIG_INIT, 0x00)) {
            M5_LIB_LOGE("Failed to initialize prodcess");
            return false;
        }
    } else {
        M5_LIB_LOGE("Failed to initialize prodcess");
        return false;
    }

    // Calibration
    if (_cfg.calibrate_offset) {
        int16_t offset{};
        if (!calibrateOffset(offset)) {
            M5_LIB_LOGE("Failed to calibrateOffset");
            return false;
        }
    }
    if (_cfg.calibrate_xtalk) {
        uint16_t xtalk{};
        if (!calibrateXtalk(xtalk)) {
            M5_LIB_LOGE("Failed to calibrateXtalk");
            return false;
        }
    }

    // Apply config
    if (!writeDistanceMode(_cfg.distance) || !writeTimingBudget(_cfg.timing_budget) ||
        !writeInterMeasurementPeriod(interval_table[m5::stl::to_underlying(_cfg.timing_budget)])) {
        M5_LIB_LOGE("Failed to apply config");
        return false;
    }

    return _cfg.start_periodic ? startPeriodicMeasurement() : true;
}

void UnitVL53L1X::update(const bool force)
{
    _updated = false;
    if (inPeriodic()) {
        elapsed_time_t at{m5::utility::millis()};
        if (force || !_latest || at >= _latest + _interval) {
            if (read_data_ready_status()) {
                Data d{};
                _updated = read_measurement(d);
                if (_updated) {
                    _latest = at;
                    _data->push_back(d);
                }
                auto ret = writeRegister8(SYSTEM_INTERRUPT_CLEAR, 0x01);
                if (!ret) {
                    M5_LIB_LOGE("Failed to write, Cannot continue measure");
                }
            }
        }
    }
}

bool UnitVL53L1X::start_periodic_measurement()
{
    if (inPeriodic()) {
        M5_LIB_LOGD("Periodic measurements are running");
        return false;
    }

    //    if (writeRegister8(SYSTEM_MODE_START, m5::stl::to_underlying(SystemMode::RangeTimed)) &&
    //                       writeRegister8(SYSTEM_INTERRUPT_CLEAR, 0x01)) {
    if (writeRegister8(SYSTEM_MODE_START, m5::stl::to_underlying(SystemMode::RangeTimed))) {
        _latest   = 0;
        _periodic = true;
    }
    return _periodic;
}

bool UnitVL53L1X::start_periodic_measurement(const vl53l1x::Distance dist, const vl53l1x::Timing tb)
{
    if (inPeriodic()) {
        M5_LIB_LOGD("Periodic measurements are running");
        return false;
    }
    return writeDistanceMode(dist) && writeTimingBudget(tb) &&
           writeInterMeasurementPeriod(interval_table[m5::stl::to_underlying(tb)]) && start_periodic_measurement();
}

bool UnitVL53L1X::stop_periodic_measurement()
{
    if (writeRegister8(SYSTEM_MODE_START, m5::stl::to_underlying(SystemMode::RangeAbort))) {
        _periodic = false;
        return true;
    }
    return false;
}

bool UnitVL53L1X::measureSingleshot(vl53l1x::Data& d)
{
    d = {};

    if (inPeriodic()) {
        M5_LIB_LOGD("Periodic measurements are running");
        return false;
    }

    if (writeRegister8(SYSTEM_INTERRUPT_CLEAR, 0x01) &&
        writeRegister8(SYSTEM_MODE_START, m5::stl::to_underlying(SystemMode::RangeSingleShort))) {
        auto timeout_at = m5::utility::millis() + _interval * 2;
        do {
            if (read_data_ready_status()) {
                return read_measurement(d);
            }
        } while (m5::utility::millis() <= timeout_at);
    }
    return false;
}

bool UnitVL53L1X::softReset()
{
    if (soft_reset()) {
        _distance = Distance::Unknown;
        _periodic = false;
        return wait_booted() && write_default_values();
    }
    return false;
}

bool UnitVL53L1X::soft_reset()
{
    if (writeRegister8(SOFT_RESET, 0x00)) {  // reset
        m5::utility::delay(1);
        if (writeRegister8(SOFT_RESET, 0x01)) {  // resume
            m5::utility::delay(1);               // need delay
            return true;
        }
    }
    return false;
}

bool UnitVL53L1X::wait_booted()
{
    uint8_t ss{};
    auto timeout_at = m5::utility::millis() + 1000;
    do {
        readRegister8(FIRMWARE_SYSTEM_STATUS, ss, 0);
        m5::utility::delay(1);
    } while (!ss && m5::utility::millis() <= timeout_at);
    return ss != 0;
}

bool UnitVL53L1X::readI2CAddress(uint8_t& i2c_address)
{
    i2c_address = 0;
    return readRegister8(I2C_SLAVE_DEVICE_ADDRESS, i2c_address, 0);
}

bool UnitVL53L1X::changeI2CAddress(const uint8_t i2c_address)
{
    if (!m5::utility::isValidI2CAddress(i2c_address)) {
        M5_LIB_LOGE("Invalid address : %02X", i2c_address);
        return false;
    }
    if (writeRegister8(I2C_SLAVE_DEVICE_ADDRESS, i2c_address) && changeAddress(i2c_address)) {
        return true;
    }
    return false;
}

bool UnitVL53L1X::write_operating_condition(const vl53l1x::Operating oc)
{
    uint8_t v{};
    if (readRegister8(EXTSUP_CONFIG, v, 0)) {
        v &= ~0x01;
        v |= m5::stl::to_underlying(oc);
        return writeRegister8(EXTSUP_CONFIG, v);
    }
    return false;
}

bool UnitVL53L1X::read_measurement(Data& d)
{
    return readRegister(RESULT_RANGE_STATUS, d.raw.data(), d.raw.size(), 0);
}

bool UnitVL53L1X::read_data_ready_status()
{
#if 0
    //    uint8_t v{};
    //    return readRegister8(GPIO_TIO_HV_STATUS, v, 0) && ((v & 0x01) == 1);
#else
    uint8_t polarity{}, v{};
    if (readRegister8(GPIO_HV_MUX_CTRL, polarity, 0) && readRegister8(GPIO_TIO_HV_STATUS, v, 0)) {
        polarity = !((polarity & 0x10) >> 4);
        return (v & 0x01) == polarity;
    }
    return false;
#endif
}

bool UnitVL53L1X::readDistanceMode(Distance& d)
{
    uint8_t v{};
    if (readRegister8(PHASECAL_CONFIG_TIMEOUT_MACROP, v, 0)) {
        d = (v == 0x14) ? Distance::Short : (v == 0x0A) ? Distance::Long : Distance::Unknown;
        return true;
    }
    return false;
}

bool UnitVL53L1X::writeDistanceMode(const Distance d)
{
    Timing tb{};
    if (readTimingBudget(tb)) {
        bool ret{};
        switch (d) {
            case Distance::Short:
                ret = writeRegister8(PHASECAL_CONFIG_TIMEOUT_MACROP, 0x14) &&
                      writeRegister8(RANGE_CONFIG_VCSEL_PERIOD_A, 0x07) &&
                      writeRegister8(RANGE_CONFIG_VCSEL_PERIOD_B, 0x05) &&
                      writeRegister8(RANGE_CONFIG_VALID_PHASE_HIGH, 0x38) &&
                      writeRegister16BE(SD_CONFIG_WOI_SD0, 0x0705) &&
                      writeRegister16BE(SD_CONFIG_INITIAL_PHASE_SD0, 0x0606);
                break;
            case Distance::Long:
                ret = writeRegister8(PHASECAL_CONFIG_TIMEOUT_MACROP, 0x0A) &&
                      writeRegister8(RANGE_CONFIG_VCSEL_PERIOD_A, 0x0F) &&
                      writeRegister8(RANGE_CONFIG_VCSEL_PERIOD_B, 0x0D) &&
                      writeRegister8(RANGE_CONFIG_VALID_PHASE_HIGH, 0xB8) &&
                      writeRegister16BE(SD_CONFIG_WOI_SD0, 0x0F0D) &&
                      writeRegister16BE(SD_CONFIG_INITIAL_PHASE_SD0, 0x0E0E);
                break;
            default:
                break;
        }
        if (ret && (tb != Timing::BudgetUnknown) ? write_timing_budget(tb, d) : true) {
            _distance = d;
            return true;
        }
    }
    return false;
}

bool UnitVL53L1X::calibrateOffset(int16_t& offset, const uint16_t targetmm)
{
    offset = 0;

    if (!writeRegister8(ALGO_PART_TO_PART_RANGE_OFFSET_MM, 0x00) || !writeRegister8(MM_CONFIG_INNER_OFFSET_MM, 0x00) ||
        !writeRegister8(MM_CONFIG_OUTER_OFFSET_MM, 0x00)) {
        return false;
    }

    uint32_t avg{}, avg_count{};
    if (writeRegister8(SYSTEM_MODE_START, m5::stl::to_underlying(SystemMode::RangeTimed))) {
        uint32_t cnt{50};
        while (cnt--) {
            auto timeout_at = m5::utility::millis() + 1000;
            bool ready{};
            do {
                ready = read_data_ready_status();
                m5::utility::delay(1);
            } while (!ready && m5::utility::millis() <= timeout_at);
            if (ready) {
                uint16_t distance{};
                if (readRegister16BE(RESULT_FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, distance, 0)) {
                    avg += distance;
                    ++avg_count;
                }
                writeRegister8(SYSTEM_INTERRUPT_CLEAR, 0x01);
            }
        }
        writeRegister8(SYSTEM_MODE_START, m5::stl::to_underlying(SystemMode::RangeAbort));

        if (!avg_count) {
            return false;
        }
        M5_LIB_LOGV("tgt:%u avg(%u):%u", targetmm, avg_count, avg / avg_count);
        offset = targetmm - (avg / avg_count);
        return writeOffset(offset);
    }
    return false;
}

bool UnitVL53L1X::readOffset(int16_t& offset)
{
    uint16_t v{};
    if (readRegister16BE(ALGO_PART_TO_PART_RANGE_OFFSET_MM, v, 0)) {
        // 11.2 to int16
        v >>= 2;
        if (v & 0x0400) {  // Sign extension
            v |= 0xFC00;
        }
        offset = (int16_t)v;
        return true;
    }
    return false;
}

bool UnitVL53L1X::writeOffset(const int16_t offset)
{
    uint16_t tmp = (uint16_t)offset;

    // check range
    if (offset > 1023 || offset < -1024) {
        M5_LIB_LOGE("Invalid offset %d", offset);
        return false;
    }
    tmp = (tmp << 2) & 0x1FFF;  // int16 to 11.2
    return writeRegister16BE(ALGO_PART_TO_PART_RANGE_OFFSET_MM, tmp);
}

bool UnitVL53L1X::calibrateXtalk(uint16_t& xtalk, const uint16_t targetmm)
{
    xtalk = 0;

    float avg{}, avg_signal_rate{}, avg_spad{};
    uint_fast8_t avg_count{}, avg_signal_rate_count{}, avg_spad_count{};

    if (writeRegister8(ALGO_CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS, 0x00) &&
        writeRegister8(SYSTEM_MODE_START, m5::stl::to_underlying(SystemMode::RangeTimed))) {
        uint32_t cnt{50};
        while (cnt--) {
            auto timeout_at = m5::utility::millis() + 1000;
            bool ready{};
            do {
                ready = read_data_ready_status();
            } while (!ready && m5::utility::millis() <= timeout_at);
            if (ready) {
                uint16_t distance{}, sr{}, spad_nb{};
                if (readRegister16BE(RESULT_FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, distance, 0)) {
                    avg += distance;
                    ++avg_count;
                }
                if (readRegister16BE(RESULT_PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0, sr, 0)) {
                    avg_signal_rate += sr;
                    ++avg_signal_rate_count;
                }
                if (readRegister16BE(RESULT_DSS_ACTUAL_EFFECTIVE_SPADS_SD0, spad_nb, 0)) {
                    avg_spad += spad_nb;
                    ++avg_spad_count;
                }
                writeRegister8(SYSTEM_INTERRUPT_CLEAR, 0x01);
            }
        }
        writeRegister8(SYSTEM_MODE_START, m5::stl::to_underlying(SystemMode::RangeAbort));

        if (!avg_count || !avg_signal_rate_count || !avg_spad_count) {
            return false;
        }

        avg /= avg_count;
        avg_signal_rate /= avg_signal_rate_count;
        avg_spad /= avg_spad_count;

        M5_LIB_LOGV("tgt:%u avg(%u):%f sr(%u):%f spad(%u):%f", targetmm, avg_count, avg, avg_signal_rate_count,
                    avg_signal_rate, avg_spad_count, avg_spad);

        uint32_t tmp = (uint16_t)(512 * (avg_signal_rate * (1 - (avg / targetmm))) / avg_spad);

        if (tmp > 65535) {
            tmp = 65535;
        }
        // tmp *= 1000;
        // tmp >>= 9;
        // xtalk = tmp;
        xtalk = (uint16_t)((tmp * 1000) >> 9);
        M5_LIB_LOGV("%x -> %u", tmp, xtalk);
        return writeXtalk(xtalk);
    }
    return false;
}

float fixed_7_9_to_float(uint16_t fixed_value)
{
    int16_t integer_part  = fixed_value >> 9;
    float fractional_part = (fixed_value & 0x01FF) / 512.0f;
    return integer_part + fractional_part;
}

bool UnitVL53L1X::readXtalk(uint16_t& xtalk)
{
    xtalk = 0;
    uint16_t v{};
    if (readRegister16BE(ALGO_CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS, v, 0)) {
        uint32_t t = v;
        xtalk      = (t * 1000 + 0x1FF) >> 9;  // ceiled
        return true;
    }
    return false;
}

bool UnitVL53L1X::writeXtalk(const uint16_t xtalk)
{
    return writeRegister16BE(ALGO_CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS, 0x0000) &&
           writeRegister16BE(ALGO_CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS, 0x0000) &&
           // convert to kiro cps by fixed 7.9
           writeRegister16BE(ALGO_CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS, (xtalk << 9) / 1000);
}

bool UnitVL53L1X::readTimingBudget(vl53l1x::Timing& tb)
{
    tb = Timing::BudgetUnknown;

    uint16_t v{};
    if (readRegister16BE(RANGE_CONFIG_TIMEOUT_MACROP_A, v, 0)) {
        switch (v) {
            case 0x001D:
                tb = Timing::Budget15ms;
                break;
            case 0x0051:
            case 0x001E:
                tb = Timing::Budget20ms;
                break;
            case 0x00D6:
            case 0x0060:
                tb = Timing::Budget33ms;
                break;
            case 0x01AE:
            case 0x00AD:
                tb = Timing::Budget50ms;
                break;
            case 0x02E1:
            case 0x01CC:
                tb = Timing::Budget100ms;
                break;
            case 0x03E1:
            case 0x02D9:
                tb = Timing::Budget200ms;
                break;
            case 0x0591:
            case 0x048F:
                tb = Timing::Budget500ms;
                break;
            default:
                break;
        }
        return true;
    }
    return false;
}

bool UnitVL53L1X::write_timing_budget(const Timing tb, const Distance dist)
{
    if (tb == Timing::BudgetUnknown || dist == Distance::Unknown) {
        M5_LIB_LOGE("Illegal arg or status %u/%u", tb, dist);
        return false;
    }
    if (tb == Timing::Budget15ms && dist == Distance::Long) {
        M5_LIB_LOGW("Long mode CANNOT set 15ms");
        return false;
    }

    if (writeRegister16BE(RANGE_CONFIG_TIMEOUT_MACROP_A_HI,
                          tb_table[m5::stl::to_underlying(_distance)][m5::stl::to_underlying(tb)][0]) &&
        writeRegister16BE(RANGE_CONFIG_TIMEOUT_MACROP_B_HI,
                          tb_table[m5::stl::to_underlying(_distance)][m5::stl::to_underlying(tb)][1])) {
        _tb = tb;
        return true;
    }
    return false;
}

bool UnitVL53L1X::readInterMeasurementPeriod(uint16_t& ms)
{
    ms = 0;
    uint32_t tmp32{};
    uint16_t cpll{};
    if (readRegister32BE(SYSTEM_INTERMEASUREMENT_PERIOD, tmp32, 0) &&
        readRegister16BE(RESULT_OSC_CALIBRATE_VAL, cpll, 0)) {
        cpll &= 0x3FF;
        ms = (uint16_t)(tmp32 / (cpll * 1.065f) + 0.5f);
        return true;
    }
    return false;
}

bool UnitVL53L1X::writeInterMeasurementPeriod(const uint16_t ms)
{
    uint16_t cpll{};
    if (readRegister16BE(RESULT_OSC_CALIBRATE_VAL, cpll, 0)) {
        cpll &= 0x3FF;

        uint32_t tmp32 = (uint32_t)(cpll * ms * 1.065f);

        if (writeRegister32BE(SYSTEM_INTERMEASUREMENT_PERIOD, tmp32)) {
            _interval = ms;
            return true;
        }
    }
    return false;
}

bool UnitVL53L1X::readDistanceThresholdWindow(vl53l1x::Window& window)
{
    uint8_t v{};
    if (readRegister8(SYSTEM_INTERRUPT_CONFIG_GPIO, v, 0)) {
        if (v & 0x20) {
            window = Window::Regular;
        } else {
            window = static_cast<Window>(v & 0x03);
        }
        return true;
    }
    return false;
}

bool UnitVL53L1X::readDistanceThresholdLow(uint16_t& mm)
{
    return readRegister16BE(SYSTEM_THRESH_LOW, mm, 0);
}
bool UnitVL53L1X::readDistanceThresholdHigh(uint16_t& mm)
{
    return readRegister16BE(SYSTEM_THRESH_HIGH, mm, 0);
}

bool UnitVL53L1X::writeDistanceThreshold(const vl53l1x::Window window, const uint16_t low, const uint16_t high)
{
    if (low > high) {
        M5_LIB_LOGE("Need high >= low %u:%u", high, low);
        return false;
    }

    uint8_t tmp{};
    if (readRegister8(SYSTEM_INTERRUPT_CONFIG_GPIO, tmp, 0)) {
        if (window == Window::Regular) {
            tmp = m5::stl::to_underlying(window);  // 0x20 go back to the regular ranging mode
        } else {
            tmp = (tmp & 0x4C) | m5::stl::to_underlying(window);
        }
        return writeRegister8(SYSTEM_INTERRUPT_CONFIG_GPIO, tmp) && writeRegister16BE(SYSTEM_THRESH_LOW, low) &&
               writeRegister16BE(SYSTEM_THRESH_HIGH, high);
    }
    return false;
}

bool UnitVL53L1X::readROI(uint8_t& wid, uint8_t& hgt)
{
    uint8_t v{};
    if (readRegister8(ROI_CONFIG_USER_ROI_REQUESTED_GLOBAL_XY_SIZE, v, 0)) {
        wid = (v & 0x0F) + 1;
        hgt = ((v >> 4) & 0x0F) + 1;
        return true;
    }
    return false;
}

bool UnitVL53L1X::readROICenter(uint8_t& center)
{
    return readRegister8(ROI_CONFIG_USER_ROI_CENTRE_SPAD, center, 0);
}

bool UnitVL53L1X::writeROI(const uint8_t wid, const uint8_t hgt)
{
    if (wid < 4 || wid > 16 || hgt < 4 || hgt > 16) {
        M5_LIB_LOGE("Invalid reagen width/heght.(valid between 4 and 16)  %u,%u", wid, hgt);
        return false;
    }

#if 0
    // Calculate the center
    uint8_t x = 15 - ((wid - 1) >> 1);
    uint8_t y = 15 - ((hgt) >> 1);
    assert(y * 16 + x < 256);
    uint8_t center = roi_region[y * 16 + x];
#endif

    uint8_t v = (wid - 1) | ((hgt - 1) << 4);
    return ((wid > 10 || hgt > 10) ? writeROICenter(199) : true) &&
           writeRegister8(ROI_CONFIG_USER_ROI_REQUESTED_GLOBAL_XY_SIZE, v);
}

bool UnitVL53L1X::writeROICenter(const uint8_t center)
{
    return writeRegister8(ROI_CONFIG_USER_ROI_CENTRE_SPAD, center);
}

}  // namespace unit
}  // namespace m5
