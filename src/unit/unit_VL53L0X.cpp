/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_VL53L0X.cpp
  @brief VL53L0X Unit for M5UnitUnified
 */
#include "unit_VL53L0X.hpp"
#include <M5Utility.hpp>

using namespace m5::utility::mmh3;
using namespace m5::unit::types;
using namespace m5::unit::vl53l0x;
using namespace m5::unit::vl53l0x::command;

// User-defined literals "_u8"
constexpr uint8_t operator""_u8(unsigned long long v)
{
    return static_cast<uint8_t>(v);
}

namespace {
constexpr uint8_t VALID_MODEL_ID{0xEE};
// Wait after MODEL_ID is valid on resume (datasheet tBOOT 1.2 ms max, with margin)
constexpr uint32_t BOOT_WAIT_MS{2};

// specific steps in the sequence
constexpr uint8_t RANGE_SEQUENCE_STEP_MSRC{0x04};
constexpr uint8_t RANGE_SEQUENCE_STEP_TCC{0x10};
constexpr uint8_t RANGE_SEQUENCE_STEP_DSS{0x28};
constexpr uint8_t RANGE_SEQUENCE_STEP_PRE_RANGE{0x40};
constexpr uint8_t RANGE_SEQUENCE_STEP_FINAL_RANGE{0x80};

struct default_value_t {
    uint8_t reg;
    uint8_t val;
};
constexpr default_value_t default_values[] = {
    {0xFF, 0x01}, {0x00, 0x00}, {0xFF, 0x00}, {0x09, 0x00}, {0x10, 0x00}, {0x11, 0x00}, {0x24, 0x01}, {0x25, 0xFF},
    {0x75, 0x00}, {0xFF, 0x01}, {0x4E, 0x2C}, {0x48, 0x00}, {0x30, 0x20}, {0xFF, 0x00}, {0x30, 0x09}, {0x54, 0x00},
    {0x31, 0x04}, {0x32, 0x03}, {0x40, 0x83}, {0x46, 0x25}, {0x60, 0x00}, {0x27, 0x00}, {0x50, 0x06}, {0x51, 0x00},
    {0x52, 0x96}, {0x56, 0x08}, {0x57, 0x30}, {0x61, 0x00}, {0x62, 0x00}, {0x64, 0x00}, {0x65, 0x00}, {0x66, 0xA0},
    {0xFF, 0x01}, {0x22, 0x32}, {0x47, 0x14}, {0x49, 0xFF}, {0x4A, 0x00}, {0xFF, 0x00}, {0x7A, 0x0A}, {0x7B, 0x00},
    {0x78, 0x21}, {0xFF, 0x01}, {0x23, 0x34}, {0x42, 0x00}, {0x44, 0xFF}, {0x45, 0x26}, {0x46, 0x05}, {0x40, 0x40},
    {0x0E, 0x06}, {0x20, 0x1A}, {0x43, 0x40}, {0xFF, 0x00}, {0x34, 0x03}, {0x35, 0x44}, {0xFF, 0x01}, {0x31, 0x04},
    {0x4B, 0x09}, {0x4C, 0x05}, {0x4D, 0x04}, {0xFF, 0x00}, {0x44, 0x00}, {0x45, 0x20}, {0x47, 0x08}, {0x48, 0x28},
    {0x67, 0x00}, {0x70, 0x04}, {0x71, 0x01}, {0x72, 0xFE}, {0x76, 0x00}, {0x77, 0x00}, {0xFF, 0x01}, {0x0D, 0x01},
    {0xFF, 0x00}, {0x80, 0x01}, {0x01, 0xF8}, {0xFF, 0x01}, {0x8E, 0x01}, {0x00, 0x01}, {0xFF, 0x00}, {0x80, 0x00},
};

constexpr uint8_t pre_phase_table[]                 = {0x18, 0x30, 0x40, 0x50};
constexpr uint16_t pre_range_timeout_macrop_table[] = {0x00AF, 0x0096, 0x0083, 0x0073};
constexpr uint8_t pre_msrc_timeout_macrop_table[]   = {0x2B, 0x25, 0x20, 0x1C};

struct final_values_t {
    uint8_t phase_hi, phase_low, vcsel, phasecal_timeout, phasecal_limit;
};

constexpr final_values_t final_values_table[] = {{0x10, 0x08, 0x02, 0x0C, 0x30},
                                                 {0x28, 0x08, 0x03, 0x09, 0x20},
                                                 {0x38, 0x08, 0x03, 0x08, 0x20},
                                                 {0x48, 0x08, 0x03, 0x07, 0x20}};

constexpr uint16_t pre_and_final_timeout_macrop_table[4][4] = {{0x02B0, 0x0296, 0x0284, 0x01EF},
                                                               {0x02AA, 0x028F, 0x01FE, 0x01E3},
                                                               {0x02A6, 0x028B, 0x01F2, 0x01D9},
                                                               {0x02A3, 0x0288, 0x01EC, 0x01D3}};

constexpr RangeStatus range_status_table[16] = {
    RangeStatus::Unknown,         RangeStatus::HardwareFailure, RangeStatus::HardwareFailure,
    RangeStatus::HardwareFailure, RangeStatus::SignalFailure,   RangeStatus::Unknown,
    RangeStatus::PhaseFailure,    RangeStatus::Unknown,         RangeStatus::MinRangeFailure,
    RangeStatus::PhaseFailure,    RangeStatus::MinRangeFailure, RangeStatus::OK,
    RangeStatus::Unknown,         RangeStatus::Unknown,         RangeStatus::Unknown,
    RangeStatus::Unknown};

// Polling interval for each mode. Continuous ranging runs back-to-back, so the period is the timing budget of the mode
// (about 31 / 195 / 31 / 18 ms, varies a little by device). Poll a little faster so that no result is missed
constexpr uint32_t interval_table[] = {30, 190, 30, 17};

}  // namespace

namespace m5 {
namespace unit {
namespace vl53l0x {

RangeStatus Data::range_status() const
{
    return range_status_table[(raw[0] & 0x78) >> 3 /* 0-15 */];
}

}  // namespace vl53l0x

const char UnitVL53L0X::name[] = "UnitVL53L0X";
const types::uid_t UnitVL53L0X::uid{"UnitVL53L0X"_mmh3};
const types::attr_t UnitVL53L0X::attr{attribute::AccessI2C};

bool UnitVL53L0X::write_default_values()
{
    for (auto&& dv : default_values) {
        if (!writeRegister8(dv.reg, dv.val)) {
            return false;
        }
    }
    return true;
}

bool UnitVL53L0X::write_default_settings()
{
    // Operating condition (read-modify-write, bit 0 only)
    {
        uint8_t v{};
        if (!readRegister8(VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, v, 0)) {
            M5_LIB_LOGE("Failed to read operation condition");
            return false;
        }
        if (_cfg.operating == Operating::Condition2V8) {
            v |= 0x01;
        } else {
            v &= ~0x01;
        }
        if (!writeRegister8(VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, v)) {
            M5_LIB_LOGE("Failed to write operation condition");
            return false;
        }
    }

    // Set I2C standard mode
    _stop = 0xff;
    if (!writeRegister8(0x88_u8, 0x00) || !writeRegister8(0x80_u8, 0x01) || !writeRegister8(0xFF_u8, 0x01) ||
        !writeRegister8(0x00_u8, 0x00) || !readRegister8(0x91_u8, _stop, 0) || !writeRegister8(0x00_u8, 0x01) ||
        !writeRegister8(0xFF_u8, 0x00) || !writeRegister8(0x80_u8, 0x00)) {
        M5_LIB_LOGE("Failed to set i2c standard mode");
        return false;
    }
    // Disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
    uint8_t v{};
    if (!readRegister8(MSRC_CONFIG_CONTROL, v, 0) || !writeRegister8(MSRC_CONFIG_CONTROL, v | 0x12)) {
        M5_LIB_LOGE("Failed to set signal");
        return false;
    }

    // Set default values
    if (!write_default_values()) {
        M5_LIB_LOGE("Failed to write default");
        return false;
    }

    // Set interrupt config
    uint8_t h{};
    if (!writeRegister8(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04) || !readRegister8(GPIO_HV_MUX_ACTIVE_HIGH, h, 0) ||
        !writeRegister8(GPIO_HV_MUX_ACTIVE_HIGH, h & ~0x10) || !writeRegister8(SYSTEM_INTERRUPT_CLEAR, 0x01)) {
        M5_LIB_LOGE("Failed to set interrupt config");
        return false;
    }

    // Specific steps in the sequence
    if (!writeRegister8(SYSTEM_SEQUENCE_CONFIG,
                        RANGE_SEQUENCE_STEP_DSS | RANGE_SEQUENCE_STEP_PRE_RANGE | RANGE_SEQUENCE_STEP_FINAL_RANGE)) {
        M5_LIB_LOGE("Failed to set specific steps in the sequence");
        return false;
    }

    // Ref(temperature) calibration
    if (!perform_single_ref_calibration(true) || !perform_single_ref_calibration(false)) {
        M5_LIB_LOGE("Failed to ref calibration");
        return false;
    }

    // Restore sequence steps
    if (!writeRegister8(SYSTEM_SEQUENCE_CONFIG,
                        RANGE_SEQUENCE_STEP_DSS | RANGE_SEQUENCE_STEP_PRE_RANGE | RANGE_SEQUENCE_STEP_FINAL_RANGE)) {
        M5_LIB_LOGE("Failed to set specific steps in the sequence");
        return false;
    }
    return true;
}

bool UnitVL53L0X::begin()
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
    uint8_t model_id{};
    if (readRegister8(MODEL_ID, model_id, 0) && model_id == 0x00) {
        // Held in reset (e.g. a previous reset was not resumed): resume it
        M5_LIB_LOGW("Held in reset, resume it");
        if (writeRegister8(SOFT_RESET, 0x01) && wait_model_id(true)) {
            readRegister8(MODEL_ID, model_id, 0);
        }
    }
    if (model_id != VALID_MODEL_ID) {
        M5_LIB_LOGE("This unit is NOT VL53L0X %x", model_id);
        return false;
    }

    // Reset to ensure clean state
    if (!soft_reset()) {
        M5_LIB_LOGE("Failed to soft_reset");
        return false;
    }

    // Default settings
    if (!write_default_settings()) {
        return false;
    }

    // Mode
    if (!writeMode(_cfg.mode)) {
        M5_LIB_LOGE("Failed to writeMode %u", _cfg.mode);
        return false;
    }

    return _cfg.start_periodic ? startPeriodicMeasurement() : true;
}

void UnitVL53L0X::update(const bool force)
{
    _updated = false;
    if (inPeriodic()) {
        const elapsed_time_t at{m5::utility::millis()};
        if (force || !_latest || m5::utility::hasElapsed(_latest, _interval, at)) {
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

bool UnitVL53L0X::start_periodic_measurement()
{
    if (inPeriodic()) {
        M5_LIB_LOGD("Periodic measurements are running");
        return false;
    }
    if (mode() == Mode::Unknown) {
        M5_LIB_LOGE("Invalid mode");
        return false;
    }

    if (writeRegister8(0x80_u8, 0x01) && writeRegister8(0xFF_u8, 0x01) && writeRegister8(0x00_u8, 0x00) &&
        writeRegister8(0x91_u8, _stop) && writeRegister8(0x00_u8, 0x01) && writeRegister8(0xFF_u8, 0x00) &&
        writeRegister8(0x80_u8, 0x00) &&
        writeRegister8(SYSTEM_RANGE_START, 0x02) &&  // Continuous (back-to-back), as ST CONTINUOUS_RANGING
        writeRegister8(SYSTEM_INTERRUPT_CLEAR, 0x01)) {
        _latest   = 0;
        _periodic = true;
    } else {
        M5_LIB_LOGE("ERROR");
    }
    return _periodic;
}

bool UnitVL53L0X::stop_periodic_measurement()
{
    if (writeRegister8(SYSTEM_RANGE_START, 0x01) && writeRegister8(0xFF_u8, 0x01) && writeRegister8(0x00_u8, 0x00) &&
        writeRegister8(0x91_u8, 0x00) && writeRegister8(0x00_u8, 0x01) && writeRegister8(0xFF_u8, 0x00)) {
        _periodic = false;
        return true;
    }
    return false;
}

bool UnitVL53L0X::measureSingleshot(vl53l0x::Data& data)
{
    data = {};

    if (inPeriodic()) {
        M5_LIB_LOGE("Periodic measurements are running");
        return false;
    }

    if (!writeRegister8(0x80_u8, 0x01) || !writeRegister8(0xFF_u8, 0x01) || !writeRegister8(0x00_u8, 0x00) ||
        !writeRegister8(0x91_u8, _stop) || !writeRegister8(0x00_u8, 0x01) || !writeRegister8(0xFF_u8, 0x00) ||
        !writeRegister8(0x80_u8, 0x00) || !writeRegister8(SYSTEM_RANGE_START, 0x01)) {
        M5_LIB_LOGE("Failed to start single shot");
        return false;
    }

    auto start_at = m5::utility::millis();
    bool done{};
    uint8_t v{};
    done = readRegister8(SYSTEM_RANGE_START, v, 0) && ((v & 0x01) == 0);
    while (!done && !m5::utility::hasElapsed(start_at, 1000)) {
        m5::utility::delay(1);
        done = readRegister8(SYSTEM_RANGE_START, v, 0) && ((v & 0x01) == 0);
    }
    if (!done) {
        M5_LIB_LOGE("Single shot start not confirmed");
        return false;
    }

    start_at = m5::utility::millis();
    do {
        if (read_data_ready_status()) {
            return read_measurement(data) && writeRegister8(SYSTEM_INTERRUPT_CLEAR, 0x01);
        }
        m5::utility::delay(1);
    } while (!m5::utility::hasElapsed(start_at, 1000));

    M5_LIB_LOGE("Timeout");
    return false;
}

bool UnitVL53L0X::read_data_ready_status()
{
    uint8_t v{};
    return readRegister8(RESULT_INTERRUPT_STATUS, v, 0) && (v & 0x07);
}

bool UnitVL53L0X::read_measurement(vl53l0x::Data& d)
{
    return readRegister(RESULT_RANGE_STATUS, d.raw.data(), d.raw.size(), 0);
}

bool UnitVL53L0X::writeSignalRateLimit(const float mcps)
{
    if (!(mcps >= 0.0f && mcps < 512.f)) {  // Also rejects NaN
        M5_LIB_LOGE("Valid range is between 0.0f and 512.0f %f", mcps);
        return false;
    }
    const uint16_t v = static_cast<uint16_t>(mcps * (1 << 7));
    return writeRegister16BE(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, v);
}

bool UnitVL53L0X::writeMode(const vl53l0x::Mode mode)
{
    if (!writeRegister8(0x80_u8, 0x01) || !writeRegister8(0xFF_u8, 0x01) || !writeRegister8(0x00_u8, 0x00) ||
        !writeRegister8(0x91_u8, _stop) || !writeRegister8(0x00_u8, 0x01) || !writeRegister8(0xFF_u8, 0x00) ||
        !writeRegister8(0x80_u8, 0x00)) {
        return false;
    }

    bool ret{};
    switch (mode) {
        case Mode::Default: {
            ret = writeSignalRateLimit(0.25f) && write_vcsel_period_range(14, 10);
        } break;
        case Mode::HighAccuracy: {
            ret = writeSignalRateLimit(0.25f) && write_vcsel_period_range(14, 10) &&
                  writeRegister16BE(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, 0x059A);
        } break;
        case Mode::HighSpeed: {
            ret = writeSignalRateLimit(0.25f) && write_vcsel_period_range(14, 10) &&
                  writeRegister16BE(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, 0x00D5);
        } break;
        case Mode::LongRange: {
            ret = writeSignalRateLimit(0.1f) && write_vcsel_period_range(18, 14);
        } break;
        default:
            break;
    }
    if (ret) {
        _interval = interval_table[m5::stl::to_underlying(mode)];
        _mode     = mode;
        return true;
    }
    return false;
}

bool UnitVL53L0X::readSignalRateLimit(float& mcps)
{
    uint16_t v{};
    mcps = std::numeric_limits<float>::quiet_NaN();
    if (readRegister16BE(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, v, 0)) {
        mcps = static_cast<float>(v) / (1 << 7);  // 9.7 to float
        return true;
    }
    return false;
}

bool UnitVL53L0X::perform_single_ref_calibration(const bool vhv)
{
    const uint8_t seq_cfg   = vhv ? 0x01 : 0x02;
    const uint8_t sys_start = (vhv ? 0x40 : 0x00) | 0x01;
    if (!writeRegister8(SYSTEM_SEQUENCE_CONFIG, seq_cfg) || !writeRegister8(SYSTEM_RANGE_START, sys_start)) {
        return false;
    }

    const auto start_at = m5::utility::millis();
    uint8_t v{};
    while (!readRegister8(RESULT_INTERRUPT_STATUS, v, 0) || (v & 0x07) == 0) {
        if (m5::utility::hasElapsed(start_at, 5000)) {
            M5_LIB_LOGE("Timeout");
            return false;
        }
        m5::utility::delay(1);
    }
    return writeRegister8(SYSTEM_INTERRUPT_CLEAR, 0x01) && writeRegister8(SYSTEM_RANGE_START, 0x00);
}

bool UnitVL53L0X::write_vcsel_period_range(const uint8_t pre_pclk, const uint8_t final_pclk)
{
    const uint8_t pre_vcsel_period   = (pre_pclk >> 1) - 1;
    const uint8_t final_vcsel_period = (final_pclk >> 1) - 1;

    if ((pre_pclk & 1) || pre_pclk < 12 || pre_pclk > 18) {
        M5_LIB_LOGE("Invalid pre_pclk %u", pre_pclk);
        return false;
    }
    if ((final_pclk & 1) || final_pclk < 8 || final_pclk > 14) {
        M5_LIB_LOGE("Invalid final_pclk %u", final_pclk);
        return false;
    }

    // pre
    const uint32_t pre_idx = (pre_pclk - 12) >> 1;  // 0-3
    if (!writeRegister8(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, pre_phase_table[pre_idx]) ||
        !writeRegister8(PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08) ||
        !writeRegister8(PRE_RANGE_CONFIG_VCSEL_PERIOD, pre_vcsel_period) ||
        !writeRegister16BE(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, pre_range_timeout_macrop_table[pre_idx]) ||
        !writeRegister8(MSRC_CONFIG_TIMEOUT_MACROP, pre_msrc_timeout_macrop_table[pre_idx])) {
        M5_LIB_LOGE("Failed to write pre");
        return false;
    }

    // final
    const uint32_t final_idx = (final_pclk - 8) >> 1;  // 0-3
    if (!writeRegister8(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, final_values_table[final_idx].phase_hi) ||
        !writeRegister8(FINAL_RANGE_CONFIG_VALID_PHASE_LOW, final_values_table[final_idx].phase_low) ||
        !writeRegister8(GLOBAL_CONFIG_VCSEL_WIDTH, final_values_table[final_idx].vcsel) ||
        !writeRegister8(ALGO_PHASECAL_CONFIG_TIMEOUT, final_values_table[final_idx].phasecal_timeout) ||
        !writeRegister8(0xFF_u8, 0x01) ||
        !writeRegister8(ALGO_PHASECAL_LIM, final_values_table[final_idx].phasecal_limit) ||
        !writeRegister8(0xFF_u8, 0x00) || !writeRegister8(FINAL_RANGE_CONFIG_VCSEL_PERIOD, final_vcsel_period) ||
        !writeRegister16BE(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                           pre_and_final_timeout_macrop_table[pre_idx][final_idx])) {
        M5_LIB_LOGE("Failed to write final");
        return false;
    }

    uint8_t scfg{};
    return readRegister8(SYSTEM_SEQUENCE_CONFIG, scfg, 0) && perform_single_ref_calibration(false) &&
           writeRegister8(SYSTEM_SEQUENCE_CONFIG, scfg);
}

bool UnitVL53L0X::softReset()
{
    // Reset command to current address, then switch to default before resume
    if (writeRegister8(SOFT_RESET, 0x00)) {
        changeAddress(DEFAULT_ADDRESS);  // Sensor reverts to default address after reset
        if (wait_model_id(false) && writeRegister8(SOFT_RESET, 0x01) &&
            wait_model_id(true)) {  // resume at default address
            _mode     = Mode::Unknown;
            _periodic = false;
            return write_default_settings();
        }
    }
    return false;
}

bool UnitVL53L0X::soft_reset()
{
    // The reset reverts the I2C address to the default: resume there, then restore the address in use
    const uint8_t addr = address();
    if (!writeRegister8(SOFT_RESET, 0x00)) {  // reset
        return false;
    }
    changeAddress(DEFAULT_ADDRESS);
    if (!wait_model_id(false) || !writeRegister8(SOFT_RESET, 0x01) || !wait_model_id(true)) {  // resume
        return false;
    }
    return addr == DEFAULT_ADDRESS || restore_i2c_address(addr);
}

// The address written while the firmware is still booting is ignored: write it until the device answers there
bool UnitVL53L0X::restore_i2c_address(const uint8_t addr)
{
    constexpr uint32_t TIMEOUT_MS{100};
    const auto start_at = m5::utility::millis();
    do {
        uint8_t v{};
        if (changeI2CAddress(addr) && readRegister8(MODEL_ID, v, 0) && v == VALID_MODEL_ID) {
            return true;
        }
        changeAddress(DEFAULT_ADDRESS);
        m5::utility::delay(1);
    } while (!m5::utility::hasElapsed(start_at, TIMEOUT_MS));
    M5_LIB_LOGE("Failed to restore the I2C address %02X", addr);
    return false;
}

// As ST VL53L0X_ResetDevice(): MODEL_ID reads 0 while in reset, and the valid ID after resume (then wait for the boot)
bool UnitVL53L0X::wait_model_id(const bool resumed)
{
    constexpr uint32_t TIMEOUT_MS{100};
    const auto start_at = m5::utility::millis();
    do {
        uint8_t v{0xFF};
        const bool read = readRegister8(MODEL_ID, v, 0);
        if (resumed ? (read && v == VALID_MODEL_ID) : (!read || v == 0x00)) {
            if (resumed) {
                m5::utility::delay(BOOT_WAIT_MS);  // The firmware is still booting (datasheet tBOOT 1.2 ms max)
            }
            return true;
        }
        m5::utility::delay(1);
    } while (!m5::utility::hasElapsed(start_at, TIMEOUT_MS));
    M5_LIB_LOGE("Timeout waiting for the %s", resumed ? "resume" : "reset");
    return false;
}

bool UnitVL53L0X::readI2CAddress(uint8_t& i2c_address)
{
    i2c_address = 0;
    return readRegister8(I2C_SLAVE_DEVICE_ADDRESS, i2c_address, 0);
}

bool UnitVL53L0X::changeI2CAddress(const uint8_t i2c_address)
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

}  // namespace unit
}  // namespace m5
