/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_VL53L0X.hpp
  @brief VL53L0X Unit for M5UnitUnified
 */
#ifndef M5_UNIT_TOF_UNIT_VL53L0X_HPP
#define M5_UNIT_TOF_UNIT_VL53L0X_HPP

#include <M5UnitComponent.hpp>
#include <m5_utility/container/circular_buffer.hpp>
#include <m5_utility/types.hpp>
#include <array>
#include <limits>  // NaN

namespace m5 {
namespace unit {
namespace vl53l0x {
/*!
  @enum Operating
  @brief Operating condition
 */
enum class Operating : uint8_t {
    ConditionStandard,  //!< Standard mode (1.6 - 1.9 V)
    Condition2V8        //!< 2V8 mode (2.6 - 3.5 V) (as default)
};

/*!
  @enum Mode
  @brief Operation Mode
 */
enum class Mode : int8_t {
    Unknown = -1,
    Default,       //!< interval 30ms  (limited to 1.2m)
    HighAccuracy,  //!< interval 200ms (limited to 1.2m)
    LongRange,     //!< interval 33ms  (limited to 2.0m)
    HighSpeed,     //!< interval 20ms  (limited to 1.2m)
};

/*!
  @enum RangeStatus
  @brief Range status interpretation
 */
enum class RangeStatus : uint8_t {
    OK,
    HardwareFailure,
    PhaseFailure,
    MinRangeFailure,
    SignalFailure,
    SigmaFailure,
    Unknown = 255
};

/*!
  @struct Data
  @brief Measurement data group
 */
struct Data {
    std::array<uint8_t, 12> raw{};  //!< RAW data
    //! @brief Range status
    RangeStatus range_status() const;
    //! @brief Is data valid?
    inline bool valid() const
    {
        return ((raw[0] & 0x78) >> 3) == 11;
    }
    /*!
      @brief range (mm)
      @retval >= 0 range (mm)
      @retval < 0 Returns a negative value if there is a range error
     */
    inline int16_t range() const
    {
        return valid() ? m5::types::big_uint16_t(raw[10], raw[11]).get() : -1;
    }
};

}  // namespace vl53l0x

/*!
  @class UnitVL53L0X
  @brief ToF unit
*/
class UnitVL53L0X : public Component, public PeriodicMeasurementAdapter<UnitVL53L0X, vl53l0x::Data> {
    M5_UNIT_COMPONENT_HPP_BUILDER(UnitVL53L0X, 0x29);

public:
    /*!
      @struct config_t
      @brief Settings for begin
     */
    struct config_t {
        //! Operatiing condition
        vl53l0x::Operating operating{vl53l0x::Operating::Condition2V8};
        //! Operation mode
        vl53l0x::Mode mode{vl53l0x::Mode::LongRange};
        //! Start periodic measurement on begin?
        bool start_periodic{true};
    };

    explicit UnitVL53L0X(const uint8_t addr = DEFAULT_ADDRESS)
        : Component(addr), _data{new m5::container::CircularBuffer<vl53l0x::Data>(1)}
    {
        auto ccfg  = component_config();
        ccfg.clock = 400 * 1000U;
        component_config(ccfg);
    }
    virtual ~UnitVL53L0X()
    {
    }

    virtual bool begin() override;
    virtual void update(const bool force = false) override;

    ///@name Settings for begin
    ///@{
    /*! @brief Gets the configration */
    inline config_t config() const
    {
        return _cfg;
    }
    //! @brief Set the configration
    inline void config(const config_t& cfg)
    {
        _cfg = cfg;
    }
    ///@}

    ///@name Measurement data by periodic
    ///@{
    /*!
      @brief Oldest measured range(mm)
      @warning Returns a negative value if there is a range error or empty
     */
    inline int16_t range() const
    {
        return !empty() ? oldest().range() : -1;
    }
    //!@brief Is valid oldest data?
    inline bool valid() const
    {
        return !empty() && oldest().valid();
    }
    //!@brief Oldest measured range status
    inline vl53l0x::RangeStatus range_status() const
    {
        return !empty() ? oldest().range_status() : vl53l0x::RangeStatus::Unknown;
    }
    ///@}

    ///@Properties
    ///@{
    /*!
      @brief Gets the inner mode
    */
    vl53l0x::Mode mode() const
    {
        return _mode;
    }
    ///@}

    ///@name Periodic measurement
    ///@{
    /*!
      @brief Start periodic measurement in the current settings
      @return True if successful
    */
    inline bool startPeriodicMeasurement()
    {
        return PeriodicMeasurementAdapter<UnitVL53L0X, vl53l0x::Data>::startPeriodicMeasurement();
    }
    /*!
      @brief Start periodic measurement
      @param mode Operation mode
      @return True if successful
    */
    inline bool startPeriodicMeasurement(const vl53l0x::Mode mode)
    {
        return PeriodicMeasurementAdapter<UnitVL53L0X, vl53l0x::Data>::startPeriodicMeasurement(mode);
    }
    /*!
      @brief Stop periodic measurement
      @return True if successful
    */
    inline bool stopPeriodicMeasurement()
    {
        return PeriodicMeasurementAdapter<UnitVL53L0X, vl53l0x::Data>::stopPeriodicMeasurement();
    }
    ///@}

    ///@name Single shot measurement
    ///@{
    /*!
      @brief Measurement single shot in the current settings
      @param[out] data Measuerd data
      @return True if successful
      @warning During periodic detection runs, an error is returned
      @warning Processing is blocked until results are returned
    */
    bool measureSingleshot(vl53l0x::Data& d);
    ///@}

    //! @brief Software reset
    bool softReset();

    /*!
      @brief Write the operation mode
      @param mode Operation mode
      @return True if successful
    */
    bool writeMode(const vl53l0x::Mode mode);

    ///@name Signal range limit
    ///@{
    /*!
      @brief Read the signal range limit
      @param[out] mcps value
      @return True if successful
     */
    bool readSignalRateLimit(float& mcps);
    /*!
      @brief Write the signal range limit
      @param mcps value
      @return True if successful
     */
    bool writeSignalRateLimit(const float mcps);
    ///@}

    ///@warning Handling warning
    ///@name I2C Address
    ///@{
    /*!
      @brief Read the I2C address
      @param i2c_address[out] I2C address
      @return True if successful
     */
    bool readI2CAddress(uint8_t& i2c_address);
    /*!
      @brief Change unit I2C address
      @param i2c_address I2C address
      @return True if successful
    */
    bool changeI2CAddress(const uint8_t i2c_address);
    ///@}

protected:
    bool start_periodic_measurement();
    inline bool start_periodic_measurement(const vl53l0x::Mode mode)
    {
        return writeMode(mode) && start_periodic_measurement();
    }
    bool stop_periodic_measurement();

    bool read_data_ready_status();
    bool read_measurement(vl53l0x::Data& d);

    bool write_default_values();
    bool perform_single_ref_calibration(bool VHV);
    bool write_vcsel_period_range(const uint8_t pre_pclk, const uint8_t final_pclk);

    bool soft_reset();

    M5_UNIT_COMPONENT_PERIODIC_MEASUREMENT_ADAPTER_HPP_BUILDER(UnitVL53L0X, vl53l0x::Data);

protected:
    vl53l0x::Mode _mode{vl53l0x::Mode::Unknown};
    uint8_t _stop{};  // stop variable
    std::unique_ptr<m5::container::CircularBuffer<vl53l0x::Data>> _data{};
    config_t _cfg{};
};

///@cond
namespace vl53l0x {
namespace command {
constexpr uint8_t SYSTEM_RANGE_START{0x00};
constexpr uint8_t SYSTEM_SEQUENCE_CONFIG{0x01};
constexpr uint8_t SYSTEM_INTERMEASUREMENT_PERIOD{0x04};
constexpr uint8_t SYSTEM_INTERRUPT_CONFIG_GPIO{0x0A};
constexpr uint8_t SYSTEM_INTERRUPT_CLEAR{0x0B};
constexpr uint8_t RESULT_INTERRUPT_STATUS{0x13};
constexpr uint8_t RESULT_RANGE_STATUS{0x14};
constexpr uint8_t RESULT_RANGE_STATUS_RESULT{0x1E};
constexpr uint8_t ALGO_PHASECAL_CONFIG_TIMEOUT{0x30};
constexpr uint8_t GLOBAL_CONFIG_VCSEL_WIDTH{0x32};
constexpr uint8_t FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT{0x44};
constexpr uint8_t MSRC_CONFIG_TIMEOUT_MACROP{0x46};
constexpr uint8_t FINAL_RANGE_CONFIG_VALID_PHASE_LOW{0x47};
constexpr uint8_t FINAL_RANGE_CONFIG_VALID_PHASE_HIGH{0x48};
constexpr uint8_t PRE_RANGE_CONFIG_VCSEL_PERIOD{0x50};
constexpr uint8_t PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI{0x51};
constexpr uint8_t PRE_RANGE_CONFIG_VALID_PHASE_LOW{0x56};
constexpr uint8_t PRE_RANGE_CONFIG_VALID_PHASE_HIGH{0x57};
constexpr uint8_t MSRC_CONFIG_CONTROL{0x60};

constexpr uint8_t FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI{0x71};
constexpr uint8_t GPIO_HV_MUX_ACTIVE_HIGH{0x84};
constexpr uint8_t I2C_SLAVE_DEVICE_ADDRESS{0x8A};
constexpr uint8_t VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV{0x89};
constexpr uint8_t SOFT_RESET{0xBF};
constexpr uint8_t MODEL_ID{0xC0};

constexpr uint8_t ALGO_PHASECAL_LIM{0x30};
}  // namespace command
}  // namespace vl53l0x
///@endcond

}  // namespace unit
}  // namespace m5
#endif
