/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_VL53L1X.hpp
  @brief VL53L1X Unit for M5UnitUnified
 */
#ifndef M5_UNIT_TOF_UNIT_VL53L1X_HPP
#define M5_UNIT_TOF_UNIT_VL53L1X_HPP

#include <M5UnitComponent.hpp>
#include <m5_utility/container/circular_buffer.hpp>
#include <m5_utility/types.hpp>
#include <array>
#include <limits>  // NaN

namespace m5 {
namespace unit {

namespace vl53l1x {

/*!
  @enum Operating
  @brief Operating condition
 */
enum class Operating : uint8_t {
    ConditionStandard,  //!< Standard mode (1.6 - 1.9 V)
    Condition2V8        //!< 2V8 mode (2.6 - 3.5 V) (as default)
};

/*!
  @enum Distance
  @brief Distance mode
 */
enum class Distance : int8_t {
    Unknown = -1,
    Short,  //!< Short (limited to 1.3m)
    Long,   //!< Long  (limited to 4.0m)
};

/*!
  @enum Timing
  @brief Timeing budget for ranging
  @warning This value does not directly represent the measurement interval
  @warning inter-measurement period needs to be set
*/
enum class Timing : int8_t {
    BudgetUnknown = -1,
    Budget15ms,   //!< 15 ms (Only possible when Short mode)
    Budget20ms,   //!< 20 ms
    Budget33ms,   //!< 33 ms
    Budget50ms,   //!< 50 ms
    Budget100ms,  //!< 100 ms
    Budget200ms,  //!< 200 ms
    Budget500ms,  //!< 500 ms
};

/*!
  @enum RangeStatus
  @brief Range status interpretation
 */
enum class RangeStatus : uint8_t {
    OK,
    SigmaFailure,
    SignalFailure,
    RangeValidMinRangeClipped,
    OutOfBounds,
    HardwareFailure,
    Unknown6,
    Wraparound,
    Unknown8,
    XtalkSignalFailure,
    SynchronizationInt,
    Unknown11,
    Unknown12,
    MinRangeFailure,
    Unknown255 = 255,
};

/*!
  @enum Window
  @brief Window detection mode
  - Below
    - If object distance > ThreshLow or no object found: no report
    - If object distance < ThreshLow and object found: report
  - Beyond
    - If object distance < ThreshHigh or no object found: no report
    - If object distance > ThreshHigh and object found: report
  - Out
    - ThreshLow < object distance < ThreshHigh: no report
    - ThreshLow > object distance > ThreshHigh: report
  - In
    - ThreshLow > object distance > ThreshHigh: no report
    - ThreshLow < object distance < ThreshHigh: report
*/
enum class Window : uint8_t {
    Below,           //!< Below a certain distance
    Beyond,          //!< Beyond a certain distance
    Out,             //!< Out of distance range (min/max), "out of Window"
    In,              //!< Within the distance range (min/max), "inside Window"
    Regular = 0x20,  //!< Regular ranging (without window)
};

/*!
  @struct Data
  @brief Measurement data group
 */
struct Data {
    std::array<uint8_t, 17> raw{};  //!< RAW data
    /*!
      @brief Range status
      @note Get the converted value from the raw value
     */
    RangeStatus range_status() const;
    /*!
      @brief Is data valid?
     */
    inline bool valid() const
    {
        return raw[0] == 9;
    }
    /*!
      @brief range (mm)
      @retval >= 0 range (mm)
      @retval < 0 Returns a negative value if there is a range error
     */
    inline int16_t range() const
    {
        return valid() ? m5::types::big_uint16_t(raw[13], raw[14]).get() : -1;
    }
};

}  // namespace vl53l1x

/*!
  @class UnitVL53L1X
  @brief ToF4M unit
*/
class UnitVL53L1X : public Component, public PeriodicMeasurementAdapter<UnitVL53L1X, vl53l1x::Data> {
    M5_UNIT_COMPONENT_HPP_BUILDER(UnitVL53L1X, 0x29);

public:
    /*!
      @struct config_t
      @brief Settings for begin
     */
    struct config_t {
        //! Operatiing condition
        vl53l1x::Operating operating{vl53l1x::Operating::Condition2V8};
        //! Distance mode
        vl53l1x::Distance distance{vl53l1x::Distance::Long};
        //! Timing budget
        vl53l1x::Timing timing_budget{vl53l1x::Timing::Budget100ms};
        //! Start periodic measurement on begin?
        bool start_periodic{true};
        //! Caliblate offset if start on begin
        bool calibrate_offset{false};
        //! Caliblate crosstalk if start on begin
        bool calibrate_xtalk{false};
    };

    explicit UnitVL53L1X(const uint8_t addr = DEFAULT_ADDRESS)
        : Component(addr), _data{new m5::container::CircularBuffer<vl53l1x::Data>(1)}
    {
        auto ccfg  = component_config();
        ccfg.clock = 400 * 1000U;
        component_config(ccfg);
    }
    virtual ~UnitVL53L1X()
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
    inline vl53l1x::RangeStatus range_status() const
    {
        return !empty() ? oldest().range_status() : vl53l1x::RangeStatus::Unknown255;
    }
    ///@}

    ///@name Properties
    ///@{
    /*!
      @brief Gets the inner distance mode
      @note Updated by writeDistanceMode
    */
    inline vl53l1x::Distance distanceMode() const
    {
        return _distance;
    }
    /*!
      @brief Gets the inner timing budget
      @note Updated by writeTimingBudget
    */
    inline vl53l1x::Timing timingBudget() const
    {
        return _tb;
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
        return PeriodicMeasurementAdapter<UnitVL53L1X, vl53l1x::Data>::startPeriodicMeasurement();
    }
    /*!
      @brief Start periodic measurement
      @param dist Distance mode
      @param tb Timing budget
      @return True if successful
    */
    inline bool startPeriodicMeasurement(const vl53l1x::Distance dist,
                                         const vl53l1x::Timing tb = vl53l1x::Timing::Budget50ms)
    {
        return PeriodicMeasurementAdapter<UnitVL53L1X, vl53l1x::Data>::startPeriodicMeasurement(dist, tb);
    }
    /*!
      @brief Stop periodic measurement
      @return True if successful
    */
    inline bool stopPeriodicMeasurement()
    {
        return PeriodicMeasurementAdapter<UnitVL53L1X, vl53l1x::Data>::stopPeriodicMeasurement();
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
    bool measureSingleshot(vl53l1x::Data& d);
    ///@}

    /*!
      @brief Read the distance mode
      @param[out] d distance
      @return True if successful
    */
    bool readDistanceMode(vl53l1x::Distance& d);
    /*!
      @brief Write the distance mode
      @param d distance
      @return True if successful
    */
    bool writeDistanceMode(const vl53l1x::Distance d);

    //! @brief Software reset
    bool softReset();

    ///@name Offset
    ///@{
    /*!
      @brief Calculate and write calibration offsets for a given target value
      @param[out] offset Calculated offset value
      @param targetmm Target distance (mm)
      @return True if successful
     */
    bool calibrateOffset(int16_t& offset, const uint16_t targetmm = 100);
    /*!
      @brief Read the offset
      @param[out] offset Offset value
      @return True if successful
     */
    bool readOffset(int16_t& offset);
    /*!
      @brief Write the offset
      @param offset Offset value
      @return True if successful
      @warning Valid range -1024 - 1023
     */
    bool writeOffset(const int16_t offset);
    ///@}

    ///@name Crosstalk
    ///@{
    /*!
      @brief Calculate and write calibration crosstalk for a given target value
      @param[out] xtalk Calculated crosstalk value
      @param targetmm Target distance (mm)
      @return True if successful
     */
    bool calibrateXtalk(uint16_t& xtalk, const uint16_t targetmm = 100);
    /*!
      @brief Read the crosstalk
      @param[out] xtalk Crosstalk value
      @return True if successful
     */
    bool readXtalk(uint16_t& xtalk);
    /*!
      @brief Write the crosstalk
      @param xtalk Crosstalk value
      @return True if successful
     */
    bool writeXtalk(const uint16_t xtalk);
    ///@}

    ///@name Timing considerations
    ///@{
    /*!
      @brief Read the timing budget
      @param[out] tb timing budget
      @return True if successful
     */
    bool readTimingBudget(vl53l1x::Timing& tb);
    /*!
      @brief Write the timing budget
      @param tb Timing budget
      @return True if successful
      @post Need to call writeInterMeasurementPeriod for adjust inter-measurement period
    */
    inline bool writeTimingBudget(const vl53l1x::Timing tb)
    {
        return write_timing_budget(tb, distanceMode());
    }
    /*!
      @brief Read the inter-measurement period(IMP)
      @param[out] ms period (ms)
      @return True if successful
    */
    bool readInterMeasurementPeriod(uint16_t& ms);
    /*!
      @brief Write the inter-measurement period(IMP)
      @param ms period (ms)
      @return True if successful
      @warning The IMP must gerater than or equal to the TB otherwise the actual IMP is double the expected value
    */
    bool writeInterMeasurementPeriod(const uint16_t ms);
    ///@}

    ///@name Window
    ///@{
    /*!
      @brief Read the window detection mode
      @param[out] window Window mode
      @return True if successful
    */
    bool readDistanceThresholdWindow(vl53l1x::Window& window);
    /*!
      @brief Read the lower distance threshould
      @param[out] mm distance (mm)
      @return True if successful
    */
    bool readDistanceThresholdLow(uint16_t& mm);
    /*!
      @brief Read the higher distance threshould
      @param[out] mm distance (mm)
      @return True if successful
    */
    bool readDistanceThresholdHigh(uint16_t& mm);
    /*!
      @brief Write the threshold and window detection mode
      @param window window mode
      @param low lower distance threshold (mm)
      @param high lower distance threshold (mm)
      @return True if successful
    */
    bool writeDistanceThreshold(const vl53l1x::Window window, const uint16_t low, const uint16_t high);
    /*!
      @brief Go back to the regular ranging mode
      @return True if successful
    */
    inline bool clearDistanceThreshold()
    {
        return writeDistanceThreshold(vl53l1x::Window::Regular, 0, 0);
    }
    ///@}

    /*!
      ROI coodinate
      @verbatim
      128,136,144,152,160,168,176,184,   192,200,208,216,224,232,240,248
      129,137,145,153,161,169,177,185,   193,201,209,217,225,233,241,249
      130,138,146,154,162,170,178,186,   194,202,210,218,226,234,242,250
      131,139,147,155,163,171,179,187,   195,203,211,219,227,235,243,251
      132,140,148,156,164,172,180,188,   196,204,212,220,228,236,244,252
      133,141,149,157,165,173,181,189,   197,205,213,221,229,237,245,253
      134,142,150,158,166,174,182,190,   198,206,214,222,230,238,246,254
      135,143,151,159,167,175,183,191,   199,207,215,223,231,239,247,255

      127,119,111,103, 95, 87, 79, 71,    63, 55, 47, 39, 31, 23, 15,  7
      126,118,110,102, 94, 86, 78, 70,    62, 54, 46, 38, 30, 22, 14,  6
      125,117,109,101, 93, 85, 77, 69,    61, 53, 45, 37, 29, 21, 13,  5
      124,116,108,100, 92, 84, 76, 68,    60, 52, 44, 36, 28, 20, 12,  4
      123,115,107, 99, 91, 83, 75, 67,    59, 51, 43, 35, 27, 19, 11,  3
      122,114,106, 98, 90, 82, 74, 66,    58, 50, 42, 34, 26, 18, 10,  2
      121,113,105, 97, 89, 81, 73, 65,    57, 49, 41, 33, 25, 17,  9,  1
      120,112,104, 96, 88, 80, 72, 64,    56, 48, 40, 32, 24, 16,  8,  0
      @endverbatim
     */
    ///@name ROI(Region Of Interest)
    ///@{
    /*!
      @brief Read the ROI width and height
      @param[out] wid width
      @param[out] hgt height
      @return True if successful
    */
    bool readROI(uint8_t& wid, uint8_t& hgt);
    /*!
      @brief Read the center of the ROI
      @param[out] center ROI coordinates
      @return True if successful
     */
    bool readROICenter(uint8_t& center);
    /*!
      @brief Write the ROI width and height
      @param wid width
      @param hgt height
      @return True if successful
      @note If wid or hgt is greater than 10, the center position is also changed
      @warning Valid param between 4 and 16
     */
    bool writeROI(const uint8_t wid, const uint8_t hgt);
    /*!
      @brief Write the center of the ROI
      @param center ROI coordinates
      @return True if successful
      @warning If the center is outside the ROI size boundary, the measurement returns an error
     */
    bool writeROICenter(const uint8_t center);
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
    bool start_periodic_measurement(const vl53l1x::Distance dist, const vl53l1x::Timing tb);
    bool stop_periodic_measurement();

    bool write_operating_condition(const vl53l1x::Operating oc);
    bool write_timing_budget(const vl53l1x::Timing tb, const vl53l1x::Distance distance);

    bool read_data_ready_status();
    bool read_measurement(vl53l1x::Data& d);

    bool soft_reset();
    bool write_default_values();
    bool wait_booted();

    M5_UNIT_COMPONENT_PERIODIC_MEASUREMENT_ADAPTER_HPP_BUILDER(UnitVL53L1X, vl53l1x::Data);

protected:
    vl53l1x::Distance _distance{vl53l1x::Distance::Unknown};
    vl53l1x::Timing _tb{vl53l1x::Timing::BudgetUnknown};

    std::unique_ptr<m5::container::CircularBuffer<vl53l1x::Data>> _data{};
    config_t _cfg{};
};

///@cond
namespace vl53l1x {
namespace command {
// clang-format off
constexpr uint16_t SOFT_RESET                                                   {0x0000};
constexpr uint16_t I2C_SLAVE_DEVICE_ADDRESS                                     {0x0001};
constexpr uint16_t OSC_MEASURED_FAST_OSC_FREQUENCY                              {0x0006};
constexpr uint16_t VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND                         {0x0008};
constexpr uint16_t VHV_CONFIG_INIT                                              {0x000B};
constexpr uint16_t ALGO_CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS                {0x0016};// [15:0] fixed point 7.9)
constexpr uint16_t ALGO_CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS            {0x0018};
constexpr uint16_t ALGO_CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS            {0x001A};
constexpr uint16_t ALGO_PART_TO_PART_RANGE_OFFSET_MM                            {0x001E};// [12:0] fixed point 11.2
constexpr uint16_t MM_CONFIG_INNER_OFFSET_MM                                    {0x0020};
constexpr uint16_t MM_CONFIG_OUTER_OFFSET_MM                                    {0x0022};
constexpr uint16_t DSS_CONFIG_TARGET_TOTAL_RATE_MCPS                            {0x0024};
constexpr uint16_t EXTSUP_CONFIG                                                {0x002E};
constexpr uint16_t GPIO_HV_MUX_CTRL                                             {0x0030};
constexpr uint16_t GPIO_TIO_HV_STATUS                                           {0x0031};
constexpr uint16_t SIGMA_ESTIMATOR_EFFECTIVE_PULSE_WIDTH_NS                     {0x0036};
constexpr uint16_t SIGMA_ESTIMATOR_EFFECTIVE_AMBIENT_WIDTH_NS                   {0x0037};
constexpr uint16_t ALGO_CROSSTALK_COMPENSATION_VALID_HEIGHT_MM                  {0x0039};
constexpr uint16_t ALGO_RANGE_IGNORE_VALID_HEIGHT_MM                            {0x003E};
constexpr uint16_t ALGO_RANGE_MIN_CLIP                                          {0x003F};
constexpr uint16_t ALGO_CONSISTENCY_CHECK_TOLERANCE                             {0x0040};
constexpr uint16_t SYSTEM_INTERRUPT_CONFIG_GPIO                                 {0x0046};
constexpr uint16_t CAL_CONFIG_VCSEL_START                                       {0x0047};
constexpr uint16_t PHASECAL_CONFIG_TIMEOUT_MACROP                               {0x004B};
constexpr uint16_t PHASECAL_CONFIG_OVERRIDE                                     {0x004D};
constexpr uint16_t DSS_CONFIG_ROI_MODE_CONTROL                                  {0x004F};
constexpr uint16_t SYSTEM_THRESH_RATE_HIGH                                      {0x0050};
constexpr uint16_t SYSTEM_THRESH_RATE_LOW                                       {0x0052};
constexpr uint16_t DSS_CONFIG_MANUAL_EFFECTIVE_SPADS_SELECT                     {0x0054};
constexpr uint16_t DSS_CONFIG_APERTURE_ATTENUATION                              {0x0057};
constexpr uint16_t MM_CONFIG_TIMEOUT_MACROP_A                                   {0x005A};
constexpr uint16_t MM_CONFIG_TIMEOUT_MACROP_B                                   {0x005C};
constexpr uint16_t RANGE_CONFIG_TIMEOUT_MACROP_A                                {0x005E};
constexpr uint16_t RANGE_CONFIG_TIMEOUT_MACROP_A_HI                             {0x005E};
constexpr uint16_t RANGE_CONFIG_VCSEL_PERIOD_A                                  {0x0060};
constexpr uint16_t RANGE_CONFIG_TIMEOUT_MACROP_B                                {0x0061};
constexpr uint16_t RANGE_CONFIG_TIMEOUT_MACROP_B_HI                             {0x0061};
constexpr uint16_t RANGE_CONFIG_VCSEL_PERIOD_B                                  {0x0063};
constexpr uint16_t RANGE_CONFIG_SIGMA_THRESH                                    {0x0064};
constexpr uint16_t RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_MCPS                   {0x0066};
constexpr uint16_t RANGE_CONFIG_VALID_PHASE_HIGH                                {0x0069};
constexpr uint16_t SYSTEM_INTERMEASUREMENT_PERIOD                               {0x006C};
constexpr uint16_t SYSTEM_GROUPED_PARAMETER_HOLD_0                              {0x0071};
constexpr uint16_t SYSTEM_THRESH_HIGH                                           {0x0072};
constexpr uint16_t SYSTEM_THRESH_LOW                                            {0x0074};
constexpr uint16_t SYSTEM_SEED_CONFIG                                           {0x0077};
constexpr uint16_t SD_CONFIG_WOI_SD0                                            {0x0078};
constexpr uint16_t SD_CONFIG_WOI_SD1                                            {0x0079};
constexpr uint16_t SD_CONFIG_INITIAL_PHASE_SD0                                  {0x007A};
constexpr uint16_t SD_CONFIG_INITIAL_PHASE_SD1                                  {0x007B};
constexpr uint16_t SYSTEM_GROUPED_PARAMETER_HOLD_1                              {0x007C};
constexpr uint16_t SD_CONFIG_QUANTIFIER                                         {0x007E};
constexpr uint16_t ROI_CONFIG_USER_ROI_CENTRE_SPAD                              {0x007F};
constexpr uint16_t ROI_CONFIG_USER_ROI_REQUESTED_GLOBAL_XY_SIZE                 {0x0080};
constexpr uint16_t SYSTEM_SEQUENCE_CONFIG                                       {0x0081};
constexpr uint16_t SYSTEM_GROUPED_PARAMETER_HOLD                                {0x0082};
constexpr uint16_t SYSTEM_INTERRUPT_CLEAR                                       {0x0086};
constexpr uint16_t SYSTEM_MODE_START                                            {0x0087};
constexpr uint16_t RESULT_RANGE_STATUS                                          {0x0089}; // result size is 17bytes
constexpr uint16_t RESULT_DSS_ACTUAL_EFFECTIVE_SPADS_SD0                        {0x008C};
constexpr uint16_t RESULT_FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0                {0x0096};
constexpr uint16_t RESULT_PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0   {0x0098};
constexpr uint16_t PHASECAL_RESULT_VCSEL_START                                  {0x00D8};
constexpr uint16_t RESULT_OSC_CALIBRATE_VAL                                     {0x00DE};
constexpr uint16_t FIRMWARE_SYSTEM_STATUS                                       {0x00E5};
//
constexpr uint16_t MODEL_ID                                                     {0x010F};
constexpr uint16_t MODULE_TYPE                                                  {0x0110};
constexpr uint16_t MASK_REVISION                                                {0x0111};
// clang-format on
}  // namespace command
}  // namespace vl53l1x
///@endcond

}  // namespace unit
}  // namespace m5
#endif
