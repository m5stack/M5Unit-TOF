/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file M5UnitUnifiedToF.hpp
  @brief Main header of M5UnitToF

  @mainpage M5Unit-TOF
  Library for UnitTOF using M5UnitUnified.
*/
#ifndef M5_UNIT_UNIFIED_TOF_HPP
#define M5_UNIT_UNIFIED_TOF_HPP

#include "unit/unit_VL53L0X.hpp"
#include "unit/unit_VL53L1X.hpp"

/*!
  @namespace m5
  @brief Top level namespace of M5stack
 */
namespace m5 {

/*!
  @namespace unit
  @brief Unit-related namespace
 */
namespace unit {

using UnitToF   = m5::unit::UnitVL53L0X;
using UnitToF4M = m5::unit::UnitVL53L1X;

}  // namespace unit
}  // namespace m5
#endif
