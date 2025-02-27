/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example using M5UnitUnified for UnitToF/ToF4M/HatTof
*/
// *************************************************************
// Choose one define symbol to match the unit you are using
// *************************************************************
#if !defined(USING_UNIT_TOF) && !defined(USING_UNIT_TOF4M) && !defined(USING_HAT_TOF)
// For UnitToF
// #define USING_UNIT_TOF
// For UnitToF4M
// #define USING_UNIT_TOF4M
// For HatToF
// #define USING_HAT_TOF
#endif
// *************************************************************
#include "main/SimpleDisplay.cpp"
