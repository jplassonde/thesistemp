#pragma once
#include <cstdint>


// DRC Settings

////////////
// VALUES //
////////////


constexpr uint8_t SIG_DET_PK_VAL = 1;
constexpr uint8_t SIG_DET_MODE = 0;
constexpr uint8_t SIG_DET_ENA = 1;
constexpr uint8_t NG_ENA = 1;
constexpr uint8_t KNEE_2_ENA = 1;
constexpr uint8_t QR_ENA = 1;
constexpr uint8_t AC_ENA = 0;
constexpr uint8_t DRC_OUT = 3;

// Values - 0x441
constexpr uint8_t GAIN_AR = 0;
constexpr uint8_t GAIN_DCAY = 0;
constexpr uint8_t MIN_ATT_GAIN = 0;
constexpr uint8_t MAX_BOOST_GAIN = 0;

// Values 0x442
constexpr uint8_t NG_MIN_GAIN = 0;
constexpr uint8_t NG_SLOPE = 3;
constexpr uint8_t QR_THRES = 0;
constexpr uint8_t QR_DCAY = 0;
constexpr uint8_t UPPER_SLOPE = 2;
constexpr uint8_t LOWER_SLOPE = 1;

// Values 0x443
constexpr uint8_t KNEE_IN_LVL = 27; //
constexpr uint8_t KNEE_OUT_LVL = 7;

// Values 0x444
constexpr uint8_t NG_KNEE_IN_LVL = 0x0F; // - 52.5 dBFS in
constexpr uint8_t NG_KNEE_OUT_LVL = 0x1C; // = -63 dBFS out

/*
constexpr uint8_t SIG_DET_PK_VAL = 1;
constexpr uint8_t SIG_DET_MODE = 0;
constexpr uint8_t SIG_DET_ENA = 1;
constexpr uint8_t NG_ENA = 1;
constexpr uint8_t KNEE_2_ENA = 1;
constexpr uint8_t QR_ENA = 1;
constexpr uint8_t AC_ENA = 0;
constexpr uint8_t DRC_OUT = 3;

// Values - 0x441
constexpr uint8_t GAIN_AR = 3;
constexpr uint8_t GAIN_DCAY = 0;
constexpr uint8_t MIN_ATT_GAIN = 1;
constexpr uint8_t MAX_BOOST_GAIN = 0;

// Values 0x442
constexpr uint8_t NG_MIN_GAIN = 0;
constexpr uint8_t NG_SLOPE = 3;
constexpr uint8_t QR_THRES = 0;
constexpr uint8_t QR_DCAY = 0;
constexpr uint8_t UPPER_SLOPE = 0;
constexpr uint8_t LOWER_SLOPE = 0;

// Values 0x443
constexpr uint8_t KNEE_IN_LVL = 0x2e;
constexpr uint8_t KNEE_OUT_LVL = 0x2e;

// Values 0x444
constexpr uint8_t NG_KNEE_IN_LVL = 0x0E;
constexpr uint8_t NG_KNEE_OUT_LVL = 0x0E;
*/
///////////////
// POSITIONS //
///////////////


// Pos - 0x440
constexpr uint8_t SIG_DET_PK_POS = 9;
constexpr uint8_t SIG_DET_MODE_POS = 7;
constexpr uint8_t SIG_DET_ENA_POS = 6;
constexpr uint8_t NG_ENA_POS = 8;
constexpr uint8_t KNEE_2_ENA_POS = 5;
constexpr uint8_t QR_ENA_POS = 4;
constexpr uint8_t AC_ENA_POS = 3;

// Pos - 0x441
constexpr uint8_t GAIN_AR_POS = 9;
constexpr uint8_t GAIN_DCAY_POS = 5;
constexpr uint8_t MIN_ATT_GAIN_POS = 2;
constexpr uint8_t MAX_BOOST_GAIN_POS = 0;

// Pos 0x442
constexpr uint8_t NG_MIN_GAIN_POS = 12;
constexpr uint8_t NG_SLOPE_POS = 10;
constexpr uint8_t QR_THRES_POS = 8;
constexpr uint8_t QR_DCAY_POS = 6;
constexpr uint8_t UPPER_SLOPE_POS = 3;
constexpr uint8_t LOWER_SLOPE_POS = 0;

// Pos 0x443
constexpr uint8_t KNEE_IN_LVL_POS = 5;
constexpr uint8_t KNEE_OUT_LVL_POS = 0;
// Pos 0x444
constexpr uint8_t NG_KNEE_IN_LVL_POS = 5;
constexpr uint8_t NG_KNEE_OUT_LVL_POS = 0;

