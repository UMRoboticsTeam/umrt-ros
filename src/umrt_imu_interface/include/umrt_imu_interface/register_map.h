//
//Created by Toni Odujinrin on 2025-06-22
//
/**
 * @file 
 * registers and imu specific constants
 */ 
#ifndef REGISTER_MAP_H
#define REGISTER_MAP_H 

//sensor registers 
constexpr uint8_t LIN_ACC_X_REG_H = 0x29; 
constexpr uint8_t LIN_ACC_X_REG_L = 0x28; 
constexpr uint8_t LIN_ACC_Y_REG_H = 0x2B; 
constexpr uint8_t LIN_ACC_Y_REG_L = 0x2A; 
constexpr uint8_t LIN_ACC_Z_REG_H = 0x2D; 
constexpr uint8_t LIN_ACC_Z_REG_L = 0x2C; 
constexpr uint8_t PITCH_REG_H = 0x1F;
constexpr uint8_t PITCH_REG_L = 0x1E;
constexpr uint8_t ROLL_REG_H = 0x1D; 
constexpr uint8_t ROLL_REG_L = 0x1C;
constexpr uint8_t HEAD_REG_H = 0x1B; 
constexpr uint8_t HEAD_REG_L = 0x1A; 
constexpr uint8_t GYR_X_REG_H = 0x15;
constexpr uint8_t GYR_X_REG_L = 0x14; 
constexpr uint8_t GYR_Y_REG_H = 0x17; 
constexpr uint8_t GYR_Y_REG_L = 0x16; 
constexpr uint8_t GYR_Z_REG_H = 0x19;
constexpr uint8_t GYR_Z_REG_L = 0x18; 
constexpr uint8_t MAG_X_REG_H = 0x0F;  
constexpr uint8_t MAG_X_REG_L = 0x0E;
constexpr uint8_t MAG_Y_REG_H = 0x11; 
constexpr uint8_t MAG_Y_REG_L = 0x10; 
constexpr uint8_t MAG_Z_REG_H = 0x13; 
constexpr uint8_t MAG_Z_REG_L = 0x12; 
constexpr uint8_t ACC_X_REG_H = 0x09; 
constexpr uint8_t ACC_X_REG_L = 0x08;
constexpr uint8_t ACC_Y_REG_H = 0x0B; 
constexpr uint8_t ACC_Y_REG_L = 0x0A; 
constexpr uint8_t ACC_Z_REG_H = 0x0D; 
constexpr uint8_t ACC_Z_REG_L = 0x0C; 
constexpr uint8_t QUA_X_REG_L = 0x22; 
constexpr uint8_t QUA_X_REG_H = 0x23; 
constexpr uint8_t QUA_Y_REG_L = 0x24; 
constexpr uint8_t QUA_Y_REG_H = 0x25;
constexpr uint8_t QUA_Z_REG_L = 0x26; 
constexpr uint8_t QUA_Z_REG_H = 0x27;
constexpr uint8_t QUA_W_REG_L = 0x20; 
constexpr uint8_t QUA_W_REG_H = 0x21;
constexpr uint8_t EUL_PITCH_REG_H = 0x1F; 
constexpr uint8_t EUL_PITCH_REG_L = 0x1E; 
constexpr uint8_t EUL_ROLL_REG_H = 0x1D; 
constexpr uint8_t EUL_ROLL_REG_L = 0x1C; 
constexpr uint8_t EUL_HEAD_REG_H = 0x1B; 
constexpr uint8_t EUL_HEAD_REG_L = 0x1A;


//control and calibration registers
constexpr uint8_t OPR_MODE_REG = 0x3D; 
constexpr uint8_t OPR_MODE_FUSION_VAL = 0x08; 
constexpr uint8_t CHIP_ID_REG = 0x00; 
constexpr uint8_t SYS_STATUS_REG = 0x39; 
constexpr uint8_t CALIB_STAT_REG = 0x35; 
constexpr uint8_t SYS_TRIGGER_REG = 0x3F;
constexpr uint8_t SYS_TRIGGER_RESET_VALUE = 0x20; 
constexpr uint8_t CHIP_ID_VAL = 0xA0;

//scale values
constexpr double QUAT_SCALE_VAL = 16384.0; 
constexpr double EUL_SCALE_VAL = 0.0625; 
constexpr double ACC_SCALE_VAL = 0.01; 
constexpr double GYR_SCALE_VAL = 0.0625; 
constexpr double MAG_SCALE_VAL = 0.0625; 



#endif