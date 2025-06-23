//
//Created by Toni Odujinrin on 2025-06-22
//
/**
 * @file 
 * Class definition for IMU Interface, provides funtions to read messages from UMRT's Embedded IMU 
 */ 


#include "../include/umrt_imu_interface/imu.h"

using std::int16_t;
using std::uint32_t;
using std::uint8_t;

Imu_Interface::Imu_Interface(const std::string& file_addr, uint8_t slave_addr) { 
    fd = open(file_addr.c_str(), O_RDWR); 
    if (fd < 0) {
        BOOST_LOG_TRIVIAL(error) << "[x] i2c file not found, using " << file_addr; 
        throw std::runtime_error("[x] could not find i2c file"); 
    }

    // Set slave address
    if (ioctl(fd, I2C_SLAVE, slave_addr) < 0) {
        BOOST_LOG_TRIVIAL(error) << "[x] Could not set I2C slave address"; 
        throw std::runtime_error("[x] could not set I2C slave address"); 
    }

    // Verify chip ID
    int32_t chip_id = i2c_smbus_read_byte_data(fd, CHIP_ID_REG);
    if (chip_id != CHIP_ID_VAL) {
        BOOST_LOG_TRIVIAL(error) << "[x] Invalid chip ID: " << (int)chip_id << ", expected "<<CHIP_ID_VAL;
        throw std::runtime_error("[x] Invalid chip ID for imu sensor"); 
    }

    // Perform power-on reset
    i2c_smbus_write_byte_data(fd, SYS_TRIGGER_REG, SYS_TRIGGER_RESET_VALUE);
    std::this_thread::sleep_for(std::chrono::milliseconds(650)); // Wait for reset

    // Set operating mode to IMU
    i2c_smbus_write_byte_data(fd, OPR_MODE_REG, OPR_MODE_FUSION_VAL); 
    std::this_thread::sleep_for(std::chrono::milliseconds(650)); // Wait for mode switch

    // Wait for calibration
    while (!check_calibration()) {
        BOOST_LOG_TRIVIAL(warning) << "[o] Waiting for calibration...";
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    BOOST_LOG_TRIVIAL(info) << "[o] IMU calibration complete";
}

Imu_Interface::~Imu_Interface() {
    if(fd >= 0){
        close(fd);   
    }
    fd = -1; 
}

bool Imu_Interface::check_calibration() {
    int32_t calib_status = i2c_smbus_read_byte_data(fd, CALIB_STAT_REG);
    if (calib_status < 0) {
        BOOST_LOG_TRIVIAL(error) << "[x] Failed to read calibration status";
        return false;
    }
    uint8_t sys_cal = (calib_status >> 6) & 0x03;
    uint8_t gyr_cal = (calib_status >> 4) & 0x03;
    uint8_t acc_cal = (calib_status >> 2) & 0x03;
    uint8_t mag_cal = calib_status & 0x03;
    BOOST_LOG_TRIVIAL(info) << "Calibration: Sys=" << (int)sys_cal << ", Gyr=" << (int)gyr_cal << ", Acc=" << (int)acc_cal << ", Mag=" << (int)mag_cal;
    return (acc_cal == 3 && gyr_cal == 3 && mag_cal == 3); // Require full calibration
}

int16_t Imu_Interface::read_16_bit_reg(const uint8_t& reg_l, const int& fd) {
    if(fd == -1){
         BOOST_LOG_TRIVIAL(error) << "[x] i2c file not found, using fd:"<<fd; 
         throw std::runtime_error("[x] i2c file not found, using fd:"); 
    }
    int retries = 3;
    uint8_t val[2]; 
    while (retries--) {   //retry i2c read 3 times then report fail 
        i2c_smbus_read_i2c_block_data(fd,reg_l,2,val); 
        if (val[1] >= 0 && val[0] >= 0) {
            return (static_cast<int16_t>((val[1]) << 8) | val[0]);
        }
        BOOST_LOG_TRIVIAL(warning) << "[o] I2C block read retry starting from register" << (int)reg_l;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    BOOST_LOG_TRIVIAL(error) << "[x] Failed to read registers ";
    return 0;
}

std::array<int16_t,3> Imu_Interface::get_mag_xyz_raw() {
    int16_t mag_x_full = read_16_bit_reg(MAG_X_REG_L, fd); 
    int16_t mag_y_full = read_16_bit_reg(MAG_Y_REG_L, fd); 
    int16_t mag_z_full = read_16_bit_reg(MAG_Z_REG_L, fd); 
    std::array<int16_t, 3> mag_xyz_raw = {mag_x_full, mag_y_full, mag_z_full};
    return mag_xyz_raw; 
}

std::array<double,3> Imu_Interface::get_mag_xyz(){
    std::array<int16_t,3> raw_mag = get_mag_xyz_raw(); 
    std::array<double,3> scaled_mag = {raw_mag[0]*MAG_SCALE_VAL,raw_mag[1]*MAG_SCALE_VAL,raw_mag[2]*MAG_SCALE_VAL}; 
    return scaled_mag; 
}

std::array<int16_t,3> Imu_Interface::get_acc_xyz_raw() {
    int16_t acc_x_full = read_16_bit_reg(ACC_X_REG_L, fd); 
    int16_t acc_y_full = read_16_bit_reg(ACC_Y_REG_L, fd); 
    int16_t acc_z_full = read_16_bit_reg(ACC_Z_REG_L, fd); 
    std::array<int16_t, 3> acc_xyz_raw = {acc_x_full, acc_y_full, acc_z_full};
    return acc_xyz_raw; 
}

std::array<double,3> Imu_Interface::get_acc_xyz(){
    std::array<int16_t,3> raw_acc = get_acc_xyz_raw(); 
    std::array<double,3> scaled_acc = {raw_acc[0]*ACC_SCALE_VAL,raw_acc[1]*ACC_SCALE_VAL,raw_acc[2]*ACC_SCALE_VAL}; 
    return scaled_acc; 
}

std::array<int16_t,3> Imu_Interface::get_gyr_xyz_raw() {
    int16_t gyr_x_full = read_16_bit_reg(GYR_X_REG_L, fd);  
    int16_t gyr_y_full = read_16_bit_reg(GYR_Y_REG_L, fd);  
    int16_t gyr_z_full = read_16_bit_reg(GYR_Z_REG_L, fd);  
    std::array<int16_t, 3> gyr_xyz_raw = {gyr_x_full, gyr_y_full, gyr_z_full};
    return gyr_xyz_raw; 
}

std::array<double,3> Imu_Interface::get_gyr_xyz(){
    std::array<int16_t,3> raw_gyr = get_gyr_xyz_raw(); 
    std::array<double,3> scaled_gyr = {raw_gyr[0]*GYR_SCALE_VAL,raw_gyr[1]*GYR_SCALE_VAL,raw_gyr[2]*GYR_SCALE_VAL}; 
    return scaled_gyr; 
}


std::array<int16_t,3> Imu_Interface::get_lin_acc_xyz_raw() {
    int16_t lin_acc_x_full = read_16_bit_reg(LIN_ACC_X_REG_L, fd); 
    int16_t lin_acc_y_full = read_16_bit_reg(LIN_ACC_Y_REG_L, fd); 
    int16_t lin_acc_z_full = read_16_bit_reg(LIN_ACC_Z_REG_L, fd); 
    std::array<int16_t, 3> lin_acc_xyz_raw = {lin_acc_x_full, lin_acc_y_full, lin_acc_z_full};
    return lin_acc_xyz_raw; 

}

std::array<double,3> Imu_Interface::get_lin_acc_xyz(){
    std::array<int16_t,3> raw_lin_acc = get_lin_acc_xyz_raw(); 
    std::array<double,3> scaled_lin_acc = {raw_lin_acc[0]*ACC_SCALE_VAL,raw_lin_acc[1]*ACC_SCALE_VAL,raw_lin_acc[2]*ACC_SCALE_VAL}; 
    return scaled_lin_acc; 
}

std::array<int16_t,4> Imu_Interface::get_qua_xyzw_raw(){
    int16_t qua_x_full = read_16_bit_reg(QUA_X_REG_L, fd); 
    int16_t qua_y_full = read_16_bit_reg(QUA_Y_REG_L, fd); 
    int16_t qua_z_full = read_16_bit_reg(QUA_Z_REG_L, fd); 
    int16_t qua_w_full = read_16_bit_reg(QUA_W_REG_L, fd); 
    std::array<int16_t, 4> qua_xyzw_raw = {qua_x_full, qua_y_full, qua_z_full, qua_w_full};
    return qua_xyzw_raw; 
}

std::array<double,4> Imu_Interface::get_qua_xyzw(){
    std::array<int16_t,4> raw_qua_xyzw = get_qua_xyzw_raw(); 
    std::array<double,4> scaled_qua_xyzw = {raw_qua_xyzw[0]/QUAT_SCALE_VAL,raw_qua_xyzw[1]/QUAT_SCALE_VAL,raw_qua_xyzw[2]/QUAT_SCALE_VAL,raw_qua_xyzw[3]/QUAT_SCALE_VAL}; 
    return scaled_qua_xyzw; 
}

std::array<int16_t,3> Imu_Interface::get_eul_rph_raw(){
    int16_t eul_roll_full = read_16_bit_reg(EUL_ROLL_REG_L, fd); 
    int16_t eul_pitch_full = read_16_bit_reg(EUL_PITCH_REG_L, fd); 
    int16_t eul_head_full = read_16_bit_reg(EUL_HEAD_REG_L, fd);  
    std::array<int16_t, 3> eul_rph_raw = {eul_roll_full, eul_pitch_full, eul_head_full};
    return eul_rph_raw; 
}

std::array<double,3> Imu_Interface::get_eul_rph(){
    std::array<int16_t,3> raw_eul_rph = get_eul_rph_raw(); 
    std::array<double,3> scaled_eul_rph = {raw_eul_rph[0]*EUL_SCALE_VAL,raw_eul_rph[1]*EUL_SCALE_VAL,raw_eul_rph[2]*EUL_SCALE_VAL}; 
    return scaled_eul_rph; 
}

double Imu_Interface::get_roll() {
    int16_t roll_full = read_16_bit_reg(ROLL_REG_L, fd);  
    double roll_deg = roll_full*GYR_SCALE_VAL; // Convert to degrees
    return roll_deg; 
}
 
double Imu_Interface::get_pitch() {
    int16_t pitch_full = read_16_bit_reg(PITCH_REG_L, fd);  
    double pitch_deg = pitch_full*GYR_SCALE_VAL; // Convert to degrees
    return pitch_deg; 
}

double Imu_Interface::get_head() {
    int16_t head_full = read_16_bit_reg(HEAD_REG_L, fd);  
    double head_deg = head_full*GYR_SCALE_VAL; // Convert to degrees
    return head_deg; 
}