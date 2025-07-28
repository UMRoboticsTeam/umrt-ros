//
//Created by Toni Odujinrin on 2025-06-22
//
/**
 * @file 
 * Class declaration for IMU Interface, provides funtions to read messages from UMRT's Embedded IMU 
 */ 

#ifndef IMU_H
#define IMU_H

#include <iostream>
extern "C" {
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}
#include <unistd.h>
#include <chrono>
#include <sys/ioctl.h> 
#include <fcntl.h>
#include <thread>
#include <stdexcept> 
#include "register_map.h"
#include <boost/log/trivial.hpp>



class Imu_Interface {
private:
    int fd{-1};
     /**
     * Reads 2 IMU 8-bit register using i2c block read 
     * @param reg_l low register . 
     * @param i2c device file descriptor  
     */

    int16_t read_16_bit_reg(const uint8_t& reg_l, const int& fd); 
    /**
     * Stalls IMU untill gyroscope and accelerometer are fully calibrated
     */
    bool check_calibration(); 


public:
    Imu_Interface() = default;  
      /**
     * Initializes IMU, and begins calibration and device configuration checks
     * @param file_addr file address for the i2c i/o device . 
     * @param slave_addr_param address of the IMU device  
     */
    Imu_Interface(const std::string& file_addr, uint8_t slave_addr_param); 
    Imu_Interface(const Imu_Interface&) = delete;
  
    ~Imu_Interface(); 
    Imu_Interface& operator=(const Imu_Interface&) = delete;

    /**
     * Get scaled magnetometer 3-axis values
     *  @return {mag_x , mag_y, mag_z}
     */ 
    std::array<double, 3> get_mag_xyz(); 


    /**
     * Get raw magnetometer 3-axis values
     *  @return {mag_x , mag_y, mag_z} 
     */  
    std::array<int16_t,3> get_mag_xyz_raw(); 


    /**
     * Get scaled accelerometer 3-axis values
     *  @return {acc_x , acc_y, acc_z} 
     */ 
    std::array<double, 3> get_acc_xyz(); 


    /**
     * Get raw accelerometer 3-axis values
     *  @return {acc_x , acc_y, acc_z} 
     */
    std::array<int16_t,3> get_acc_xyz_raw();


    /**
     * Get scaled gyroscope 3-axis values
     * @return {gyr_x , gyr_y, gyr_z} 
     */ 
    std::array<double, 3> get_gyr_xyz();


    /**
     * Get raw gyroscope 3-axis values
     *  @return {gyr_x , gyr_y, gyr_z} 
     */ 
    std::array<int16_t,3> get_gyr_xyz_raw();


    /**
     * Get scaled linear acceleration 3-axis values
     *  @return {lin_x , lin_y, lin_z} 
     */  
    std::array<double,3> get_lin_acc_xyz(); 


    /**
     * Get raw linear acceleration 3-axis values
     * @return {lin_x , lin_y, lin_z} 
     */ 
    std::array<int16_t,3> get_lin_acc_xyz_raw(); 


    /**
     * Get scaled euler 3-axis values
     * @return {eul_x , eul_y, eul_z} 
     */
    std::array<double,3> get_eul_rph(); 


    /**
     * Get raw euler 3-axis values
     * @return {eul_x , eul_y, eul_z}
     */
    std::array<int16_t,3> get_eul_rph_raw(); 


    /**
     * Get scaled quaternion 4-axis values
     *  @return {quaternion_x , quaternion_y, quaternion_z, quaternion_w}  
     */
    std::array<double,4> get_qua_xyzw(); 

    /**
     * Get raw quaternion 4-axis values
     * @return {quaternion_x , quaternion_y, quaternion_z, quaternion_w}  
     */
    std::array<int16_t,4> get_qua_xyzw_raw();

    /**
     * Get raw pitch
     * @return double value 
     */
    double get_pitch(); 

    /**
     * Get raw roll
     * @return double value 
     */
    double get_roll(); 

    /**
     * Get raw head
     * @return double value 
     */
    double get_head();  
};

#endif