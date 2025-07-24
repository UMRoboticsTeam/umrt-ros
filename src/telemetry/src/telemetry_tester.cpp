#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <ctime>
#include <cstdlib>
#include "rclcpp/rclcpp.hpp"
#include "telemetry_interfaces/msg/imu.hpp" 
#include "telemetry_interfaces/msg/gps.hpp"
typedef telemetry_interfaces::msg::Gps gps_message; 
typedef telemetry_interfaces::msg::Imu imu_message;


class GPS_IMU_publisher: public rclcpp::Node
{
  public:
    GPS_IMU_publisher()
    : Node("gps_imu_publisher")
    {
      gps_publisher = this->create_publisher<gps_message>("gps", 10);
      imu_publisher = this->create_publisher<imu_message>("imu",10); 
      timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&GPS_IMU_publisher::timer_callback, this));
    }
  private:
    void timer_callback()
    {
      std::srand(std::time(nullptr));
      auto gps = gps_message();
      auto imu = imu_message(); 
      gps.longitude = std::rand()%100; 
      gps.latitude = std::rand()%100; 
      imu.roll = std::rand()%150; 
      imu.pitch = std::rand()%150; 
      imu.yaw = std::rand()%150; 
      imu.heading = std::rand()%360; 
  	  

      gps_publisher->publish(gps);
      imu_publisher->publish(imu); 
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<gps_message>::SharedPtr gps_publisher;
    rclcpp::Publisher<imu_message>::SharedPtr imu_publisher;
  };



  int main(int argc, char * argv[])
  {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPS_IMU_publisher>());
    rclcpp::shutdown();
    return 0;
  }