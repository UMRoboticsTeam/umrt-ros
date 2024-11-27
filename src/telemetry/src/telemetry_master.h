#ifndef TELEMETRY_MASTER
#define TELEMETRY_MASTER 

#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "telemetry_interfaces/msg/telemetry.hpp" 
#include "telemetry_interfaces/msg/imu.hpp" 
#include "telemetry_interfaces/msg/gps.hpp"




typedef telemetry_interfaces::msg::Telemetry telemetry_message;
typedef telemetry_interfaces::msg::Gps gps_message; 
typedef telemetry_interfaces::msg::Imu imu_message; 

class TelemetryMaster: public rclcpp::Node{
	private:
		gps_message::SharedPtr current_gps_data{}; 
		imu_message::SharedPtr current_imu_data{}; 

		void gps_callback(gps_message::SharedPtr gps_data); 
		void imu_callback(imu_message::SharedPtr imu_data);
		void publisher_callback(); 

		rclcpp::Publisher<telemetry_message>::SharedPtr telemetry_publisher; 
		rclcpp::Subscription<gps_message>::SharedPtr gps_subscriber; 
		rclcpp::Subscription<imu_message>::SharedPtr imu_subscriber; 
		rclcpp::TimerBase::SharedPtr timer; 

		

	public: 
			TelemetryMaster(); 
			TelemetryMaster(std::chrono::milliseconds interval_ms); 

}; 

#endif