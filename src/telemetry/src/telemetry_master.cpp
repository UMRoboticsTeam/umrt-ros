#include "telemetry_master.h"
#include <chrono>
using std::placeholders::_1; 


TelemetryMaster::TelemetryMaster():TelemetryMaster(std::chrono::milliseconds(500)){}; 

TelemetryMaster::TelemetryMaster(std::chrono::milliseconds interval_ms):Node("telemetry_master"){
	telemetry_publisher = this->create_publisher<telemetry_message>("telemetry",10); 
	gps_subscriber = this->create_subscription<gps_message>("gps", 10, std::bind(&TelemetryMaster::gps_callback,this,_1)); 
	imu_subscriber = this->create_subscription<imu_message>("imu", 10, std::bind(&TelemetryMaster::imu_callback, this,_1)); 
	timer = this-> create_wall_timer(interval_ms, std::bind(&TelemetryMaster::publisher_callback,this)); 
}; 



void TelemetryMaster::gps_callback(gps_message::SharedPtr gps_data){
	//get gps messages then update current_gps_data value 
	//do some processing if needed
	current_gps_data = gps_data; 
}; 

void TelemetryMaster::imu_callback(imu_message::SharedPtr imu_data){
	//get imu messages then update current_imu_data value
	//do some processing if needed
	current_imu_data = imu_data; 
}; 

void TelemetryMaster::publisher_callback(){
	auto telemetry = telemetry_message(); 
	telemetry.time_stamp = std::time(nullptr); 
	telemetry.gps_latitude = current_gps_data? current_gps_data->latitude:0; 
	telemetry.gps_longitude = current_gps_data?current_gps_data->longitude:0; 
	telemetry.roll = current_imu_data?current_imu_data->roll:0; 
	telemetry.pitch = current_imu_data?current_imu_data->pitch:0; 
	telemetry.yaw = current_imu_data?current_imu_data->yaw:0; 
	telemetry.heading = current_imu_data?current_imu_data->heading:0; 
	telemetry_publisher->publish(telemetry); 
}; 



