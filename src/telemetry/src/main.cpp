#include "rclcpp/rclcpp.hpp"
#include "telemetry_master.h"
#include <memory>

using std::placeholders::_1;




int main(int argc, char * argv[]){

	rclcpp::init(argc, argv); 
	rclcpp::spin(std::make_shared<TelemetryMaster>()); 
	rclcpp::shutdown(); 
	return 0; 
}