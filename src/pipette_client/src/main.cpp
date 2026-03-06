#include <rclcpp/rclcpp.hpp>
#include "pipette_client/pipette_client_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<pipette_client::PipetteClientNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("pipette_client"), 
                 "Fatal error: %s", e.what());
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}
