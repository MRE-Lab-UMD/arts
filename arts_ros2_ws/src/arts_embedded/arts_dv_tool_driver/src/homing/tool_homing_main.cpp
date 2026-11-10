
#include "arts_dv_tool_driver/homing/tool_homing_node.hpp"


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ToolHomingNode>());
    rclcpp::shutdown();
    return 0;
}
