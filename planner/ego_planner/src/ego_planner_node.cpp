#include <rclcpp/rclcpp.hpp>

#include <ego_planner/ego_replan_fsm.h>

using namespace ego_planner;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("ego_planner_node");

    EGOReplanFSM rebo_replan;

    rebo_replan.init(node);

    spin(node);
    rclcpp::shutdown();

    return 0;
}