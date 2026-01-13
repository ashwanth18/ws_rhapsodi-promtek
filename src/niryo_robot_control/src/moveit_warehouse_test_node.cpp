#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_warehouse/robot_state_storage.h>
#include <warehouse_ros_sqlite/database_connection.hpp>


int main(int argc, char** argv){
    
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("moveit_warehouse_test_node");

    // Load robot model (adjust robot description path as needed)
    robot_model_loader::RobotModelLoader robot_model_loader(node,"robot_description");
    auto robot_model = robot_model_loader.getModel();

    // Init MoveGroupInterface 
    moveit::planning_interface::MoveGroupInterface move_group(node,"arm");

    // Connect to SQLite warehouse database 
    warehouse_ros::DatabaseConnection::Ptr conn = std::make_shared<warehouse_ros_sqlite::DatabaseConnection>();
    conn->setParams("/home/ashwanth/rhapsodi-promtek/warehouse_data.sqlite",0.5)
    if(!conn->connect()){
        RCLCPP_ERROR(node->get_logger(),"Failed to connect to warehouse database");
        return 1;
}

// Create RobotStateStorage 
moveit_warehouse::RobotStateStorage rs(conn);

// Get list of known state names
std::vector<std::string> state_names;
rs.getKnownRobotStates(state_names);
if (state_names.empty()){
    RCLCPP_ERROR(node->get_logger(),"No states found in warehouse");
    return 1;
}

// Print available states
for (const auto& name : state_names) 
{
    RCLCPP_INFO(node->get_logger(),"Known state: %s",name.c_str());
}
    // Load a specific named state
    std::string target_state = "containerA";
    moveit_warehouse::RobotStateWithMetadataConstPtr state_metadata;
    if(!rs.hasRobotState(target_name)){
        RCLCPP_ERROR(node->get_logger(),"State %s not found in warehouse",target_name.c_str());
        return 1;
    }
    moveit_warehouse::RobotStateWithMetadata msg(rs.getRobotStateStorage()->getMetaData());
    rs.getRobotState(msg,target_name);
    const moveit_msgs::msg::RobotState& rs_msg = static_cast<const moveit_msgs::msg:RobotState&>(*msg);



    //Convert to moveit::core::RobotState
    moveit::core::RobotState target_state(robot_model);
    moveit::core::robotStateMsgToRobotState(rs_msg,target_state);

    // Set as target and plan/execute
    move_group.setJointValueTarget(target_state);
    // Plan and execute
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if(move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
        move_group.execute(plan);
        RCLCPP_INFO(node->get_logger(), "Moved to state %s",target_name.c_str());
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to plan to state %s",target_name.c_str());
    }

    rclcpp::shutdown();
    return 0;
}
