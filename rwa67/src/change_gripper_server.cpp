#include <memory>
#include <string>
#include <unistd.h> 
#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/srv/change_gripper.hpp"
#include "change_gripper_server.hpp"
#include "std_msgs/msg/u_int8.hpp"

std::string ChangeGripperServer::table_name(uint8_t table_no){
    if(m_table_names.find(table_no)!=m_table_names.end()){
        return m_table_names.at(table_no);
    }
    else{
        RCLCPP_ERROR(this->get_logger(),"Undefined table number!");
        return std::string("UNDEFINED"); 
    }
}

std::string ChangeGripperServer::gripper_name(uint8_t gripper_type){
    if(m_part_names.find(gripper_type)!=m_part_names.end()){
        return m_part_names.at(gripper_type);
    }
    else{
        RCLCPP_ERROR(this->get_logger(),"Undefined gripper type!");
        return std::string("UNDEFINED");
    }
}

template <typename T>
void ChangeGripperServer::wait_for_ready(T client){
    // Wait for the service to become available
    while (!client->wait_for_service(std::chrono::milliseconds(100)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for %s to become ready...", client->get_service_name());
    }

}

void ChangeGripperServer::handle_change_gripper(const std::shared_ptr<custom_msgs::srv::ChangeGripper::Request> request, 
    std::shared_ptr<custom_msgs::srv::ChangeGripper::Response> response){
    bool success{true};
    std::string message="Change gripper successful!";
    auto store_result = [&success,&message,response](){
            response->success = success;
            response->message = message;
    };

    // report status
    std::string table_name{this->table_name(request->table)};
    std::string gripper_name{this->gripper_name(request->gripper_type)};
    RCLCPP_INFO(this->get_logger(),"Changing to %s at %s", gripper_name.c_str(), table_name.c_str());

    if(m_emulate_only){
        response->success = true;
        response->message = "emulated success";
        return; // skip the rest
    }

    // move to table
    move_to_table_(request->table,success,message);
    store_result();
    if(success==false)return;
    // sleep(1);
    
    // enter tool changer
    enter_tool_changer_(request->table,request->gripper_type,success,message);
    store_result();
    if(success==false)return;
    // sleep(1);

    // call ariac change gripper service
    ariac_change_gripper_(request->gripper_type, success, message);
    store_result();
    if(success==false)return;
    // sleep(1);

    // exit tool change
    exit_tool_changer_(request->table,request->gripper_type,success,message);
    store_result();
    if(success==false)return;

    RCLCPP_INFO(this->get_logger(), "Change gripper successful!");

}

void ChangeGripperServer::move_to_table_(const uint8_t table_no, bool& success, std::string &message){
    wait_for_ready(m_totable_client);
    auto req = std::make_shared<robot_commander_msgs::srv::MoveRobotToTable::Request>();
    
    switch (table_no)
    {
        case custom_msgs::srv::ChangeGripper::Request::TABLE1:
            // go to table 1
            req->kts = robot_commander_msgs::srv::MoveRobotToTable::Request::KTS1;
            break;

        case custom_msgs::srv::ChangeGripper::Request::TABLE2:
            // go to table 2
            req->kts = robot_commander_msgs::srv::MoveRobotToTable::Request::KTS2;
            break;

        default:
            RCLCPP_ERROR(this->get_logger(),"Wrong table number received in change gripper request!");
            break;
    }
    auto result = m_totable_client->async_send_request(req);
    
    if(result.wait_for(std::chrono::seconds(m_timeout))==std::future_status::timeout){
        RCLCPP_ERROR(this->get_logger(),"Timeout moving robot to table %d", int(req->kts));
        success=false;
        message="Timeout moving to table.";
        return;
    }

    if(result.get()->success==true){
        RCLCPP_INFO(this->get_logger(),"Successfully moved robot to table %d", int(table_no+1));
    }
    else{
        RCLCPP_ERROR(this->get_logger(),"Failed moved robot to table %d", int(table_no+1));
        success = false;
        message = "Failed moving to table.";
        return;
    }
}

void ChangeGripperServer::parse_table_gripper_type_(const uint8_t table_no, const uint8_t type, 
    std::string &kts_req, std::string &type_req){
    switch (table_no)
    {
        case custom_msgs::srv::ChangeGripper::Request::TABLE1:
            kts_req="kts1";
            break;
        case custom_msgs::srv::ChangeGripper::Request::TABLE2:
            kts_req="kts2";
            break;
        default:
            RCLCPP_ERROR(this->get_logger(),"Wrong table number received in change gripper request!");
            break;
    }
    switch (type)
    {
            // tray gripper
        case custom_msgs::srv::ChangeGripper::Request::TRAY_GRIPPER:
            type_req="trays";
            break;
            // part gripper
        case custom_msgs::srv::ChangeGripper::Request::PART_GRIPPER:
            type_req="parts";
            break;
        default:
            RCLCPP_ERROR(this->get_logger(),"Wrong gripper type received in change gripper request!");
            break;
    }
}

void ChangeGripperServer::enter_tool_changer_(const uint8_t table_no, const uint8_t type, 
    bool& success, std::string &message){
    wait_for_ready(m_enter_tc_client);
    auto req = std::make_shared<robot_commander_msgs::srv::EnterToolChanger::Request>();
    std::string kts_req;
    std::string type_req;

    parse_table_gripper_type_(table_no, type, kts_req, type_req);
    
    req->changing_station=kts_req;
    req->gripper_type=type_req;

    auto result = m_enter_tc_client->async_send_request(req);

    if(result.wait_for(std::chrono::seconds(m_timeout))==std::future_status::timeout){
        RCLCPP_ERROR(this->get_logger(),"Timeout entering tool changer at table %d",  int(table_no));
        success=false;
        message="Timeout entering tool changer.";
        return;
    }

    if(result.get()->success==true){
        RCLCPP_INFO(this->get_logger(),"Successfully entered tool changer at table %d", int(table_no));
    }
    else{
        RCLCPP_ERROR(this->get_logger(),"Failed to enter tool changer at table %d", int(table_no));
        success = false;
        message = "Failed entering tool changer";
        return;
    }

}

void ChangeGripperServer::ariac_change_gripper_(uint8_t type, bool& success, std::string &message){
    wait_for_ready(m_ariac_cg_client);
    uint8_t gripper_type;
    switch (type)
    {
            // tray gripper
        case custom_msgs::srv::ChangeGripper::Request::TRAY_GRIPPER:
            gripper_type = ariac_msgs::srv::ChangeGripper::Request::TRAY_GRIPPER;
            break;
            // part gripper
        case custom_msgs::srv::ChangeGripper::Request::PART_GRIPPER:
            gripper_type = ariac_msgs::srv::ChangeGripper::Request::PART_GRIPPER;
            break;
        default:
            RCLCPP_ERROR(this->get_logger(),"Wrong gripper type received in change gripper request!");
            break;
    }

    auto req = std::make_shared<ariac_msgs::srv::ChangeGripper::Request>();
    req->gripper_type = gripper_type;

    auto result = m_ariac_cg_client->async_send_request(req); // no callback is registered, wait for result

    if(result.wait_for(std::chrono::seconds(m_timeout))==std::future_status::timeout){
        RCLCPP_ERROR(this->get_logger(),"Timeout calling ariac change gripper service!");
        success=false;
        message="Timeout calling ariac change gripper service.";
        return;
    }

    if(result.get()->success){
        // report status
        RCLCPP_INFO(this->get_logger(),"Successful calling ariac change gripper service.");
    }
    else{
        success = false;
        message = std::string("failed");
        RCLCPP_INFO(this->get_logger(),"%s",result.get()->message.c_str());
        RCLCPP_INFO(this->get_logger(),"Failed to call ariac change gripper service.");
    }

}

void ChangeGripperServer::exit_tool_changer_(const uint8_t table_no, const uint8_t type, 
    bool& success, std::string &message){
    wait_for_ready(m_exit_tc_client);
    auto req = std::make_shared<robot_commander_msgs::srv::ExitToolChanger::Request>();
    std::string kts_req;
    std::string type_req;

    parse_table_gripper_type_(table_no, type, kts_req, type_req);
    
    req->changing_station=kts_req;
    req->gripper_type=type_req;

    auto result = m_exit_tc_client->async_send_request(req);

    if(result.wait_for(std::chrono::seconds(m_timeout))==std::future_status::timeout){
        RCLCPP_ERROR(this->get_logger(),"Timeout exiting tool changer at table %d",  int(table_no));
        success=false;
        message="Timeout exiting tool changer.";
        return;
    }

    if(result.get()->success==true){
        RCLCPP_INFO(this->get_logger(),"Successfully exited tool changer at table %d", int(table_no));
    }
    else{
        RCLCPP_ERROR(this->get_logger(),"Failed to exit tool changer at table %d", int(table_no));
        success = false;
        message = "Failed exiting tool changer";
        return;
    }

}

int main(int argc, char **argv)
{   
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ChangeGripperServer>("change_gripper_server");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
}