#ifndef __CHANGE_GRIPPER_SERVER_HPP__
#define __CHANGE_GRIPPER_SERVER_HPP__

#include <string>
#include <memory>
#include <map>
#include <functional>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/srv/change_gripper.hpp"
#include "ariac_msgs/srv/change_gripper.hpp"
#include "robot_commander_msgs/srv/move_robot_to_table.hpp"
#include "robot_commander_msgs/srv/enter_tool_changer.hpp"
#include "robot_commander_msgs/srv/exit_tool_changer.hpp"

// auto table_names_map = 

class ChangeGripperServer : public rclcpp::Node {
    public:
        /**
         * @brief Construct a new Change Gripper Server object
         * 
         * @param node_name 
         */
        ChangeGripperServer(std::string node_name): Node(node_name)
        {   
            this->declare_parameter("emulate_only",false);
            m_emulate_only = this->get_parameter("emulate_only").as_bool();

            // create callback groups
            m_cbg_server = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            m_cbg_client = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

            // create the server for chaning gripper
            m_chg_gripper_server = this->create_service<custom_msgs::srv::ChangeGripper>(
                "/ariac/custom_change_gripper", 
                std::bind(&ChangeGripperServer::handle_change_gripper, this, std::placeholders::_1, std::placeholders::_2),
                rmw_qos_profile_services_default,
                m_cbg_server
            );

            // create client for ariac change gripper service
            m_ariac_cg_client = this->create_client<ariac_msgs::srv::ChangeGripper>("/ariac/floor_robot_change_gripper",
                rmw_qos_profile_services_default, m_cbg_client);

            // clients for all robot_commander_msgs interfaces
            m_totable_client = this->create_client<robot_commander_msgs::srv::MoveRobotToTable>("/commander/move_robot_to_table",
                rmw_qos_profile_services_default,m_cbg_client);
            m_enter_tc_client = this->create_client<robot_commander_msgs::srv::EnterToolChanger>("/commander/enter_tool_changer",
                rmw_qos_profile_services_default,m_cbg_client);
            m_exit_tc_client = this->create_client<robot_commander_msgs::srv::ExitToolChanger>("/commander/exit_tool_changer",
                rmw_qos_profile_services_default,m_cbg_client);
        }

    private:
        // ====================== class methods ======================
        /**
         * @brief function for handling change gripper service call
         * 
         * @param request 
         * @param response 
         */
        void handle_change_gripper(const std::shared_ptr<custom_msgs::srv::ChangeGripper::Request> request, 
            std::shared_ptr<custom_msgs::srv::ChangeGripper::Response> response);

        /**
         * @brief return the table name accorind to table no
         * 
         * @param table_no 
         * @return std::string 
         */
        std::string table_name(uint8_t table_no);

        /**
         * @brief template function for checking if a service is ready
         * 
         * @tparam T client type
         * @param client 
         */
        template <typename T>
        void wait_for_ready(T client);

        /**
         * @brief return gripper name according to gripper type
         * 
         * @param gripper_type 
         * @return std::string 
         */
        std::string gripper_name(uint8_t gripper_type);

        /**
         * @brief function for calling move to table service
         * 
         * @param table_no the number of the table
         * @param success 
         * @param message 
         */
        void move_to_table_(const uint8_t table_no, bool& success, std::string &message);

        /**
         * @brief function for calling enter tool changer service
         * 
         * @param table_no 
         * @param type part or gripper 
         * @param success
         * @param message
         */
        void enter_tool_changer_(const uint8_t table_no, const uint8_t type,bool& success, std::string &message);

        /**
         * @brief function for calling ariac change gripper service
         * 
         * @param type gripper type
         * @param success 
         * @param message 
         */
        void ariac_change_gripper_(uint8_t type, bool& success, std::string &message);

        /**
         * @brief parse table number and gripper type to return the proper string for enter
         * and exit tool changer request
         * @param table_no 
         * @param type 
         * @param kts_req 
         * @param type_req 
         */
        void parse_table_gripper_type_(const uint8_t table_no, const uint8_t type, std::string &kts_req, std::string &type_req);

        /**
         * @brief function for calling exit tool changer service
         * 
         * @param table_no 
         * @param type 
         * @param success 
         * @param message 
         */
        void exit_tool_changer_(const uint8_t table_no, const uint8_t type,bool& success, std::string &message);

        // ====================== class attributes ======================
        /**
         * @brief custom server for changing gripper, handles robot name, table number
         * and gripper type
         */
        rclcpp::Service<custom_msgs::srv::ChangeGripper>::SharedPtr m_chg_gripper_server;

        /**
         * @brief client for calling ariac change gripper service for floor robot
         * 
         */
        rclcpp::Client<ariac_msgs::srv::ChangeGripper>::SharedPtr m_ariac_cg_client;

        /**
         * @brief client for calling ariac change gripper service for ceiling robot
         * 
         */
        rclcpp::Client<ariac_msgs::srv::ChangeGripper>::SharedPtr m_crobot_client;

        /**
         * @brief client for the move table service call
         * 
         */
        rclcpp::Client<robot_commander_msgs::srv::MoveRobotToTable>::SharedPtr m_totable_client;

        /**
         * @brief client for the enter tool changer service call
         * 
         */
        rclcpp::Client<robot_commander_msgs::srv::EnterToolChanger>::SharedPtr m_enter_tc_client;

        /**
         * @brief client for the exit tool changer service call
         * 
         */
        rclcpp::Client<robot_commander_msgs::srv::ExitToolChanger>::SharedPtr m_exit_tc_client;


        /**
         * @brief callback group for client service calls
         * 
         */
        rclcpp::CallbackGroup::SharedPtr m_cbg_client;

        /**
         * @brief callback group for server
         * 
         */
        rclcpp::CallbackGroup::SharedPtr m_cbg_server;

        /**
         * @brief the timeout duration for waiting for service result, in seconds
         * 
         */
        int m_timeout{15};

        /**
         * @brief map to table number in string
         * 
         */
        inline static const std::unordered_map<uint8_t,std::string> m_table_names{
            {custom_msgs::srv::ChangeGripper::Request::TABLE1, "table 1"},
            {custom_msgs::srv::ChangeGripper::Request::TABLE2, "table 2"}
        };

        /**
         * @brief map to part name in string
         * 
         */
        inline static const std::map<uint8_t,std::string> m_part_names{
            {custom_msgs::srv::ChangeGripper::Request::TRAY_GRIPPER, "tray gripper"},
            {custom_msgs::srv::ChangeGripper::Request::PART_GRIPPER, "part gripper"}
        };

        bool m_emulate_only = false;
 
};



#endif