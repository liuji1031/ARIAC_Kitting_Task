#ifndef __SHIP_ORDER_HPP__
#define __SHIP_ORDER_HPP__

#include <string>
#include <functional>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "ariac_msgs/srv/move_agv.hpp"
#include "std_srvs/srv/trigger.hpp" 

class ShipOrder : public rclcpp::Node {
    public:
        ShipOrder(std::string node_name): 
            Node(node_name)
        {
            this->declare_parameter("total_agv_number",4);
            m_total_agv = this->get_parameter("total_agv_number").as_int();
            RCLCPP_INFO(this->get_logger(),"Total AGV number: %d", m_total_agv);

            // make the subscriber callback reentrant so it can call the services for 
            // multiple AGVs at the same time (might happen)
            m_reentrant_cb_grp = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
            rclcpp::SubscriptionOptions sub_options;
            sub_options.callback_group = m_reentrant_cb_grp;

            m_agv_sub = this->create_subscription<std_msgs::msg::UInt8>("move_agv_warehouse",10,
                std::bind(&ShipOrder::agv_sub_callback, this, std::placeholders::_1), sub_options);

            // initialize lock clients for all agvs
            m_lock_clients.reserve(m_total_agv);
            for(int i_agv{0};i_agv<m_total_agv;i_agv++){
                // create service name based on agv number, e.g., "agv1_lock_tray"
                std::string service_name = std::string("/ariac/agv")+std::to_string(i_agv+1)+std::string("_lock_tray");

                // push back into the vector
                m_lock_clients.push_back(  this->create_client<std_srvs::srv::Trigger>(service_name)  );
            }

            // initialize move agv clients
            m_move_clients.reserve(m_total_agv);
            for(int i_agv{0};i_agv<m_total_agv;i_agv++){
                // create service name based on agv number, e.g., "move_agv1"
                std::string service_name = std::string("/ariac/move_agv")+std::to_string(i_agv+1);

                // push back into the vector
                m_move_clients.push_back(  this->create_client<ariac_msgs::srv::MoveAGV>(service_name)  );
            }
            

        }

    private:
        // ------------- member functions -------------- 

        /**
         * @brief the callback sends an async service call to move
         * the agv according the incoming msg
         * 
         * @param msg the agv number to be moved to warehouse
         */
        void agv_sub_callback(std_msgs::msg::UInt8 msg);

        /**
         * @brief make service call to lock tray on agv
         * 
         * @param agv_n the agv number
         */
        auto lock_tray(uint8_t agv_n);

        /**
         * @brief make service call to send agv to the warehouse
         * 
         * @param agv_n the agv number
         */
        void move_agv_warehouse(uint8_t agv_n);

        /**
         * @brief callback for handling future returned by the call to the lock
         * tray service
         * 
         * @param future 
         */
        void lock_response_callback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);

        /**
         * @brief callback for handling future of move agv service call
         * 
         * @param future 
         */
        void move_agv_response_callback(rclcpp::Client<ariac_msgs::srv::MoveAGV>::SharedFuture future);

        /**
         * @brief template function for checking if a service is ready
         * 
         * @tparam T client type
         * @param client 
         */
        template <typename T>
        void wait_for_ready(T client);

        // ------------- member variables -------------- 

        /**
         * @brief the total number of agvs to be controlled, e.g., 4
         * 
         */
        int m_total_agv;

        /**
         * @brief subscribes to a topic "" to check which agv needs to go to
         * the warehouse
         */
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr m_agv_sub;

        /**
         * @brief a vector of client for moving the agv
         * 
         */
        std::vector<rclcpp::Client<ariac_msgs::srv::MoveAGV>::SharedPtr> m_move_clients;

        /**
         * @brief client for locking the tray on the agv
         * 
         */
        std::vector<rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> m_lock_clients;

        /**
         * @brief create a reentrant callback group for the subscription
         * 'move_agv_warehouse'
         * 
         */
        rclcpp::CallbackGroup::SharedPtr m_reentrant_cb_grp;

};

#endif