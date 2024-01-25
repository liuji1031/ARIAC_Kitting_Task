#pragma once

#include <vector>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <ariac_msgs/msg/agv_status.hpp>
#include <ariac_msgs/msg/order.hpp>
#include <ariac_msgs/srv/submit_order.hpp>
#include <std_msgs/msg/string.hpp>
#include <cstdint>
#include <cstdlib>


/**
 * @brief Class for the client
 *
 */
class SubmitOrders : public rclcpp::Node
{
public:
    SubmitOrders() : Node("submit_orders_cpp")
    {   // set the agv number monitored by this node
        this->declare_parameter("agv_number",1);
        agv_number_ = this->get_parameter("agv_number").as_int();

        // create callback groups
        callback_grp1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_grp2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = callback_grp1_;

        // agv status subscriber
        agv_stat_sub_ = this->create_subscription<ariac_msgs::msg::AGVStatus>(
            "/ariac/agv1_status", 10, 
            std::bind(&SubmitOrders::agv_stat_callback, this, std::placeholders::_1),sub_options);

        // submit order client
        client_ = this->create_client<ariac_msgs::srv::SubmitOrder>("/ariac/submit_order",rmw_qos_profile_services_default, callback_grp2_);

        agv_reached_ = false;

        // subscriber for storing order information
        order_sub_ = this->create_subscription<ariac_msgs::msg::Order>("/ariac/orders",10,
            std::bind(&SubmitOrders::order_sub_callback, this, std::placeholders::_1));

        submitted_order_pub_ = this->create_publisher<std_msgs::msg::String>("/ariac/submitted_order",10);
    }

private:
    //  ---------------- attributes ------------------

    /**
     * @brief Indicates whether agv has reached warehouse or not
     *
     * value: (true): AGV reached warehouse 
     */
    bool agv_reached_;

    /**
     * @brief the number of the agv the node is monitoring
     * 
     */
    int agv_number_;

    /**
     * @brief subscriber to the /ariac/order topic
     * 
     */
    rclcpp::Subscription<ariac_msgs::msg::Order>::SharedPtr order_sub_;

    /**
     * @brief vector for storing all anounced orders
     * 
     */
    std::vector<ariac_msgs::msg::Order> orders_;

    /**
     * @brief Shared pointer to create_wall_timer object
     *
     */
    rclcpp::TimerBase::SharedPtr timer_;
    /**
     * @brief Shared pointer to create_client object
     *
     */
    rclcpp::Client<ariac_msgs::srv::SubmitOrder>::SharedPtr client_;

    /**
     * @brief Subscribes to araic_msgs/msg/AGVStatus
     * 
     */
    rclcpp::Subscription<ariac_msgs::msg::AGVStatus>::SharedPtr agv_stat_sub_;

    /**
     * @brief callback group for agv status subscription
     * 
     */
    rclcpp::CallbackGroup::SharedPtr callback_grp1_;

    /**
     * @brief publisher for already submitted orders
     * 
     */
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr submitted_order_pub_;


    //  ---------------- methods -------------------

    /**
     * @brief callback group for service call
     * 
     */
    rclcpp::CallbackGroup::SharedPtr callback_grp2_;

    /**
     * @brief Call the client
     *
     */
    auto call_client(std::string order_id);

    /**
     * @brief Callback function for the timer
     *
     */
    void timer_cb();

    /**
     * @brief Callback function for the client
     *
     * This function is called when the client receives a response from the server
     * @param future Shared pointer to the future object.
     *  A future is a value that indicates whether the call and response is finished (not the value of the response itself)
     */
    void response_callback(rclcpp::Client<ariac_msgs::srv::SubmitOrder>::SharedFuture future);

    /**
     * @brief Callback funciton for the AGVStatus subscriber
     * 
     * @param msg 
     */
    void agv_stat_callback(const ariac_msgs::msg::AGVStatus::SharedPtr msg);

    /**
     * @brief subscriber callback to the /ariac/order topic
     * 
     * @param msg 
     */
    void order_sub_callback(ariac_msgs::msg::Order msg);

    /**
     * @brief search through stored orders and return one with the 
     * agv that's arrived in the warehouse
     * 
     * @return std::string the order id
     */
    std::string find_order_id();

    /**
     * @brief remove submitted order from stored orders
     * 
     */
    void remove_order(std::string order_id);
};
