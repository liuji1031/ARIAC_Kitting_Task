#include <rclcpp/rclcpp.hpp>
#include <ariac_msgs/msg/agv_status.hpp>
#include <ariac_msgs/msg/submission_condition.hpp>
#include <ariac_msgs/msg/kitting_task.hpp>
#include <chrono>
#include <memory>
#include <iostream>
#include <string>
#include <unistd.h>
#include "submit_orders.hpp"

using namespace std::chrono_literals;

auto SubmitOrders::call_client(std::string order_id_)
{
    // Wait for the service to become available
    while (!client_->wait_for_service(std::chrono::milliseconds(100)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            
        }
        RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    // Create a request and send it to the server
    auto request = std::make_shared<ariac_msgs::srv::SubmitOrder::Request>();
    request->order_id = order_id_;

    auto future_result = client_->async_send_request(request, std::bind(&SubmitOrders::response_callback, this, std::placeholders::_1));
    return future_result;
}

void SubmitOrders::agv_stat_callback(const ariac_msgs::msg::AGVStatus::SharedPtr msg)
{
    // Checking if the AGV has reached the Warehous
    if(msg->location == ariac_msgs::msg::AGVStatus::WAREHOUSE && agv_reached_==false){

        agv_reached_ = true;
        // report
        RCLCPP_INFO_STREAM(this->get_logger(), "AGV"<<agv_number_<<" has reached the Warehouse");
        
        // find order id
        std::string order_id = this->find_order_id();
        RCLCPP_INFO(this->get_logger(), "Order id to be submitted: %s", order_id.c_str());
        if(order_id.empty()==true){
            return;
        }
        sleep(5);
        // call service submit order
        // RCLCPP_INFO(this->get_logger(), "Calling client...");
        auto future = this->call_client(order_id);

        // wait for result
        // RCLCPP_INFO(this->get_logger(), "Waiting for future...");
        future.wait();
        if(future.get()->success==true){
            // remove order from orders
            remove_order(order_id);

            // publish the submitted order id
            std_msgs::msg::String msg;
            msg.data = order_id;
            submitted_order_pub_->publish(msg);
        }

    }
    
}

std::string SubmitOrders::find_order_id(){
    std::string id;
    for(auto order : orders_){
        if(order.kitting_task.agv_number == agv_number_){
            id = order.id.c_str();
            break;
        }
    }
    return id;
}

void SubmitOrders::order_sub_callback(ariac_msgs::msg::Order msg){
    // only store order that matches the node's agv
    if(int(msg.kitting_task.agv_number)!=agv_number_)return;  
    RCLCPP_INFO(this->get_logger(),"Storing new order, id: %s", msg.id.c_str());
    orders_.push_back(msg);
}

void SubmitOrders::timer_cb()
{

    if (agv_reached_){
        RCLCPP_INFO_STREAM(this->get_logger(), "AGV has entered warehouse thus \
                                                requesting Server now.");
        // Get order_id from SubmissionCondition message
        auto order_submit_ = std::make_shared<ariac_msgs::msg::SubmissionCondition>();

        call_client(order_submit_->order_id);
    }
    
}


void SubmitOrders::response_callback(rclcpp::Client<ariac_msgs::srv::SubmitOrder>::SharedFuture future)
{
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready)
    {
        if(future.get()->success){
            RCLCPP_INFO_STREAM(this->get_logger(), "Orders have been submitted successfully. Extra info: \n" <<
                                                future.get()->message);
        }else{
            RCLCPP_INFO_STREAM(this->get_logger(), "Orders submission not successful. Extr info: \n" <<
                                                future.get()->message);
        }
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
}

void SubmitOrders::remove_order(std::string order_id){
    int i{0};
    bool found{false};
    // find the order that matches the order id
    for(;i<int(this->orders_.size()); i++){
        if(this->orders_.at(i).id.compare(order_id)==0) // 0 means the same
        {   
            found=true;
            break;
        }
    }
    if(found==true){
        // remove the order so won't submit twice
        this->orders_.erase(this->orders_.begin()+i);
        // RCLCPP_INFO(this->get_logger(),"Order %s submitted and removed!", order_id.c_str());
    }
}


int main(int argc, char **argv)
{   
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SubmitOrders>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
}