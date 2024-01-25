#include <chrono>
#include <functional>
#include <memory>
#include <future>
#include "ship_order.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

auto ShipOrder::lock_tray(uint8_t agv_n){
    RCLCPP_INFO(this->get_logger(),"Locking tray on AGV %d", agv_n);

    int agv_n0{agv_n-1}; // convert to 0 based indexing
    assert(agv_n0>=0);

    // check if service is ready
    auto client = m_lock_clients.at(agv_n0);
    wait_for_ready(client);

    // build request
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    // send request
    auto future = client->async_send_request(request, std::bind(&ShipOrder::lock_response_callback,this,std::placeholders::_1));

    return future;
}

void ShipOrder::agv_sub_callback(std_msgs::msg::UInt8 msg){
    // figure out the agv number
    uint8_t agv_n{msg.data};

    // build request and lock tray on agv
    auto lock_tray_future = lock_tray(agv_n);

    // check the future of lock tray before move AGV
    lock_tray_future.wait();
    if(lock_tray_future.get()->success==true){
        // send agv to the warehouse
        move_agv_warehouse(agv_n);
    }
}

template <typename T>
void ShipOrder::wait_for_ready(T client){
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

void ShipOrder::move_agv_warehouse(uint8_t agv_n){
    RCLCPP_INFO(this->get_logger(),"Moving AGV %d to warehouse", agv_n);

    int agv_n0{agv_n-1}; // convert to 0 based indexing
    assert(agv_n0>=0);

    // check if service is ready
    auto client = m_move_clients.at(agv_n0);
    wait_for_ready(client);

    // build request
    auto request = std::make_shared<ariac_msgs::srv::MoveAGV::Request>();
    request->location = ariac_msgs::srv::MoveAGV::Request::WAREHOUSE;

    auto future = client->async_send_request(request, std::bind(&ShipOrder::move_agv_response_callback,this,std::placeholders::_1));
}

void ShipOrder::lock_response_callback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future){
    auto status = future.wait_for(std::chrono::milliseconds(500));
    if( status == std::future_status::ready ){
        if( future.get()->success == true){ 
            RCLCPP_INFO(this->get_logger(),"Lock tray result: success=%d, message=%s",int(future.get()->success), future.get()->message.c_str());
        }
        else{
            RCLCPP_ERROR(this->get_logger(),"Lock tray result: success=%d, message=%s",int(future.get()->success), future.get()->message.c_str());
        }
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Service still in progress...");
    }
}

void ShipOrder::move_agv_response_callback(rclcpp::Client<ariac_msgs::srv::MoveAGV>::SharedFuture future){
    auto status = future.wait_for(std::chrono::milliseconds(500));
    if( status == std::future_status::ready ){
        if( future.get()->success == true){ 
            RCLCPP_INFO(this->get_logger(),"Move AGV result: success=%d, %s",int(future.get()->success), future.get()->message.c_str());
        }
        else{
            RCLCPP_ERROR(this->get_logger(),"Move AGV result: success=%d, %s",int(future.get()->success), future.get()->message.c_str());
        }
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Service still in progress...");
    }
}

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<ShipOrder>("ship_order");
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
}



