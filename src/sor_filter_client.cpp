// C++ libraries
#include <iostream>
#include <string.h>
// ROS2 libraries
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

class StatisticalOutlierRemovalFilterClient : public rclcpp::Node{
    public:
        StatisticalOutlierRemovalFilterClient(): Node("point_cloud2_filter_client"){
            sor_filter_client = create_client<std_srvs::srv::SetBool>("/sor_filter");

            while (!sor_filter_client->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) {
                    std::cout << "Service Not Available!" << std::endl;
                    rclcpp::shutdown();
                    return;
                }
                std::cout << "Waiting for Service to Be Available!" << std::endl;
            }

            call_for_filter_service(sor_filter_is_activated);  
        }

    private:
        bool sor_filter_is_activated = true;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr sor_filter_client;

        void call_for_filter_service(bool data)
        {
            std::cout << "Asking to Filter a PointCloud2..." << std::endl;
            auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
            request->data = data;

            auto future = sor_filter_client->async_send_request(request);
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) != rclcpp::FutureReturnCode::SUCCESS){
                RCLCPP_ERROR(this->get_logger(), "Failed to Call Service");
                return;
            }

            auto response = future.get();
            std::cout << response->message << std::endl;
        }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  auto sor_filter_client_node = std::make_shared<StatisticalOutlierRemovalFilterClient>();
  rclcpp::spin(sor_filter_client_node);

  rclcpp::shutdown();
  return 0;
}