// C++ libraries
#include <iostream>
#include <string.h>
// ROS2 libraries
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
// PCL libraries
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

class StatisticalOutlierRemovalFilterServer: public rclcpp::Node
{
    public:
        StatisticalOutlierRemovalFilterServer(): Node("point_cloud2_filter_server"){
            std::cout << "Waiting to Filter a PointCloud2..." << std::endl;

            filter_server = this->create_service<std_srvs::srv::SetBool>(
                "/sor_filter",
                std::bind(&StatisticalOutlierRemovalFilterServer::filter_callback, this, std::placeholders::_1, std::placeholders::_2)
            );
        }
    private:
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr filter_server;
        const char *outlier_file_path = "/home/usuario/Workspaces/personal_ws/src/statistical_outlier_removal_filter/pcd/reconstruction_outliers.pcd";
        
        void apply_sor_filter_to_pcd(){
            pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

            std::cout << "Reading PCD file..." << std::endl;
            pcl::PCDReader reader;
            const char *input_file_path = "/home/usuario/Workspaces/personal_ws/src/statistical_outlier_removal_filter/pcd/reconstruction.pcd";
            reader.read<pcl::PointXYZ>(input_file_path, *input_cloud);

            std::cout << "Applying SOR filter" << std::endl;
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statistical_outlier_removal_filter;
            statistical_outlier_removal_filter.setInputCloud(input_cloud);
            statistical_outlier_removal_filter.setMeanK(50);
            statistical_outlier_removal_filter.setStddevMulThresh(1.0);
            statistical_outlier_removal_filter.setNegative (true);
            statistical_outlier_removal_filter.filter(*cloud_filtered);
            
            std::cout << "PointCloud2 filtered!" << std::endl;
            pcl::PCDWriter writer;
            writer.write<pcl::PointXYZ>(outlier_file_path, *cloud_filtered, false);
        }

        void filter_callback(std_srvs::srv::SetBool::Request::SharedPtr request, std_srvs::srv::SetBool::Response::SharedPtr response){
            if(request->data == true){
                apply_sor_filter_to_pcd();
                response->success = true;
                response->message = "SOR Filter Applied!";
            }
            else{
                response->success = false;
                response->message = "SOR Filter Not Applied!";
            }
        }

};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    
    auto sor_filter_server_node = std::make_shared<StatisticalOutlierRemovalFilterServer>();
    rclcpp::spin(sor_filter_server_node);
    
    rclcpp::shutdown();
    return (0);
}