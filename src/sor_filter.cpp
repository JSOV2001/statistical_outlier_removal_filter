// C++ libraries
#include <iostream>
#include <string.h>
// ROS2 libraries
#include <rclcpp/rclcpp.hpp>
// PCL libraries
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

class StatisticalOutlierRemovalFilter: public rclcpp::Node{
    public:
        StatisticalOutlierRemovalFilter(): Node("point_cloud2_filter"){
            std::cout << "About to filter a PointCloud2" << std::endl;
            this->filter_pcd_point_cloud2();
        }
    private:
        const char *inlier_file_path = "/home/usuario/Workspaces/personal_ws/src/statistical_outlier_removal_filter/pcd/inliers.pcd";
        const char *outlier_file_path = "/home/usuario/Workspaces/personal_ws/src/statistical_outlier_removal_filter/pcd/outliers.pcd";
        
        void filter_pcd_point_cloud2(){
            pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

            std::cout << "Reading PCD file" << std::endl;
            pcl::PCDReader reader;
            const char *input_file_path = "/home/usuario/Workspaces/personal_ws/src/statistical_outlier_removal_filter/pcd/input.pcd";
            reader.read<pcl::PointXYZ>(input_file_path, *input_cloud);

            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statistical_outlier_removal_filter;
            statistical_outlier_removal_filter.setInputCloud(input_cloud);
            statistical_outlier_removal_filter.setMeanK(50);
            statistical_outlier_removal_filter.setStddevMulThresh(1.0);
            statistical_outlier_removal_filter.filter(*cloud_filtered);

            pcl::PCDWriter writer;
            writer.write<pcl::PointXYZ>(inlier_file_path, *cloud_filtered, false);

            std::cout << "New filtered PointCloud2: outliers.pcd" << std::endl;
            statistical_outlier_removal_filter.setNegative (true);
            statistical_outlier_removal_filter.filter(*cloud_filtered);
            
            std::cout << "New filtered PointCloud2: outliers.pcd" << std::endl;
            writer.write<pcl::PointXYZ>(outlier_file_path, *cloud_filtered, false);
        }

};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    
    auto sor_filter_node = std::make_shared<StatisticalOutlierRemovalFilter>();
    rclcpp::spin(sor_filter_node);
    
    rclcpp::shutdown();
    return (0);
}