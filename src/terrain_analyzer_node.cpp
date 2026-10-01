#include "terrain_analyzer/terrain_analyzer.hpp"

#include<rclcpp/rclcpp.hpp>
#include<sensor_msgs/msg/point_cloud2.hpp>
#include<sensor_msgs/point_cloud2_iterator.hpp>

#include<Eigen/Dense>

#include<cmath>
#include<memory>
#include<vector>

class TerrainAnalyzerNode : public rclcpp::Node
{
    public:
      TerrainAnalyzerNode()
      : Node("terrain_analyzer_node")
      {
        pointcloud_topic_ = this->declare_parameter<std::string>("pointcloud_topic","/livox/lidar");

        min_points_ = this->declare_parameter<int>("min_points",30);

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pointcloud_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&TerrainAnalyzerNode::pointCloudCallback,this,std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(),"Terrain Analyzer started.");
        RCLCPP_INFO(this->get_logger(),"PointCloud topic: %s",pointcloud_topic_.c_str());
      }
    private:
      void pointCloudCallback(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg
      )
      {
        std::vector<Eigen::Vector3d> points;

        try{
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg,"x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg,"y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg,"z");

            for(
                ; iter_x != iter_x.end(); 
                ++iter_x, ++iter_y, ++iter_z
            )
            {
                const double x = static_cast<double>(*iter_x);
                const double y = static_cast<double>(*iter_y);
                const double z = static_cast<double>(*iter_z);

                if (
                    !std::isfinite(x) ||
                    !std::isfinite(y) ||
                    !std::isfinite(z)
                )
                {
                    continue;
                }
                points.emplace_back(x,y,z);
            }
                
        }
        catch (const std::exception & e)
        {
            RCLCPP_ERROR(this->get_logger(),"Failed to read PointCloud2: %s",e.what());
            return;
        }
        if (points.size() < static_cast<std::size_t>(min_points_))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(),*this->get_clock(),2000,"Too few valid points: %zu",points.size());
            return;
        }
        const auto result = analyzer_.analyze(points);

        if(!result.valid)
        {
            RCLCPP_WARN(this->get_logger(),"Terrain analysis failed.");
            return;
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(),1000,"points=%zu | slope=%.2f deg | " "planarity=%.3f | scattering=%.3f",points.size(),result.slope * 180.0 / M_PI,result.planarity,result.scattering);
      }

      std::string pointcloud_topic_;
      int min_points_;

      terrain_analyzer::TerrainAnalyzer analyzer_;

      rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
        subscription_;
};

int main(int argc,char ** argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<TerrainAnalyzerNode>());
    rclcpp::shutdown();

    return 0;
}