#include "terrain_analyzer/terrain_analyzer.hpp"
#include "terrain_analyzer/msg/terrain_analysis.hpp"

#include<rclcpp/rclcpp.hpp>
#include<sensor_msgs/msg/point_cloud2.hpp>
#include<sensor_msgs/point_cloud2_iterator.hpp>

#include<Eigen/Dense>

#include<cmath>
#include<memory>
#include<vector>

#include<unordered_map>
#include<functional>

struct GridIndex
{
    int x;
    int y;
    bool operator==(const GridIndex & other)const
    {
        return x == other.x && y == other.y;
    }
};

struct TerrainCell
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    double lambda1 = 0.0;
    double lambda2 = 0.0;
    double lambda3 = 0.0;

    double linearity = 0.0;
    double planarity = 0.0;
    double scattering = 0.0;
    
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();

    double slope = 0.0;
    double height_std = 0.0;

    double residual_mean = 0.0;
    double residual_std = 0.0;

    double outlier_ratio = 0.0;

    std::size_t point_count = 0;
    bool valid = false;

    uint8_t terrain_type = 0;
};

uint8_t classifyTerrain(const TerrainCell & cell)
{
    if (!cell.valid)
    {
        return 0;
    }
    if (cell.slope < 10.0 * M_PI / 180.0 && cell.planarity > 0.8)
    {
        return 1;
    }
    if (cell.slope >= 10.0 * M_PI / 180.0 && cell.slope < 30.0 * M_PI / 180.0 && cell.planarity > 0.7)
    {
        return 2;
    }
    if (cell.scattering > 0.1)
    {
        return 3;
    }
    if (cell.planarity < 0.5)
    {
        return 4;
    }
    if (cell.linearity > 0.6)
    {
        return 5;
    }
    return 0;
};

struct GridIndexHash
{
    std::size_t operator()(const GridIndex & index)const
    {
        const std::size_t h1 = std::hash<int>{}(index.x);
        const std::size_t h2 = std::hash<int>{}(index.y);
        return h1 ^ (h2 << 1);
    }
};

const char * terrainTypeToString(uint8_t type)
{
    switch (type)
    {
        case 1: 
            return "FLAT";
        case 2: 
            return "SLOPE";
        case 3: 
            return "ROUGH";
        case 4: 
            return "IRREGULAR";
        case 5: 
            return "EDGE";
        default: 
            return "UNKNOWN";
    }
}
class TerrainAnalyzerNode : public rclcpp::Node
{
    public:
      TerrainAnalyzerNode()
      : Node("terrain_analyzer_node")
      {
        pointcloud_topic_ = this->declare_parameter<std::string>("pointcloud_topic","/livox/lidar");

        min_points_ = this->declare_parameter<int>("min_points",30);
        cell_size_ = this->declare_parameter<double>("cell_size",0.5);

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pointcloud_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&TerrainAnalyzerNode::pointCloudCallback,this,std::placeholders::_1)
        );
        terrain_pub_= this->create_publisher<terrain_analyzer::msg::TerrainAnalysis>("/terrain_analysis",10);

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

        std::unordered_map<GridIndex,std::vector<Eigen::Vector3d>,GridIndexHash> grids;
        for (const auto & point : points)
        {
            const int grid_x = static_cast<int>(std::floor(point.x() / cell_size_));
            const int grid_y = static_cast<int>(std::floor(point.y() / cell_size_));
            GridIndex index{grid_x, grid_y};
            grids[index].push_back(point);
        }

        if (points.size() < static_cast<std::size_t>(min_points_))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(),*this->get_clock(),2000,"Too few valid points: %zu",points.size());
            return;
        }

        // const auto result = analyzer_.analyze(points);
        // if(!result.valid)
        // {
        //     RCLCPP_WARN(this->get_logger(),"Terrain analysis failed.");
        //     return;
        // }
        // RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(),1000,"points=%zu | slope=%.2f deg | " "planarity=%.3f | scattering=%.3f",points.size(),result.slope * 180.0 / M_PI,result.planarity,result.scattering);
        
        std::vector<TerrainCell> terrain_cells;
        int flat_count = 0;
        int slope_count = 0;
        int rough_count = 0;
        int irregular_count = 0;
        int edge_count = 0;
        int unknown_count = 0;

        for (const auto & grid : grids)
        {
            const GridIndex & index = grid.first;
            const auto & grid_points = grid.second;

            const double center_x = (static_cast<double>(index.x) + 0.5) * cell_size_;
            const double center_y = (static_cast<double>(index.y) + 0.5) * cell_size_;

            if (grid_points.size() < static_cast<std::size_t>(min_points_))
            {
                continue;
            }

            const auto grid_result = analyzer_.analyze(grid_points);
            if (!grid_result.valid)
            {
                continue;
            }
            const double center_z = grid_result.mean_height;
            
            TerrainCell cell;

            cell.x = center_x;
            cell.y = center_y;
            cell.z = center_z;

            cell.lambda1 = grid_result.lambda1;
            cell.lambda2 = grid_result.lambda2;
            cell.lambda3 = grid_result.lambda3;

            cell.linearity = grid_result.linearity;
            cell.planarity = grid_result.planarity;
            cell.scattering = grid_result.scattering;

            cell.normal = grid_result.normal;

            cell.slope = grid_result.slope;

            cell.height_std = grid_result.height_std;

            cell.residual_mean = grid_result.residual_mean;
            cell.residual_std = grid_result.residual_std;

            cell.outlier_ratio = grid_result.outlier_ratio;

            cell.point_count = grid_points.size();

            cell.valid = true;

            cell.terrain_type = classifyTerrain(cell);
            switch (cell.terrain_type)
            {
                case 1:
                    flat_count++;
                    break;
                case 2:
                    slope_count++;
                    break;
                case 3:
                    rough_count++;
                    break;
                case 4:
                    irregular_count++;
                    break;
                case 5:
                    edge_count++;
                    break;
                default:
                    unknown_count++;
                    break;
            }

            terrain_cells.push_back(cell);

            //RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(),1000,"grid=(%d,%d) | points=%zu |" "slope=&.2f deg | planarity=%.3f | scattering=%.3f",index.x,index.y,grid_points.size(),result.slope * 180.0 / M_PI,result.planarity,result.scattering);
            RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(),1000,"grid=(%d,%d) | " "center=(%.2f,%.2f,%.2f) | " "type=%s | " "points=%zu | " "slope=%.2f deg | " "normal=(%.3f,%.3f,%.3f) | " "planarity=%.3f | " "scattering=%.3f | " "lambda=[%.5f, %.5f, %.5f] | " "height_std=%.5f",index.x,index.y,cell.x,cell.y,cell.z,terrainTypeToString(cell.terrain_type),grid_points.size(),cell.slope * 180.0 / M_PI,cell.normal.x(),cell.normal.y(),cell.normal.z(),cell.planarity,cell.scattering,cell.lambda1,cell.lambda2,cell.lambda3,cell.height_std);
        }

        terrain_analyzer::msg::TerrainAnalysis terrain_msg;

        terrain_msg.header = msg->header;

        for (const auto & cell : terrain_cells)
        {
            terrain_msg.x.push_back(cell.x);
            terrain_msg.y.push_back(cell.y);
            terrain_msg.z.push_back(cell.z);

            terrain_msg.lambda1.push_back(cell.lambda1);
            terrain_msg.lambda2.push_back(cell.lambda2);
            terrain_msg.lambda3.push_back(cell.lambda3);

            terrain_msg.linearity.push_back(cell.linearity);
            terrain_msg.planarity.push_back(cell.planarity);
            terrain_msg.scattering.push_back(cell.scattering);

            terrain_msg.normal_x.push_back(cell.normal.x());
            terrain_msg.normal_y.push_back(cell.normal.y());
            terrain_msg.normal_z.push_back(cell.normal.z());

            terrain_msg.slope.push_back(cell.slope);

            terrain_msg.mean_height.push_back(cell.z);
            terrain_msg.height_std.push_back(cell.height_std);

            terrain_msg.residual_mean.push_back(cell.residual_mean);
            terrain_msg.residual_std.push_back(cell.residual_std);

            terrain_msg.outlier_ratio.push_back(cell.outlier_ratio);
            terrain_msg.terrain_type.push_back(cell.terrain_type);

            terrain_msg.point_count.push_back(static_cast<uint32_t>(cell.point_count));
        }
        terrain_pub_->publish(terrain_msg);

        RCLCPP_INFO(this->get_logger(),"Terrain summary: " "FLAT=%d | " "SLOPE=%d | " "ROUGH=%d | " "IRREGULAR=%d | " "EDGE=%d | " "UNKNOWN=%d", flat_count,slope_count,rough_count,irregular_count,edge_count,unknown_count);
      }  

      std::string pointcloud_topic_;
      int min_points_;
      double cell_size_;

      terrain_analyzer::TerrainAnalyzer analyzer_;

      rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
      rclcpp::Publisher<terrain_analyzer::msg::TerrainAnalysis>::SharedPtr terrain_pub_;
};

int main(int argc,char ** argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<TerrainAnalyzerNode>());
    rclcpp::shutdown();

    return 0;
}