#include "terrain_analyzer/terrain_analyzer.hpp"
#include "terrain_analyzer/msg/terrain_analysis.hpp"

#include<rclcpp/rclcpp.hpp>
#include<sensor_msgs/msg/point_cloud2.hpp>
#include<sensor_msgs/point_cloud2_iterator.hpp>
#include<visualization_msgs/msg/marker_array.hpp>

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

    double confidence = 0.0;

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
        map_radius_ = this->declare_parameter<double>("map_radius",5.0);

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pointcloud_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&TerrainAnalyzerNode::pointCloudCallback,this,std::placeholders::_1)
        );
        terrain_pub_= this->create_publisher<terrain_analyzer::msg::TerrainAnalysis>("/terrain_analysis",10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/terrain_markers",10);

        RCLCPP_INFO(this->get_logger(),"Terrain Analyzer started.");
        RCLCPP_INFO(this->get_logger(),"PointCloud topic: %s",pointcloud_topic_.c_str());
      }
    private:
      TerrainCell fuseTerrainCell(
        const TerrainCell & old_cell,
        const TerrainCell & new_cell
      )
      {
        const double alpha = 0.3;

        TerrainCell fused = old_cell;

        fused.x = new_cell.x;
        fused.y = new_cell.y;
        fused.z = (1.0 - alpha) * old_cell.z + alpha * new_cell.z;

        fused.lambda1 = (1.0 - alpha) * old_cell.lambda1 + alpha * new_cell.lambda1;
        fused.lambda2 = (1.0 - alpha) * old_cell.lambda2 + alpha * new_cell.lambda2;
        fused.lambda3 = (1.0 - alpha) * old_cell.lambda3 + alpha * new_cell.lambda3;

        fused.linearity = (1.0 - alpha) * old_cell.linearity + alpha * new_cell.linearity;
        fused.planarity = (1.0 - alpha) * old_cell.planarity + alpha * new_cell.planarity;
        fused.scattering = (1.0 - alpha) * old_cell.scattering + alpha * new_cell.scattering; 

        fused.slope = (1.0 - alpha) * old_cell.slope + alpha * new_cell.slope;

        fused.height_std = (1.0 - alpha) * old_cell.height_std + alpha * new_cell.height_std;

        fused.residual_mean = (1.0 - alpha) * old_cell.residual_mean + alpha * new_cell.residual_mean;
        fused.residual_std = (1.0 - alpha) * old_cell.residual_std + alpha * new_cell.residual_std;

        fused.outlier_ratio = (1.0 - alpha) * old_cell.outlier_ratio + alpha * new_cell.outlier_ratio;

        fused.confidence = (1.0 - alpha) * old_cell.confidence + alpha * new_cell.confidence;

        fused.point_count = new_cell.point_count;
        fused.normal = ((1.0 - alpha) * old_cell.normal + alpha * new_cell.normal).normalized();

        fused.valid = true;

        fused.terrain_type = classifyTerrain(fused);
        
        return fused;
      }
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

            const double point_confidence = std::min(static_cast<double>(grid_points.size()) / 200.0, 1.0);
            const double planarity_confidence = std::max(0.0, std::min(cell.planarity, 1.0));
            const double outlier_confidence = std::max(0.0, 1.0 - std::min(cell.outlier_ratio, 1.0));

            cell.confidence = 0.5 * point_confidence + 0.3 * planarity_confidence + 0.2 * outlier_confidence;

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

        // terrain_analyzer::msg::TerrainAnalysis terrain_msg;
        // terrain_msg.header = msg->header;
        // for (const auto & cell : terrain_cells)
        // {
        //     terrain_msg.x.push_back(cell.x);
        //     terrain_msg.y.push_back(cell.y);
        //     terrain_msg.z.push_back(cell.z);

        //     terrain_msg.lambda1.push_back(cell.lambda1);
        //     terrain_msg.lambda2.push_back(cell.lambda2);
        //     terrain_msg.lambda3.push_back(cell.lambda3);

        //     terrain_msg.linearity.push_back(fused_cell.linearity);
        //     terrain_msg.planarity.push_back(fused_cell.planarity);
        //     terrain_msg.scattering.push_back(fused_cell.scattering);

        //     terrain_msg.normal_x.push_back(fused_cell.normal.x());
        //     terrain_msg.normal_y.push_back(fused_cell.normal.y());
        //     terrain_msg.normal_z.push_back(fused_cell.normal.z());

        //     terrain_msg.slope.push_back(fused_cell.slope);

        //     terrain_msg.mean_height.push_back(fused_cell.z);
        //     terrain_msg.height_std.push_back(fused_cell.height_std);

        //     terrain_msg.residual_mean.push_back(fused_cell.residual_mean);
        //     terrain_msg.residual_std.push_back(fused_cell.residual_std);

        //     terrain_msg.outlier_ratio.push_back(fused_cell.outlier_ratio);
        //     terrain_msg.terrain_type.push_back(fused_cell.terrain_type);

        //     terrain_msg.point_count.push_back(static_cast<uint32_t>(fused_cell.point_count));
        // }

        // terrain_pub_->publish(terrain_msg);
        for (const auto & cell : terrain_cells)
        {
            const int grid_x = static_cast<int>(std::floor(cell.x / cell_size_));
            const int grid_y = static_cast<int>(std::floor(cell.y / cell_size_));

            GridIndex index{grid_x, grid_y};

            auto it = terrain_map_.find(index);

            if (it == terrain_map_.end())
            {
                terrain_map_[index] = cell;
            }
            else
            {
                it->second = fuseTerrainCell(it->second, cell);
            }
        }
        for (auto it = terrain_map_.begin(); it != terrain_map_.end(); )
        {
            const auto & cell = it->second;
            const double distance = std::sqrt(cell.x * cell.x + cell.y * cell.y);

            if (distance > map_radius_)
            {
                it = terrain_map_.erase(it);
            }
            else
            {
                ++it;
            }
        }

        terrain_analyzer::msg::TerrainAnalysis terrain_msg;
        terrain_msg.header = msg->header;

        for (const auto & item : terrain_map_)
        {
            const auto & cell = item.second;

            if (!cell.valid)
            {
                continue;
            }

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

            terrain_msg.confidence.push_back(cell.confidence);
        }
        terrain_pub_->publish(terrain_msg);
        
        for (auto it = terrain_map_.begin(); it != terrain_map_.end(); )
        {
            const auto & cell = it->second;
            const double distance = std::sqrt(cell.x * cell.x + cell.y * cell.y);
            
            if (distance > map_radius_)
            {
                it = terrain_map_.erase(it);
            }
            else
            {
                ++it;
            }
        }

        visualization_msgs::msg::MarkerArray marker_array;

        int marker_id = 0;
        for (const auto & item : terrain_map_)
        {
            const auto & cell = item.second;
            if (!cell.valid)
            {
                continue;
            }
            visualization_msgs::msg::Marker marker;

            marker.header.stamp = this->now();
            marker.header.frame_id = "base_link";

            marker.ns = "terrain";
            marker.id = marker_id++;

            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = cell.x;
            marker.pose.position.y = cell.y;
            marker.pose.position.z = cell.z;

            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = cell_size_;
            marker.scale.y = cell_size_;

            marker.scale.z = 0.05;

            if (cell.terrain_type == 1)
            {
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
            }
            else if (cell.terrain_type == 2)
            {
                marker.color.r = 1.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
            }
            else if (cell.terrain_type == 3)
            {
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
            }else if (cell.terrain_type == 4)
            {
                marker.color.r = 1.0;
                marker.color.g = 0.5;
                marker.color.b = 0.0;
            }else if (cell.terrain_type == 5)
            {
                marker.color.r = 0.0;
                marker.color.g = 0.5;
                marker.color.b = 1.0;
            }
            else
            {
                marker.color.r = 0.5;
                marker.color.g = 0.5;
                marker.color.b = 0.5;
            }

            marker.color.a = 0.8;
            marker_array.markers.push_back(marker);                                                                                   
        }
        RCLCPP_INFO(this->get_logger(),"Terrain map cell: %zu",terrain_map_.size());
        marker_pub_->publish(marker_array);

        RCLCPP_INFO(this->get_logger(),"Terrain summary: " "FLAT=%d | " "SLOPE=%d | " "ROUGH=%d | " "IRREGULAR=%d | " "EDGE=%d | " "UNKNOWN=%d", flat_count,slope_count,rough_count,irregular_count,edge_count,unknown_count);
      }  

      std::string pointcloud_topic_;
      int min_points_;
      double cell_size_;
      double map_radius_;

      terrain_analyzer::TerrainAnalyzer analyzer_;

      std::unordered_map<GridIndex, TerrainCell, GridIndexHash> terrain_map_;

      rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
      rclcpp::Publisher<terrain_analyzer::msg::TerrainAnalysis>::SharedPtr terrain_pub_;
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc,char ** argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<TerrainAnalyzerNode>());
    rclcpp::shutdown();

    return 0;
}