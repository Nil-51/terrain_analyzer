#ifndef TERRAIN_ANALYZER__TERRAIN_ANALYZER_HPP_
#define TERRAIN_ANALYZER__TERRAIN_ANALYZER_HPP_

#include <Eigen/Dense>

#include <vector>
#include <cstddef>

namespace terrain_analyzer
{
    struct TerrainResult
    {
        //PCA特征值
        double lambda1 = 0.0;
        double lambda2 = 0.0;
        double lambda3 = 0.0;

        //PCA几何特征
        double linearity = 0.0;
        double planarity = 0.0;
        double scattering = 0.0;

        //局部地形法向量
        Eigen::Vector3d normal = Eigen::Vector3d::Zero();

        //地形坡度
        double slope = 0.0;

        //高度统计
        double mean_height = 0.0;
        double height_std = 0.0;

        //PCA平面残差统计
        double residual_mean = 0.0;
        double residual_std = 0.0;

        //高斯异常点比例
        double outlier_ratio = 0.0;

        //是否计算成功
        bool valid = false;

    };
class TerrainAnalyzer
{
    public:
      TerrainAnalyzer() = default;

      TerrainResult analyze(const std::vector<Eigen::Vector3d> & points);

    private:
      Eigen::Vector3d computeCentroid(const std::vector<Eigen::Vector3d> & points);

      Eigen:: Matrix3d computeCovariance(
        const std::vector<Eigen::Vector3d> & points,
        const Eigen::Vector3d & centroid
      );
      
      bool computePCA(
        const Eigen::Matrix3d & covariance,
        Eigen::Vector3d & eigenvalues,
        Eigen::Matrix3d & eigenvectors
      );

      double computeSlope(const Eigen::Vector3d & normal);
      double computeLinearity(const Eigen::Vector3d & eigenvalues);
      double computePlanarity(const Eigen::Vector3d & eigenvalues);
      double computeScattering(const Eigen::Vector3d & eigenvalues);

      void computeHeightStatistics(
        const std::vector<Eigen::Vector3d> & points,
        double & mean,
        double & stddev 
      );

      void computeResidualStatistics(
        const std::vector<Eigen::Vector3d> & points,
        const Eigen::Vector3d & centroid,
        const Eigen::Vector3d & normal,
        double & mean,
        double & stddev,
        double & outlier_ratio
      );


};

} //namespace terrain_analyzer

#endif  // TERRAIN_ANALYZER__TERRAIN_ANALYZER_HPP_