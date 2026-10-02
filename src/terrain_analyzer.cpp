#include "terrain_analyzer/terrain_analyzer.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace terrain_analyzer
{
    TerrainResult TerrainAnalyzer::analyze(
        const std::vector<Eigen::Vector3d> & points
    )
    {
        TerrainResult result;
   
        //点太少，无法进行可靠PCA
        if (points.size() < 3) {
            return result;
        }

        //计算质心
        const Eigen::Vector3d centroid = computeCentroid(points);
        //计算协方差矩阵
        const Eigen::Matrix3d covariance = computeCovariance(points,centroid);

        //PCA
        Eigen::Vector3d eigenvalues;
        Eigen::Matrix3d eigenvectors;

        if (!computePCA(covariance,eigenvalues,eigenvectors))
        {
            return result;
        }

        result.lambda1 = eigenvalues(2);
        result.lambda2 = eigenvalues(1);
        result.lambda3 = eigenvalues(0);

        result.normal = eigenvectors.col(0).normalized();

        //坡度
        result.slope = computeSlope(result.normal);

        //PCA几何特征
        result.linearity = computeLinearity(eigenvalues);
        result.planarity = computePlanarity(eigenvalues);
        result.scattering = computeScattering(eigenvalues);

        //高度统计
        computeHeightStatistics(points,result.mean_height,result.height_std);

        //PCA平面残差统计
        computeResidualStatistics(points,centroid,result.normal,result.residual_mean,result.residual_std,result.outlier_ratio);

        result.valid = true;

        return result;  
         
    }
    //质心计算
    Eigen::Vector3d TerrainAnalyzer::computeCentroid(
        const std::vector<Eigen::Vector3d> & points
    )
    {
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        
        for (const auto & point : points)
        {
            centroid += point;
        }
        centroid /= static_cast<double>(points.size());

        return centroid;
    }
    //协方差矩阵
    Eigen::Matrix3d TerrainAnalyzer::computeCovariance(
        const std::vector<Eigen::Vector3d> & points,
        const Eigen::Vector3d & centroid
    )
    {
        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();

        for (const auto & point :points){
            const Eigen::Vector3d diff = point - centroid;

            covariance += diff * diff.transpose();
        }
        covariance /= static_cast<double>(points.size());

        return covariance;
    }
    //PCA
    bool TerrainAnalyzer::computePCA(
        const Eigen::Matrix3d & covariance,
        Eigen::Vector3d & eigenvalues,
        Eigen::Matrix3d & eigenvectors
    )
    {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>
          solver(covariance);
        
        if (solver.info() != Eigen::Success)
        {
            return false;
        }
        eigenvalues = solver.eigenvalues();
        eigenvectors = solver.eigenvectors();

        return true;
    }
    //坡度计算
    double TerrainAnalyzer::computeSlope(
        const Eigen::Vector3d & normal
    )
    {
        const Eigen::Vector3d n = normal.normalized();

        const double nz = std::clamp(
            std::abs(n.z()),
            0.0,
            1.0
        );
        return std::acos(nz);
    }

    //Linear
    double TerrainAnalyzer::computeLinearity(
        const Eigen::Vector3d & eigenvalues
    )
    {
        const double lambda1 = eigenvalues(2);
        const double lambda2 = eigenvalues(1);

        if (lambda1 < 1e-12){
            return 0.0;
        }
        return(lambda1 - lambda2) / lambda1;
    }
    //Planarity
    double TerrainAnalyzer::computePlanarity(
        const Eigen::Vector3d & eigenvalues
    )
    {
        const double lambda1 = eigenvalues(2);
        const double lambda2 = eigenvalues(1);
        const double lambda3 = eigenvalues(0);

        if (lambda1 < 1e-12){
            return 0.0;
        }
        return (lambda2 - lambda3) / lambda1;
    }
    //Scattering
    double TerrainAnalyzer::computeScattering(
        const Eigen::Vector3d & eigenvalues
    )
    {
        const double lambda1 = eigenvalues(2);
        const double lambda3 = eigenvalues(0);

        if (lambda1 < 1e-12){
            return 0.0;
        }
        return lambda3 / lambda1;
    }

    //高度统计
    void TerrainAnalyzer::computeHeightStatistics(
        const std::vector<Eigen::Vector3d> & points,
        double & mean,
        double & stddev
    )
    {
        mean = 0.0;
        stddev = 0.0;

        if (points.empty()){
            return;
        }
        
        for (const auto & point : points){
            mean += point.z();
        }
        mean /= static_cast<double>(points.size());
        double variance = 0.0;

        for (const auto & point : points){
            const double diff = point.z() - mean;

            variance += diff * diff;
        }
        variance /=static_cast<double>(points.size());
        
        stddev = std::sqrt(variance);
    }

    //平面残差Gaussian
    void TerrainAnalyzer::computeResidualStatistics(
        const std::vector<Eigen::Vector3d> & points,
        const Eigen::Vector3d & centroid,
        const Eigen::Vector3d & normal,
        double & mean,
        double & stddev,
        double & outlier_ratio
    )
    {
        mean = 0.0;
        stddev = 0.0;
        outlier_ratio = 0.0;

        if (points.empty()){
            return;
        }

        std::vector<double> residuals;
        residuals.reserve(points.size());

        for (const auto & point : points){
            const double residual = normal.dot(point - centroid);

            residuals.push_back(residual);
            mean += residual;
        }

        mean /= static_cast<double>(residuals.size());
        double variance = 0.0;

        for (const double residual : residuals){
            const double diff = residual - mean;

            variance += diff * diff;
        }

        variance /= static_cast<double>(residuals.size());
        stddev = std::sqrt(variance);

        if (stddev > 1e-12){
            const double threshold = mean + 3.0 * stddev;
            std::size_t outliers = 0;

            for (const double residual : residuals){
                if (residual > threshold){
                    ++outliers;
                }
            }
            outlier_ratio = static_cast<double>(outliers) / static_cast<double>(residuals.size());
        }
    }


}    //namespace terrain_analyzer