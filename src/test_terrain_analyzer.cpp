#include "terrain_analyzer/terrain_analyzer.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

using terrain_analyzer::TerrainAnalyzer;
using terrain_analyzer::TerrainResult;

constexpr double PI = 3.14159265358979323846;

void printResult(
  const std::string & name,
  const TerrainResult & result)
{
  std::cout << "\n========================================\n";
  std::cout << "Terrain: " << name << "\n";
  std::cout << "========================================\n";

  if (!result.valid) {
    std::cout << "Result: INVALID\n";
    return;
  }

  std::cout << std::fixed << std::setprecision(6);

  std::cout << "lambda1       = "
            << result.lambda1 << "\n";

  std::cout << "lambda2       = "
            << result.lambda2 << "\n";

  std::cout << "lambda3       = "
            << result.lambda3 << "\n";

  std::cout << "linearity     = "
            << result.linearity << "\n";

  std::cout << "planarity     = "
            << result.planarity << "\n";

  std::cout << "scattering    = "
            << result.scattering << "\n";

  std::cout << "normal        = ["
            << result.normal.x() << ", "
            << result.normal.y() << ", "
            << result.normal.z() << "]\n";

  std::cout << "slope(rad)    = "
            << result.slope << "\n";

  std::cout << "slope(deg)    = "
            << result.slope * 180.0 / PI << "\n";

  std::cout << "mean_height   = "
            << result.mean_height << "\n";

  std::cout << "height_std    = "
            << result.height_std << "\n";

  std::cout << "residual_mean = "
            << result.residual_mean << "\n";

  std::cout << "residual_std  = "
            << result.residual_std << "\n";

  std::cout << "outlier_ratio = "
            << result.outlier_ratio << "\n";
}


// ------------------------------------------------------------
// 1. 水平平面
// z = 0
// ------------------------------------------------------------
std::vector<Eigen::Vector3d> createFlatPlane()
{
  std::vector<Eigen::Vector3d> points;

  for (int x = -5; x <= 5; ++x) {
    for (int y = -5; y <= 5; ++y) {

      points.emplace_back(
        x * 0.1,
        y * 0.1,
        0.0);
    }
  }

  return points;
}


// ------------------------------------------------------------
// 2. 30° 坡面
// z = x * tan(30°)
// ------------------------------------------------------------
std::vector<Eigen::Vector3d> createSlope()
{
  std::vector<Eigen::Vector3d> points;

  const double slope =
    std::tan(30.0 * PI / 180.0);

  for (int x = -5; x <= 5; ++x) {
    for (int y = -5; y <= 5; ++y) {

      const double xx = x * 0.1;
      const double yy = y * 0.1;

      const double zz =
        xx * slope;

      points.emplace_back(
        xx,
        yy,
        zz);
    }
  }

  return points;
}


// ------------------------------------------------------------
// 3. 带高斯噪声的平面
// z ~ N(0, sigma²)
// ------------------------------------------------------------
std::vector<Eigen::Vector3d> createNoisyPlane()
{
  std::vector<Eigen::Vector3d> points;

  std::default_random_engine generator(42);

  std::normal_distribution<double> noise(
    0.0,
    0.01);

  for (int x = -5; x <= 5; ++x) {
    for (int y = -5; y <= 5; ++y) {

      const double xx = x * 0.1;
      const double yy = y * 0.1;
      const double zz = noise(generator);

      points.emplace_back(
        xx,
        yy,
        zz);
    }
  }

  return points;
}


// ------------------------------------------------------------
// 4. 平面 + 凸起异常点
// ------------------------------------------------------------
std::vector<Eigen::Vector3d> createPlaneWithOutliers()
{
  auto points = createFlatPlane();

  // 添加一组明显高于地面的点
  points.emplace_back(0.0, 0.0, 0.20);
  points.emplace_back(0.05, 0.0, 0.25);
  points.emplace_back(-0.05, 0.0, 0.25);
  points.emplace_back(0.0, 0.05, 0.25);
  points.emplace_back(0.0, -0.05, 0.25);

  return points;
}


int main()
{
  TerrainAnalyzer analyzer;

  // 1. 水平面
  {
    auto points = createFlatPlane();

    auto result =
      analyzer.analyze(points);

    printResult(
      "Flat Plane",
      result);
  }

  // 2. 30° 坡
  {
    auto points = createSlope();

    auto result =
      analyzer.analyze(points);

    printResult(
      "30 Degree Slope",
      result);
  }

  // 3. 噪声平面
  {
    auto points = createNoisyPlane();

    auto result =
      analyzer.analyze(points);

    printResult(
      "Noisy Plane",
      result);
  }

  // 4. 平面 + 异常凸起
  {
    auto points =
      createPlaneWithOutliers();

    auto result =
      analyzer.analyze(points);

    printResult(
      "Plane With Outliers",
      result);
  }

  return 0;
}