#include <random>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>

#include "../include/utils.hpp"
#include "../include/pedestrian.hpp"


double Utils::proxemicsFunction(double t_a, double t_x, double t_sigma, double t_d_min)
{
   if (t_x > M_PI)
      t_x = 2.0 * M_PI - t_x;

   // DEBUG
   t_d_min = 0.7;

   return ((t_a / (t_sigma * std::sqrt(2.0 * M_PI))) * std::pow(M_E, (-0.5 * std::pow((t_x / t_sigma), 2.0)))) + t_d_min;
}


float Utils::randomFloat(float t_min, float t_max)
{
   std::random_device rd;
   std::uniform_real_distribution<double> distr{t_min, t_max};
   std::default_random_engine eng(rd());
   return (float)distr(eng);
}


std::vector<std::vector<std::string>> Utils::readInPaths(const std::string& t_path)
{
   std::vector<std::vector<std::string>> content{};
   std::vector<std::string> row{""};
   std::string line{""};
   std::string word{""};

   std::ifstream input_file{t_path};

   if (input_file.is_open()) {

      while (std::getline(input_file, line)) {
         row.clear();

         std::stringstream ss{line};

         while (std::getline(ss, word, ';')) 
            row.emplace_back(word);
         
         content.emplace_back(row);
      }
   } else
      std::cout << "Could not open file\n";

  return content;
}


Point3f Utils::getNearestPointToWall(const std::pair<Point3f, Point3f>& t_wall, Point3f t_point)
{
   Vector3f relativeEnd, relativePos, relativeEndScal, relativePosScal;
	float dotProduct;
	Point3f nearestPoint;

	// Create Vector Relative to Wall's 'start'
	relativeEnd = t_wall.second - t_wall.first;	// Vector from wall's 'start' to 'end'
	relativePos = t_point - t_wall.first;

	// Scale Both Vectors by the Length of the Wall
	relativeEndScal = relativeEnd;
	relativeEndScal.normalize();

	relativePosScal = relativePos * (1.0F / relativeEnd.length());

	// Compute Dot Product of Scaled Vectors
	dotProduct = relativeEndScal.dot(relativePosScal);

	if (dotProduct < 0.0)		// Position of Agent i located before wall's 'start'
		nearestPoint = t_wall.first;
	else if (dotProduct > 1.0)	// Position of Agent i located after wall's 'end'
		nearestPoint = t_wall.second;
	else						// Position of Agent i located between wall's 'start' and 'end'
		nearestPoint = (relativeEnd * dotProduct) + t_wall.first;

	return nearestPoint;
}


double Utils::circularMean(const std::vector<double>& t_angles)
{
   double sum_sin{0.0};
   double sum_cos{0.0};

   for (const auto& angle : t_angles) {
      sum_sin += std::sin(angle);
      sum_cos += std::cos(angle);
   }

   return std::atan2(sum_sin, sum_cos);
}


double Utils::angularDifferenceSigned(double t_angle_from, double t_angle_to)
{
   return std::atan2(std::sin(t_angle_to - t_angle_from), std::cos(t_angle_to - t_angle_from));
}


double Utils::angularDifferenceUnsigned(double t_angle_a, double t_angle_b)
{
   return M_PI - std::abs(std::abs(t_angle_a - t_angle_b) - M_PI); 
}


double Utils::normalizeAngle(double t_angle)
{
   if (t_angle < 0.0)
      return t_angle + 2.0 * M_PI;
   else if (t_angle > (2.0 * M_PI))
      return t_angle - 2.0 * M_PI;
   else
      return t_angle;
}


double Utils::localAngle(const Point3f& t_from_position, const Point3f& t_to_position)
{
   return Utils::normalizeAngle(std::atan2(t_to_position.y - t_from_position.y, t_to_position.x - t_from_position.x));
}


Point3f Utils::lineLineIntersection(const Point3f& t_robot_position, const Point3f& t_robot_extrapolated_position, const Point3f& t_person_position, const Point3f& t_person_extrapolated_position)
{
	double a1{t_robot_extrapolated_position.y - t_robot_position.y};
	double b1{t_robot_position.x - t_robot_extrapolated_position.x};
	double c1{a1 * t_robot_position.x + b1 * t_robot_position.y};
	
	double a2{t_person_extrapolated_position.y - t_person_position.y};
	double b2{t_person_position.x - t_person_extrapolated_position.x};
	double c2{a2 * t_person_position.x + b2 * t_person_position.y};

	double determinant{a1 * b2 - a2 * b1};

	if (determinant == 0.0)
		return Point3f{0.0, 0.0, 0.0};
	else {
		double x{(b2 * c1 - b1 * c2) / determinant};
		double y{(a1 * c2 - a2 * c1) / determinant};
		return Point3f{x, y, 0.0};
	}
}


double Utils::euclideanDistance(const Point3f& t_point_a, const Point3f& t_point_b)
{
   return std::sqrt(std::pow(t_point_b.x - t_point_a.x, 2.0) + std::pow(t_point_b.y - t_point_a.y, 2.0));
}


std::vector<std::pair<Point3f, double>> Utils::createLaserScannerDataPoints(const Point3f& t_robot_position, double t_robot_heading, std::vector<std::pair<Point3f, Point3f>> t_obstacles)
{
   std::vector<std::pair<Point3f, double>> data_points{};

   // TODO: define externally
   const double DATA_POINTS_MAX_RANGE{10.0};

   // TODO: define externally
   for (double i = (-M_PI / 2.0); i <= (M_PI / 2.0); i += 0.05) {
      double x_r = t_robot_position.x + std::cos(normalizeAngle(t_robot_heading + i));
      double y_r = t_robot_position.y + std::sin(normalizeAngle(t_robot_heading + i));

      std::vector<Point3f> intersection_points{};

      for (const auto& obstacle_segment : t_obstacles) {
         auto intersection_point{lineLineIntersection(t_robot_position, Point3f{x_r, y_r, 0.0}, obstacle_segment.first, obstacle_segment.second)};

         // check if intersection point belongs to wall segment
         // add data point to list
         double angle_robot_to_intersection_point{Utils::localAngle(t_robot_position, intersection_point)};

         if ((intersection_point.x != 0.0 && intersection_point.y != 0.0) &&
               (intersection_point.x <= std::max(obstacle_segment.first.x, obstacle_segment.second.x) && intersection_point.x >= std::min(obstacle_segment.first.x, obstacle_segment.second.x)) &&
               (intersection_point.y <= std::max(obstacle_segment.first.y, obstacle_segment.second.y) && intersection_point.y >= std::min(obstacle_segment.first.x, obstacle_segment.second.y)) &&
               Utils::angularDifferenceUnsigned(t_robot_heading, angle_robot_to_intersection_point) <= M_PI / 2.0 &&
               Utils::euclideanDistance(t_robot_position, intersection_point) < DATA_POINTS_MAX_RANGE) {
                  intersection_points.emplace_back(intersection_point);
         }
      }

      if (intersection_points.empty())
         data_points.emplace_back(std::make_pair(Point3f{t_robot_position.x + DATA_POINTS_MAX_RANGE * std::cos(Utils::normalizeAngle(t_robot_heading + i)), t_robot_position.y + DATA_POINTS_MAX_RANGE * std::sin(Utils::normalizeAngle(t_robot_heading + i)), 0.0}, i));
      else {
         std::pair<Point3f, double> min_point{Point3f{}, MAXFLOAT};
 
         for (const auto& point : intersection_points) {
            auto euclidean_distance{Utils::euclideanDistance(t_robot_position, point)};
            if (euclidean_distance < min_point.second)
               min_point = std::make_pair(point, euclidean_distance);
         }
         data_points.emplace_back(std::make_pair(min_point.first, i));
      }
   }

   return data_points;
}
