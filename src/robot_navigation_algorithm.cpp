#include <fstream>
#include <algorithm>
#include "predictive_proxemics_navigation/robot_navigation_algorithm.h"


RobotNavigationAlgorithm::RobotNavigationAlgorithm() : m_group_detector{GroupDetector{}}
{

}


RobotNavigationAlgorithm::RobotNavigationAlgorithm(float t_time_step, float t_max_speed, const std::vector<std::vector<int>> t_traversible, const Point3f& t_start, const Point3f& t_goal) : m_group_detector{GroupDetector{}}
{
	// TODO: DEBUG
	m_debug = true;

	m_start_position = t_start;
	m_goal_position = t_goal;

	m_traversible = t_traversible;


	// for (auto& row : m_traversible) {
	// 	for (auto& pixel : row) {
	// 		if (pixel == 0) {
	// 			pixel = 100;
	// 		} else {
	// 			pixel = 0;
	// 		}
	// 	}
	// }

	m_a_star = AStar();


    m_time_step = t_time_step;
	m_max_speed = t_max_speed;
	m_goal = t_goal;

	m_waypoints.emplace_front(t_start);
	m_waypoints.emplace_back(t_goal);

	m_navigation_waypoint_met = 0.2;
    m_prediction_horizon = 3.0;
    m_proxemics_function_a_weight = 2.0;
    m_proxemics_function_sigma = 1.0;
    m_evasion_proxemics_data_points_step = 0.05;
    m_evasion_inner_score_intersection_weight = 0.5;
    m_evasion_inner_score_intrusion_weight = 0.5;

	m_prediction_distance_threshold = 1.2 * m_prediction_horizon + m_max_speed * m_prediction_horizon;

	/*
	for (int i{0}; i < m_agents->size() - 1; ++i)
		m_sensed_positions.emplace_back(std::deque<Point3f>{});
		*/
}


void RobotNavigationAlgorithm::sense(const Point3f& t_robot_position, const std::vector<Point3f>& t_pedestrian_positions)
{

	if (!m_inital_path_set && !m_no_global_planner) {
		applyGlobalPlanner();
		m_inital_path_set = true;
	}

	// set own position
	m_own_positions.emplace_back(Point3f{t_robot_position.x + m_start_position.x, t_robot_position.y + m_start_position.y, 0.0});

	if (m_own_positions.size() >= 10)
		m_own_positions.pop_front();

	// TODO: store pedestrian positions	
	for (const auto& pedestrian : t_pedestrian_positions) {

		bool found{false};
		for (auto& sensed : m_sensed_positions) {
			if (!sensed.second.empty()) {
				if (Utils::euclideanDistance(pedestrian, sensed.second.at(sensed.second.size() - 1)) < 0.5) {
					found = true;
					sensed.second.emplace_back(Point3f{pedestrian.x, pedestrian.y, 0.0});
				}
			}
		}

		if (!found) {
			std::deque<Point3f> deque{};
			deque.emplace_back(Point3f{pedestrian.x, pedestrian.y, 0.0});
			m_sensed_positions.emplace_back(std::make_pair("", deque));
		}
	}

	// TODO: test: delete pedestrians 
	int i{0};
	for (const auto& position : t_pedestrian_positions) {
		bool found{false};
		for (const auto& sensed : m_sensed_positions) {
			if (!sensed.second.empty()) {
				if (!(position.x == sensed.second.at(sensed.second.size() - 1).x && position.y == sensed.second.at(sensed.second.size() - 1).y)) {
					found = true;
					break;
				}
			}
		}

		if (!found) {
			m_sensed_positions.erase(m_sensed_positions.begin() + i);
			continue;
		}
		++i;
	}
}


void RobotNavigationAlgorithm::noGlobalPlanner()
{
	m_no_global_planner = true;
}


void RobotNavigationAlgorithm::applyGlobalPlanner()
{
	// FIXME
	return;

	// TODO: change again
	Point3f current_robot_position{};
	if (m_own_positions.size() == 0)
		current_robot_position = m_waypoints.at(0);
	else
		current_robot_position = m_own_positions.at(m_own_positions.size() - 1);

	if (m_debug) {
		std::cout << "m_own_positions:\n";
		for (const auto& p : m_own_positions) {
			std::cout << p << std::endl;
		}
	}

	// DEBUG
	std::cout << "current robot position: " << current_robot_position << std::endl;

	//AStar global_planner{AStar{(int) current_robot_position.x * 20, (int) current_robot_position.y * 20, m_goal.x * 20, m_goal.y * 20, m_traversible}};

	m_a_star.applyAStar((int) (current_robot_position.x * 20), (int) (current_robot_position.y * 20), m_goal.x * 20, m_goal.y * 20);

	m_waypoints.clear();

	// DEBUG
	/*
	if (m_debug) {
		std::cout << "a star path waypoints:" << std::endl;
		for (const auto& w : global_planner.pathAStar)
			std::cout << w.x << " " << w.y << std::endl;

		std::cout << "a star selected waypoints:" << std::endl;
		for (const auto& w : global_planner.pathAStarSelectedPoints)
			std::cout << w.x << " " << w.y << std::endl;

		std::cout << "a star dense waypoints:" << std::endl;
		for (const auto& w : global_planner.pathAStarDense)
			std::cout << w.x << " " << w.y << std::endl;

		std::cout << "a star smoothed waypoints:" << std::endl;
		for (const auto& w : global_planner.pathAStarSelectedPoints) {
			std::cout << w.x << " " << w.y << std::endl;
			m_waypoints.emplace_front(Point3f{w.x / 20.0, w.y / 20.0, 0.0});
		}
	}

	std::cout << "path length: " << global_planner.pathAStarDense.size() << std::endl;
	*/

	for (const auto& w : m_a_star.pathAStarSmoothed) {
		// std::cout << w.x << " " << w.y << std::endl;
		m_waypoints.emplace_front(Point3f{w.x / 20.0, w.y / 20.0, 0.0});
	}

	std::ofstream log_file{"astar_log.txt"};
	if (log_file.is_open()) {
		log_file << "x;y\n";

		for (const auto& w : m_a_star.pathAStarSmoothed) {
			log_file << w.x << ";" << w.y << "\n";
		}
		log_file.close();
	}
}


bool RobotNavigationAlgorithm::obstacleCollisionDetected(const Point3f& t_position)
{
	int new_x{(int) (t_position.x * 20)};
	int new_y{(int) (t_position.y  * 20)};

	if (new_x > 0 && new_x < m_traversible.at(0).size() && new_y > 0 && new_y < m_traversible.size()) {
		if (m_traversible.at((int) t_position.y * 20).at((int) t_position.x * 20) > 0)
			return true;
		else return false;
	}
	return false;
}


Vector3f RobotNavigationAlgorithm::drivingForce()
{
    const float T = 0.54F;	// relaxation time based on (Moussaid et al., 2009)
	Vector3f e_i, f_i;

	std::cout << "waypoints:\n";
	for (const auto& w : m_waypoints)
		std::cout << w << std::endl;

	if (m_waypoints.at(0).x == m_own_positions.at(m_own_positions.size() - 1).x && m_waypoints.at(0).y == m_own_positions.at(m_own_positions.size() - 1).y) {
		std::cout << "same waypoint, popping!\n";

		if (m_waypoints.size() > 1)
			m_waypoints.pop_front();
	}


    // compute desired direction
	e_i = m_waypoints.at(0) - m_own_positions.at(m_own_positions.size() - 1);
	e_i.normalize();

    // compute driving force
	f_i = ((m_desired_speed * e_i) - m_velocity_vector) * (1 / T);

	std::cout << "current pos: " << m_own_positions.at(m_own_positions.size() - 1) << std::endl;
	std::cout << "waypoints at 0: " << m_waypoints.at(0) << std::endl;
	std::cout << "ei: " << e_i << std::endl;
	std::cout << "desired speed: " << m_desired_speed << std::endl;
	std::cout << "driving force: " << f_i << std::endl;

	return f_i;
}


void RobotNavigationAlgorithm::plan()
{
	// if (!m_no_global_planner)
	// 	applyGlobalPlanner();

	if (m_debug)
		std::cout << "current evasion waypoint: " << m_evasion_waypoint << std::endl;

	bool pedestrian_collision_detected{false};

	// DEBUG
	std::cout << "current position: " << m_current_robot_position << std::endl;


	double distance_to_goal{Utils::euclideanDistance(m_own_positions.at(m_own_positions.size() - 1), m_waypoints.at(m_waypoints.size() - 1))};

	/*
	if (obstacleCollisionDetected()) {

		if (m_debug)
			std::cout << "obstacle collsion detected, applying a star...\n";

		m_navigation_mode == NORMAL;
		m_waypoints.clear();
		applyGlobalPlanner();
	}
	*/

	/*
	if (distance_to_goal > m_distance_to_goal && m_waypoints.size() > 1)
		m_waypoints.pop_front();

	*/
	if (m_debug)
		std::cout << "current speed: " << m_current_speed << std::endl;

	auto waypoint_difference{Utils::euclideanDistance(m_own_positions.at(m_own_positions.size() - 1), m_waypoints.at(0))};

	if ((waypoint_difference < m_navigation_waypoint_met) && m_waypoints.size() > 1) {

		if (m_waypoints.at(0).x == m_evasion_waypoint.x && m_waypoints.at(0).y == m_evasion_waypoint.y) {

			// DEBUG
			std::cout << "evasion waypoint met!\n";
			//applyGlobalPlanner();

			bool flag{false};
			while (true) {

				auto distance_to_goal{Utils::euclideanDistance(m_goal, m_waypoints.at(0))};
				auto distance_to_robot{Utils::euclideanDistance(m_current_robot_position, m_waypoints.at(0))};

				if (distance_to_goal < 2.0 || distance_to_robot > 4.0)
					break;

				// DEBUG
				std::cout << "removing current waypoint\n";

				if (m_waypoints.size() > 1)
					m_waypoints.pop_front();
			}

		}

		if (m_waypoints.size() > 1)
			m_waypoints.pop_front();
	}
		

	/*
	auto available_space{calculateAvailableSpace()};

	if (m_debug)
		std::cout << "available space: " << available_space << std::endl;
		*/

	// set desired_speed
	float min_distance{MAXFLOAT};
	for (const auto& sensed : m_sensed_positions) {
		auto distance{Utils::euclideanDistance(m_own_positions.at(m_own_positions.size() - 1), sensed.second.at(sensed.second.size() - 1))};

		if (distance < min_distance) {
			min_distance = distance;
		}
	}

	m_desired_speed = speedFunction(min_distance);

	int number_of_waypoints{m_waypoints.size()};

	if (number_of_waypoints < m_number_of_waypoints && m_navigation_mode == EVASION_DRIFTING) {
		m_navigation_mode = EVASION;
	}

	m_number_of_waypoints = number_of_waypoints;

	double delta_t{0.0};

	// robot's position
	double r_velocity{0.0};
	double r_yaw_rate{0.0};
	double r_acceleration{0.0};

	// TODO: change
	double r_delta_t{0.05};

	double r_theta{0.0};

	int d_n{0};
	m_own_positions.size() >= 5 ? d_n = 5 : d_n = m_own_positions.size();

	Point3f r_last_position{};
	std::vector<Point3f> p_last_positions{};

	if (!m_own_positions.empty())
		r_last_position = m_own_positions.at(m_own_positions.size() - 1);

	// FIXME: introduces bugs!
	for (int j{0}; j < m_sensed_positions.size(); ++j) {
		if (!m_sensed_positions.at(j).second.empty())
			p_last_positions.emplace_back(m_sensed_positions.at(j).second.at(m_sensed_positions.at(j).second.size() - 1));
	}

	Point3f current_position{};
	if (d_n > 1) {

		// get robot data
		std::vector<double> robot_yaw_rates{};
		std::vector<double> robot_velocities{};

		for (int r_i{(int)m_own_positions.size() - d_n}; r_i < m_own_positions.size() - 1; ++r_i) {
			auto velocity{static_cast<Vector3f>(m_own_positions.at(r_i + 1) - m_own_positions.at(r_i)).length()};

			r_velocity += velocity;
			robot_velocities.emplace_back(velocity);

			Point3f old_position{m_own_positions.at(r_i)};
			current_position = m_own_positions.at(r_i + 1);

			auto theta{atan2(current_position.y - old_position.y, current_position.x - old_position.x)};

			if (theta < 0.0)
				theta += 2 * M_PI;

			r_theta = theta;
			robot_yaw_rates.emplace_back(theta);
		}

		m_current_robot_position = current_position;
		m_current_robot_heading = r_theta;

		r_velocity /= (d_n * r_delta_t);

		m_current_speed = r_velocity;

		for (int y_i{0}; y_i < robot_yaw_rates.size() - 1; ++y_i)
			r_yaw_rate += std::abs(robot_yaw_rates[y_i + 1] - robot_yaw_rates[y_i]);
		r_yaw_rate /= (robot_yaw_rates.size() - 1);

		for (int v_i{0}; v_i < robot_velocities.size() - 1; ++v_i)
			r_acceleration += std::abs(robot_velocities[v_i + 1] - robot_velocities[v_i]);
		r_acceleration /= (robot_velocities.size() - 1);

		if (r_yaw_rate == 0.0)
			r_yaw_rate += 0.0000001;
		
		std::vector<std::vector<double>> pedestrian_values{};

		// check if evasion drifting is over
		/*
		if (m_navigation_mode == EVASION_DRIFTING) {
			double global_angle_to_waypoint{std::atan2(m_evasion_waypoint.y - current_position.y, m_evasion_waypoint.x - current_position.x)};

			if (global_angle_to_waypoint < 0.0)
				global_angle_to_waypoint += 2 * M_PI;

			double relative_angle_to_waypoint{global_angle_to_waypoint - r_theta};
			if (relative_angle_to_waypoint < 0.0)
				relative_angle_to_waypoint += 2 * M_PI;

			// TODO: define externally
			if (relative_angle_to_waypoint < 0.3 || relative_angle_to_waypoint > 2 * M_PI - 0.3) {
				m_navigation_mode = EVASION;
			}
		}
		*/

		std::vector<int> predicted_pedestrian_indeces{};

		if (m_debug)
			std::cout << "pedestrian calculation...\n";

		// calculate values for detected pedestrians
		for (auto j{0}; j < m_sensed_positions.size(); ++j) {

			if (Utils::euclideanDistance(m_sensed_positions.at(j).second.at(m_sensed_positions.at(j).second.size() - 1), m_own_positions.at(m_own_positions.size() - 1))
				< m_prediction_distance_threshold) {
					predicted_pedestrian_indeces.emplace_back(j);
				}

			double p_velocity{0.0};
			double p_yaw_rate{0.0};
			double p_acceleration{0.0};
			double p_theta{0.0};

			pedestrian_values.emplace_back(std::vector<double>{});

			std::vector<double> pedestrian_yaw_rates{};
			std::vector<double> pedestrian_velocities{};



			for (int r_i{(int)m_sensed_positions.at(j).second.size() - d_n}; r_i < m_sensed_positions.at(j).second.size() - 1; ++r_i) {
				auto velocity{static_cast<Vector3f>(m_sensed_positions.at(j).second.at(r_i + 1) - m_sensed_positions.at(j).second.at(r_i)).length()};

				p_velocity += velocity;

				if (velocity < 3.0)
					pedestrian_velocities.emplace_back(velocity);

				Point3f old_position{m_sensed_positions.at(j).second.at(r_i)};
				Point3f current_position{m_sensed_positions.at(j).second.at(r_i + 1)};

				auto theta{Utils::localAngle(old_position, current_position)};

				p_theta = theta;

				pedestrian_yaw_rates.emplace_back(theta);
			}

			p_velocity /= (d_n * r_delta_t);

			if (pedestrian_yaw_rates.size() > 0) {
				for (int y_i{0}; y_i < pedestrian_yaw_rates.size() - 1; ++y_i)
					p_yaw_rate += std::abs(pedestrian_yaw_rates[y_i + 1] - pedestrian_yaw_rates[y_i]);

				p_yaw_rate /= pedestrian_yaw_rates.size() - 1;
			}

			if (pedestrian_velocities.size() > 0) {
				for (int v_i{0}; v_i < pedestrian_velocities.size() - 1; ++v_i)
					p_acceleration += std::abs(pedestrian_velocities[v_i + 1] - pedestrian_velocities[v_i]);
				p_acceleration /= pedestrian_velocities.size() - 1;
			}

			pedestrian_values.at(j).emplace_back(p_velocity);
			pedestrian_values.at(j).emplace_back(p_yaw_rate);
			pedestrian_values.at(j).emplace_back(p_acceleration);
			pedestrian_values.at(j).emplace_back(p_theta);

		}

		// in prediction
		while (delta_t <= m_prediction_horizon) {

			// FIXME: this is a hack, remove!
			// if (Utils::euclideanDistance(m_own_positions.at(m_own_positions.size() - 1), m_waypoints.at(m_waypoints.size() - 1)) < 3.0)
				//  break;

			m_pedestrian_positions_at_evasion.clear();

			std::vector<Utils::GroupInformation> evasion_groups{};

			std::vector<Utils::PedestrianInformation> pedestrian_information;

			double r_new_x{(1 / pow(r_yaw_rate, 2.0)) * ((r_velocity * r_yaw_rate + r_acceleration * r_yaw_rate * delta_t)
										* sin(r_theta + r_yaw_rate * delta_t)
										+ r_acceleration * cos(r_theta + r_yaw_rate * delta_t)
										- r_velocity * r_yaw_rate * sin(r_theta) - r_acceleration * cos(r_theta))};

			double r_new_y{(1 / pow(r_yaw_rate, 2.0)) * ((-r_velocity * r_yaw_rate - r_acceleration * r_yaw_rate * delta_t)
												* cos(r_theta + r_yaw_rate * delta_t)
												+ r_acceleration * sin(r_theta + r_yaw_rate * delta_t)
												+ r_velocity * r_yaw_rate * cos(r_theta) - r_acceleration * sin(r_theta))};

			Point3f r_new_position{r_last_position.x + (float)r_new_x, r_last_position.y + (float)r_new_y, 0.0};

			/*
			if (obstacleCollisionDetected(r_new_position)) {

				if (m_debug)
					std::cout << "obstacle collsion detected, applying a star...\n";

				m_navigation_mode == NORMAL;
				//applyGlobalPlanner();
				break;
			}
			*/

			// position for pedestrians
			// TODO: selected pedestrians only
			for (auto j{0}; j < pedestrian_values.size(); ++j) {

				bool predict{false};
				for (const auto& index : predicted_pedestrian_indeces) {
					if (index == j) {
						predict = true;
						break;
					}
				}

				// FIXME
				if (predict || true) {
					double p_velocity{pedestrian_values.at(j).at(0)};
					double p_yaw_rate{pedestrian_values.at(j).at(1)};
					double p_acceleration{pedestrian_values.at(j).at(2)};
					double p_theta{pedestrian_values.at(j).at(3)};

					// ugly but necessary
					if (p_yaw_rate == 0.0)
						p_yaw_rate += 0.0000001;

					auto p_new_x{(1 / pow(p_yaw_rate, 2.0)) * ((p_velocity * p_yaw_rate + p_acceleration * p_yaw_rate * delta_t)
												* sin(p_theta + p_yaw_rate * delta_t)
												+ p_acceleration * cos(p_theta + p_yaw_rate * delta_t)
												- p_velocity * p_yaw_rate * sin(p_theta) - p_acceleration * cos(p_theta))};

					auto p_new_y{(1 / pow(p_yaw_rate, 2.0)) * ((-p_velocity * p_yaw_rate - p_acceleration * p_yaw_rate * delta_t)
														* cos(p_theta + p_yaw_rate * delta_t)
														+ p_acceleration * sin(p_theta + p_yaw_rate * delta_t)
														+ p_velocity * p_yaw_rate * cos(p_theta) - p_acceleration * sin(p_theta))};

					Point3f pedestrian_new_position{p_last_positions.at(j).x + p_new_x, p_last_positions.at(j).y + p_new_y, 0.0};

					double predicted_difference{static_cast<Vector3f>(r_new_position - pedestrian_new_position).length()};

					m_pedestrian_positions_at_evasion.emplace_back(Utils::PedestrianInformation{j, pedestrian_new_position, p_theta, p_velocity, predicted_difference});
				}
			}

			auto group_information = m_group_detector.detectGroups(m_pedestrian_positions_at_evasion);

			// calculate distance to robot
			for (auto& group : group_information)
				group.distance = Utils::euclideanDistance(r_new_position, group.position);

			// check nearest person first, if it is in personal space

			// TODO

			std::sort(group_information.begin(), group_information.end(), [](auto& left, auto& right) {
				return left.distance < right.distance;
			});

			for (const auto& group : group_information) {
				auto is_in_personal_space{isInPersonalSpace(r_new_position, group.position, group.theta, group.speed, group.max_distance)};
					
				// evade!
				if (is_in_personal_space && m_navigation_mode != EVASION_DRIFTING && m_navigation_mode != COLLISON_AVOIDANCE) {

					if (m_debug)
						std::cout << "evading!" << std::endl;

					if (m_navigation_mode == EVASION) {
						bool found{false};
						for (const auto& w : m_waypoints) {
							if (w.x == m_evasion_waypoint.x && w.y == m_evasion_waypoint.y) {
								found = true;
								break;
							}
						}

						/*
						if (found)
							m_waypoints.pop_front();
							*/
					}

					evasion_groups.emplace_back(group);

					//m_navigation_mode = EVASION_DRIFTING;

				}
			}

			// TODO: filter out last evasion
			// last evasion position (person)
			// if last evasion in proxemic area + last evasion position ~ current pos -> delete
			
			if (evasion_groups.size() > 0) {

				if (m_debug)
					std::cout << "delta_t: " << delta_t << ", evading!\n";

				std::cout << "evasions: " << evasion_groups.size() << std::endl;

				double evasion_distance{0.0};
				bool found{false};
				int max_i{0};
				double max_angular_difference{0.0};
				double min_distance{MAXFLOAT};
				for (int i{0}; i < evasion_groups.size(); ++i) {

					auto distance = Utils::euclideanDistance(r_new_position, evasion_groups.at(i).position);

					// DEBUG
					std::cout << "evasion group position: " << evasion_groups.at(i).position << ", distance: " << distance << std::endl;
					std::cout << "m_last_evasion_in_proxemic_area: " << m_last_evasion_in_proxemic_area << std::endl;
					std::cout << "evasion distance: " << distance << std::endl;
					std::cout << "m_evasion_distance: " << m_evasion_distance << std::endl;
					std::cout << "position difference: " << Utils::euclideanDistance(evasion_groups.at(i).position, m_last_evasion_position) << std::endl;

					auto angular_difference{Utils::angularDifferenceUnsigned(r_theta, evasion_groups.at(i).theta)};
					if (evasion_distance < min_distance) {
						min_distance = distance;
						max_i = i;
						found = true;
						evasion_distance = distance;
					}
				}

				std::cout << "later\n";
				std::cout << "evasion group position: " << evasion_groups.at(max_i).position << ", distance: " << evasion_distance << std::endl;
				std::cout << "m_last_evasion_in_proxemic_area: " << m_last_evasion_in_proxemic_area << std::endl;
				std::cout << "evasion distance: " << evasion_distance << std::endl;
				std::cout << "m_evasion_distance: " << m_evasion_distance << std::endl;
				std::cout << "position difference: " << Utils::euclideanDistance(evasion_groups.at(max_i).position, m_last_evasion_position) << std::endl;

				// skip
				if (m_last_evasion_in_proxemic_area && evasion_distance > m_evasion_distance && Utils::euclideanDistance(evasion_groups.at(max_i).position, m_last_evasion_position) < 0.1) {

					if (m_debug)
						std::cout << "skipping...\n";

				} else {

					if (m_debug)
						std::cout << "not skipping, evading...\n";

					if (delta_t == 0.0)
						m_last_evasion_in_proxemic_area = true;
					else 
						m_last_evasion_in_proxemic_area = false;

					auto group{evasion_groups.at(max_i)};
					m_evasion_distance = Utils::euclideanDistance(r_new_position, group.position);

					m_last_evasion_position = group.position;

					pedestrian_collision_detected = true;

					evade(delta_t == 0.0, r_new_position, r_theta, group.position, group.theta, group.speed, group.max_distance);
					break;
				}
			}

			/*
			if (pedestrian_collision_detected)
				break;
				*/

			delta_t += 0.1;
		}
	}

	if (!pedestrian_collision_detected) {
		if (m_debug) {
			std::cout << "current mode: " << m_navigation_mode << std::endl;
			std::cout << "current evasion waypoint: " << m_current_evasion_waypoint << std::endl;
			std::cout << "current waypoint: " << m_waypoints.at(0) << std::endl;
			std::cout << "no collsions detected\n";
		}
			

		if (m_current_evasion_waypoint == m_waypoints.at(0) && m_waypoints.size() > 1) {
			m_waypoints.pop_front();
		}
			

		m_navigation_mode = NORMAL;
	}



	/*
	if (!pedestrian_collision_detected && m_waypoints.size() > 1) {
		m_waypoints.pop_front();
	}
	*/
}


Point3f RobotNavigationAlgorithm::act()
{
	return m_waypoints.at(0);
	// if (!m_waypoints.empty()) {
		
	// 	// compute social force
	// 	Vector3f acceleration{drivingForce()};

	// 	// compute new velocity
	// 	m_velocity_vector = m_velocity_vector + acceleration * m_time_step;

	// 	// truncate velocity if it exceeds maximum speed
	// 	if (m_velocity_vector.lengthSquared() > (m_desired_speed * m_desired_speed)) {
	// 		m_velocity_vector.normalize();
	// 		m_velocity_vector *= m_desired_speed;
	// 	}

	// 	// compute new position
	// 	Point3f new_position{m_own_positions.at(m_own_positions.size() - 1) + m_velocity_vector * m_time_step};

	// 	return std::vector<float>{new_position.x, new_position.y, (float)m_current_robot_heading, m_desired_speed};
	// }

	// return std::vector<float>{m_goal.x, m_goal.y, (float)m_current_robot_heading, m_desired_speed};
}


double RobotNavigationAlgorithm::speedFunction(double t_distance)
{
	return m_max_speed;
	if (t_distance > 1.2)
		return m_max_speed;
	else {
		if (t_distance < 0.45)
			return 0.1;
		else
			return ((m_max_speed - 0.1) / 0.75) * t_distance + ((0.12 - 0.45 * m_max_speed) / 0.75);
	}
}


bool RobotNavigationAlgorithm::isInPersonalSpace(Vector3f t_robot_position, Vector3f t_pedestrian_position, double t_pedestrian_bearing_angle, double t_pedestrian_velocity, double t_min_distance)
{
	// get angle of robot in relation to pedestrian
	auto theta{atan2(t_robot_position.y - t_pedestrian_position.y, t_robot_position.x - t_pedestrian_position.x)};

	if (theta < 0.0)
		theta += 2 * M_PI;

	auto relative_angle_to_robot{theta - t_pedestrian_bearing_angle};

	if (relative_angle_to_robot < 0.0)
		relative_angle_to_robot += 2 * M_PI;

	// change angle because of partly defined function
	if (relative_angle_to_robot > M_PI)
		relative_angle_to_robot = 2 * M_PI - relative_angle_to_robot;

	double sigma{0.6};
	double d_min{1.2};

	auto a{t_pedestrian_velocity * m_proxemics_function_a_weight};

	auto social_distance{Utils::proxemicsFunction(a, relative_angle_to_robot, m_proxemics_function_sigma, t_min_distance)};

	bool is_in_personal_space{false};
	(static_cast<Vector3f>(t_robot_position - t_pedestrian_position).length() - social_distance < 0.0) ? is_in_personal_space = true : is_in_personal_space = false;

	return is_in_personal_space;
}


void RobotNavigationAlgorithm::evade(bool t_in_proxemic_area, Point3f t_predicted_robot_position, double t_robot_theta, Point3f t_predicted_person_position, double t_person_theta, double t_person_velocity, double t_min_distance)
{
	m_current_pedestrian_position = t_predicted_person_position;

	// DEBUG
	std::cout << "evading! sim time: " << m_sim_time << ", predicted robot position: " << t_predicted_robot_position << ", predicted person position: " << t_predicted_person_position << std::endl;
	std::cout << "person theta: " << t_person_theta * 180 / M_PI << ", person velocity: " << t_person_velocity << ", min distance: " << t_min_distance << std::endl;
	
	// compute points for proxemics
	std::vector<Point3f> proxemics_points{};

	std::vector<Point3f> points{};

	for (double i{0}; i < 2 * M_PI + 0.05; i += 0.05) {
		auto a{t_person_velocity * m_proxemics_function_a_weight};
		points.emplace_back(Point3f{i, Utils::proxemicsFunction(a, i, m_proxemics_function_sigma, t_min_distance), 0.0});
	}

	std::ofstream log_file{};
	
	if (m_debug)
		log_file.open("proxemic_points.csv");

	/*
	if (m_debug && log_file.is_open()) {
		log_file << m_agents->at(0).position().x << ";" << m_agents->at(0).position().y << "\n";
		log_file << t_predicted_robot_position.x << ";" << t_predicted_robot_position.y << "\n";
		log_file << t_predicted_person_position.x << ";" << t_predicted_person_position.y << "\n";
	}
	*/

	double cartesian_person_theta{t_person_theta - M_PI_2};

	std::vector<Point3f> cartesian_points{};
	for (const auto& p : points) {
		Point3f cartesian_point{(p.y * sin(p.x)), (p.y * cos(p.x)), 1.0};

		Matrix3f rotation_matrix{Matrix3f{cos(cartesian_person_theta), -sin(cartesian_person_theta), 0.0,
			sin(cartesian_person_theta), cos(cartesian_person_theta), 0.0,
			0.0, 0.0, 1.0}};

		Point3f rotated_point{rotation_matrix * cartesian_point};

		Point3f translated_point{t_predicted_person_position.x + rotated_point.x, t_predicted_person_position.y + rotated_point.y, 0.0};

		if (m_debug && log_file.is_open())
			log_file << translated_point.x << ";" << translated_point.y << "\n"; 

		cartesian_points.emplace_back(translated_point);
	}

	// LOG
	if (m_debug)
		log_file.close();

	m_current_proxemic_boundary = cartesian_points;

	// choose the nearest border point if the robot is already in the proxemic area
	if (t_in_proxemic_area) {

		double angle_robot_person{Utils::localAngle(m_current_robot_position, t_predicted_person_position)};

		double angle_person_robot{Utils::localAngle(t_predicted_person_position, m_current_robot_position)};

		bool robot_has_same_direction{Utils::angularDifferenceUnsigned(m_current_robot_heading, t_person_theta) < (M_PI / 2.0)};

		// find intersection point
		double x_p{t_predicted_person_position.x + std::cos(t_person_theta)};
		double y_p{t_predicted_person_position.y + std::sin(t_person_theta)};

		double x_r{m_current_robot_position.x + std::cos(m_current_robot_heading)};
		double y_r{m_current_robot_position.y + std::sin(m_current_robot_heading)};

		Point3f intersection_point{Utils::lineLineIntersection(m_current_robot_position, Point3f{x_r, y_r, 0.0}, t_predicted_person_position, Point3f{x_p, y_p, 0.0})};

		if (m_debug)
			std::cout << "intersection point: " << intersection_point << std::endl;

		double intersection_point_difference{std::sqrt(std::pow(t_predicted_person_position.x - intersection_point.x, 2.0) + std::pow(t_predicted_person_position.y - intersection_point.y, 2.0))};

		auto a{t_person_velocity * m_proxemics_function_a_weight};
		double score_intersection{intersection_point_difference / Utils::proxemicsFunction(a, 0.0, m_proxemics_function_sigma, 1.2)};

		if (score_intersection > 1.0)
			score_intersection = 1.0;

		score_intersection = 1.0 - score_intersection;

		// calculate score for intrusion
		double distance_to_person{std::sqrt(std::pow(t_predicted_person_position.x - m_current_robot_position.x, 2.0) + std::pow(t_predicted_person_position.y - m_current_robot_position.y, 2.0))};

		double proxemic_boundary_angle{M_PI - std::abs(std::abs(t_person_theta - angle_person_robot) - M_PI)};

		double score_intrusion{(distance_to_person - 1.2) / (Utils::proxemicsFunction(a, proxemic_boundary_angle, m_proxemics_function_sigma, 1.2) - 1.2)};

		if (score_intrusion > 1.0)
			score_intrusion = 1.0;

		if (score_intrusion < 0.0)
			score_intrusion = 0.0;

		score_intrusion = 1.0 - score_intrusion;

		if (m_debug)
			std::cout << "intrusion: distance to person: " << distance_to_person << ", max boundary: " << Utils::proxemicsFunction(a, proxemic_boundary_angle, m_proxemics_function_sigma, 1.2) << std::endl;

		// check if robot is facing away: angle_robot_intersection_point != heading
		double angle_robot_intersection_point{Utils::localAngle(m_current_robot_position, t_predicted_person_position)};

		bool robot_is_facing_away{Utils::angularDifferenceUnsigned(m_current_robot_heading, angle_robot_intersection_point) > 1.0};

		double angle_person_intersection_point{Utils::localAngle(t_predicted_person_position, intersection_point)};

		bool intersection_point_in_proxemic_area{(Utils::euclideanDistance(t_predicted_person_position, intersection_point) < Utils::proxemicsFunction(a, angle_person_intersection_point, m_proxemics_function_sigma, 1.2)) && Utils::angularDifferenceUnsigned(Utils::localAngle(m_current_robot_position, intersection_point), m_current_robot_heading) < 1.0};

		robot_is_facing_away = robot_is_facing_away && !intersection_point_in_proxemic_area;

		double score{0.0};
		if (robot_is_facing_away)
			score = score_intrusion;
		else {
			score = m_evasion_inner_score_intersection_weight * score_intersection + m_evasion_inner_score_intrusion_weight * score_intrusion;
		}

		if (m_debug)
			std::cout << "score intrusion: " << score_intrusion << ", score intersection: " << score_intersection << ", overall score: " << score << std::endl;

		double evasion_offset_angle{score * (M_PI / 2.0)};

		double sign{(t_person_theta - angle_person_robot >= 0.0 && t_person_theta - angle_person_robot <= M_PI) || (t_person_theta - angle_person_robot <= -M_PI && t_person_theta - angle_person_robot >= -(2.0 * M_PI)) ? 1.0 : -1.0};

		evasion_offset_angle *= sign;

		if (robot_has_same_direction)
			evasion_offset_angle *= -1.0;

		double evasion_angle{Utils::normalizeAngle(evasion_offset_angle + m_current_robot_heading)};

		bool robot_is_behind_person{Utils::angularDifferenceUnsigned(Utils::localAngle(t_predicted_person_position, m_current_robot_position), t_person_theta) > M_PI_2};

		// robot is behind person
		if (!robot_has_same_direction && robot_is_facing_away && robot_is_behind_person)
			evasion_angle = m_current_robot_heading;

		double person_inverse_heading{Utils::normalizeAngle(t_person_theta - M_PI)};

		if (robot_is_facing_away && (Utils::angularDifferenceUnsigned(m_current_robot_heading, t_person_theta) > M_PI_4 &&
			Utils::angularDifferenceUnsigned(m_current_robot_heading, person_inverse_heading) > M_PI_4))
        	evasion_angle = m_current_robot_heading;

		if (m_debug)
			std::cout << "evasion offset angle: " << evasion_offset_angle * 180 / M_PI << std::endl;

		// find closest point to offset angle + robot heading
		std::pair<Point3f, double> closest_point{std::pair<Point3f, double>{Point3f{}, MAXFLOAT}};
		
		bool found{false};
		for (const auto& p : cartesian_points) {

			double angle_robot_proxemic_point{Utils::localAngle(m_current_robot_position, p)};
			double angular_difference{Utils::angularDifferenceUnsigned(evasion_angle, angle_robot_proxemic_point)};

			if (angular_difference < closest_point.second) {
				found = true;
				closest_point = std::pair<Point3f, double>{p, angular_difference};
			}
		}

		if (m_debug)
			std::cout << "the new waypoint is: " << closest_point.first << std::endl;

		// extrapolate waypoint if no proxemic point is found
		if (!found) {

			closest_point.first = Point3f{m_current_robot_position.x + std::cos(evasion_angle), m_current_robot_position.y + std::sin(evasion_angle), 0.0};

			if (m_debug) {
				std::cout << "current robot position: " << m_current_robot_position << ", current robot theta: " <<  m_current_robot_heading * 180 / M_PI << std::endl;
				std::cout << "closest point not found! extrapolating and using point " << closest_point.first << std::endl;
			}
		}

		if (m_waypoints.size() > 1)
			m_waypoints.pop_front();

		m_waypoints.emplace_front(Point3f{closest_point.first.x, closest_point.first.y, 0.0});
		m_evasion_waypoint = closest_point.first;
		m_current_evasion_waypoint = closest_point.first;

		m_number_of_waypoints++;

	// not in proxemics area, evade preemtively	
	} else {

		// compute angles relative to the robot's position
		std::vector<double> relative_angles{};
		for (const auto& p : cartesian_points) {
			Point3f relative_point{p.x - m_current_robot_position.x, p.y - m_current_robot_position.y, 0.0};

			relative_angles.emplace_back(atan2(relative_point.y, relative_point.x));
		}

		double min_angle_difference{MAXFLOAT};
		double max_angle_difference{0.0};

		Point3f min_difference_point{};
		Point3f max_difference_point{};

		for (int i = 0; i < cartesian_points.size() - 1; ++i) {
			
			// compute angle robot to point
			double angle_robot_point{Utils::localAngle(m_current_robot_position, cartesian_points.at(i))};

			// compute angle point to neighbour
			double angle_point_neighbor{Utils::localAngle(cartesian_points.at(i + 1), cartesian_points.at(i))};

			// compute difference
			double angle_difference{Utils::angularDifferenceUnsigned(angle_point_neighbor, angle_robot_point)};

			if (angle_difference < min_angle_difference) {
				min_angle_difference = angle_difference;
				min_difference_point = cartesian_points.at(i);
			}

			if (angle_difference > max_angle_difference) {
				max_angle_difference = angle_difference;
				max_difference_point = cartesian_points.at(i);
			}
		}

		// select correct waypoint
		Point3f evasion_waypoint_1{min_difference_point};
		Point3f evasion_waypoint_2{max_difference_point};

		// local angle to waypoints
		double angle_to_waypoint_1{Utils::localAngle(m_current_robot_position, evasion_waypoint_1)};

		double angle_to_waypoint_2{Utils::localAngle(m_current_robot_position, evasion_waypoint_2)};

		double relative_angle_to_waypoint_1{Utils::angularDifferenceUnsigned(angle_to_waypoint_1, t_robot_theta)};
		double relative_angle_to_waypoint_2{Utils::angularDifferenceUnsigned(angle_to_waypoint_2, t_robot_theta)};

		if (m_debug) {
			std::cout << "angle to waypoint 1: " << angle_to_waypoint_1 * 180 / M_PI << std::endl;
			std::cout << "angle to waypoint 2: " << angle_to_waypoint_2 * 180 / M_PI << std::endl;
			std::cout << "relative angle to waypoint 1: " << relative_angle_to_waypoint_1 * 180 / M_PI << std::endl;
			std::cout << "relative angle to waypoint 2: " << relative_angle_to_waypoint_2 * 180 / M_PI << std::endl;
			std::cout << "relative angle to waypoint 2: " << relative_angle_to_waypoint_2 * 180 / M_PI << std::endl;	
			std::cout << "relative angle to waypoint 2: " << relative_angle_to_waypoint_2 * 180 / M_PI << std::endl;
		}

		// new approach
		double angle_person_wp1{std::atan2(evasion_waypoint_1.y - t_predicted_person_position.y, evasion_waypoint_1.x - t_predicted_person_position.x)};
		double angle_person_wp2{std::atan2(evasion_waypoint_2.y - t_predicted_person_position.y, evasion_waypoint_2.x - t_predicted_person_position.x)};
		double angle_person_robot{std::atan2(m_current_robot_position.y - t_predicted_person_position.y, m_current_robot_position.x - t_predicted_person_position.x)};

		if (angle_person_wp1 < 0.0)
			angle_person_wp1 += 2 * M_PI;

		if (angle_person_wp2 < 0.0)
			angle_person_wp2 += 2 * M_PI;

		if (angle_person_robot < 0.0)
			angle_person_robot += 2 * M_PI;

		double mean_robot_wp1_angle{Utils::angularDifferenceUnsigned(Utils::circularMean({angle_person_robot, angle_person_wp1}), t_person_theta)};
		double mean_robot_wp2_angle{Utils::angularDifferenceUnsigned(Utils::circularMean({angle_person_robot, angle_person_wp2}), t_person_theta)};

		double wayness_score_wp1{std::abs(mean_robot_wp1_angle / (M_PI / 2))};
		double wayness_score_wp2{std::abs(mean_robot_wp2_angle / (M_PI / 2))};

		if (wayness_score_wp1 > 1.0)
			wayness_score_wp1 = 1.0;

		if (wayness_score_wp2 > 1.0)
			wayness_score_wp2 = 1.0;

		// angle scores
		double angle_score_wp1{1.0 - std::abs(relative_angle_to_waypoint_1 / (M_PI / 2))};
		if (angle_score_wp1 < 0.0)
			angle_score_wp1 = 0.0;

		double angle_score_wp2{1.0 - std::abs(relative_angle_to_waypoint_2 / (M_PI / 2))};
		if (angle_score_wp2 < 0.0)
			angle_score_wp2 = 0.0;

		if (m_debug) {
			std::cout << "angle score wp1: " << angle_score_wp1;
			std::cout << " angle score wp2: " << angle_score_wp2;
			std::cout << " wayness wp1: " << wayness_score_wp1;
			std::cout << " wayness wp2: " << wayness_score_wp2 << std::endl;
		}
		
		double relative_velocity_person{t_person_velocity / m_current_speed};
		if (relative_velocity_person > 1.0)
			relative_velocity_person = 1.0;

		double weight_angle{1.0 - relative_velocity_person};
		double weight_wayness{relative_velocity_person};

		if (m_debug) {
			std::cout << "weight angle: " << weight_angle << std::endl;
			std::cout << "weight wayness: " << weight_wayness << std::endl;
		}

		double score_wp1{(weight_angle * angle_score_wp1 + weight_wayness * wayness_score_wp1) / (weight_angle + weight_wayness)};
		double score_wp2{(weight_angle * angle_score_wp2 + weight_wayness * wayness_score_wp2) / (weight_angle + weight_wayness)};

		if (m_debug)
			std::cout << "score wp1: " << score_wp1 << ", score wp2: " << score_wp2 << std::endl;

		if (score_wp1 < score_wp2) {

			if (m_debug)
				std::cout << "I've chosen waypoint " << evasion_waypoint_2 << std::endl;

			
			if (m_waypoints.size() > 1)
				m_waypoints.pop_front();

			m_waypoints.emplace_front(Point3f{evasion_waypoint_2.x, evasion_waypoint_2.y, 0.0});
			m_evasion_waypoint = evasion_waypoint_2;
			m_current_evasion_waypoint = evasion_waypoint_2;
			m_current_other_waypoint = evasion_waypoint_1;

			if (m_debug)
				std::cout << "set new waypoint: " << evasion_waypoint_1 << std::endl;

			m_number_of_waypoints++;
		}
		else {

			if (m_debug)
				std::cout << "I've chosen waypoint " << evasion_waypoint_1 << std::endl;

			if (m_waypoints.size() > 1)
				m_waypoints.pop_front();

			m_waypoints.emplace_front(Point3f{evasion_waypoint_1.x, evasion_waypoint_1.y, 0.0});
			m_evasion_waypoint = evasion_waypoint_1;
			m_current_evasion_waypoint = evasion_waypoint_1;
			m_current_other_waypoint = evasion_waypoint_2;

			if (m_debug)
				std::cout << "set new waypoint: " << evasion_waypoint_1 << std::endl;

			m_number_of_waypoints++;
		}
	}

	// DEBUG
	std::cout << "waypoints: ";
	for (const auto& w : m_waypoints)
		std::cout << w << " ";
	std::cout << std::endl;

	m_current_pedestrian_heading = t_person_theta;
}


bool RobotNavigationAlgorithm::verifyEvasionWaypoint(int t_id, Point3f t_evasion_waypoint)
{
	bool is_in_personal_space{false};
	for (const auto& i : m_pedestrian_positions_at_evasion) {
		if (i.id != t_id && isInPersonalSpace(t_evasion_waypoint, i.position, i.heading, i.speed, false))
			is_in_personal_space = true;
	}
	return !is_in_personal_space;
}
