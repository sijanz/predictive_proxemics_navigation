#include "../include/agent.hpp"


Agent::Agent(float t_speed, const std::deque<Point3f> t_waypoints) : m_desired_speed{t_speed}, m_path{t_waypoints}
{
	m_position = m_path.at(0);
	m_path.pop_front();
}


Point3f Agent::path()
{
    Vector3f distanceCurr, distanceNext;

	distanceCurr = m_path[0] - m_position;			// distance to current waypoint

	if (m_path.size() > 2) {
		distanceNext = m_path[1] - m_position;		// distance to next waypoint

		// set next waypoint as current waypoint if next waypoint is nearer
		if (distanceNext.lengthSquared() < distanceCurr.lengthSquared()) {
			m_path.push_back(m_path.front());
			m_path.pop_front();

			distanceCurr = distanceNext;
		}
	}

	// move front point to back if within radius
	if (distanceCurr.lengthSquared() < (0.5 * 0.5)) {
		m_path.push_back(m_path.front());
		m_path.pop_front();
	}

	return m_path.front();
}
