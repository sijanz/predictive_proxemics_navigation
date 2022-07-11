#include "predictive_proxemics_navigation/pedestrian.h"
#include "predictive_proxemics_navigation/utils.h"

Pedestrian::Pedestrian(int t_id)
{
	m_id = t_id;
	m_desired_speed = 1.2;
}

Pedestrian::Pedestrian(int t_id, const Point3f& t_position)
{
	m_id = t_id;
	m_desired_speed = 1.2;
    m_position = t_position;
}
