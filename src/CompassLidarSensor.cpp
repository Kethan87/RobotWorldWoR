/*
 * CompassLidarSensor.cpp
 *
 *  Created on: Oct 17, 2023
 *      Author: kethan
 */

#include "CompassLidarSensor.hpp"
#include "Logger.hpp"
#include "Robot.hpp"
#include "RobotWorld.hpp"
#include "Wall.hpp"
#include "Shape2DUtils.hpp"
#include "MathUtils.hpp"
#include "RobotShape.hpp"
#include "MathUtils.hpp"
#include "DistanceStimulus.hpp"

#include <random>



namespace Model
{

	bool CompassLidarSensor::particleFilter = true;
	wxPoint CompassLidarSensor::lastPosition(1025,1025);
	double lidarStddev = 10;

	CompassLidarSensor::CompassLidarSensor(Robot& aRobot) : AbstractSensor(aRobot) {
		// TODO Auto-generated constructor stub

	}

	std::shared_ptr< AbstractStimulus > CompassLidarSensor::getStimulus() const
	{
		Robot* robot = dynamic_cast<Robot*>(agent);
		if(robot)
		{
			std::random_device rd{};
			std::mt19937 gen{rd()};
			std::normal_distribution<> noiseLidar{0,Model::lidarStddev};
			double angle = 0;
			std::vector<WallPtr> walls = RobotWorld::getRobotWorld().getWalls();
			Stimuli stimulus;
			wxPoint robotLocation = robot->getPosition();
			for (int i = 0; i < LASERBEAMS; i++)
			{
				double shortestDistanceIntersection = LASERBEAM_LENGTH;
				bool objectFound = false;
				double currentAngle = Utils::MathUtils::toRadians(angle + (i * 2));
				for(std::shared_ptr<Wall> wall : walls)
				{
				wxPoint wallPoint1 = wall->getPoint1();
				wxPoint wallPoint2 = wall->getPoint2();
				std::vector<std::pair<double, double>> anglesAndDistances;
				    wxPoint laserEndpoint{static_cast<int>(robotLocation.x + std::cos(currentAngle) * LASERBEAM_LENGTH) ,
				    					static_cast<int>(robotLocation.y + std::sin(currentAngle) * LASERBEAM_LENGTH )};
				    wxPoint interSection = Utils::Shape2DUtils::getIntersection(wallPoint1,wallPoint2,robotLocation,laserEndpoint);
				    if(interSection != wxDefaultPosition)
				    {
						double distance = Utils::Shape2DUtils::distance(robotLocation,interSection);
						if(distance < shortestDistanceIntersection)
						{
							objectFound = true;
							shortestDistanceIntersection = distance;
						}
				    }
				}
				if(!objectFound)
				{
					stimulus.push_back(DistanceStimulus(noAngle, noDistance));
				} else
				{
					stimulus.push_back(DistanceStimulus(currentAngle, shortestDistanceIntersection + noiseLidar(gen)));
				}
			}
			return std::make_shared<DistanceStimuli>(stimulus);

		}

		return std::make_shared< DistanceStimulus >( noAngle,noDistance);
	}

	std::shared_ptr< AbstractPercept > CompassLidarSensor::getPerceptFor( std::shared_ptr< AbstractStimulus > anAbstractStimulus) const
	{
		Robot* robot = dynamic_cast< Robot* >( agent);
		if (robot)
		{
			wxPoint robotLocation = robot->getPosition();

			DistanceStimuli* distanceStimuli = dynamic_cast< DistanceStimuli* >( anAbstractStimulus.get());
			if(distanceStimuli)
			{
				std::vector<DistancePercept> endPoints;
				for(int i = 0; i < distanceStimuli->stimuli.size(); ++i)
				{
					if(distanceStimuli->stimuli.at(i).distance == noDistance)
					{
						endPoints.push_back(DistancePercept(wxPoint(noObject, noObject)));
					} else
					{
						wxPoint endPoint{static_cast< int >(std::cos(distanceStimuli->stimuli.at(i).angle)*distanceStimuli->stimuli.at(i).distance),
								static_cast< int >(std::sin(distanceStimuli->stimuli.at(i).angle)*distanceStimuli->stimuli.at(i).distance)};
						endPoints.push_back(DistancePercept(endPoint));
					}
				}
				return std::make_shared<DistancePercepts>(endPoints);
			}
		}
		return std::make_shared<OrientationPercept>( noAngle,invalidDistance, 0, 0);
	}

}

