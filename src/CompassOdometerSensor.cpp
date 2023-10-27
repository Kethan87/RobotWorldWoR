/*
 * CompassSensor.cpp
 *
 *  Created on: Oct 4, 2023
 *      Author: kethan
 */

#include "CompassOdometerSensor.hpp"

#include "Logger.hpp"
#include "Robot.hpp"
#include "RobotWorld.hpp"
#include "Wall.hpp"
#include "Shape2DUtils.hpp"
#include "MathUtils.hpp"
#include "OrientationStimulus.hpp"

#include <iostream>
#include <random>
namespace Model
{

	wxPoint CompassOdometerSensor::lastPosition(1025, 1025);
	bool CompassOdometerSensor::kalmanFilter = true;
	double odometerSttdev = 1;
	double compasStddev = 2;


	CompassOdometerSensor::CompassOdometerSensor(Robot& aRobot) : AbstractSensor(aRobot)
	{
		// TODO Auto-generated constructor stub
	}

	std::shared_ptr< AbstractStimulus > CompassOdometerSensor::getStimulus() const
	{
		Robot* robot = dynamic_cast<Robot*>(agent);
		if(robot)
		{
			std::random_device rd{};
			std::mt19937 gen{rd()};
		    std::normal_distribution<> noiseOdometer{0,Model::odometerSttdev};
		    std::normal_distribution<> noiseCompass{0,Model::compasStddev};
		    wxPoint robotLocation = robot->getPosition();
		    double randomAngleDeviation = Utils::MathUtils::toRadians(noiseCompass(gen));
		    double angle = Utils::Shape2DUtils::getAngle(robot->getFront()) + randomAngleDeviation;
		    wxPoint distancePoint{static_cast<int>(robotLocation.x + std::cos(angle)),
				static_cast<int>(robotLocation.y + std::sin(angle))};
		    double distance = Utils::Shape2DUtils::distance(lastPosition, distancePoint) + noiseOdometer(gen);
		    uint16_t deltaXPosition = std::abs(lastPosition.x - distancePoint.x);
		    uint16_t deltaYPosition = std::abs(lastPosition.y - distancePoint.y);
//		    std::cout << "DELTA X POSITION STIMULUS: " << deltaXPosition << std::endl;
//		    std::cout << "DELTA Y POSITION STIMULUS: " << deltaYPosition << std::endl;
		    if(lastPosition.x == FIRSTRUN_POSITION && lastPosition.y == FIRSTRUN_POSITION)
		    {
		    	distance = 0;
		    	deltaXPosition = 0;
		    	deltaYPosition = 0;
		    }
//		    lastPosition = robot->getPosition();
		    lastPosition = distancePoint;
//		    std::cout << "STIMULUS ANGLE: " << angle << std::endl;
		    return std::make_shared< OrientationStimulus >(angle, distance, deltaXPosition, deltaYPosition);
		}

		return std::make_shared< OrientationStimulus >( noAngle,noDistance, INVALID_DELTA_X, INVALID_DELTA_Y);
	}

	std::shared_ptr< AbstractPercept > CompassOdometerSensor::getPerceptFor( std::shared_ptr< AbstractStimulus > anAbstractStimulus) const
	{
		Robot* robot = dynamic_cast< Robot* >( agent);
		if (robot)
		{
			wxPoint robotLocation = robot->getPosition();
			OrientationStimulus* orientationStimulus = dynamic_cast< OrientationStimulus* >( anAbstractStimulus.get());
			if(orientationStimulus)
			{
				if(orientationStimulus->distance == noDistance)
				{
					return std::make_shared<OrientationPercept>( noObject,noObject, INVALID_DELTA_X, INVALID_DELTA_Y);
				}
				return std::make_shared<OrientationPercept>(orientationStimulus->distance, orientationStimulus->angle, orientationStimulus->deltaX, orientationStimulus->deltaY);
			}
		}
		return std::make_shared<OrientationPercept>( noAngle,invalidDistance, INVALID_DELTA_X, INVALID_DELTA_Y);
	}




}
