/*
 * CompassSensor.h
 *
 *  Created on: Oct 4, 2023
 *      Author: kethan
 */

#ifndef SRC_COMPASSODOMETERSENSOR_HPP_
#define SRC_COMPASSODOMETERSENSOR_HPP_
#include "Config.hpp"

#include "AbstractSensor.hpp"
#include "DistancePercept.hpp"
#include "OrientationPercept.hpp"

#include <vector>

namespace Model
{
	class Robot;
	typedef std::shared_ptr<Robot> RobotPtr;
	extern double odometerSttdev;
	extern double compasStddev;


	class CompassOdometerSensor : public AbstractSensor
	{
	public:
		explicit CompassOdometerSensor(Robot& aRobot);

		static bool kalmanFilter;
		static wxPoint lastPosition;
		/**
		 *
		 */
		virtual std::shared_ptr< AbstractStimulus > getStimulus() const override;

		virtual std::shared_ptr< AbstractPercept > getPerceptFor( std::shared_ptr< AbstractStimulus > anAbstractStimulus) const override;

	private:
		const int FIRSTRUN_POSITION = 1025;

	};
}

#endif /* SRC_COMPASSSENSOR_HPP_ */
