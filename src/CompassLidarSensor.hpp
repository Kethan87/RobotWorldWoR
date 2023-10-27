/*
 * CompassLidarSensor.hpp
 *
 *  Created on: Oct 17, 2023
 *      Author: kethan
 */

#ifndef SRC_COMPASSLIDARSENSOR_HPP_
#define SRC_COMPASSLIDARSENSOR_HPP_

#include "Config.hpp"

#include "AbstractSensor.hpp"
#include "DistancePercept.hpp"
#include "OrientationPercept.hpp"

#include <vector>

namespace Model
{
	class Robot;
	typedef std::shared_ptr<Robot> RobotPtr;
	class CompassLidarSensor : public AbstractSensor
	{
	public:
		static wxPoint lastPosition;
		static bool particleFilter;
		static double lidarStddev;
		explicit CompassLidarSensor(Robot& aRobot);


		virtual std::shared_ptr< AbstractStimulus > getStimulus() const override;

		virtual std::shared_ptr< AbstractPercept > getPerceptFor( std::shared_ptr< AbstractStimulus > anAbstractStimulus) const override;
	private:
		const int FIRSTRUN_POSITION = 1025;
		const int LASERBEAM_LENGTH = 1024;
		const int LASERBEAMS = 180;
		std::vector<wxPoint> lidarMeasures;
	};
}

#endif /* SRC_COMPASSLIDARSENSOR_HPP_ */
