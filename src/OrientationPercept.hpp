/*
 * OrientationPercept.h
 *
 *  Created on: Oct 11, 2023
 *      Author: kethan
 */
#ifndef SRC_ORIENTATIONPERCEPT_HPP_
#define SRC_ORIENTATIONPERCEPT_HPP_
#include "Config.hpp"

#include "DistanceStimulus.hpp"
#include "Point.hpp"

#include <limits>
namespace Model
{

	class OrientationPercept : public AbstractPercept
	{
		public:
			OrientationPercept(double aDistance,double aAngle, uint16_t aDeltaX, uint16_t aDeltaY);

			virtual std::string asString() const override
			{
				return "DistancePercept: distance " + std::to_string(distance) + ", angle: " + std::to_string(angle);
			}

			/**
			 * Returns a description of the object with all data of the object usable for debugging
			 */
			virtual std::string asDebugString() const override
			{
				return asString();
			}

		double angle;
		double distance;
		uint16_t deltaX;
		uint16_t deltaY;
		};
}

#endif /* SRC_ORIENTATIONPERCEPT_H_ */
