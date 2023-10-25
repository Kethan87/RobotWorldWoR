#ifndef SRC_ORIENTATIONSTIMULUS_HPP_
#define SRC_ORIENTATIONSTIMULUS_HPP_

#include "DistanceStimulus.hpp"

namespace Model
{

	const uint8_t INVALID_DELTA_X = -3;
	const uint8_t INVALID_DELTA_Y = -3;
	class OrientationStimulus : public AbstractStimulus{
	public:
		OrientationStimulus(double aDistance, double aAngle, uint16_t aDeltaX, uint16_t aDeltaY);


		virtual std::string asString() const override
		{
			return "DistancePercept: " + std::to_string(angle) + ", " + std::to_string(distance);
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

#endif /* SRC_ORIENTATIONSTIMULUS_HPP_ */
