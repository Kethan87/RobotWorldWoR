/*
 * OrientationPercept.cpp
 *
 *  Created on: Oct 11, 2023
 *      Author: kethan
 */

#include "OrientationPercept.hpp"
namespace Model
{
	OrientationPercept::OrientationPercept(double aDistance,double aAngle, uint16_t aDeltaX, uint16_t aDeltaY)
			: distance(aDistance), angle(aAngle), deltaX(aDeltaX), deltaY(aDeltaY){

	}

}
