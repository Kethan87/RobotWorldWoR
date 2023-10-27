/*
 * KalmanFilter.hpp
 *
 *  Created on: Oct 27, 2023
 *      Author: kethan
 */

#ifndef SRC_KALMANFILTER_HPP_
#define SRC_KALMANFILTER_HPP_
#include "Matrix.hpp"
#include <iostream>

class KalmanFilter {
public:
	KalmanFilter(double aProcessErrorX, double aProcessErrorY,  double anObservationErrorAngle, double anObservationErrorDistance, const uint8_t aDelta);

	template< typename T, const std::size_t M, const std::size_t N>
	Matrix< T, M, N > predictedStateVector(double deltaT, T stateVector, T procesError, double scalar);

	template< typename T, const std::size_t M, const std::size_t N>
	Matrix< T, M, N > predictedProcesError(double deltaT, double processorErrorX, double processorErrorY);

	template< typename T, const std::size_t M, const std::size_t N>
	Matrix< T, M, N > KalManGain(Matrix< T, M, N> predictedProcesError, double observationErrorX, double observationErrorY);

	template< typename T, const std::size_t M, const std::size_t N>
	Matrix< T, M, N > measureMent(double stateVectorFuture, double procesErrorFuture);

	template< typename T, const std::size_t M, const std::size_t N>
	Matrix< T, M, N > adjustedStateVector(Matrix< T, M, N > predictedStateVector, Matrix< T, 2, 2 > KalManGain, Matrix< T, M, N > measureMent);

	template< typename T, const std::size_t M, const std::size_t N>
	Matrix< T, M, N > adjustedProcesError(Matrix< T, M, N > KalManGain, Matrix< T, M, N > predictedProcesError);


	std::pair<double, double> kalManFilter(uint16_t oldXValue, uint16_t oldaYValue, uint16_t newXValue, uint16_t newYValue, double scalar);
	virtual ~KalmanFilter();

private:
	double processErrorX;
	double processErrorY;
	double observationErrorAngle;
	double observationErrorDistance;
	const uint8_t delta;
	bool firstTimeKalman;
};

#endif /* SRC_KALMANFILTER_HPP_ */
