/*
 * KalmanFilter.cpp
 *
 *  Created on: Oct 27, 2023
 *      Author: kethan
 */

#include "KalmanFilter.hpp"

KalmanFilter::KalmanFilter(double aProcessErrorX, double aProcessErrorY, double anObservationErrorAngle, double anObservationErrorDistance, const uint8_t aDelta) : processErrorX(aProcessErrorX), processErrorY(aProcessErrorY), observationErrorAngle(anObservationErrorAngle), observationErrorDistance(anObservationErrorDistance), delta(aDelta), firstTimeKalman(true){
	// TODO Auto-generated constructor stub
}

KalmanFilter::~KalmanFilter() {
	// TODO Auto-generated destructor stub
}


template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N > KalmanFilter::predictedStateVector(double deltaT, T stateVectorVariable1, T stateVectorVariable2, double scalar)
{
	Matrix< T, M, N > result;
	Matrix< T, 2, 2 > A{{1,deltaT},{0,1}};
	Matrix< T, M, N > variables{{{stateVectorVariable1}},{{stateVectorVariable2}}};
	Matrix< T, 2, 1 > error{{{0.5 * std::pow(1, 2)}},{{1}}};
	error = error * scalar;
	result = ((A * variables) + error);
	return result;
}

template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N > KalmanFilter::predictedProcesError(double deltaT,double processorErrorX, double processorErrorY)
{
	Matrix< T, M, N > result;
	Matrix< T, M, N > identity{{1,deltaT},{0,1}};
	Matrix< T, M, N > identityTranspose = identity.transpose();
	if(firstTimeKalman)
	{
		double leftAbove = pow(processorErrorX,2);
		double rightUnder = pow(processorErrorY,2);
		result.at(0, 0) = leftAbove;
		result.at(0, 1) = 0;
		result.at(1, 0) = 0;
		result.at(1, 1) = rightUnder;
		firstTimeKalman = false;
	} else
	{
		result.at(0, 0) = processorErrorX;
		result.at(0, 1) = 0;
		result.at(1, 0) = 0;
		result.at(1, 1) = processorErrorY;
	}
	result = identity * result * identityTranspose;
	result.at(0, 1) = 0;
	result.at(1, 0) = 0;
	return result;
}


template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N > KalmanFilter::KalManGain(Matrix< T, M, N> predictedProcesError, double observationErrorX, double observationErrorY)
{
	Matrix< T, M, N > result;
	double leftAbove = pow(observationErrorX,2);
	double rightUnder = pow(observationErrorY,2);
	result.at(0, 0) = leftAbove;
	result.at(0, 1) = 0;
	result.at(1, 0) = 0;
	result.at(1, 1) = rightUnder;
	Matrix<double, 2, 2> identity{{1,0},{0,1}};
	identity = identity.transpose();
	result = result * identity;
	result = result + predictedProcesError;
	result.at(0,0) = predictedProcesError.at(0,0)/ result.at(0,0);
	result.at(1,1) = predictedProcesError.at(1,1) /result.at(1,1);

	return result;
}

template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N > KalmanFilter::measureMent(double stateVectorFuture, double procesErrorFuture)
{
	Matrix< T, M, N > result;
	Matrix< T, 2, 2 > identity{{1,0},{0,1}};
	Matrix< T, 2, 1 > variables{{{stateVectorFuture}},{{procesErrorFuture}}};
	result = identity * variables;
	return result;

}

template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N > KalmanFilter::adjustedStateVector(Matrix< T, M, N > predictedStateVector, Matrix< T, 2, 2 > KalManGain, Matrix< T, M, N > measureMent)
{
	Matrix< T, M, N > result;
	Matrix< T, M, N > check;
 	result = predictedStateVector + (KalManGain * (measureMent - predictedStateVector));
 	check = measureMent - predictedStateVector;
	return result;
}

template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N > KalmanFilter::adjustedProcesError(Matrix< T, M, N > KalManGain, Matrix< T, M, N > predictedProcesError)
{
	Matrix< T, M, N > result;
	Matrix< T, M, N > identity{{1,0},{0,1}};
	result = (identity - KalManGain) * predictedProcesError;
	return result;
}

std::pair<double, double> KalmanFilter::kalManFilter(uint16_t oldXValue, uint16_t oldaYValue, uint16_t newXValue, uint16_t newYValue, double scalar) // IS USED IN robot.cpp in at line 509
{
	Matrix<double, 2, 1> newPredictedStateVector = predictedStateVector<double, 2, 1>(delta, oldXValue, oldaYValue, scalar);
	Matrix<double, 2, 2> newPredictedProcesError = predictedProcesError<double, 2, 2>(delta, processErrorX, processErrorY);
	Matrix<double, 2, 2> kalManGain = KalManGain(newPredictedProcesError, observationErrorAngle, observationErrorDistance);
	Matrix<double, 2, 1> newMeasurement = measureMent<double, 2, 1>(newXValue, newYValue);
	Matrix<double, 2, 1> newAdjustedStateVector = adjustedStateVector(newPredictedStateVector, kalManGain, newMeasurement);
	Matrix<double, 2, 2> newAdjustedProcessError = adjustedProcesError(kalManGain, newPredictedProcesError);
	processErrorX = newAdjustedProcessError.at(0, 0);
	processErrorY = newAdjustedProcessError.at(1,1);
	std::pair<double, double> result(newAdjustedStateVector.at(0,0), newAdjustedStateVector.at(1,0));

	return result;

}


