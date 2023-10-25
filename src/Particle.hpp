#ifndef SRC_PARTICLE_HPP_
#define SRC_PARTICLE_HPP_

#include "Point.hpp"
#include "DistancePercepts.hpp"
#include "CompassLidarSensor.hpp"
#include "RobotWorld.hpp"
#include "Wall.hpp"
#include "Shape2DUtils.hpp"

#include <random>

namespace Model
{
//	static uint16_t totalParticleWeight = 0;
class Particle {
public:
	Particle(wxPoint aPosition, double aWeight);
	Particle();
	virtual ~Particle();
	PointCloud measurement(wxPoint position);
	Particle& operator=(const Particle& particle);

	wxPoint getPosition();
	void setPosition(wxPoint newPosition);
	PointCloud getLidarMeasurements();
	double getWeight();
	void setWeight(double newWeight);
	double getLidarStddev();


private:
	wxPoint position;
	double weight;
	double lastWeight;
	PointCloud lidarMeasurements;
	double lidarStddev;
	const int LASERBEAM_LENGTH;
};

}

#endif /* SRC_PARTICLE_HPP_ */
