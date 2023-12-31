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
class Particle {
public:
	Particle(wxPoint aPosition, double aWeight);
	virtual ~Particle();
	/**
	 *
	 * @param position position of the particle
	 * @return Pointcloud within there the measurements of the lidar from the particle position
	 */
	PointCloud measurementLidar(wxPoint position);
	/**
	 *
	 * @param particle
	 * @return a particle that has been assigned to the given particle
	 */
	Particle& operator=(const Particle& particle);

	wxPoint getPosition();
	void setPosition(wxPoint newPosition);
	PointCloud getLidarMeasurements();
	void setLidarMeasurements(const PointCloud& newLidarMeasurements);
	double getWeight();
	void setWeight(double newWeight);
	double getLidarStddev();


private:
	wxPoint position;
	double weight;
	PointCloud lidarMeasurements;
	double lidarStddev;
	const int LASERBEAM_LENGTH;
	const int LASERBEAMS = 180;
};

}

#endif /* SRC_PARTICLE_HPP_ */
