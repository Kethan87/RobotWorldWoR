#include "Particle.hpp"

namespace Model
{
	Particle::Particle(wxPoint aPosition, double aWeight): position(aPosition), weight(aWeight), lidarMeasurements(measurement(aPosition)), lidarStddev(10), LASERBEAM_LENGTH(1024), lastWeight(0) {
		// TODO Auto-generated constructor stub
	}

	Particle::Particle() : position(wxPoint(0,0)), weight(0.0), lidarMeasurements(measurement(position)), lidarStddev(10), LASERBEAM_LENGTH(1024), lastWeight(0)
	{

	}

	Particle::~Particle() {
		// TODO Auto-generated destructor stub
	}

	PointCloud Particle::measurement(wxPoint position)
	{
		std::random_device rd{};
		std::mt19937 gen{rd()};
		std::normal_distribution<> noiseLidar{CompassLidarSensor::lidarStddev * -1,CompassLidarSensor::lidarStddev};
		double angle = 0;
		std::vector<WallPtr> walls = RobotWorld::getRobotWorld().getWalls();
		PointCloud measurements;
		for (int i = 0; i < 180; i++)
		{
			double shortestDistanceIntersection = LASERBEAM_LENGTH;
			bool objectFound = false;
			double currentAngle = Utils::MathUtils::toRadians(angle + (i * 2));
			for (std::shared_ptr<Wall> wall : walls) {
				wxPoint wallPoint1 = wall->getPoint1();
				wxPoint wallPoint2 = wall->getPoint2();
				std::vector<std::pair<double, double>> anglesAndDistances;
				wxPoint laserEndpoint { static_cast<int>(position.x+ std::cos(currentAngle) * LASERBEAM_LENGTH + noiseLidar(gen)),
					static_cast<int>(position.y + std::sin(currentAngle) * LASERBEAM_LENGTH + noiseLidar(gen)) };
				wxPoint interSection = Utils::Shape2DUtils::getIntersection(wallPoint1, wallPoint2, position, laserEndpoint);
				if (interSection != wxDefaultPosition) {
					double distance = Utils::Shape2DUtils::distance(position,interSection);
					if (distance < shortestDistanceIntersection) {
						objectFound = true;
						shortestDistanceIntersection = distance;
					}
				}
			}
			if (!objectFound) {
				measurements.push_back(DistancePercept(wxPoint(noObject, noObject)));
			} else {
				wxPoint endPoint{static_cast< int >( position.x + std::cos(currentAngle)*shortestDistanceIntersection),
												static_cast< int >( position.y + std::sin(currentAngle)*shortestDistanceIntersection)};
				measurements.push_back(DistancePercept(endPoint));
			}
		}
		return measurements;
	}


	Particle& Particle::operator =(const Particle& particle)
	{
		if(this != &particle)
		{
			position = particle.position;
			weight = particle.weight;
			lidarMeasurements = measurement(particle.position);
		}
		return *this;
	}

	wxPoint Particle::getPosition()
	{
		return position;
	}

	PointCloud Particle::getLidarMeasurements()
	{
		return lidarMeasurements;
	}

	double Particle::getWeight()
	{
		return weight;
	}

	double Particle::getLidarStddev()
	{
		return lidarStddev;
	}

	void Particle::setWeight(double newWeight)
	{
		weight = newWeight;
	}

	void Particle::setPosition(wxPoint newPosition)
	{
		position = newPosition;
	}
}