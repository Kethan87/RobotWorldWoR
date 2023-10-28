/*
 * ParticleFilter.cpp
 *
 *  Created on: Oct 23, 2023
 *      Author: kethan
 */

#include "ParticleFilter.hpp"

namespace Model
{
	ParticleFilter::ParticleFilter(const std::vector<Particle>& aParticlesForFilter, const uint16_t A_NUMBER_OF_PARTICLES) : particlesForFilter(aParticlesForFilter),  NUMBER_OF_PARTICLES(A_NUMBER_OF_PARTICLES), totalWeight(calculateTotalWeight(particlesForFilter)) {
		// TODO Auto-generated constructor stub

	}

	std::vector<Particle> ParticleFilter::initializeParticleFilter()
	{

		if(Model::CompassLidarSensor::particleFilter)
		{
			std::random_device rd;
			std::mt19937 gen(rd());
			std::uniform_real_distribution<> dis(0, 1024);
			for(int i = 0; i < NUMBER_OF_PARTICLES; ++i)
			{
				Particle particle(
						wxPoint(static_cast<int>(dis(gen)),
								static_cast<int>(dis(gen))), 1);
				particlesForFilter.push_back(particle);
			}
		} else
		{
			while(!particlesForFilter.empty())
			{
				particlesForFilter.pop_back();
			}
		}
		return particlesForFilter;
	}

	void ParticleFilter::controlUpdate(double deltaX, double deltaY, double controlErrorX, double controlErrorY) {
	    for (int i = 0; i < NUMBER_OF_PARTICLES; ++i) {

	        double noiseX = generateRandomError(controlErrorX);
	        double noiseY = generateRandomError(controlErrorY);
	        double predicted_x = particlesForFilter.at(i).getPosition().x + deltaX * 1 + noiseX;
	        double predicted_y = particlesForFilter.at(i).getPosition().y + deltaY * 1 + noiseY;


	        particlesForFilter.at(i).setPosition(wxPoint(static_cast<int>(predicted_x), static_cast<int>(predicted_y)));


	        particlesForFilter.at(i).setLidarMeasurements(particlesForFilter.at(i).measurementLidar(wxPoint(static_cast<int>(predicted_x), static_cast<int>(predicted_y))));
	    }
	}

	void ParticleFilter::measurementUpdate(PointCloud lidarMeasurements) {
		for(int i = 0; i < NUMBER_OF_PARTICLES; ++i)
		{
			double totalDifferenceMeasurement = 0.0;
			for(unsigned long j = 0; j < particlesForFilter.at(i).getLidarMeasurements().size(); ++j)
			{
				double distancePointMeasure = 0.0;
				distancePointMeasure = Utils::Shape2DUtils::distance(particlesForFilter.at(i).getLidarMeasurements().at(j).point, lidarMeasurements.at(j).point);

				totalDifferenceMeasurement += distancePointMeasure;
			}
			double probability = 1.0 / (1.0 + totalDifferenceMeasurement);

			double particleWeight = particlesForFilter.at(i).getWeight();
			particleWeight *= probability;
			particlesForFilter.at(i).setWeight(particleWeight);
		}
	}


	std::vector<Particle> ParticleFilter::resampling() {
	    std::random_device rd{};
	    std::mt19937 gen{rd()};
	    std::vector<Particle> resampleParticles;
	    double totalWeight = calculateTotalWeight(particlesForFilter);

	    std::vector<double> weights;
	    for (unsigned long i = 0; i < particlesForFilter.size(); ++i) {
	        weights.push_back(particlesForFilter.at(i).getWeight() / totalWeight);
	    }

	    std::discrete_distribution<int> dist(weights.begin(), weights.end());

	    for (int i = 0; i < NUMBER_OF_PARTICLES; ++i) {
	        int index = dist(gen);
	        resampleParticles.push_back(particlesForFilter.at(index));
	    }

	    for (Particle& particle : resampleParticles) {
	        particle.setWeight(1.0);
	    }

	    return resampleParticles;
	}


	double ParticleFilter::calculateTotalWeight(std::vector<Particle> particlesForFilter)
	{
		double totalWeight = 0.0;
		for(unsigned long i = 0; i < particlesForFilter.size(); ++i)
		{
			totalWeight += particlesForFilter.at(i).getWeight();
		}
		return totalWeight;
	}


	double ParticleFilter::generateRandomError(double standardDeviation) {
	    std::random_device rd;
	    std::mt19937 gen(rd());
	    std::normal_distribution<double> error(0, standardDeviation);
	    return error(gen);
	}

	std::vector<Particle> ParticleFilter::getParticles()
	{
		return particlesForFilter;
	}

	void ParticleFilter::setParticles(const std::vector<Particle>& newParticles)
	{
		particlesForFilter = newParticles;
	}



	ParticleFilter::~ParticleFilter() {
		// TODO Auto-generated destructor stub
	}
}

