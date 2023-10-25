/*
 * ParticleFilter.cpp
 *
 *  Created on: Oct 23, 2023
 *      Author: kethan
 */

#include "ParticleFilter.hpp"

namespace Model
{
	ParticleFilter::ParticleFilter(std::vector<Particle> aParticlesForFilter, const uint16_t A_NUMBER_OF_PARTICLES) : belief(0.0), totalWeight(calculateTotalWeight(particlesForFilter)), particlesForFilter(aParticlesForFilter),  NUMBER_OF_PARTICLES(A_NUMBER_OF_PARTICLES) {
		// TODO Auto-generated constructor stub

	}

	std::vector<Particle> ParticleFilter::initializeParticleFilter()
	{

		if(Model::CompassLidarSensor::particleFilter)
		{
			std::random_device rd;
			std::mt19937 gen(rd());
			std::uniform_real_distribution<> dis(0, 1024);
			particlesForFilter.resize(NUMBER_OF_PARTICLES);
			for(int i = 0; i < NUMBER_OF_PARTICLES; ++i)
			{
				Particle particle(wxPoint(static_cast<int>(dis(gen)), static_cast<int>(dis(gen))), 1);
				particlesForFilter.at(i) = particle;
				for(int j = 0; j < particle.getLidarMeasurements().size(); ++j)
				{
					std::cout << "Number Particle: " << i << " " <<  "X" << j << ": " << particle.getLidarMeasurements().at(j).point.x << std::endl;
					std::cout << "Number Particle: " << i << " " << "Y" << j << ": " << particle.getLidarMeasurements().at(j).point.y << std::endl;
				}
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

	void ParticleFilter::controlUpdate(PointCloud lidarMeasurements)
	{
	}

	void ParticleFilter::measurementUpdate(PointCloud lidarMeasurements) {
		for(int i = 0; i < NUMBER_OF_PARTICLES; ++i)
		{
			double totalDifferenceMeasurement = 0.0;
			for(int j = 0; j < particlesForFilter.at(i).getLidarMeasurements().size(); ++j)
			{
				double distancePointMeasure = Utils::Shape2DUtils::distance(particlesForFilter.at(i).getLidarMeasurements().at(j).point, lidarMeasurements.at(j).point);
				totalDifferenceMeasurement += fabs(distancePointMeasure);
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
	    for (int i = 0; i < static_cast<int>(particlesForFilter.size()); ++i) {
	        weights.push_back(particlesForFilter.at(i).getWeight() / totalWeight);
	    }

	    std::discrete_distribution<int> dist(weights.begin(), weights.end());

	    for (int i = 0; i < NUMBER_OF_PARTICLES; ++i) {
	        int index = dist(gen);
	        resampleParticles.push_back(particlesForFilter.at(index));
	    }
	    return resampleParticles;
	}

	double ParticleFilter::calculateTotalWeight(std::vector<Particle> particlesForFilter)
	{
		double totalWeight = 0;
		for(int i = 0; i < static_cast<int>(particlesForFilter.size()); ++i)
		{
			totalWeight += particlesForFilter.at(i).getWeight();
		}
		return totalWeight;
	}




	ParticleFilter::~ParticleFilter() {
		// TODO Auto-generated destructor stub
	}
}

