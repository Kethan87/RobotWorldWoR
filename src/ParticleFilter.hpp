#ifndef SRC_PARTICLEFILTER_HPP_
#define SRC_PARTICLEFILTER_HPP_


#include "Particle.hpp"
#include "DistancePercepts.hpp"
#include "Shape2DUtils.hpp"
namespace Model
{
	class ParticleFilter {
	public:
		ParticleFilter(std::vector<Particle> aParticlesForFilter, const uint16_t A_NUMBER_OF_PARTICLES);
		std::vector<Particle> initializeParticleFilter();
		void controlUpdate(PointCloud lidarMeasurements);
		void measurementUpdate(PointCloud lidarMeasurements);
		std::vector<Particle> resampling();
		double calculateTotalWeight(std::vector<Particle> particlesForFilter);
		void getTotalWeight();
		virtual ~ParticleFilter();
	private:
		double belief;
		std::vector<Particle> particlesForFilter;
		const uint16_t NUMBER_OF_PARTICLES;
		double totalWeight;
	};
}

#endif /* SRC_PARTICLEFILTER_HPP_ */
