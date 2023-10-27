#ifndef SRC_PARTICLEFILTER_HPP_
#define SRC_PARTICLEFILTER_HPP_


#include "Particle.hpp"
#include "DistancePercepts.hpp"
#include "Shape2DUtils.hpp"
namespace Model
{
	class ParticleFilter {
	public:
		ParticleFilter(const std::vector<Particle>& aParticlesForFilter, const uint16_t A_NUMBER_OF_PARTICLES);
		std::vector<Particle> initializeParticleFilter();
		std::vector<Particle> controlUpdate(double delta_x, double delta_y, double controlErrorX, double controlErrorY);
		std::vector<Particle> measurementUpdate(PointCloud lidarMeasurements);
		std::vector<Particle> resampling();
		double generateRandomError(double standardDeviation);
		double calculateTotalWeight(std::vector<Particle> particlesForFilter);
		std::vector<Particle>& operator=(const std::vector<Particle>& particles);
		std::vector<Particle> getParticles();
		void setParticles(const std::vector<Particle>& newParticles);
		void getTotalWeight();
		virtual ~ParticleFilter();
	private:
		std::vector<Particle> particlesForFilter;
		const uint16_t NUMBER_OF_PARTICLES;
		double totalWeight;
	};
}

#endif /* SRC_PARTICLEFILTER_HPP_ */
