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

		/**
		 * Initialization of the particles
		 * @return a vector of particles random on the field
		 */
		std::vector<Particle> initializeParticleFilter();

		/**
		 *
		 * @param delta_x Movement of the Robot in X in delta (last en current position of robot)
		 * @param delta_y Movement of the Robot in Y in delta (last en current position of robot)
		 * @param controlErrorX Error for Movement X position
		 * @param controlErrorY Error for Movement Y position
		 * @return vector with Particles
		 */
		void controlUpdate(double delta_x, double delta_y, double controlErrorX, double controlErrorY);
		/**
		 *
		 * @param lidarMeasurements measurements of the particl with the lidar
		 */
		void measurementUpdate(PointCloud lidarMeasurements);
		/**
		 *
		 * @return new particles that has been chosen due to their weight
		 */
		std::vector<Particle> resampling();
		/**
		 *
		 * @param standardDeviation
		 * @return a random generated double by the given standarddeviation
		 */
		double generateRandomError(double standardDeviation);
		/**
		 *
		 * @param particlesForFilter
		 * @return the total weight of the particles from the particle filter
		 */
		double calculateTotalWeight(std::vector<Particle> particlesForFilter);
		/**
		 *
		 * @param particles
		 * @return assign the vector with particles to another vector with particles
		 */
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
