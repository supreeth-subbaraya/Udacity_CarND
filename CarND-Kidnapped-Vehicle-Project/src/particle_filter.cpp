/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	num_particles = 20;

	double stdX = std[0];
	double stdY = std[1];
	double stdTheta = std[2];

	default_random_engine generator;

	normal_distribution<double> distX(x, stdX);
	normal_distribution<double> distY(y, stdY);
	normal_distribution<double> distTheta(theta, stdTheta);

	// Initialize the particles
	for(int i = 0; i < num_particles; i++)
	{
		Particle tempParticle;

		tempParticle.x = distX(generator);
		tempParticle.y = distY(generator);
		tempParticle.theta = distTheta(generator);
		tempParticle.weight = 1.0;
		tempParticle.id = i;

		particles.push_back(tempParticle);
		weights.push_back(tempParticle.weight);
	}
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	default_random_engine generator;

	for(int i = 0; i < num_particles; i++)
	{
		double x = particles[i].x;
		double y = particles[i].y;
		double theta = particles[i].theta;
		double predX, predY, predTheta;

		if (fabs(yaw_rate) > 0.0001)
		{
			predX = x + (velocity/yaw_rate) * (sin(theta+yaw_rate*delta_t) - sin(theta) );
			predY = y + (velocity/yaw_rate) * (cos(theta) -cos(theta+yaw_rate*delta_t));
			predTheta = theta + yaw_rate * delta_t;
		}
		else
		{
			predX = x + velocity * cos(theta) * delta_t;
			predY = y + velocity * sin(theta) * delta_t;
			predTheta = theta;
		}

		normal_distribution<double> distX(predX, std_pos[0]);
		normal_distribution<double> distY(predY, std_pos[1]);
		normal_distribution<double> distTheta(predTheta, std_pos[2]);

		particles[i].x = distX(generator);
		particles[i].y = distY(generator);
		particles[i].theta = distTheta(generator);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.

	for( auto& observation : observations )
	{
		double minDist = numeric_limits<double>::max();

		for( auto& prediction : predicted )
		{
			double obsX = observation.x;
			double obsY = observation.y;
			double predX = prediction.x;
			double predY = prediction.y;

			double distance = dist(obsX, obsY, predX, predY);

			if (distance < minDist)
			{
				minDist = distance;
				observation.id = prediction.id;
			}
		}
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	//  Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	for( int i = 0; i < num_particles; i++)
	{
		double pX = particles[i].x;
		double pY = particles[i].y;
		double pTheta = particles[i].theta;
		particles[i].weight = 1.0;

		vector<LandmarkObs> predictions;
		vector<LandmarkObs> transformedObs;

		// Obtain the predictions
		for(unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++ )
		{
			double landmarkX = map_landmarks.landmark_list[j].x_f;
			double landmarkY = map_landmarks.landmark_list[j].y_f;
			int landmarkId = map_landmarks.landmark_list[j].id_i;

			if(fabs(landmarkX - pX) <= sensor_range && fabs(landmarkY - pY) <= sensor_range)
			{
				predictions.push_back(LandmarkObs{landmarkId, landmarkX, landmarkY});
			}
		}

		// Obtain the transformed onservations
		for(auto& observation : observations)
		{
			LandmarkObs tObs;

			double vehicleObsX = observation.x;
			double vehicleObsY = observation.y;

			tObs.x = pX + cos(pTheta) * vehicleObsX - sin(pTheta) * vehicleObsY;
			tObs.y = pY + sin(pTheta) * vehicleObsX + cos(pTheta) * vehicleObsY;

			transformedObs.push_back(tObs);
		}

		dataAssociation(predictions, transformedObs);

		double stdX = std_landmark[0];
		double stdY = std_landmark[1];
		double normTerm = 1.0 / ( 2.0 * M_PI * stdX * stdY);
		double stdX2 = pow(stdX, 2);
		double stdY2 = pow(stdY, 2);

		// Find the multi variate gaussian prob.
		for( auto& tObs : transformedObs )
		{
			double tformX = tObs.x;
			double tformY = tObs.y;
			double tformId = tObs.id;

			for( auto& prediction : predictions)
			{
				double predId = prediction.id;

				if( tformId == predId)
				{
					double predX = prediction.x;
					double predY = prediction.y;

					double weight = normTerm * exp(-1.0 * ((pow(tformX - predX, 2) / (2.0 * stdX2)) + (pow(tformY - predY, 2) / (2.0 * stdY2))));
					particles[i].weight *= weight;
				}
			}
		}
		weights[i] = particles[i].weight;
	}
}

void ParticleFilter::resample() {
	// Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	vector<Particle> newParticles;

	default_random_engine gen;

	uniform_int_distribution<int> particleIdx(0, num_particles-1);
	int index = particleIdx(gen);

	double maxWeight =  *max_element(weights.begin(), weights.end());
	double beta = 0.0;

	uniform_real_distribution<double> maxW(0.0, 2.0 * maxWeight);

	for( int i = 0; i < num_particles; i++)
	{
		beta = beta + maxW(gen);

		while (beta > weights[index])
		{
			beta -= weights[index];
			index = (index + 1) % num_particles;
		}
		newParticles.push_back(particles[index]);
	}

	particles = newParticles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations,
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;

	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
