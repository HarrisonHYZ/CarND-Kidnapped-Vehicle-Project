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
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	
	cout<<"Init Starts!"<<endl;//Added by Harrison

	default_random_engine gen;
	double std_x, std_y, std_theta, gps_x, gps_y; // Standard deviations for x, y, and theta

	// TODO: Set standard deviations for x, y, and theta
	 std_x = std[0];
	 std_y = std[1];
	 std_theta = std[2];
	 gps_x = x;
	 gps_y = y;	 

	// This line creates a normal (Gaussian) distribution for x
	normal_distribution<double> dist_x(gps_x, std_x);
	
	// TODO: Create normal distributions for y and theta
	normal_distribution<double> dist_y(gps_y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);

	// Create 10 particle
	num_particles = 10;
	
	for (int i = 0; i < num_particles; ++i) {
		double sample_x, sample_y, sample_theta;
		
		// TODO: Sample  and from these normal distrubtions like this: 
		//	 sample_x = dist_x(gen);
		//	 where "gen" is the random engine initialized earlier.
		
		 sample_x = dist_x(gen);
		 sample_y = dist_y(gen);
		 sample_theta = dist_theta(gen);	

		 Particle particle={
		 	i,
		 	sample_x,
		 	sample_y,
		 	sample_theta,
		 	1.0
		 };

		 particles.push_back(particle);
		 
		 // Print your samples to the terminal.
		 cout << "Particle " << i << " " << sample_x << " " << sample_y << " " << sample_theta << " " << 1.0 << endl;
	}

	is_initialized = true;

	cout<<"Init end!"<<endl;//Added by Harrison
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	cout<<"Prediction Starts!"<<endl;//Added by Harrison

	default_random_engine gen;

	// This line creates a normal (Gaussian) distribution for x, y, theta
	// normal_distribution<double> dist_x(0, std_pos[0]);
	// normal_distribution<double> dist_y(0, std_pos[1]);
	// normal_distribution<double> dist_theta(0, std_pos[2]);

	if(yaw_rate == 0.0){
		for(int i=0; i<num_particles; i++){
			// double noise_x, noise_y, noise_theta;
			// noise_x = dist_x(gen);
		 // 	noise_y = dist_y(gen);
		 // 	noise_theta = dist_theta(gen);

			particles[i].x = particles[i].x + velocity * delta_t * cos(particles[i].theta);
			particles[i].y = particles[i].y + velocity * delta_t * sin(particles[i].theta);
			particles[i].theta = particles[i].theta;

			// This line creates a normal (Gaussian) distribution for x, y, theta
			normal_distribution<double> dist_x(particles[i].x, std_pos[0]);
			normal_distribution<double> dist_y(particles[i].y, std_pos[1]);
			normal_distribution<double> dist_theta(particles[i].theta, std_pos[2]);

			particles[i].x = dist_x(gen);
		 	particles[i].y = dist_y(gen);
		 	particles[i].theta = dist_theta(gen);

			//angle normalization
    		while (particles[i].theta> M_PI) particles[i].theta-=2.*M_PI;
    		while (particles[i].theta<-M_PI) particles[i].theta+=2.*M_PI;

    		//cout<<"Particle "<<i<<" is "<<particles[i].x<<", "<<particles[i].y<<", "<<particles[i].theta<<endl;//Added by Harrison
		}
	}
	else{
		for(int i=0; i<num_particles; i++){
			// double noise_x, noise_y, noise_theta;
			// noise_x = dist_x(gen);
		 // 	noise_y = dist_y(gen);
		 // 	noise_theta = dist_theta(gen);

			particles[i].x = particles[i].x + (velocity / yaw_rate) * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
			particles[i].y = particles[i].y + (velocity / yaw_rate) * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
			particles[i].theta = particles[i].theta + yaw_rate * delta_t;

			// This line creates a normal (Gaussian) distribution for x, y, theta
			normal_distribution<double> dist_x(particles[i].x, std_pos[0]);
			normal_distribution<double> dist_y(particles[i].y, std_pos[1]);
			normal_distribution<double> dist_theta(particles[i].theta, std_pos[2]);

			particles[i].x = dist_x(gen);
		 	particles[i].y = dist_y(gen);
		 	particles[i].theta = dist_theta(gen);

			//angle normalization
    		while (particles[i].theta> M_PI) particles[i].theta-=2.*M_PI;
    		while (particles[i].theta<-M_PI) particles[i].theta+=2.*M_PI;

    		//cout<<"Particle "<<i<<" is "<<particles[i].x<<", "<<particles[i].y<<", "<<particles[i].theta<<endl;//Added by Harrison
		}
	}

	cout<<"Prediction End!"<<endl;//Added by Harrison
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	cout<<"dataAssociation Starts!"<<endl;//Added by Harrison

	for(int i=0; i<observations.size(); i++){
		observations[i].id = predicted[0].id;
		int distance = dist(observations[i].x, observations[i].y, predicted[0].x, predicted[0].y);

		for(int j=1; j<predicted.size(); j++){
			int distance_new = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);

			if(distance_new < distance){
				distance = distance_new;
				observations[i].id = predicted[j].id;
			}
		}
	}

	// Added by Harrison
	// for(int i=0; i<observations.size(); i++){
	// 	cout<<"Observation "<<i<<" with closest landmark is "<<observations[i].id<<", "<<observations[i].x<<", "<<observations[i].y<<endl;//Added by Harrison
	// }
	// Added by Harrison

	cout<<"dataAssociation End!"<<endl;//Added by Harrison
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
	
	// Added by Harrison. To see the map data
	// for(int m=0; m<map_landmarks.landmark_list.size(); m++){
	// 	cout<<map_landmarks.landmark_list[m].id_i<<"___"<<map_landmarks.landmark_list[m].x_f<<"___"<<map_landmarks.landmark_list[m].y_f<<endl;
	// }
	// Added by Harrison
	

	cout<<"updateWeights Starts!"<<endl;//Added by Harrison

	for(int m=0; m<num_particles; m++){

		cout<<"This is particle "<<m<<endl;//Added by Harrison

		//	 Predict all the map landmarks within the sensor_range for a particular particle
		std::vector<LandmarkObs> predicted;
		for(int i=0; i<map_landmarks.landmark_list.size(); i++){

			double distance = dist(particles[m].x, particles[m].y, map_landmarks.landmark_list[i].x_f, map_landmarks.landmark_list[i].y_f);

			if(distance <= sensor_range){
				LandmarkObs LO_pre;
				LO_pre.id = map_landmarks.landmark_list[i].id_i;
				LO_pre.x = map_landmarks.landmark_list[i].x_f;
				LO_pre.y = map_landmarks.landmark_list[i].y_f;

				predicted.push_back(LO_pre);
			}
		}

		//	 calculate normalization term
		double sig_x = std_landmark[0];
		double sig_y = std_landmark[1];
		double gauss_norm = 1/(2 * M_PI * sig_x * sig_y);
		// cout<<"gauss_norm is "<<gauss_norm<<endl; // Added by Harrison

		// There is no landmarks within the sensor range!!!
		// if(predicted.empty()){

		// 	cout<<"predicted landmarks is empty!"<<endl;//Added by Harrison

		// 	//	 Transfer all the observations to the map coordinates for a particular particle
		// 	std::vector<LandmarkObs> trans_observations;
		// 	for(int j=0; j<observations.size(); j++){
		// 		LandmarkObs LO_obs;
		// 		LO_obs.id = 0;
		// 		LO_obs.x = particles[m].x + (cos(particles[m].theta) * observations[j].x) - (sin(particles[m].theta) * observations[j].y);
		// 		LO_obs.y = particles[m].y + (sin(particles[m].theta) * observations[j].x) + (cos(particles[m].theta) * observations[j].y);

		// 		trans_observations.push_back(LO_obs);
		// 	}

		// 	// Added by Harrison
		// 	for(int i=0; i<trans_observations.size(); i++){
		// 		cout<<"trans_observations"<<i<<" is "<<trans_observations[i].id<<", "<<trans_observations[i].x<<", "<<trans_observations[i].y<<endl;//Added by Harrison
		// 	}
		// 	// Added by Harrison

		// 	//	 Calculate the final weight for this particular particle
		// 	for(int n=0; n<trans_observations.size(); n++){
		// 		// 	 calculate exponent
		// 		double exponent = pow(sensor_range,2)/(2 * pow(sig_x,2)) + pow(sensor_range,2)/(2 * pow(sig_y,2));

		// 		// 	 calculate weight using normalization terms and exponent
		// 		particles[m].weight *= gauss_norm * exp(-exponent);
		// 	}
		// }
		// else{// There are landmarks within the sensor range

			//cout<<"predicted landmarks is not empty!"<<endl;//Added by Harrison

			// Added by Harrison
			// for(int i=0; i<predicted.size(); i++){
			// 	cout<<"Landmarks within sensor range: Landmark "<<i<<" is "<<predicted[i].id<<", "<<predicted[i].x<<", "<<predicted[i].y<<endl;//Added by Harrison
			// }
			// Added by Harrison

			//	 Transfer all the observations to the map coordinates for a particular particle
			std::vector<LandmarkObs> trans_observations;
			for(int j=0; j<observations.size(); j++){
				LandmarkObs LO_obs;
				LO_obs.id = 0;
				LO_obs.x = particles[m].x + cos(particles[m].theta) * observations[j].x - sin(particles[m].theta) * observations[j].y;
				LO_obs.y = particles[m].y + sin(particles[m].theta) * observations[j].x + cos(particles[m].theta) * observations[j].y;

				trans_observations.push_back(LO_obs);
			}

			// Added by Harrison
			// for(int i=0; i<trans_observations.size(); i++){
			// 	cout<<"trans_observations"<<i<<" is "<<trans_observations[i].id<<", "<<trans_observations[i].x<<", "<<trans_observations[i].y<<endl;//Added by Harrison
			// }
			// Added by Harrison

			dataAssociation(predicted, trans_observations);

			//	 Calculate the final weight for this particular particle
			for(int n=0; n<trans_observations.size(); n++){
				// 	 calculate exponent
				double x_obs = trans_observations[n].x;
				// cout<<"x_obs is "<<x_obs<<endl; // Added by Harrison
				double y_obs = trans_observations[n].y;
				// cout<<"y_obs is "<<y_obs<<endl; // Added by Harrison
				double mu_x = map_landmarks.landmark_list[trans_observations[n].id-1].x_f;
				// cout<<"mu_x is "<<mu_x<<endl; // Added by Harrison
				double mu_y = map_landmarks.landmark_list[trans_observations[n].id-1].y_f;
				// cout<<"mu_y is "<<mu_y<<endl; // Added by Harrison
				double exponent = pow(x_obs - mu_x,2)/(2 * pow(sig_x,2)) + pow(y_obs - mu_y,2)/(2 * pow(sig_y,2));
				// cout<<"exponent is "<<exponent<<endl; // Added by Harrison

				// 	 calculate weight using normalization terms and exponent
				particles[m].weight *= gauss_norm * exp(-exponent);
				// cout<<"weight is "<<particles[m].weight<<endl; // Added by Harrison
			}
		// }

		//cout<<"The weight of particle " << m <<" before normalization is: "<<particles[m].weight<<endl;//Added by Harrison
		//cout<<"TEST!!!"<<endl;//Added by Harrison
	}

	//	 Normalize all the final weights for all particles
	double sum_weight = 0.0;

	for(int m=0; m<num_particles; m++){
		sum_weight += particles[m].weight;
	}

	for(int m=0; m<num_particles; m++){
		particles[m].weight /= sum_weight;

		cout<<"The weight of particle " << m <<" after normalization is: "<<particles[m].weight<<endl;//Added by Harrison
	}

	cout<<"updateWeights Ends!"<<endl;//Added by Harrison
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

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
