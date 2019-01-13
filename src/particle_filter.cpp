/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

using std::normal_distribution;
using std::default_random_engine;
using std::discrete_distribution;
//using std::uniform_real_distribution;//no longer using

void ParticleFilter::init(double x, double y, double theta, double std[]) {
    num_particles = 100;//you can play with this for me 10 usually worked pretty well, but 100 seems more eliable, and still seems fast
    
    normal_distribution<double> xDist(x,std[0]);//construct normal distributions around passed in x,y, and theta value to sample particles from
    normal_distribution<double> yDist(y,std[1]);
    normal_distribution<double> tDist(theta,std[2]);
    default_random_engine gen;//use this to get a random values from normal distibutions
    
    for(int i=0; i<num_particles;i++){
        Particle p={i,xDist(gen),yDist(gen),tDist(gen),1.0};//id is just index
        particles.push_back(p);//add particle to particle list
        weights.push_back(1.0);
    }
    is_initialized=true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate) {
    default_random_engine gen;//use this to get a random values from normal distibutions
    normal_distribution<double> xDist(0.0,std_pos[0]);//normal distributions to sample noise from
    normal_distribution<double> yDist(0.0,std_pos[1]);
    normal_distribution<double> tDist(0.0,std_pos[2]);
    
    double d;
    if(yaw_rate!=0){
        d=velocity/yaw_rate;
    }else{
        d=velocity*delta_t;
    }
    
    for(int i=0; i<num_particles;i++){//use bike motion model to calculate and update each particle's x,y and theta with some noise
        double cosPT=cos(particles[i].theta);//do common stuff once
        double sinPT=sin(particles[i].theta);
        particles[i].x+=xDist(gen);
        particles[i].y+=yDist(gen);
        
        if(yaw_rate!=0){
            double theta=particles[i].theta+yaw_rate*delta_t;
            particles[i].x+=d*(sin(theta)-sinPT);//update particle's state
            particles[i].y+=d*(cosPT-cos(theta));
            particles[i].theta=fmod(theta+tDist(gen),2*M_PI);//fmod to keep between 0 and 2PI
        }else{
            particles[i].x+=d*cosPT;//update particle's state
            particles[i].y+=d*sinPT;
        }
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks) {
    double norm=1.0/(2*M_PI*std_landmark[0]*std_landmark[1]);//stuff same for everything calc once
    double varX=2*std_landmark[0]*std_landmark[0];
    double varY=2*std_landmark[1]*std_landmark[1];
    
    for(int i=0;i<particles.size();i++){//for each particle
        weights[i]=1.0;//reset particle weight to 1
        for(int j=0;j<observations.size();j++){//for each observation
            double sinT=sin(particles[i].theta);//transform to map coords using this particle's coords
            double cosT=cos(particles[i].theta);
            double x=particles[i].x+(cosT * observations[j].x)-(sinT * observations[j].y);
            double y=particles[i].y+(sinT * observations[j].x)+(cosT * observations[j].y);
            
            double bestD=sensor_range;//set distance to infinity? or sensor_range?Possible that do not find any landmarks within sensor_range and get error?
            double dis;
            Map::single_landmark_s lm;//find closest landmark to map coords
            for(int k=0;k<map_landmarks.landmark_list.size();k++){
                dis=dist(x,y,map_landmarks.landmark_list[k].x_f,map_landmarks.landmark_list[k].y_f);//if set bestD to Infinity instead of seno_range at start don't need actual distance
                if(dis<bestD){
                    bestD=dis;
                    lm=map_landmarks.landmark_list[k];
                }
            }
            //observations[j].id=lm.id_i;//link landmark to that observation(unecessary)?
            weights[i]*=norm*exp(-(pow(x-lm.x_f,2)/varX+pow(y-lm.y_f,2)/varY));//multipy observation weights(calculated with multivariate gaussian dist) into particle weight
        }
        particles[i].weight=weights[i];
    }
}

void ParticleFilter::resample() {
    default_random_engine gen;
    discrete_distribution<int> distr(weights.begin(), weights.end());//create probabilty distribution for indices of particles
    
    vector<Particle> p;//list of resampled particles
    
    for(int i=0;i<num_particles;i++){
        p.push_back(particles[distr(gen)]);
    }
    
    
    //started keeping track of weights in weights array so no longer using either of 2 methods below
    
    /*uniform_real_distribution<double> distr(0.0,1.0);
    int ind=(int) (distr(gen)*num_particles);
    double maxw=-1.0;//could make 0.0
    for(int i=0;i<num_particles;i++){//get highest weight of any particle
        if(maxw<particles[i].weight){
            maxw=particles[i].weight;
        }
    }
    maxw*=2.0;
    double beta=0.0;
    
    for(int i=0;i<num_particles;i++){//"spin wheel" num_particles time
        beta+=distr(gen)*maxw;
        while(beta>particles[ind].weight){
            beta-=particles[ind].weight;
            ind=(ind+1)%num_particles;
        }
        p.push_back(particles[ind]);//choose particle "wheel lands on"
    }*/
    //another simple method that could be used for resampling(not as good, particles stored further into list have lower liklihood of being chosen)
    /* double sum=0.0;
     for(int i=0;i<num_particles;i++){//sum particle weights
     sum+=particles[i].weight;
     }
     uniform_real_distribution<double> distr(0.0,sum);
     while(p.size()<num_particles){//until have picked enough particles
     for(int i=0;i<num_particles;i++){//for each particle pick it with probabi
     if(distr(gen)<particles[i].weight){
     p.push_back(particles[i]);
     }
     }
     }*/
    particles=p;
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations,
                                     const vector<double>& sense_x,
                                     const vector<double>& sense_y) {
    // particle: the particle to which assign each listed association,
    //   and association's (x,y) world coordinates mapping
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates
    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
    vector<int> v = best.associations;
    std::stringstream ss;
    copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
    vector<double> v;
    
    if (coord == "X") {
        v = best.sense_x;
    } else {
        v = best.sense_y;
    }
    
    std::stringstream ss;
    copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
