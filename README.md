# Particle Filter Localization
This code as is will interact with the Simulator found [here](https://github.com/udacity/self-driving-car-sim/releases)
## Dependencies:
##### To build and run as is this project requires:

###### To Run:
  1. [Simulator](https://github.com/udacity/self-driving-car-sim/releases)
  2. [uWebSocketIO](https://github.com/uWebSockets/uWebSockets)

###### To Build:
  3. cmake >= 3.5
    * All OSes: [click here for installation instructions](https://cmake.org/install/)
  4. make >= 4.1 (Linux, Mac), 3.81 (Windows)
    * Linux: make is installed by default on most Linux distros
    * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
    * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
  5. gcc/g++ >= 5.4
    * Linux: gcc / g++ is installed by default on most Linux distros
    * Mac: same as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
    * Windows: recommend using [MinGW](http://www.mingw.org/)

## Running the Code:

The main program can be built and run by running the following commands from the project's top directory(this has already been done):

If no changes made to code just run:
1. Run it: `./build/particle_filter `

Or build with changes and run:

1. Remove old build directory and make new one: `rm -r build && mkdir build && cd build`
2. Compile: `cmake .. && make`
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
3. Run it: `./particle_filter `

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.
## Code Structure:
In src the following files:
 1. main.cpp:
      Loads map data from file and handles server interaction, receives telemetry data, initial state from 'GPS', velocity and yaw rate of the car, and updates from sensors of relative position of observed landmarks to car.
      Passes velocity, yaw rate, and observations to particle filter
 2. particle_filter.cpp:
 Contains and manages particles, updating their states using velocity, and yaw rate passed to it in main. Then matches observations to landmarks for each particle to determine likelihood that particle is in same state as actual car given observations from the car's sensors. Then resamples the particles it is managing based on this likelihood.

##### INPUT: values provided by the simulator to the c++ program

// sense noisy position data from the simulator(at initialization to simulate GPS data)

["sense_x"]

["sense_y"]

["sense_theta"]

// get the previous velocity and yaw rate to predict the particle's transitioned state

["previous_velocity"]

["previous_yawrate"]

// receive noisy observation data from the simulator, in a respective list of x/y values

["sense_observations_x"]

["sense_observations_y"]


##### OUTPUT: values provided by the c++ program to the simulator

// best particle values used for calculating the error evaluation

["best_particle_x"]

["best_particle_y"]

["best_particle_theta"]

//Optional message data used for debugging particle's sensing and associations

// for respective (x,y) sensed positions ID label

["best_particle_associations"]

// for respective (x,y) sensed positions

["best_particle_sense_x"] <= list of sensed x positions

["best_particle_sense_y"] <= list of sensed y positions

## Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory.

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

### All other data the simulator provides, such as observations and controls.
> * Map data provided by 3D Mapping Solutions GmbH.
