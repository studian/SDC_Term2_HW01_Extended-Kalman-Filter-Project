[image1]: ./simul_results/EKF_results.png
[image2]: ./simul_results/EKF_simul.png

# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project you will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

![alt text][image1]

**My build environments are use to `cmake` in the `Ubuntu 16.04`.**

It must be installed [uWebSocketIO](https://github.com/uWebSockets/uWebSockets).
Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

**Other Important Dependencies**
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros

## 1. Build & Make
* Clone this repo.
* mkdir build
* cd build
* cmake ..
* make
* ./ExtendedKF

### Editor Settings
We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:
* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

### Code Style
Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## 2. Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

If you want to do optional process, you must download and use to version of `project-assistant-ready` of `CarND-Extended-Kalman-Filter-Project`

I did not yet.

## 3. Project Results

Here are Cmake based result.

* Cmake result is follows:
![alt text][image2]
