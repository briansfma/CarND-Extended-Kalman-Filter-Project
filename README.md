# Object Tracking via Extended Kalman Filter
Udacity Self-Driving Car Engineer Nanodegree Program

This project utilizes an extended kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Maintaining low RMSE values is the main focus of this project, as any object tracking software would be useless in the real-world if it is not accurate.

[//]: # (Image References)
[image1]: runtime_example.jpg "Runtime Example"

## Basic Build Instructions

Running the code requires connecting to the Udacity CarND Term 2 Simulator, which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see the uWebSocketIO Starter Guide page in the classroom within the EKF Project lesson for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by executing in the Linux bash:

```
$ git clone https://github.com/briansfma/CarND-Extended-Kalman-Filter-Project
$ cd CarND-Extended-Kalman-Filter-Project
$ mkdir build
$ cd build
$ cmake ..
$ make
$ ./ExtendedKF
```

## Running the Program

If it is not running already, launch the project.

```
$ cd CarND-Extended-Kalman-Filter-Project
$ cd build
$ ./ExtendedKF
```

When `ExtendedKF` has initialized successfully, it will output to terminal

```
Listening to port 4567
```

Launch the simulator `term2_sim.exe`. Select "Project 1/2: EKF and UKF" from the menu. Upon successful connection to the simulator, `ExtendedKF` will output to terminal

```
Connected!!!
```

Click the "Start" button and the simulator will run. `ExtendedKF` will begin outputting `x` (position) and `P` (covariance) values to the terminal. The error values will be outputted to the simulation screen itself under "RMSE". Green triangular markers denote where the Kalman Filter believes the object is.

![alt text][image1]

For reference, the project rubric requires RMSE values equal to or less than [.11, .11, 0.52, 0.52]. This code should perform consistently to the example image across multiple situations. 

## Other Important Dependencies

* cmake >= 3.5
    * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
    * Linux: make is installed by default on most Linux distros
    * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
    * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
    * Linux: gcc / g++ is installed by default on most Linux distros
    * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
    * Windows: recommend using [MinGW](http://www.mingw.org/)
