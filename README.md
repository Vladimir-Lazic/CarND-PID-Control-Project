# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---
[//]: # (Image References)

[image1]: ./car.gif "Result"

![alt text][image1]

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

### Simulator
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```
## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Implementation

In this repo I have implemented a PID controller and make two instances used for steering angle of the vehicle and throttle. The steering angle value was enough to make the car safely make a lap around track, but in order to allow for a bit smoother ride, a second instance of the PID controller was used for throttle value. The implementation minimizes the Cross Track Error (cte). 

## The effect each of the P, I, D components

The basic idea is to use the PID controller to adjust the the vehicle position on the track. The P, or proportional component of the regulator is the component that makes the greatest adjustments while the CTE is the highest. While we are the furthest from the desired position the P component allows us to get on track. The D component is used to avoid overshooting, caused by the P component. The effects of the D components is while we slowly approach the desired path, we make small adjustments to the controls in order not to go of path once we reach it. The I component is used to correct against the bias of the PI components. If its the situation that after a certain period of time we are still not at the desired location the I component makes corrections in order to get on track. 

## Tunning Hyperparameters

The parameters were tunned manually. For both instances of the PID controller, steering and throttle the tunning was done as follows. First I tested with only P, then only D, then only I components. The I ran the combinations. The main goal was to get the car to go around the track and stay within lane boundaries. The job was done with only P and I components. P being set to 0.15, I being set to 2.5. However I did notice a great deal of wheels swiftly going left to right. So I decided to engage the D component with a small coefficient value, 0.00005. Upon running the simulator it appears to have reduced the wheels from oscillating left and right. 

The idea for the throttle PID controller was to ensure a smoother ride. For instance while approaching turns the car might benefit from a lower velocity in order to safely make a turn and go smoother along a curved path. For that I found I only have use with the I component it being set to 15. For the final throttle value if subtract the output values of the I controller from a constant value of 0.75 in order to normalize the throttle value. 
