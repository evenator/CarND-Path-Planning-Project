CarND-Path-Planning-Project
===========================

Project 1 of Term 3 of the Udacity Self-Driving Car Engineer Nanodegree Program

---

About This Project
------------------

The assignment for this project was to perform path planning for a simulated car on a simulated
highway track. The track was a six lane highway (three lanes in each direction) with other simulated
cars on the track. The goal was for the car to drive around the track at the speed limit of 50 MPH,
change lanes to avoid traffic, and avoid collisions and uncomfortable maneuvers (high
jerk/acceleration).

Lane Following
--------------

The `LaneKeepPlanner` class implements lane keeping and adaptive cruise control. Using the `Map`
class, it generates waypoints along the route in current lane at 30, 50, 70, and 90 meters ahead
of the car's current position. These points are then converted from Frenet coordinates to global
XY coordinates using the `Map`. The points are then converted to local XY coordinates (the local
frame's origin is the car's current pose). The local XY points are used as tie-points in a cubic
spline.

The cubic spline is used to generate a dense path for the vehicle to follow. Each point is spaced
0.02 seconds apart. The speed control is achieved by generating the points iteratively with a speed
setpoint of 21 meters per second when there is no car in front of the vehicle, or to follow the
car in front by matching the leader's speed when it is 20 meters ahead. This is accomplished using
bang-bang acceleration with a maximum acceleration of 7.5 meters/s^2.

To ensure smooth driving, each new plan is a continuation of the last plan sent. On each planning
iteration, the previous plan is truncated to 10 points and the remainder of the plan is regenerated
up to 50 points.

Lane Changing
--------------

Because the `LaneKeepPlanner` generates smooth splining paths, it can be used to perform a lane-
change maneuver just by requesting it to plan with a different target lane. The logic to decide
which lane to be in and when to make lane changes is governed by the `FSMPlanner`.

On each update, the `FSMPlanner` checks the distance to the nearest leader vehicle in all three
lanes to identify the "best" lane. The best lane is the lane with the most space between the vehicle
and the closest leader. If the best lane is not the current lane, the `FSMPlanner` must decide
whether to change lanes. To avoid unecessary lane changes and unstable oscillation between lanes,
the `FSMPlanner` does not consider a lane change unless the best lane has at least 10 meters more
space ahead than the current lane and the current lane has less than 50 meters of space ahead.

If the conditions to change lanes are met, the `FSMPlanner` checks the obstacle list to determine
if it is safe to change lanes. The `FSMPlanner` will only command a lane change if the destination
lane has no cars from 50 meters behind the vehicle to 25 meters ahead of it. If that area is clear
of other cars, the `FSMPlanner` changes the target lane of the `LaneKeepPlanner`, and the vehicle
changes lanes.

Building and Running
---------------------

### Dependencies

Build a docker image from the included Dockerfile, or install these dependencies:

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

You'll also need the Simulator, which you can download from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).


### Basic Build Instructions

1. Clone this repository.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

