# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## My Implementation:

My implementation can be broken down into three parts. In the three parts, the lane assignment is done and for the trajectory generation, I used the same implementation with splines that makes use of previous path points for smooth trajectory generation. Speed increments/decrements for each frame were chosen so that the maximum jerk and total accerlation limits are not violated.

I chose to have all my codes in main.cpp so it's easier to read and debug, as the contents were not too long.

### Assignment of detected cars for the left lane, current lane, right lane (lines 280~323)

I first created a struct car to store left, current, and right lane cars separately. I checked if the absolute distance to each car is near my car before assigning lanes to the cars, as cars too far away from the ego vehicle are not needed for behaviour planning. I used the velocity of each car multiplied by 0.02s(each frame time) * number of previous path sizes to get more accurate s position of each vehicle. This in effect is a prediction of a few frames from the past into the current frame considering the delay of information transfer from the simulator. But this prediction is not the prediction of what each car will do for the future, this is something that I will work on later when I find time.

For the s value, I used the predicted s value of each car and subtracted the ego-car's s value so the difference in s for each car in each lane was stored in the struct.

### Extraction of various vehicle speed, distance, gap information from the detected cars (lines 326~399)

Then I sorted the lane assigned left,current, and right lanes cars based on their distance to figure out the speed of the very first front car (if exists) in my lane, in my right lane, and my left lane. The same was done for the very first car behind me (if exists) for the three lanes. Of course, if the ego car is located on the very left lane, there will be no lane assigned cars for the left side, so nothing will be updated --- similar for the very right lane. Initial values were chosen so that the ego-car can change lanes when needed. (i.e. all the vehicle speeds in the front are assigned as the desired speed of 49.5mph unless a car is detected that has less speed than that. This helps in the PLCL or PLCR finite states, which will be described below, so that the transition to LCL and LCR can happen if the speed of the car in the front for the lane I'm trying to change into is at least as fast as my current speed.)

The gap for the left and right lane was basically the smallest s value in the detected cars, respectively for the left and the right lane. For the current lane, I didn't consider the car behind me but only for the car in front of me, and I stored the distance to the very first car in front of me, if it exists. If there were no cars in front of the ego-car, then I intialized the gap_mylane to the ALLOWED_DISTANCE, so that the PLCL or PLCR transitions won't happen, and the car will just try to speed up until the desired speed of 49.5mph is reached.

### Finite state machine for behavior planning (lines 402 ~ 548)

I chose to use the Finite State Machine introduced as part of the behaviour planning lesson. I chose to have KL, PLCL, PLCR, LCL, and LCR states. The reason I chose the FSM is that it's easy to plan logics that can be robust, as there are so many cases and corner cases to consider if FSM is not used, and it will take a long time to figure out all the detailed conditions that will be fail-proof. conditional statements in the main call-back function were utilized so that FSM update can be done on every frame, regardless of other conditions.

The FSM is initialized with the KL = Keep Lane state, and it keeps this state unless there's a car in front that is at least 5m/s (which is 5*2.24 mph for the simulator) slower than me and is within ALLOWED_DISTANCE of 25m. If this condition is met, then transition to PLCL (Plan Lane Change Left) and PLCR (Plan Lane Change Right) will be considered. First, a comparison between the gap on the right side and the left side are done, and the side that has more gap will be selected for the Plan Lane Change state. If the ego-car is located on the very left-lane, then only PLCR will be considered, and if it is on the very rightmost lane, then only PLCL will be considered, regardless of the gap comparison. If both sides have equal gaps, then preference will be given to the left lane, as left lane in real highway situations will have higher speeds and therefore more favorable. If staying within the KL state, then too_close flag will be set to true when there's a slow car in front of us, which in turn will trigger decrements of VEL_INC will happen in order to slow down as to not hit the car in front of us. If there's no slow car in the front, then same VEL_INC increments will happen so that the car can speed up until the desired speed of 49.5mph is reached.

For PLCL and PLCR states, the ego-car will decelerate until the ego-car's speed is slower than the car in the front, but not too slow as that can cause the car to completely stop. I set this speed setting to 90% of the speed of the car in front of us. If the ego-car becomes too slow, it's hard to safely change into other lanes that have cars going at certain speeds. The condition to transition to back to the KL state was if the distance between the ego-car and the car in the front has become large enough (>= ALLOWED_DISTANCE) so that there's enough space to try to accelerate. Also, if the ego-car is in the middle lane and if the other side has less gap than the current plan (i.e. if I'm in the middle lane with PLCL state, if the situtation on the right lane changes so that there's more space on the right lane, then the FSM will transition back to KL so that a chance to transition into PLCR instead is provided). Check against the lane position is done so that the car doesn't go out of bounds. For transitioning condition to LCL or LCR, respectively, was set so that if the gap in the interested lane is big enough (greater or equal to SAFE_GAP_TO_CL) AND the first car in the front in that lane (if exists) is as fast as my car's speed AND the first car behind me in that lane (if exists) is as slow as my car's speed, then the transition will be made. 

For LCL and LCR states, I set the ego-car to accelerate without exceeding accleration limit or jerk limit when transitioning into the desired lane. Also, I set a transition flag which is set to true on the edge-transition, and is set to false once the ego-car is within some tolerance of the desired lane (0.9m). The tolerance had to be sufficient as sometimes the car doesn't fully go to the very center of the lane fast enough (especially on curved parts of the highway). This condition can cause the car to keep acclerating until the center position is accurately reached, and it can cause collision to the car in the front (if exists). So the tolerance had to be set to almost half the car's width. Upon satisfying this condition, which means the car has completed the transition to the desired lane, the FSM changes back to the KL state.

Future works:
I want to not only look at the lanes right next to me but the other lane as well (i.e. the 3rd lane if I'm in the 1st lane) and using some clever cost functions, decide in some cases to change all the way over to the 3rd lane if that lane is deemed safe to go into, so that the ego-car can get to the goal as fast as possible.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

