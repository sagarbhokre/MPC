# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

## Implementation details
The controller uses a predictive model to control the actuators inside the sdc simulator. Steering actuator and throttle actuator are computed using IpOpt module. The IpOpt module considers the current state and the coefficients to be fit while trying to reduce the cost values.


Following is the sequence of events involved in computing the actuator control values:
1. Convert coordinates from global space to local space, compute current state of the system
2. Compute a 3rd degree polynomial coefficient set representing the path to be followed
3. Set the lower and upper constraint bounds for state variables, actuator values
4. Compute intermediate constraint values for state variables using update equations
5. Compute the cost function depending on gain parameters set imperically
6. Return the actuator and predicted positions back to the simulator

#### Coordinate space conversion and polynomial fitting
The coordinates returned by the simulator are global coordinates. These need to be converted to local coordinates. Following part of the code does the conversion:
```
for(unsigned int i=0; i < ptsx.size(); i++) {
    double shift_x = ptsx[i] - px;
    double shift_y = ptsy[i] - py;
    ptsx[i] = shift_x*cos(0-psi) - shift_y*sin(0-psi);
    ptsy[i] = shift_x*sin(0-psi) + shift_y*cos(0-psi);
}
```
A 3rd order polynomial is fit on these local coordinates
```
auto coeffs = polyfit(ptsx2, ptsy2, 3);
```

State of the system is comprised of the following parameters:
1. Current position x coordinate (px)
2. Current position y coordinate (py)
3. Current position orientation (psi)
4. Current velocity (v)
5. Cross track error (cte)
6. Error in orientation (epsi)


#### Computing intermediate constraint values
Update equations used to set intermediate constraint values are as follows:

```
x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v_[t+1] = v[t] + a[t] * dt
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```

Other constraints set to enable the optimizer include the state variable value range, and actuator limitations.
1. State variables are constrained to range of +/- 1.0e^19
2. The steering values are constrained to a max of +/- 25 degrees
3. Throttle actuators are constrained to a max of +/- 1


#### Computing cost for optimization
These values are computed while trying to optimize the cost function. The cost function is a weighted sum of the following parameters
1. Cross track error (CTE)
2. Deflection from expected orientation (epsi)
3. Difference between current velocity and reference velocity (ev)
4. Steering actuator value (delta)
5. Throttle acutator value (a)
6. Change in steering actuator values (d_delta)
7. Change in throttle actuator values (d_a)

### Handling Latency
To handle the latency involved in actuator controls taking effect, the delta and acceleration variables need to be constrained. 

Snippet mentioned below computes iterations to be considered for constraining the parameters.
```
double latency = 0.10; //(sec)
unsigned int latency_iter = latency/dt;
```

Following snippet constrains the delta parameter until the latency sets in.
```
if(i < delta_start + latency_iter) {
    vars_lowerbound[i] = delta_prev;
    vars_upperbound[i] = delta_prev;
}
```
Similar change is implemented to constrain acceleration parameter. 

Eventually, the actuator control value passed to the simulator is offset be the latency_iteration as shown below

```
result.push_back(solution.x[delta_start + latency_iter])
```


## Misc

To handle repeated restarts and compilation a couple of utilities are developed to make the process smooth and fast.

update_params utility allows setting the gain parameters on the go while the controller and simulator are running.

```
./update_params -c 1500 -e 3000 -v 0.14 -d 30000 -a 80 -p 290 -q 50;
-c -- cte gain param
-e -- epsi gain param
-v -- velocity error gain param
-d -- delta gain param
-a -- acceleration gain parameter
-p -- d_delta gain parameter
-q -- d_a gain parameter
```


./restart_game

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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

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

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
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
