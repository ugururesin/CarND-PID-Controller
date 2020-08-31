# CarND-PID-Controller
Self-Driving Car Engineer Nanodegree Program
---
## Introduction  
A proportional–integral–derivative controller (**PID controller** or three-term controller) is a control loop mechanism employing feedback that is widely used in industrial control systems and a variety of other applications requiring continuously modulated control. A PID controller continuously calculates an error value e(t) as the difference between a desired setpoint (SP) and a measured process variable (PV) and applies a correction based on proportional, integral, and derivative terms (denoted P, I, and D respectively), hence the name.  
The distinguishing feature of the PID controller is the ability to use the three control terms of proportional, integral and derivative influence on the controller output to apply accurate and optimal control.

![](img/PID.png)  

### Proportional (P) Control  
Let's say ego car moves with a constant velocity and our reference trajectory would be the x-axis. Thus, the y-axis will represent the distance between the ego car and the reference trajectory line. Let's call this Cross-Track-Error (CTE in short).  
So, how do we set the steering angle?  

In proportional control, **steering_angle = -t * CTE**  (t is a factor)  

The problem with the proportional control is that the ego car overshoots!  

### Proportional-Derivative (PD) Control  
To avoid overshooting problem coming with the proportional control, derivative (D) term is added. As ego car moves with the initial steering angle, the CTE would decrease for a period of time. However, the steering angle is also needed to be decreased as well.  

In proportional-derivative control, **steering_angle = -tp * CTE - td * (d/dt)CTE**  

If there is a systematic bias (e.g. a misalignment between front-wheel angles mechanically), the PD controller can not handle this.

### Proportional-Integral-Derivative (PID) Control  
To overcome the overshooting problem and the systematic bias, proportional–integral–derivative controller is used.  
An example plot is provided below to compare p, pi and pid controllers.

![](img/PID_plot.png)  
---
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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.  

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

---
## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

**Compilation**  
The code can compile.  
![](img/compile.png)    

**Implementation**  
Here, heuristic approach (trial-and-error) is used so as to observe the outcome of the different parameters.  

First, **PID.cpp** file is modified to initialize PID coefficients.  
Also counters added to calculate average error.

In **main.cpp** file, the main function is modified so as to send PID parameters into it.
<pre><code>
int main(int argc, char *argv[])
{
  uWS::Hub h;
  
  //INITILIZE PID VARIABLES
  PID pid;
  double init_Kp = atof(argv[1]); //the proportional term deals with how far the car is from the reference
  double init_Ki = atof(argv[2]); //the integral term deals with the systematic bias (like misalignment of wheels)
  double init_Kd = atof(argv[3]); //the derivative term is to prevent hard oscillation around the reference
  pid.Init(init_Kp, init_Ki, init_Kd);
  ...
</pre></code>
  
Then, steering value is calculated based on PID controller.  
<pre><code>
// STEERING VALUE ERROR by CTE
pid.UpdateError(cte);
steer_value -= pid.TotalError(); 
</pre></code>

**Reflection**  
In this study, heuristic approach is used so as to observe the effects of the pid parameters.  

1. **P-controller** is tried with following p-i-d parameters: 0.15-0.0-0.0  
The ego vehicle oscillated quite a lot with this controller and then went off the road.  

![](img/p_only_off.png)  

See the full video: [./videos/p_only.mov](./videos/p_only.mov).  

2. **PD-controller** is tried with following p-i-d parameters: 0.15-0.0-2.5  
This prevents the oscillation.    

See the full video: [./videos/pd_ideal.mov](./videos/pd_ideal.mov).  

3. **PID-controller** is tried with some p-i-d parameters.
Since there were no systematic bias in the ego vehicle, there were no difference between PD and PID controllers for small integral values.  
However, **higher integral values** cause that steering angles began to change very rapidly, causing the vehicle to lose control!!!  
![](img/high_integral.gif)  
(4 times faster!)

See the full video: [./videos/pid_high_integral.mov](./videos/pid_high_integral.mov).  

Thus, a lower integral value is set for the pid controller.  
See the full video: [./videos/pid_ideal.mov](./videos/pid_ideal.mov).  


**Simulation**  
No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle).  

The simulation for the ideal pid controller (parameters = 0.15, 0.001, 2.5) is shown below:  

![](img/sim_full.gif)  
(6 times faster!)

See the full video: [./videos/pid_ideal.mov](./videos/pid_ideal.mov)  

---
### Future Improvements  
In this study, 2 improvements can be done as described below.  

1. The vehicle speed was increased to 50 mph to see the controller's steering performance at a higher speed.  
<pre><code>
std::endl;
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.5;
</code></pre> 
As a result, even if the vehicle did not leave the road completely, oscillation increased too much and some wheels slipped out of the outer lane.  
![](img/off1.png) ![](img/off2.png)   
This can be improved by adding a 2nd PID controller for "throttle" as well.  

2. Instead of a heuristic approach, twiddle can be used to reach the optimum pid parameters. However, in this study, heuristic approach is choosen so as to demonstrate the effects of the different parameters.  

Enthusiasts can get better results by trying these two suggestions.  

### References  
Udacity's Q&A Session: https://www.youtube.com/watch?v=YamBuzDjrs8
https://en.wikipedia.org/wiki/PID_controller