# Control and Trajectory Tracking for Autonomous Vehicle
This is the project of the last course of [Udacity Self-Driving Car Engineer Nanodegree Program](https://github.com/udacity/nd013-c6-control-starter/tree/master)

# Project Overview
In this project, applying the skills to design a PID controller to perform vehicle trajectory tracking. Given a trajectory as an array of locations, and a simulation environment, design and code a PID controller and test its efficiency on the CARLA simulator used in the industry.

# Usage
The whole project is done in Udacity workspace with using the CARLA simulator and pygame engine provided.

Below are the steps on running the project:

- Clone the <a href="https://github.com/udacity/nd013-c6-control-starter/tree/master" target="_blank">repository</a>

    `git clone https://github.com/udacity/nd013-c6-control-starter.git`

Replace the files following before running the next steps:
1.  [main.cpp](/project/pid_controller/main.cpp)
2.  [pid_controller.cpp](/project/pid_controller/pid_controller.cpp)
3.  [pid_controller.h](/project/pid_controller/pid_controller.h)
4.  [simulatorAPI.py](/project/simulatorAPI.py)

## Run Carla Simulator

Open new window

* `su - student`
// Will say permission denied, ignore and continue
* `cd /opt/carla-simulator/`
* `SDL_VIDEODRIVER=offscreen ./CarlaUE4.sh -opengl`

## Compile and Run the Controller

Open new window

* `cd nd013-c6-control-starter/project`
* `./install-ubuntu.sh`
* `cd pid_controller/`
* `rm -rf rpclib`
* `git clone https://github.com/rpclib/rpclib.git`
* `cmake .`
* `make` (This last command compiles your c++ code, run it after every change in your code)

## Testing

To test your installation run the following commands. (Recommend to open a new window in order not to mess up with previous window when the code changes)

* `cd nd013-c6-control-starter/project`
* `./run_main_pid.sh`
// This will silently fail `ctrl + C` to stop
* `./run_main_pid.sh` (again)
Go to desktop mode to see CARLA

If error bind is already in use, or address already being used

* `ps -aux | grep carla`
* `kill id`

## Evaluate the Efficiency
After testing the project correctly, the values of the error and the pid command are saved in thottle_data.txt and steer_data.txt. Plot the saved values using the command (in project directory):

```
python3 plot_pid.py
```

If there are some library missing, a few additional python modules are needed to be installed: 

```
pip3 install pandas
pip3 install matplotlib
```

# Result(Questions Answering is Included)
## Add the plots to your report and explain them (describe what you see)
Here with the screenshot of the result video. By following the youtube [link](), a short video/demo taken from the simulator can be viewed. Also, the data of two plots, Throttle plot and Steering Plot, are extracted from this demo.
![Alt Text](/project/pid_controller/screenshot/screenshot.txt)

By using the command mentioned in [Evaluate the Efficiency](#evaluate-the-efficiency) section, the [steering plot]() and [throttle plot]() are generated.

**Steering Plot**
![Steering Plot]()

From the graph, there are two curves plotted which the blue curve shows the steering error and the orange curve shows the steering output. The steering error is the angle difference between the current steering and the desired steering suggested by the vectorial field. The vectorial field has two characteristics:
- Suggests the steering to move in the direction between first to last waypoint. (Average calculated)
- Suggests the steering compensation if the projected position of the car is in the left or right position in order to make the car move in the right position

Due to PID controller's natural characteristics, it is noted that the steering error is proportional to the steering output, but the derivative term prevent the proportinal overshooting by the car, and therefore, some disturbances or ups-and-downs curve are showing in the curve. This kind of scene is showing when the waypoints left behind the car and the car reaches offset (spiral becomes zero). The largest disturbance is showing in the graph where the iteration is close to 100, that is when the car is going to turn right and the car is totally reaching offset. After some compensating and overshooting, the car is able to control back and move back to its direction.

**Throttle Plot**
![Throttle Plot]()

From the graph, there are three curves plotted which the blue curve shows the throttle error, the green curve shows the throttle output and th orange curve shows the brake output. The throttle is the difference between current speed and the desired speed suggested by the vectorial field. 

Similar to the steering plot, throttle error is propotional to the throttle output and so as the derivative term is preventing the proportional overshooting by the car. From the values shown, the throttle output and the brake output are always positive, but the throttle error might be positive or negative. When throttle error is positive, the throttle output shows positive and the brake output is zero; whereas when throttle error is negative, the throttle output shows zero and the brake output is positive small value. 

## What is the effect of the PID according to the plots, how each part of the PID affects the control command?
As mentioned in the Steering Plot and Throttle Plot from the previous [section](#add-the-plots-to-your-report-and-explain-them-describe-what-you-see), due to PID controller's natural characteristics, it is noted that the steering error is proportional to the steering output, but the derivative term prevent the proportinal overshooting by the car, and so goes to throttle error and throttle output. The integral term is a very small value which close to zero value when there is no crashing or drifting happened. It will levitate when there are some happening to the car. 

## How would you design a way to automatically tune the PID parameters?
As in this project, all the parameters of the throttle and steering PID controller are tuned by try-and-error method. There are two problems that stopping the PID parameters from tuning automatically. The first problem maybe quite easier is to make a bash file to auto restarting the CARLA stimulator and returning the experiment to the starting point once the parameter has been tuned to find a best value. However, to solve the first problem, the second problem is needed to be solved first which is speeding the stimulator and the environment (crashing problem). On selecting the best PID parameter, obviously it is finding the smallest root-mean-square deviation(RMSE) of the parameters. In order to tune to find the best parameter, a feedback is needed from the output of the PID controller. However, PID controller itself is a open-loop system which means it is a one-way input-output controller and it does not return a feedback to the system. Therefore, by adding feedback and behaviour from the current output to the parameter selection, it may tune automatically to find the best parameter and stop the process of tuning. 

## PID controller is a model free controller, i.e. it does not use a model of the car. Could you explain the pros and cons of this type of controller?
**Pros**
1. PID controller itself has been created by certain formula and it is perceived. Therefore, it is no need to create again and it is just needed to be tuned.
2. Due to its simplicity, PID controller can solve most of the equations related to proportional, derivative and integral.

**Cons**
1. PID controller is only suitable for continuous problem, and therefore, it is not suitable to be used in discontinuous or singularity problems.
2. PID controller is needed to be tuned until getting the best result. (Wasting a lot of time)

## What would you do to improve the PID controller?


## Project Instructions

In the previous project you built a path planner for the autonomous vehicle. Now you will build the steer and throttle controller so that the car follows the trajectory.

You will design and run the a PID controller as described in the previous course.

In the directory [/pid_controller](https://github.com/udacity/nd013-c6-control-starter/tree/master/project/pid_controller)  you will find the files [pid_controller.cpp](https://github.com/udacity/nd013-c6-control-starter/blob/master/project/pid_controller/pid_controller.cpp)  and [pid_controller.h](https://github.com/udacity/nd013-c6-control-starter/blob/master/project/pid_controller/pid_controller.h). This is where you will code your pid controller.
The function pid is called in [main.cpp](https://github.com/udacity/nd013-c6-control-starter/blob/master/project/pid_controller/main.cpp).

### Step 1: Build the PID controller object
Complete the TODO in the [pid_controller.h](https://github.com/udacity/nd013-c6-control-starter/blob/master/project/pid_controller/pid_controller.h) and [pid_controller.cpp](https://github.com/udacity/nd013-c6-control-starter/blob/master/project/pid_controller/pid_controller.cpp).

Run the simulator and see in the desktop mode the car in the CARLA simulator. Take a screenshot and add it to your report. The car should not move in the simulation.
### Step 2: PID controller for throttle:
1) In [main.cpp](https://github.com/udacity/nd013-c6-control-starter/blob/master/project/pid_controller/main.cpp), complete the TODO (step 2) to compute the error for the throttle pid. The error is the speed difference between the actual speed and the desired speed.

Useful variables:
- The last point of **v_points** vector contains the velocity computed by the path planner.
- **velocity** contains the actual velocity.
- The output of the controller should be inside [-1, 1].

2) Comment your code to explain why did you computed the error this way.

3) Tune the parameters of the pid until you get satisfying results (a perfect trajectory is not expected).

### Step 3: PID controller for steer:
1) In [main.cpp](https://github.com/udacity/nd013-c6-control-starter/blob/master/project/pid_controller/main.cpp), complete the TODO (step 3) to compute the error for the steer pid. The error is the angle difference between the actual steer and the desired steer to reach the planned position.

Useful variables:
- The variable **y_points** and **x_point** gives the desired trajectory planned by the path_planner.
- **yaw** gives the actual rotational angle of the car.
- The output of the controller should be inside [-1.2, 1.2].
- If needed, the position of the car is stored in the variables **x_position**, **y_position** and **z_position**

2) Comment your code to explain why did you computed the error this way.

3) Tune the parameters of the pid until you get satisfying results (a perfect trajectory is not expected).

### Step 4: Evaluate the PID efficiency
The values of the error and the pid command are saved in thottle_data.txt and steer_data.txt.
Plot the saved values using the command (in nd013-c6-control-refresh/project):

```
python3 plot_pid.py
```

You might need to install a few additional python modules: 

```
pip3 install pandas
pip3 install matplotlib
```

Answer the following questions:
- Add the plots to your report and explain them (describe what you see)
- What is the effect of the PID according to the plots, how each part of the PID affects the control command?
- How would you design a way to automatically tune the PID parameters?
- PID controller is a model free controller, i.e. it does not use a model of the car. Could you explain the pros and cons of this type of controller?
- (Optional) What would you do to improve the PID controller?


### Tips:

- When you wil be testing your c++ code, restart the Carla simulator to remove the former car from the simulation.
- If the simulation freezes on the desktop mode but is still running on the terminal, close the desktop and restart it.
- When you will be tuning the PID parameters, try between those values:

