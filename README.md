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

By using the command mentioned in [Evaluate the Efficiency](#evaluate-the-efficiency) section, the [steering plot](/project/pid_controller/screenshot/steering_plot.png) and [throttle plot](/project/pid_controller/screenshot/throttle_plot.png) are generated.

**Steering Plot**

![Steering Plot](/project/pid_controller/screenshot/steering_plot.png)

From the graph, there are two curves plotted which the blue curve shows the steering error and the orange curve shows the steering output. The steering error is the angle difference between the current steering and the desired steering suggested by the vectorial field. The vectorial field has two characteristics:
- Suggests the steering to move in the direction between first to last waypoint. (Average calculated)
- Suggests the steering compensation if the projected position of the car is in the left or right position in order to make the car move in the right position

Due to PID controller's natural characteristics, it is noted that the steering error is proportional to the steering output, but the derivative term prevent the proportinal overshooting by the car, and therefore, some disturbances or ups-and-downs curve are showing in the curve. This kind of scene is showing when the waypoints left behind the car and the car reaches offset (spiral becomes zero). The largest disturbance is showing in the graph where the iteration is close to 100, that is when the car is going to turn right and the car is totally reaching offset. After some compensating and overshooting, the car is able to control back and move back to its direction.

**Throttle Plot**

![Throttle Plot](/project/pid_controller/screenshot/throttle_plot.png)

From the graph, there are three curves plotted which the blue curve shows the throttle error, the green curve shows the throttle output and th orange curve shows the brake output. The throttle is the difference between current speed and the desired speed suggested by the vectorial field. 

Similar to the steering plot, throttle error is propotional to the throttle output and so as the derivative term is preventing the proportional overshooting by the car. From the values shown, the throttle output and the brake output are always positive, but the throttle error might be positive or negative. When throttle error is positive, the throttle output shows positive and the brake output is zero; whereas when throttle error is negative, the throttle output shows zero and the brake output is positive small value. 

## What is the effect of the PID according to the plots, how each part of the PID affects the control command?
As mentioned in the Steering Plot and Throttle Plot from the previous [section](#add-the-plots-to-your-report-and-explain-them-describe-what-you-see), due to PID controller's natural characteristics, it is noted that the steering error is proportional to the steering output, but the derivative term prevent the proportinal overshooting by the car, and so goes to throttle error and throttle output. The integral term is a very small value which close to zero value when there is no crashing or drifting happened. It will levitate when there are some happening to the car. 

## How would you design a way to automatically tune the PID parameters?
As in this project, all the parameters of the throttle and steering PID controller are tuned by try-and-error method. There are two problems that stopping the PID parameters from tuning automatically. The first problem maybe quite easier is to make a bash file to auto restarting the CARLA stimulator and returning the experiment to the starting point once the parameter has been tuned to find a best value. However, to solve the first problem, the second problem is needed to be solved first which is speeding the stimulator and the environment (crashing problem). On selecting the best PID parameter, obviously it is finding the smallest root-mean-square deviation(RMSE) of the parameters. In order to tune to find the best parameter, a feedback is needed from the output of the PID controller. However, PID controller used in this project is a open-loop system (perhaps it is a meta version) which means it is a one-way input-output controller and it does not return a feedback to the system. Therefore, by adding feedback and behaviour from the current output to the parameter selection, it may tune automatically to find the best parameter and stop the process of tuning. 

## PID controller is a model free controller, i.e. it does not use a model of the car. Could you explain the pros and cons of this type of controller?
**Pros**
1. PID controller itself has been created by certain formula and it is perceived. Therefore, it is no need to create again and it is just needed to be tuned.
2. Due to its simplicity, PID controller can solve most of the equations related to proportional, derivative and integral.

**Cons**
1. PID controller is only suitable for continuous problem, and therefore, it is not suitable to be used in discontinuous or singularity problems.
2. PID controller is needed to be tuned until getting the best result. (Wasting a lot of time)

## What would you do to improve the PID controller?
At first in this project, the original PID controller is used to carry out in the simulator. However, the parameters are very hard to tune to meet the requirements or satisfication. Therefore, two ways of PID controller is used to calculate the throttle error and steering error by two different vectorial fields. Instead of directly calculating the error by using the beginning and the final waypoints to to get the desired speed and angle, an average of waypoints is calculated and used to calculate the desired speed and angle for a short distance. This is to get the control of speed and angle easily to avoid drifting or overturning due to some circumstances such as turning right/left and after avoiding obstacles. Although this can be avoided by adjusting the threshold of steering, but the errors calculated with just one PID are not enough to avoid such problems occuring with keep tuning the parameters. 

# Bugs/Future Improvement
During project, the stimulation is too slow and keep crashing. This may bring a lot of annoying and dissatisfying while studying or tuning the parameters to get the satisfying result. As mentioned before, if the stimulation can be improved to become faster(with high FPS and fast calculation), the parameter tuning process can be automatically done, but not with try-and-error method. Besides, there is no map or final destination for the stimulation running. Although the stimulator can keep going and running as usual (if there is no crashing), but there is no ending point (perhaps that is the project's requirement, but it doesn't mention). Besides, the tuned parameters cannot get the same result as always because the desired speed and angle calculated everytime is different even though the obstacles and the environment are same. Therefore, if the calculated desired speed and angle can be stored and used to calculate the current values by minimizing the difference of the values, perhaps the parameter tuning process could be faster and the results are not so big different at every time.

Due to the limitation of time and basic learning of PID controller, the project is using the simple PID to control the speed and steering angle of the car in the stimulator. This PID is not so suitable to be used in the vehicle to control such important aspects. As mentioned before, this PID controller is an open-loop system which it is not suitable to be used if applying on the autonomous drving system becauses a closed-loop system can provide the feedback from outcome in order to improve its accuracy, but an open-loop system cannot. Perhaps in the future, there is a closed-loop PID controller being introduced by Udacity.

# Conclusion
Through this project, a simple PID controller is learnt to calculate the throttle error and steering error in order to control the movement of the vehicle where the braking, speeding and steering turning. The parameters of the PID are needed to be tuned to get the satisfying result. However, due to some issues persisting, there are some improvement that needed to be fixed in the future. 

