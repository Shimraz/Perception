# Perception
Perception is the process by which robots map sensor measurements into internal representation of their environment. This internal representation is a task-oriented knowledge representation upon which to control, make decision and plan actions.
Ultrasonic sensor (distance): The ultrasonic sensor measures distance to an object up to a maximum of 255cm (or 100 inches) away. It does this by sending out high frequency sound waves that bounce off any object in range, and measuring how long it takes the sound to return to the sensor. In the software, you can select whether the distance is given in centimetres or inches. The ultrasonic sensor also has a “listen only” mode that can detect whether another robot is using an ultrasonic sensor nearby. In this mode, the sensor listens for signals but does not send them. Obstacle avoidance in mobile robots refers to a robot’s ability to autonomously adjust its path in response to objects in its environment. This is achieved either through proximal (reactive) or distal (rule-based) control. The difference between these two control paradigms can be subtle. Under proximal control, the robot’s sensors are tied directly to the motor controls and the motor speeds respond to the sensor input directly. One way to implement this is to create a weighted matrix that converts the sensor inputs into motor speeds. You can use a two-dimensional array with the number of rows corresponding to the number of distance sensors and the number of columns corresponding to the number of motors. For example, if we consider a two-wheel differential-wheels robot with eight sensors, then the array would have eight rows of two columns each. In this case each row can be thought of as a two item list specifying the left and right motor speeds respectively. This array is then used in the avoidance algorithm. The algorithm used as well as the values of the weighted matrix depends on the particular implementation that you are planning. The weights of the matrix are determined empirically. For example the Braitenberg controller implements the control algorithm below.
speed[i] += matrix[j][i] * (1.0 – (sensorsValue[j] / 512)); 

The speed[] array is a two item array representing the speed of the left and right motor, the matrix[] array holds the values in the weighted matrix, and the sensorsValue[] array holds the sensor values. The 512 normalizes the sensor values and the values i and j are values from a nested for loop that is used to iterate over the sensor values and weighted matrix [1]. Vision sensors use images captured by a camera to determine presence, orientation, and accuracy of parts.

1. Exploring Pioneer
Program an exploring robot capable of detecting and avoiding obstacles. Use pioneer-p3dx differential drive robot shown in Figure 1. The task is to program pioneer to wander around in its environment without colliding into obstacles. Each sensor detects and returns a 3-dimensional point in the sensor reference frame. Find a logical way to use measurements from all six sensors to decide in which direction to turn to avoid colliding into obstacles.
![image](https://user-images.githubusercontent.com/43060427/130222274-5acc6ca5-8f5c-44c9-b5b3-ae9aa8841475.png)
Robot in the environment is show in Figure 2.
Here the range finder sensor used is ultrasonic sensor. Six such sensors are kept at exact position to find the obstacles to some particular angles.
![image](https://user-images.githubusercontent.com/43060427/130222298-f9c2aa5e-a8f5-47e4-963e-8e6c9ef5f79f.png)
The code uses Braitenberg method [1] to find the obstacles and the main program
is as follows (The whole program can be find in the attached matlab file):
![image](https://user-images.githubusercontent.com/43060427/130222329-7d3e4f73-c854-457b-95c0-93dc7bbdcc8e.png)
The robot position for non real time values is obtained by running the program and
plotting the position against time axis in command window using
plot(positionX,positionY).
The code for which is given in the same file.

2. Tracking Pioneer
Us the camera system to track pioneer as it explores the environment. For
tracking, the robot is fitted with a green sphere that is detected as a green
blob in the camera image. Read the image as a grayscale image to minimize
the amount to data to transfer between matlab and v-rep. The camera has a
special filter that allow it to detect only green pixels above a specified
threshold. Therefore, the resulting image will have zero intensity except at the
sphere. Use the information to develop a tracker (controllers) that keeps the
sphere in the centre of the camera image plane.
To solve this the camera’s yaw and pitch motor is initialized and PID controller is
used to control the 2 motors.
The code for this can be found in the attached Matlab file named
Tracking_Pioneer_p3dx.m and the camera used here is of 32*32 pixels resolution.
![image](https://user-images.githubusercontent.com/43060427/130222396-519b3dd4-7eca-4cfb-b0bb-1b3f3074ad2a.png)

Figure 4 Tracked pioneer

![image](https://user-images.githubusercontent.com/43060427/130222407-368484a2-381a-44a1-8fb8-e4ad155d5860.png)
The robot position for non real time values is obtained by running the program and plotting the position against time axis in command window using plot(positionX,positionY).
The code for which is given in the same file.


3. Leader Pioneer
This task requires you to implement a leader-follower multi-robot
system―the leader pioneer is installed with a skirt of ultrasonic sensor (as in
subtask 2) with which is detects and avoids obstacles while exploring the
environment. The second pioneer is fitted with an RGB-D (RGB-Depth)
sensor with which it should search pioneer one and track it at a distance of 1
m (between camera centre and green sphere).
![image](https://user-images.githubusercontent.com/43060427/130222447-79f0eff0-abe9-4dbe-8a08-43dd8498eff5.png)

Figure 6 Leader-follower robots in their environment
Here the rgbd camera is taken as a object and the output from that is given to the
controller to run the follower motor.
The code for this can be found in the attached file.
![image](https://user-images.githubusercontent.com/43060427/130222463-d0ab685b-ef90-4c11-ad52-429378d42f11.png)
Figure 7 Leader and follower robot position from the robot object handle
The robot position for non real time values is obtained by running the program and plotting the position against time axis in command window using plot(positionX,positionY).
The code for which is given in the same file.
