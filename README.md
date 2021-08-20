# Perception
Perception is the process by which robots map sensor measurements into internal representation of their environment. This internal representation is a task-oriented knowledge representation upon which to control, make decision and plan actions.
Ultrasonic sensor (distance): The ultrasonic sensor measures distance to an object up to a maximum of 255cm (or 100 inches) away. It does this by sending out high frequency sound waves that bounce off any object in range, and measuring how long it takes the sound to return to the sensor. In the software, you can select whether the distance is given in centimetres or inches. The ultrasonic sensor also has a “listen only” mode that can detect whether another robot is using an ultrasonic sensor nearby. In this mode, the sensor listens for signals but does not send them. Obstacle avoidance in mobile robots refers to a robot’s ability to autonomously adjust its path in response to objects in its environment. This is achieved either through proximal (reactive) or distal (rule-based) control. The difference between these two control paradigms can be subtle. Under proximal control, the robot’s sensors are tied directly to the motor controls and the motor speeds respond to the sensor input directly. One way to implement this is to create a weighted matrix that converts the sensor inputs into motor speeds. You can use a two-dimensional array with the number of rows corresponding to the number of distance sensors and the number of columns corresponding to the number of motors. For example, if we consider a two-wheel differential-wheels robot with eight sensors, then the array would have eight rows of two columns each. In this case each row can be thought of as a two item list specifying the left and right motor speeds respectively. This array is then used in the avoidance algorithm. The algorithm used as well as the values of the weighted matrix depends on the particular implementation that you are planning. The weights of the matrix are determined empirically. For example the Braitenberg controller implements the control algorithm below.
speed[i] += matrix[j][i] * (1.0 – (sensorsValue[j] / 512)); 

The speed[] array is a two item array representing the speed of the left and right motor, the matrix[] array holds the values in the weighted matrix, and the sensorsValue[] array holds the sensor values. The 512 normalizes the sensor values and the values i and j are values from a nested for loop that is used to iterate over the sensor values and weighted matrix [1]. Vision sensors use images captured by a camera to determine presence, orientation, and accuracy of parts.

1. Exploring Pioneer
Program an exploring robot capable of detecting and avoiding obstacles. Use pioneer-p3dx differential drive robot shown in Figure 1. The task is to program pioneer to wander around in its environment without colliding into obstacles. Each sensor detects and returns a 3-dimensional point in the sensor reference frame. Find a logical way to use measurements from all six sensors to decide in which direction to turn to avoid colliding into obstacles.
