# CarryME - Autonomous Self Balancing Robot 

![Untitled](https://user-images.githubusercontent.com/50642442/135082275-61aa604b-c46e-4383-885a-40e70179e4a5.jpg)

CarryME is a new smart self balancing cart that makes a quick grocery trip even quicker. With combination of self-balancing robot and computer vision algorithm the robot will be able to do the most important things, carry stuff securely and efficiently while tracking the user. 
CarryME is an autonomous robot system that includes several controllers and sensors which
detect the environment and help CarryMe to avoid obstacles and navigate autonomously.
The robot gets input from depth camera that can classify any object and it's distance from the robot.
The goal of this project is to design a two-wheel balancing robot controlled via computer vision algorithm enabling it to sense it's environment.

# Key Components

1.An Arduino Mega. Using the Mega because the project needs three hardware interrupt pins.

2.A Bosch BNO055 IMU. Gives ready-to-use tilt angle estimation as well as gyro and accelerometer raw readings.

3.An ODrive motor controller (56V). Enables very smooth current control for the two motors.

4.Bluetooth module HC-05 for the communication between the Raspberry and the Arduino.

5.Two hoverboard hub motors. Very affordable, powerful and easy-to-use motors.

6.Raspberry Pi 4 8G RAM, only for high level tasks.

7.Intel® RealSense™ depth camera for precision and clarity. This allows for High
 Definition 1080P captures, accurate depth sensing and fluid motion tracking, which enables 3D
 perception, robot mapping and obstacle avoidance.
 
8.5A DC-DC Step Down Adjustable Power Supply Module.The Raspberry Pi requires a 5V input power supply. To achieve this from a 36V battery, a
converter is needed.

9.Samsung Hoverboard Battery 36 V 4.4 AH. 

# BALANCING CONTROL 
PID
Balancing control is performed by a PID cascade, as showed in the next picture. 
This way it is possible to balance the robot even if you move the center mass or run it in a ramp. 
It will find a new balance angle that allows it to be balanced and stopped. 
In fact both the controllers are PI only, the derivative gain is set to 0 because it causes the robot to shake even with small gain.

![image](https://user-images.githubusercontent.com/50642442/134984028-422dd8f7-1184-43fa-a301-5dcd271db9a1.png)

PID implementation is as simple as this:

pTerm = Kp * error;

sum += error;

iTerm = Ki * sum*Ts;

dTerm = Kd * (error - lastError) / Ts;

Cn = pTerm + iTerm + dTerm;

lastError = error;

For PID tuning I used a Bluetooth module which allows to adjust Kp,Ki,Kd for both the controllers in real time. This way you can immediately view the effects and reach the desired behavior for the robot. In this video you can see it successfully balanced for the first time .

# Motion control
Moving forward and backward is quite easy with this PID cascade setup, you just have to give a set point to the first controller and it will calculate the appropriate leaning angle to reach that speed. Something like this:

setAngle = calcCn1(instSpeed - setSpeed);
  
instSpeed= calcCn2(angle - setAngle);

In order to turn the robot I'm attenuating the speed in one wheel, depending on the side it needs to turn. This way the robot keeps the balance as both wheels are reflecting the control system speed. Implementation looks like this:

 instSpeedL = instSpeedR = instSpeed;
 
 motorSpeedL(instSpeedL * factorL);
 
 motorSpeedR(instSpeedL * factorR); 
 
 0 ≤ factorL ≤ 1,     0 ≤ factorR ≤ 1  
 
To rotate on its axis, what I did is giving an opposite offset speed to the wheels. With the wheels rotating symmetric speeds it will perform a spin and stay balanced, completing the implementation it will look this way:

 motorSpeedL(instSpeedL * factorL + spinSpeed);
 
 motorSpeedR(instSpeedL * factorR - spinSpeed); 
 
 If spinSpeed is positive the robot will spin clockwise, otherwise it will spin counter clockwise.
 
 # Pose estimation - ML model
 
 ![image](https://user-images.githubusercontent.com/50642442/134986101-0bc2869a-da38-458b-af67-47df4e3b4681.png)
 
Pose estimation refers to computer vision techniques that detect human figures in images and videos. The pose estimation models take a processed camera image as the input and output information about keypoints. Using these deteceting points in the RGB image and depth image from RealSense camera, I was able to calculate the distance in centimeters to a 33 keypoints in the human body.
For the full guide on this section click [here](https://github.com/Razg93/Skeleton-Tracking-using-RealSense-depth-camera). 
 
