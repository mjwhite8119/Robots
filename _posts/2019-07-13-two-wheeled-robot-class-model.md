---
layout: post
title:  "Two-wheeled robot class model"
date:   2019-07-13 15:35:44 -0700
categories: Projects
permalink: /two-wheeled-robot-model
---
This project will describe the programming for a two-wheeled differential drive robot.  The project will make use of the Robot Operating System (ROS) so the code examples will detail how this can be used.  There are two functionality goals for this project.  The first is to get it move forward a specified distance, and the second is to have it turn on the spot a certain number of degrees.  Although these may seem like simple tasks it does present some programming challenges requiring control feedback loops and PID controllers.  The plan is to start with a basic programming model and build it up into a fully autonomous mobile robot. 

I’ll start by building a program model of a two-wheeled robot to implement its internal functionality, such as driving the wheels and counting encoder pulses.  This model won’t include any functionality to control how it gets to its destination.  That will be the job of the control loops described later.  In the first stage only the drive train is implemented for mobility.  Other functionality such as laser and ultrasonic range finders will be added later.  The programming model is written using C++ classes but you should be able to translate it into Python, which is the other language widely used with ROS.  Here’s a block diagram of the model:

![Robots]({{site.url}}{{site.baseurl}}/assets/images/Two-Wheeled-Robot-Slides.011.jpeg)

After detailing the two-wheeled robot model I’ll go on to describe the control loops that are used to control the speed, position and orientation of the robot.  The position and orientation is known in the ROS world as its “pose”.  The outer loop will simply wait to receive a move command that it sends to a control loop to change the robot’s pose.

![/Robots]({{site.url}}{{site.baseurl}}/assets/images/Two-Wheeled-Robot-Slides.003.jpeg)

The pose control loop tells the robot how fast it should spin each wheel and this will cascade down to the speed control loop that determines how much power to apply.  One of the challenges is to get the robot to go in a straight line.  This is not an easy as you may think unless the two motors just happen to be perfect clones of each other.  In reality this is never the case, so feedback control must be used to compensate for this.  Once the motors are spinning and the robot is moving its position and orientation is tracked so as it can be stopped after it has travelled a certain distance.

![/Robots]({{site.url}}{{site.baseurl}}/assets/images/Two-Wheeled-Robot-Slides.005.jpeg)

Wheeled Robot Kinematics

When working with robots the word kinematics comes up a lot.  Kinematics simply describes how an object moves.  This project will only look at the internal kinematics of the robot, which describes the relationship between its wheel rotation and how it moves.   For internal kinematics the robot will move within a local reference frame, as shown in the following diagram.

![/Robots]({{site.url}}{{site.baseurl}}/assets/images/Control-Theory-Slides.009.jpeg)

Note that since the robot has standard fixed wheels that it cannot move sideways.  Therefore, the value of the Y component in the internal reference frame will always be zero.  Contrast this to external kinematics that describes the robot’s position and orientation relative to a point in a global reference frame, such as a room.  In this case, the Y component can be non-zero since we can drive the robot to any location in the room using a series of discrete maneuvers.

![/Robots]({{site.url}}{{site.baseurl}}/assets/images/Control-Theory-Slides.012.jpeg)

Hardware Architecture

The robot has a motor attached to each wheel and is balanced with wheel casters.  Each motor has an attached encoder.  The compute architecture is very simple at this stage.  It uses a single ESP32 micro controller for both the robot’s internal functionality, such as the actuation of its motors, and also its external control where we drive it towards a new position and orientation.  Here’s a picture of the rig that I’m using but the program should work with any two-wheeled differential drive robot.

![Robot image]({{site.url}}{{site.baseurl}}/assets/images/Project1Robot.JPG)

Program Setup

Before we get into the details of the two-wheeled robot model I need to go over the program setup.  I included this in an earlier post but I wanted to keep it all in one place.  I have ROS installed on a Raspberry Pi with the Ubuntu Mate OS.  All of the development is done within the Arduino IDE which includes the ROS library for Arduino plus some ESP32 specific libraries.  There are several online tutorials that show you how to setup the Arduino IDE for ROS.  In order to connect to ROS you’ll need to specify the network address of the machine on which the ROS server is hosted (the Raspberry Pi in my case) together with the port that it’s listening on.  You also have to create a node handle to manage the connection between your control program and the ROS server.

{% highlight cpp %}
#include <ros.h> 

// RaspberryPi host address. Default ROS port is 11411
IPAddress server(192,168,0,ROS_HOST);      // Set the rosserial socket server IP address
const uint16_t serverPort = ROS_PORT;    // Set the rosserial socket server port

// Define the node handle to roscore
ros::NodeHandle nh;
bool nodeHandleCreated = false; 
{% endhighlight %}

You then include the ROS message type Pose. The format of the Pose message looks like this:

![Robot image]({{site.url}}{{site.baseurl}}/assets/images/Pose_message_type.png)

It has two sets of parameters for passing position and orientation.  For a wheeled robot you only need to use three of these parameters: position x, y, and orientation y.  This will form the command vector for requesting a new reference pose.  The reference pose is the final position and orientation that you want the robot to end up in.  The pose error vector will hold the difference between the robot’s current pose and its reference pose.

{% highlight cpp %}
// Include the Pose message
#include <geometry_msgs/Pose.h>

// Define a variable to hold the Pose message
geometry_msgs::Pose pose_msg;

// Subscriber definition
ros::Subscriber<geometry_msgs::Pose> *sub_pose;

// Create matrix to hold the command input
// X position, Y position, Orientation
float refPose[3] = {0.0, 0.0, 0.0};
const byte X_POS = 0;
const byte Y_POS = 1;
const byte PSI = 2;

// Error matrix for feedback loop
float poseError[3] = {0.0, 0.0, 0.0};

// Set callback flag

bool commandCalled = false;
{% endhighlight %}

In the setup() you first connect to WiFi. The way in which you do this is dependent on the micro controller that you’re using so I just put a function here. You then initialize and start a node handle that allows you to communicate with the ROS server. The ROS server is a publish/subscribe protocol (basically the ROS version of MQTT). It has a very rich collection of data structures, called messages, that you can use within topics. The Pose message that I described earlier is just one of these message types. A subscribed topic holds the message that your program is interested in. When you subscribe to a topic you need to provide a callback function to process the received message. In my case, the message is a command to move the robot to a new position and orientation. After subscribing to the pose command topic I create an instance of a two-wheeled robot. This is a C++ model that implements all of the functionality of the robot. The code I’m showing here is just the program to control the robot. The model is a work-in-progress and I intend to build on it over time to make it a fully autonomous wheeled robot. Right now it just implements the code for motion control.

{% highlight cpp %}
//--------------------------------------------//
// Setup
//--------------------------------------------//
void setup() {

// Connect to WiFi
connectWiFi();

// Initialize the ROS nodehandle.
nh.getHardware()->setConnection(server, serverPort);

// Starting node
nh.initNode();
nodeHandleCreated = true;
delay(1000);

// Setup subscriber with callback function to receive the command
sub_pose = new ros::Subscriber<geometry_msgs::Pose>("/pose_cmd", commandCb);
nh.subscribe(*sub_pose);

// Create a robot. Pass the ROS node handle.
robot = new TwoWheeledRobot(&nh);
}
{% endhighlight %}

The callback function receives a message to move the robot to a new position and orientation. I’ll go over the callback function in a later article when I document the control loops.

TwoWheeledRobot Class

The model uses C++ classes to represent all of the robot’s components.  The following diagram shows how all of these components relate to each other.  The two-wheeled robot has a differential drive system, meaning that it has one motor for each wheel.  Each of the motors has an optical encoder attached.  This is all neatly wrapped up in a drive train.  The front and back wheel casters are also represented, but at this stage they don’t serve any function from a programming perspective.

![Robots]({{site.url}}{{site.baseurl}}/assets/images/Two-Wheeled-Robot-Slides.011.jpeg)

The drive train is used to encapsulate all of the functionality that makes the robot move around.  At this stage that’s pretty much all of the robot functionality but we can build on this to add new functionality as the project progresses, such as laser and sonar range finders. 

Looking at the code.  The TwoWheeledRobot constructor takes in the node handle that’s used to communicate with ROS.  The geometry of the robot is defined, which will be useful when we want to avoid obstacles later on.  We then attach the drive train that will encapsulate all of our mobility functionality.  Finally, the front and back caster wheels are added.  The Wheel class takes in the wheel diameter that may not be particularly useful for the caster but will be a crucial part of the odometry when it comes to the powered wheels.  The wheel type is also passed in, which is not used right now but I plan to use it later to configure the wheel geometry.

{% highlight cpp %}
class TwoWheeledRobot
{
public:

// --- Constructor ---
TwoWheeledRobot(ros::NodeHandle * nodeHandle);

// --- Robot configuration ---

// Define the shape and size of the robot
struct Geometry {
uint8_t type = CYLINDER;
float radius = 0.11;
float length = 0.20;
};

// Drive train of robot
DriveTrain driveTrain;

// Unmotorized wheels
const float casterDiameter = 0.020; // caster diameter in meters 
Wheel frontCaster = Wheel(casterDiameter, CASTER);
Wheel backCaster = Wheel(casterDiameter, CASTER); 
};
{% endhighlight %}

The constructor just takes in the ROS node handle for use in publishing the robot’s odometry.

{% highlight cpp %}
// ----------------------------- Constructor ---------------------
TwoWheeledRobot::TwoWheeledRobot(ros::NodeHandle * nodeHandle)
:driveTrain(nodeHandle)
{ 
}
{% endhighlight %}
DriveTrain Class

The DriveTrain class encapsulates all of the robot’s mobility functionality. The following diagram shows the drive train configuration parameters on the left and its control functions on the right.

![Robots]({{site.url}}{{site.baseurl}}/assets/images/Two-Wheeled-Robot-Slides.013.jpeg)
The first thing to explain is the motor pinGroup.  The pinGroup is a structure of GPIO pins that are used to attach the motors and encoders to the micro controller.  There are three pins for the motor, two to control the direction and one for the PWM signal that applies the power.  The encoder A and B pins are used to control a quadrature encoder.  The pins are grouped so as we can easily reference them together depending on the motor we are controlling.   I have this structure in a configuration file to extract it from the DriveTrain class.

{% highlight cpp %}
// Define the GPIO pins for the motors
static struct DRAM_ATTR MotorPins {
const byte motorDir1; // motor direction pin1
const byte motorDir2; // motor direction pin2
const byte enable; // Enable PMW 
const byte encoderA; // encoder channel A
const byte encoderB; // encoder channel B
} motorPinGroup[2] = {27, 26, 25, 36, 37, 
14, 12, 13, 38, 39};
{% endhighlight %}

Since the DriveTrain class has two wheels I assign a pinGroup number to the left and right wheels. The wheel diameter and type is also defined, which we pass to the Wheel class constructor to create each wheel. We also want to know how far apart the wheels are since this is critical to calculating path trajectory.

{% highlight cpp %}
class DriveTrain
{
public:

// Constructor
DriveTrain(ros::NodeHandle * nodeHandle);

uint8_t leftWheelPinGroup = 0; // GPIO pin group config.h
uint8_t rightWheelPinGroup = 1; // GPIO pin group config.h

const float wheelDiameter = 0.063; // wheel diameter in meters
const uint8_t wheelType = STANDARD_FIXED;
const float wheelSeparation = 0.179; // wheel separation in meters

Wheel leftWheel = Wheel(leftWheelPinGroup, wheelDiameter, wheelType);
Wheel rightWheel = Wheel(rightWheelPinGroup, wheelDiameter, wheelType);
{% endhighlight %}

The DriveTrain is responsible for keeping track of the current position and orientation state of the robot. The ROS message type Odometry is used for this purpose. The node handle is used to connect to the ROS server so as we can publish the current state of the robot. This is used by the control loop that drives the robot to its requested pose.

{% highlight cpp %}
// Pointer to the ROS node
ros::NodeHandle * nh_; 

// Robot state 
nav_msgs::Odometry state;

// Define publisher for ROS
ros::Publisher* pub_odom;
{% endhighlight %}

The constructor initializes the robot’s state variables that track the robot’s position, orientation, and velocity. The publisher is also initialized to write the Odometry state out to the ROS server. Finally a periodic timer is started to update the robot’s state every 50 milliseconds.

{% highlight cpp %}
// ------------------ Constructor ---------------------------------
DriveTrain::DriveTrain(ros::NodeHandle * nodeHandle) 
:nh_(nodeHandle)
{ 
// Attach robot to the global odometry frame
state.header.frame_id = "/odom"; // Global odometry frame
state.child_frame_id = "/base_link"; // Robot local frame

// Setup publisher to report current state
pub_odom = new ros::Publisher("/odom", &state);
nh_->advertise(*pub_odom);

// Start state update timer
const esp_timer_create_args_t periodic_timer_args = {.callback = &updateStateISR};
esp_timer_create(&periodic_timer_args, &stateUpdateTimer);
esp_timer_start_periodic(stateUpdateTimer, updatePeriodMicros); // Time in milliseconds (50) 
instances[0] = this; 
}
{% endhighlight %}
