---
layout: post
title:  "Two-wheeled robot class model - part 1"
excerpt: "This project will describe the programming class model for a two-wheeled differential drive robot.  This first part will describe the overall model hierachy and the drive train."
date:   2019-07-12 15:35:44 -0700
category: Projects
permalink: /twr-model-part1
---
This project will describe the programming for a two-wheeled differential drive robot.  There are two functionality goals for this project.  The first is to get it move forward a specified distance, and the second is to have it turn on the spot a certain number of degrees.  Although these may seem like simple tasks it does present some programming challenges requiring control feedback loops and PID controllers.  The plan is to start with a basic programming model and build it up into a fully autonomous mobile robot.  The project will make use of the *Robot Operating System* (ROS) so the code examples will detail how this can be used.  However, the model does not require *ROS* and is still perfectly functional without it.  You would however, need to implement another method of communication such as *BlueTooth*.

I’ll start by building a program model of a two-wheeled robot to implement its internal functionality, such as driving the wheels and counting encoder pulses.  This model won’t include any functionality to control how it gets to its destination.  That will be the job of the control loops described later.  In the first stage only the drive train is implemented for mobility.  Other functionality such as laser and ultrasonic range finders will be added later.  The programming model is written using C++ classes but you should be able to translate it into Python, which is the other language widely used in robotics.  Here’s a block diagram of the model:

![Robots]({{site.url}}{{site.baseurl}}/assets/images/Two-Wheeled-Robot-Slides.011.jpeg)

After detailing the two-wheeled robot model I’ll go on to describe the control loops that are used to control the speed, position and orientation of the robot.  The position and orientation is known as its “pose”.  The outer loop will simply wait to receive a move command that it sends to a control loop to change the robot’s pose.

![/Robots]({{site.url}}{{site.baseurl}}/assets/images/Two-Wheeled-Robot-Slides.003.jpeg)

The pose control loop tells the robot how fast it should spin each wheel and this will cascade down to the speed control loop that determines how much power to apply.  One of the challenges is to get the robot to go in a straight line.  This is not an easy as you may think unless the two motors just happen to be perfect clones of each other.  In reality this is never the case, so feedback control must be used to compensate for this.  Once the motors are spinning and the robot is moving its position and orientation is tracked so as it can be stopped after it has travelled a certain distance.

![/Robots]({{site.url}}{{site.baseurl}}/assets/images/Two-Wheeled-Robot-Slides.005.jpeg)

### Wheeled Robot Kinematics

When working with robots the word kinematics comes up a lot.  Kinematics simply describes how an object moves.  This project will only look at the internal kinematics of the robot, which describes the relationship between its wheel rotation and how it moves.   For internal kinematics the robot will move within a local reference frame, as shown in the following diagram.

![/Robots]({{site.url}}{{site.baseurl}}/assets/images/Control-Theory-Slides.009.jpeg)

Note that since the robot has standard fixed wheels that it cannot move sideways.  Therefore, the value of the Y component in the internal reference frame will always be zero.  Contrast this to external kinematics that describes the robot’s position and orientation relative to a point in a global reference frame, such as a room.  In this case, the Y component can be non-zero since we can drive the robot to any location in the room using a series of discrete maneuvers.

![/Robots]({{site.url}}{{site.baseurl}}/assets/images/Control-Theory-Slides.012.jpeg)

### Hardware Architecture

The robot has a motor attached to each wheel and is balanced with wheel casters.  Each motor has an attached encoder.  The compute architecture is very simple at this stage.  It uses a single ESP32 micro controller for both the robot’s internal functionality, such as the actuation of its motors, and also its external control where we drive it towards a new position and orientation.  Here’s a picture of the rig that I’m using but the program should work with any two-wheeled differential drive robot.

![Robot image]({{site.url}}{{site.baseurl}}/assets/images/Project1Robot.JPG)

### Program Setup

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

### TwoWheeledRobot Class

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
### DriveTrain Class

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

The main tasks of the drive train is to set the speed of its wheels, and to report out its current pose state.  For wheeled robots this is commonly referred to as odometry.  The pose state is updated 20 times a second using a timer interrupt routine.  Functions to get the local pose and velocity are pretty self explanatory.  They simply return the vectors for those states.  I’ll start with the wheel speed control.

{% highlight cpp %}
//--- Drive train functions ---
void setWheelSpeeds(float leftWheelSpeed, float rightWheelSpeed);

void publishState();

float * getLocalPose();

float * getLocalVelocity();

void printLocalPose();

private:
  // --- Odometry state variables and methods ---

  // Keeps track of encoder pulses from each wheel
  uint32_t leftPositionLast_ = 0, rightPositionLast_ = 0;
  const int maxPulsesPerSecond_ = 600;

  static DriveTrain * instances [1];

  // Static instance to update robot state
  static void updateStateISR(void *pArg)
  {
    if (DriveTrain::instances[0] != NULL)
      DriveTrain::instances[0]->updateState_();
  }

  // Instance member to update robot state. Called from updateStateISR
  void updateState_(); 
};
{% endhighlight %}

### Wheel Speed Control

Setting the wheel speed is undoubtedly the most important control function for the DriveTrain.  The wheel speed is passed in as a ratio between -1 and +1, where a negative value would send the robot backwards and a positive value moves it forward.  Zero would stop the motor.  The motor however, only understands pulses per second for its control parameter, so we have to translate from the speed ratio to a pulse per second rate.  For example, if our maximum pulses per second is say 600 then that would represent a speed ratio of 1.0 forward or -1.0 backwards.  To reduce the speed we send in a fractional value, 0.5 for example.  This would cut our pulses per second down to 300.  This should all make more sense when we look at the motor and encoder models.

{% highlight cpp %}
// -----------------------------------------------------------------
// Set the left and right wheel speeds 
// Input is a speed ratio between -1.0 (backwards) and 1.0 (forward) 
// Output is a wheel speed in pulses per/sec
// -----------------------------------------------------------------
void DriveTrain::setWheelSpeeds(float leftWheelSpeed, float rightWheelSpeed) {

  // Translate the speed ratio into pulses per/sec.
  // Where speed ratio of 1.0 equals 600 pulses per/sec
  int leftPulseSetpoint = int(maxPulsesPerSecond_ * leftWheelSpeed);
  int rightPulseSetpoint = int(maxPulsesPerSecond_ * rightWheelSpeed);
  
  leftWheel.setSpeed(leftPulseSetpoint); // Pulses per/sec
  rightWheel.setSpeed(rightPulseSetpoint); // Pulses per/sec 
} 
{% endhighlight %}

### Odometry

Another function of the DriveTrain is to continuously update the robot’s odometry state and publish it to the outside world.  For this we use the ROS Odometry message type which is shown in the following diagram.  The Pose part of the message is the same as we saw earlier.  What’s been added is the Twist message that holds the linear and angular velocity.  The child_frame_id points to the robot itself and is usually called the base_link.  This is connected to the Odometry frame via the header.  This allows us to track the robot within a global frame.

![Robots]({{site.url}}{{site.baseurl}}/assets/images/ROS-Slides.002.jpeg)

The following diagram shows the difference between the local (internal kinematic) frame and the global (external kinematic) frame, referred to as the Odometry frame when dealing with wheeled robots.  Notice that the symbol for orientation has change from ψ (psi) to θ (theta) when we go from the local frame to the global frame.

![/Robots]({{site.url}}{{site.baseurl}}/assets/images/Control-Theory-Slides.012.jpeg)

The following code shows how the Odometry message is published to ROS for a wheeled robot.  Not all of the data variables are used since we’re only working on a 2D plane.  If this were a drone then all of the variables would be used.  A project note here is that at this stage I’m reporting out the robot’s local pose instead of its global pose.  For an Odometry message this should actually be tracking the X, Y, theta location within a workspace (your living room).  I’ll switch to the global frame later in the project.  See the next section for more on this.

{% highlight cpp %}
// ------------------------------------------------------
// Publish current robot state to ROS
// ------------------------------------------------------
void DriveTrain::publishState() { 
  
  // Add the timestamp
  state.header.stamp = nh_->now(); 

  state.pose.pose.position.x = local_pose[X_POS];
  state.pose.pose.position.y = local_pose[Y_POS]; // zero since cannot move sideways
  state.pose.pose.orientation.y = local_pose[PSI]; // theta

  state.twist.twist.linear.x = local_velocity[X_VELOCITY];
  state.twist.twist.linear.y = local_velocity[Y_VELOCITY];  // Can't move instantaniously sideways
  state.twist.twist.angular.y = local_velocity[Y_ANGULAR]; // angular yaw velocity in radians

  // Publish
  pub_odom->publish( &state );
} 
{% endhighlight %}

Updating the state requires some understanding of wheeled robot kinematics.

### Internal Kinematics

Internal kinematics describes the relationship between a system’s internal variables and its motion.  In the case of a wheeled robot our internal variables would be its wheels, which can rotate and cause motion, assuming that they are in contact with the ground.  For a differential drive robot each wheel can move at a different speed, therefore we need to look at how each wheel contributes to the robot’s overall motion.  When we’re dealing with internal kinematics we can describe its movement in terms of a local frame.  In the robot’s local frame forward motion is indicated by the X direction.  To get its total movement in the X direction we can simply add the distance covered by each wheel and divide the result by 2. 

So what about the Y direction?  If we had a Swedish wheel, also known as a Mecanum wheel, it would be able to move in any direction.  However, for this project I’m using standard wheels that can only roll in the direction of their orientation.  Therefore, its movement in the Y direction is always going to be zero.  This is referred to as a kinematic constraint.  We’re constrained to not move in the Y direction.

The orientation is the angle measured from the X-axis of the local frame to the forward orientation of the robot.  So when we first start out the value of ψ would be 0.0, meaning that the robot would be aligned with its own local X-axis.  In the local frame there is no notion of coordinates such as east and west or front and back of building.  Those coordinate frames will be mapped to when we consider external kinematics.  In order to rotate the robot each wheel would need to move at a different speed therefore covering a different distance.  To compute the rotation you would subtract the left wheel distance from the right wheel distance and divide it by the length of the wheelbase.  The following diagram summarizes all of this.

![/Robots]({{site.url}}{{site.baseurl}}/assets/images/Control-Theory-Slides.010.jpeg)

### Update State

Implementing the robot’s kinematics in code would look like the following.  The left and right wheel positions are obtained from the Wheel class.  This is the distance travelled since the robot was switched on.  The calculated pose is put into a pose vector.  I named the vector local_pose since the calculated pose is in the robot’s local frame.  The velocity is also calculated and placed in the velocity vector.  This process is called every 50 milliseconds. 
   
{% highlight cpp %}
// ------------------------------------------------------
// Update the current robot state 
// ------------------------------------------------------
void DriveTrain::updateState_() {
  
  // --- Get the distance delta since the last period ---  
  float leftPosition = leftWheel.currentPosition();
  float rightPosition = rightWheel.currentPosition();

  // --- Update the local position and orientation ---
  local_pose[X_POS] = (leftPosition + rightPosition) / 2.0; // distance in X direction 
  local_pose[Y_POS] = 0.0; // distance in Y direction
  local_pose[PSI] = (rightPosition - leftPosition) / wheelSeparation; // Change in orientation

  // --- Update the velocity ---
  float leftDistance = (leftPosition - leftPositionLast_); 
  float rightDistance = (rightPosition - rightPositionLast_);
  float delta_distance = (leftDistance + rightDistance) / 2.0; 
  float delta_theta = (rightDistance - leftDistance) / wheelSeparation; // in radians
 
  local_velocity[X_VELOCITY] = delta_distance / updatePeriodMicros; // Linear x velocity
  local_velocity[Y_VELOCITY] = 0.0; 
  local_velocity[Y_ANGULAR] = (delta_theta / updatePeriodMicros); // In radians per/sec

  // ---  Save the last position values ---
  leftPositionLast_ = leftPosition;    
  rightPositionLast_ = rightPosition; 
} 
{% endhighlight %}

The pose and velocity are available in vector form, which is made use of in the control loops.  For debugging purposes it can also be useful to print the pose out to the ROS console.  This is akin to printing to the serial port except that you don’t need to attach the robot to a really long USB cable.  You can also echo the published Odometry message out to the Linux terminal.  The published message can be recorded for later playback in a simulator.

{% highlight cpp %}
// ------------------------------------------------------
// Returns the local robot state
// ------------------------------------------------------
float * DriveTrain::getLocalPose() { 
  return local_pose;
}

// ---------------------------------------------------
// Return local robot velocity of robot in meters/sec 
// ---------------------------------------------------
float * DriveTrain::getLocalVelocity() {
  return local_velocity;
}

// ------------------------------------------------------
// Returns the local robot state
// ------------------------------------------------------
void DriveTrain::printLocalPose() { 
  ROSLOGString("Local pose X, Y, Psi");
  ROSLOG3F(local_pose[X_POS], local_pose[Y_POS], local_pose[PSI]);
}
{% endhighlight %}

That completes the DriveTrain class.  The [next post](twr-model-part2) will look at the Wheel, DCMotor, and Encoder classes.
