---
layout: post
title:  "Two-wheeled robot control"
date:   2019-07-13 15:35:44 -0700
category: Projects
permalink: /twr-model-control
---
The first two [posts](twr-model-part1) described the programming model of the two-wheeled robot.  In this post I’ll show the control loops that drive the robot to its requested destination and orientation.  The control loops are arranged in cascaded fashion where the outer command loop calls the pose loop which calls the speed control loop.  I described the speed loop in a previous post, so here I’ll detail the command and pose control loops.

![Robots]({{site.url}}{{site.baseurl}}/assets/images/Two-Wheeled-Robot-Slides.002.jpeg)

### Command Control Loop

The command control loop is the outermost loop of our cascaded loop strategy.  It’ll simply loop waiting for a command to be sent via a callback routine.  The command will direct the robot to move to a new position and orientation, called a pose.  For this we use a message type appropriately called “Pose”.  The Pose message type is explained in the second post of this series.  Once the command is received it will be sent to the pose control loop that is responsible for moving the robot to the new position and orientation.  Once the new pose is obtained the program goes back to waiting for a command.  The process is shown in the flow diagram below.

![Robots]({{site.url}}{{site.baseurl}}/assets/images/Two-Wheeled-Robot-Slides.003.jpeg)

The callback function receives a message to move the robot to a new position and orientation.  The robot’s current pose is added to the new reference pose.  This is then used within the pose control loop to apply power to the motors and move the robot.  A flag is set to tell the main loop that a command has been received.  The ROSLOGx lines are home grown debug statements that print out to the ROS console.  If anyone is interested in how these are implemented please let me know. 

{% highlight cpp %}
//--------------------------------------------//
// Command callback
//--------------------------------------------//
void commandCb( const geometry_msgs::Pose& cmd) {

  // Print the new pose command message to the ROS console
  ROSLOGString("Received X, Y, Psi");
  ROSLOG3F(cmd.position.x, cmd.position.y, cmd.orientation.y);
  
  // Get the robot's current pose
  float *currentPose = robot->getLocalPose();

  // Print the robot's current pose to the ROS console
  robot->driveTrain.printLocalPose();

  // Assign new target pose. Current state + new pose.
  refPose[X_POS] = currentPose[X_POS] + cmd.position.x;
  refPose[Y_POS] = currentPose[Y_POS] + cmd.position.y;
  refPose[PSI] = currentPose[PSI] + cmd.orientation.y / (180/PI); // in radians 

  // Print target pose to the ROS console
  ROSLOGString("Target Pose X, Y, Psi");
  ROSLOG3F(refPose[X_POS], refPose[Y_POS], refPose[PSI]);
  
  // Set the commandCalled flag
  commandCalled = true;  
}
{% endhighlight %}

The main loop simply waits for a command to be received via the ROS subscribe protocol.  The nh.spinOnce() statement polls the ROS server for new messages.  Once the message is received and processed by the callback, the new reference pose is sent to the pose control loop.

{% highlight cpp %}
//-------------------------------------------------//
// Main command loop
//-------------------------------------------------//
void loop() {
    
  // Loop until command is sent.
  if (commandCalled) {
    // Position control loop
    poseController(refPose);

    // Reset commandCalled flag
    commandCalled = false;
  }  

  // Wait for subscribed messages 
  if (nodeHandleCreated) {nh.spinOnce();}
  delay(1000);
}
{% endhighlight %}

### Pose Control Loop

Once we have a command request to move the robot we need to implement a controller to guide it to the new destination and orientation.  The controller is passed the reference pose that was computed in the command callback function.  The reference pose becomes the new setpoint for the pose controller.  The loop starts by calculating the difference between the robot’s current pose and its reference pose.  The difference is referred to as the error.  Given the pose error the controller will determine the speed setpoint for the wheels, which is then fed into the speed control loop.  To complete the loop the new pose state is read from the robot and an updated error calculation is made.  The loop continues until the pose error is driven to zero.  There are separate loops for the destination and orientation control.

![Robots]({{site.url}}{{site.baseurl}}/assets/images/Two-Wheeled-Robot-Slides.004.jpeg)

The first thing the poseController does is get the current pose from the robot.  The current pose is obtained using a vector containing the X, Y positions and the ψ orientation, as shown in the above diagram.  The current pose is subtracted from the reference pose using a function that subtracts two matrixes.  The original linear and angular error values are saved since the pose error will constantly change as we process the move loops.  We first process the linear move followed by the angular move.  The fabs() function returns the absolute value of a float variable.  For the angular move the control function takes in an arc radius around which the robot will turn.  In this case, the arc length is zero requesting that we turn on the spot.  There’s more detail on this later in the post.


{% highlight cpp %}
//--------------------------------------------//
// Control the position of the robot
//--------------------------------------------//
void poseController(float * refPose) {

  // Get the current x position of the robot
  float *currentPose = robot->driveTrain.getLocalPose();

  // Subtract the target pose from the current pose to get the error
  Matrix.Subtract(refPose, currentPose, 3, 1, poseError);

  ROSLOGFLabel("Start error X", poseError[X_POS]);
  ROSLOGFLabel("Start error Psi", poseError[PSI]);

  // Save the original pose errors
  float linear_error = poseError[X_POS];
  float angular_error = poseError[PSI];
   
  // -------- Process linear error ------------------
  if (fabs(linear_error) > 0.0) { 
    moveLinear();
  } 

  // -------- Process angular error -----------------
  if (fabs(angular_error) > 0.0) {
    // Pass in the arc length
    moveAngular(0.0); // Turn on the spot
  }   
}
{% endhighlight %}

### Linear Move

The code to move the robot in a straight line towards the reference destination is shown below.  We first save the direction sign, positive for forward direction and negative for backwards.  This will be used within the control loop to determine if we’ve overshot our target position.  We set our wheel to the maximum speed and initialize a loop counter.  The loop counter is used to implement a timeout.  This is especially useful during testing so as you’re not running after the robot if the code is incorrect.  Initializing the wheel to maximum speed does result in a jolt at the beginning of the move.  This is mitigated somewhat in the speed control loop where it takes several time periods to reach the maximum PWM value.  However, I do intend to implement a smoother startup transition at a later point. 

We’re now ready to enter the control loop that continues until the X position error is driven to zero.  The loop is processed every 50 milliseconds and is timed out after 5 seconds for testing purposes.  If the sign of the pose error changes then we have gone passed our target position and we immediately break the loop and stop the motors.  We want to slow down as we reach the target position so I’ve setup a couple of constants that start to reduce the motor speeds as we get to within 15% of the reference position.  The Ki value ensures that we maintain a minimum speed at all times.  These values can be tuned experimentally.  The calculation will result in the maximum speed exceeding 1.0 during most of the run so we account for this condition in the code.

The new left and right wheel speed values are sent to the motors, and we go on to read the new pose value from the robot.  The new error is calculated and we return to the top of the control loop.  The loop is exited when the error is driven to zero and the command to stop the motors is sent.

{% highlight cpp %}
// PI control for pose loop. Adjust gain constants as necessary
const float Kp = 1.5; // Gets within 15% of target 
const float Ki = 0.3; // Maintains a minimum speed of 0.3
//----------------------------------------------//
// Move in a straight line forward or backward
//----------------------------------------------//
void moveLinear() {

  // Save the direction of motion
  int dir = sgn(int(poseError[X_POS] * 1000)); 
  
  // Set min and max speed.
  float maxSpeed = 1.0;
  float wheelSpeed = maxSpeed;
  int loopCounter = 0; // Loop counter for timeout
  
  while (fabs(poseError[X_POS]) > 0.0) {

    // Process the loop every 50 milliseconds
    current_time = nh.now().toSec();
    while(nh.now().toSec() < current_time + poseLoopPeriod) {
        //wait 50ms
    }

    // Timeout after 5 seconds
    if (loopCounter > timeOut) { ROSLOGString("Timed out!"); break; } 

    // End loop if the direction sign has changed (went passed the target)
    if ( sgn( int(poseError[X_POS] * 1000) ) != dir ) { break; }
    
    // Actuate the wheels 
    
    // Slows down when we get near the target position
    // Sets wheel speed proportionally to the position error
    wheelSpeed = (poseError[X_POS] * Kp) + (Ki * dir); // Kp = 2.0, Ki = 0.3

    // Keep max speed to 1.0
    if (fabs(wheelSpeed) > maxSpeed) { 
      wheelSpeed = (maxSpeed * dir); // Max 1.0
    } 
    ROSLOGFLabel("wheelSpeed", wheelSpeed);
    
    robot->setWheelSpeeds(wheelSpeed, wheelSpeed); // left and right wheel

    // Get the current x position of the robot
    float *currentPose = robot->getLocalPose();

    // Calculate the error
    Matrix.Subtract(refPose, currentPose, 3, 1, poseError);

    publishToROS(currentPose);  // Publish robot state to ROS
     
    loopCounter++;
    
  } // End while loop

  // Stop the wheels
  robot->setWheelSpeeds(0.0, 0.0);  // Stop
  ROSLOGString("--- Done linear movement ---");
}
{% endhighlight %}

Note that since the robot is referenced in the local frame the move along the linear X position always sends the robot straight ahead regardless of where it’s initially pointing.  There’s no global X position within a space that we’re trying to get to.  I’ve tried to illustrate this in the following diagram.  In the next series of posts I’m going to place the robot within a global frame in preparation for mapping and localization.

![/Robots]({{site.url}}{{site.baseurl}}/assets/images/Control-Theory-Slides.009.jpeg)

### Angular Move

Before looking at the control loop that implements a change in orientation we need to define a point around which the robot’s wheels can rotate.  This point is called the instantaneous center of rotation (ICR) and defines a point around which both wheels follow a circular motion.  To obtain an angular movement the inside wheel would need to spin slower than the outside wheel, which can be accomplished successfully with a differential drive robot.  The following diagram shows the math involved.

![/Robots]({{site.url}}{{site.baseurl}}/assets/images/Control-Theory-Slides.011.jpeg)

The following function is a member of the DriveTrain class.  It takes in an arc radius and returns the ratio between the inside and outside wheels.  If you follow the calculation through you’ll find that an arc radius of zero will return a ratio of 1.0, resulting in the robot turning on the spot.

{% highlight cpp %}
// ----------------------------------------------------------------
// Calculate the ratio between the two wheels while driving in
// an arc. arcRadius is the Instantaneous Center of Rotation (ICR)
// ----------------------------------------------------------------
float DriveTrain::calculateArcWheelRatios(float arcRadius) {
  
  // Get the distance that each wheel has to travel
  float insideWheel = 2*PI * ( arcRadius - (wheelSeparation/2) );
  float outsideWheel = 2*PI * ( arcRadius + (wheelSeparation/2) );

  // Return the ratio between the two wheels 
  return (insideWheel / outsideWheel);
}
{% endhighlight %}

The moveAngular() function follows a similar pattern to the moveLinear() function.  Since our wheels need to move at different speeds the ratio between the left and right wheel is obtained from the calculateArcWheelRatios() function, explained above.  The resulting ratio will be between 1.0 and 0.0 so it must therefore applied to the slower spinning inside wheel.  For turns, I’ve set the maximum speed to 50%.  We then enter the main orientation control loop.

The primary difference between the orientation and linear control loops is that the wheels are moving at different speeds.  We first calculate the speed for the outside wheel and apply our ratio to get the inside wheel speed.  Note that the ratio will be 1.0 for a turn on the spot and that one wheel will be moving forward while the other is going backwards.  This will be the case whenever the arc radius is less than half the length of the wheel base.  At some point we have to decide which wheel is the inside wheel.  This depends of course on whether we’re turning left or right.  The determination is made and sent to the robot to actuate its motors.  The remainder of the process is the same as for the linear move.

{% highlight cpp %}
//----------------------------------------------//
// Move in an arc clockwise or anti-clockwise
//----------------------------------------------//
void moveAngular(float arcRadius) {

  // Calculate the relative wheel speeds based on the 
  // radius of the turn.
  float ratio = robot->calculateArcWheelRatios(arcRadius);
  
  // Save the direction of motion
  int dir = sgn(int(poseError[PSI] * 1000)); 

  // Use half of max speed for turn.
  float maxSpeed = 0.5;

  // Set initial wheel speeds
  float outsideWheelSpeed = maxSpeed;
  float insideWheelSpeed = maxSpeed * ratio;
  
  int loopCounter = 0; // Loop counter for timeout
  
  while (fabs(poseError[PSI]) > 0.0) {

    // Process the loop every 50 milliseconds
    current_time = nh.now().toSec();
    while(nh.now().toSec() < current_time + poseLoopPeriod) {
        //wait 50ms
    }    

    // Timeout after 10 seconds
    if (loopCounter > (timeOut*2)) { ROSLOGString("Timed out!"); break; } 

    // End while loop if the direction sign has changed (went passed the target)
    if (sgn( int(poseError[PSI] * 1000) ) != dir) { break; }

    // Slows down when we get near the target orientation
    // Sets the outside wheel speed proportionally to the orientation error
    float outsideWheelSpeed = (poseError[PSI] * Kp) + (Ki * dir); // Kp = 3.0, Ki = 0.3

    // Keep max speed to 1.0
    if (fabs(outsideWheelSpeed) > maxSpeed) { 
      outsideWheelSpeed = maxSpeed; // Max 1.0
    } 

    // Calculate the inside wheel speed. Ratio will be -1 for turn on the spot
    insideWheelSpeed = outsideWheelSpeed * ratio;
    ROSLOG2FLabel("Wheel speeds (left,right)", insideWheelSpeed, outsideWheelSpeed);
    
    // Actuate the wheels
    if (poseError[PSI] > 0.0) { // Moving anti-clockwise. Inside wheel is left
      robot->setWheelSpeeds(insideWheelSpeed, outsideWheelSpeed); // (left wheel, right wheel)
    } 
    else { // Moving clockwise. Outside wheel is left 
      robot->setWheelSpeeds(outsideWheelSpeed, insideWheelSpeed); // (left wheel, right wheel) 
    }

    // Get the current x position of the robot
    float *currentPose = robot->getLocalPose();

    // Calculate the error
    Matrix.Subtract(refPose, currentPose, 3, 1, poseError); 

    publishToROS(currentPose);  // Publish robot state to ROS
    
    loopCounter++;
    
  } // End while loop

  // Stop the wheels
  robot->setWheelSpeeds(0.0, 0.0);  // Stop
  ROSLOGString("--- Done angular movement ---");
} 
{% endhighlight %}

This completes phase one of the project.  Please let me know if this series of posts has been useful.  If so, I’ll continue to write updates on the project as it progresses.  In the next phase I’ll be working at controlling the robot within a global reference frame, such as a living room (external kinematics).  I’d also like to try and use an MPU for dead reckoning to confirm the odometry results coming from the encoders.

