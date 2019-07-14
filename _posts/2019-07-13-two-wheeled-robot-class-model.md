---
layout: post
title:  "Two-wheeled Robot Class Model"
date:   2019-07-13 15:35:44 -0700
categories: Projects
permalink: /two-wheeled-robot-model
---
This project will describe the programming for a two-wheeled differential drive robot.  The project will make use of the Robot Operating System (ROS) so the code examples will detail how this can be used.  There are two functionality goals for this project.  The first is to get it move forward a specified distance, and the second is to have it turn on the spot a certain number of degrees.  Although these may seem like simple tasks it does present some programming challenges requiring control feedback loops and PID controllers.  The plan is to start with a basic programming model and build it up into a fully autonomous mobile robot. 

I’ll start by building a program model of a two-wheeled robot to implement its internal functionality, such as driving the wheels and counting encoder pulses.  This model won’t include any functionality to control how it gets to its destination.  That will be the job of the control loops described later.  In the first stage only the drive train is implemented for mobility.  Other functionality such as laser and ultrasonic range finders will be added later.  The programming model is written using C++ classes but you should be able to translate it into Python, which is the other language widely used with ROS.  Here’s a block diagram of the model:

![Robots](/assets/images/Two-Wheeled-Robot-Slides.011.jpeg)

After detailing the two-wheeled robot model I’ll go on to describe the control loops that are used to control the speed, position and orientation of the robot.  The position and orientation is known in the ROS world as its “pose”.  The outer loop will simply wait to receive a move command that it sends to a control loop to change the robot’s pose.

![/Robots](/assets/images/Two-Wheeled-Robot-Slides.003.jpeg)

The pose control loop tells the robot how fast it should spin each wheel and this will cascade down to the speed control loop that determines how much power to apply.  One of the challenges is to get the robot to go in a straight line.  This is not an easy as you may think unless the two motors just happen to be perfect clones of each other.  In reality this is never the case, so feedback control must be used to compensate for this.  Once the motors are spinning and the robot is moving its position and orientation is tracked so as it can be stopped after it has travelled a certain distance.

![/Robots](/assets/images/Two-Wheeled-Robot-Slides.005.jpeg)

Wheeled Robot Kinematics

When working with robots the word kinematics comes up a lot.  Kinematics simply describes how an object moves.  This project will only look at the internal kinematics of the robot, which describes the relationship between its wheel rotation and how it moves.   For internal kinematics the robot will move within a local reference frame, as shown in the following diagram.

![/Robots](/assets/images/Control-Theory-Slides.009.jpeg)

Note that since the robot has standard fixed wheels that it cannot move sideways.  Therefore, the value of the Y component in the internal reference frame will always be zero.  Contrast this to external kinematics that describes the robot’s position and orientation relative to a point in a global reference frame, such as a room.  In this case, the Y component can be non-zero since we can drive the robot to any location in the room using a series of discrete maneuvers.

![/Robots](/assets/images/Control-Theory-Slides.012.jpeg)

Hardware Architecture

The robot has a motor attached to each wheel and is balanced with wheel casters.  Each motor has an attached encoder.  The compute architecture is very simple at this stage.  It uses a single ESP32 micro controller for both the robot’s internal functionality, such as the actuation of its motors, and also its external control where we drive it towards a new position and orientation.  Here’s a picture of the rig that I’m using but the program should work with any two-wheeled differential drive robot.

![/Robots](/assets/images/Project1Robot.JPG)

{% highlight ruby %}
def print_hi(name)
  puts "Hi, #{name}"
end
print_hi('Tom')
#=> prints 'Hi, Tom' to STDOUT.
{% endhighlight %}

Check out the [Jekyll docs][jekyll-docs] for more info on how to get the most out of Jekyll. File all bugs/feature requests at [Jekyll’s GitHub repo][jekyll-gh]. If you have questions, you can ask them on [Jekyll Talk][jekyll-talk].

[jekyll-docs]: https://jekyllrb.com/docs/home
[jekyll-gh]:   https://github.com/jekyll/jekyll
[jekyll-talk]: https://talk.jekyllrb.com/
