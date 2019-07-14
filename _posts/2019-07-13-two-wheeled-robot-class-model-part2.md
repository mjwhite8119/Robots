---
layout: post
title:  "Two-wheeled robot class model - part 2"
date:   2019-07-13 15:35:44 -0700
category: Projects
permalink: /twr-model-part2
---
This is part 2 or the [two-wheeled robot project](/twr-model-part1) using ROS.  In this post I’ll detail the Wheel, DCMotor, and Encoder classes, which will complete the robot model piece of the project.

### Wheel Class

A wheel has several properties probably the most important of which is its diameter and whether or not it is powered.  To understand the wheel’s kinematic constraints we’d also need to know whether it is fixed or steerable, and whether it’s holonomic or nonholonomic.  A holonomic wheel, such as an Mecanum wheel, is able to move instantaneously in any direction.  The Wheel class needs to encapsulate all of these properties.  In the following diagram the wheel’s configuration is shown on the left and the functions are listed on the right.

![Robots]({{site.url}}{{site.baseurl}}/assets/images/Two-Wheeled-Robot-Slides.014.jpeg)

### Wheel Configuration

 The Wheel class keeps track of the wheel diameter and a wheel type.  The wheel type lets us understand the kinematic constraints of the robot.  For instance, a Mecanum wheel will allow the robot to move instantaneously in any direction whereas a fixed wheel would only allow movement in a straight line.  If the wheel is powered then it will also add a DCMotor object.  Here’s a list of possible wheel types that we may what to represent in the Wheel class.

{% highlight cpp %}
// Define wheel types 
enum WheelTypes {
  STANDARD_FIXED = 1,
  STANDARD_FIXED_POWERED,
  STANDARD_FIXED_CONNECTED,
  STANDARD_FIXED_CONNECTED_POWERED,
  STANDARD_STEERABLE,
  STANDARD_STEERABLE_CONNECTED,
  CASTER,
  SWEDISH,
  MECANUM,
  SPHERIC
};
{% endhighlight %}
As a programming note, I currently don’t use the wheel types to make any decisions in the code.  I could for instance pass in one of the powered wheel types that tells the Wheel class to create a DCMotor object.  Right now I have a class constructor for each type of wheel, powered or unpowered.  I’m planning on enhancing the class to use the wheel type attribute at a later point.

### The Code

There are two constructors in the Wheel class.  One is for a powered wheel and the other is for an unpowered wheel.  Both constructors accept the wheel diameter and wheel type parameters.  The currentPosition() function computes the distance travelled since the robot was started.  The setSpeed() function just passes the speed setpoint onto the DCMotor class.

{% highlight cpp %}
class Wheel
{ 
  public:   
    // Constructor for powered wheels   
    Wheel(uint8_t pinGroup, float wheelDiameter, uint8_t wheelType);

    // Constructor for unpowered wheels   
    Wheel(float wheelDiameter, uint8_t wheelType);

    // -------------------- Member variables -----------------  
    
    // --- Configuration (physical) properties ---
    float diameter; // wheel diameter in meters
    uint8_t type; 
  
    DCMotor motor;

    // ---  Kinematic (motion) properties ----   
    const float distancePerPulse = (PI * diameter) / Encoder::PPR;   // PI*D/PPR  

    // Current direction of the wheel
    int currentDirection = 0;
    
    // Get the current linear_x position of the wheel
    float currentPosition();

    // ------------------- Convenience methods ------------------
          
    // Set wheel speed in meters/sec
    void setSpeed(float speed);

    // Encoder pulses from attached motor
    int32_t encoderPulses();
}; 
{% endhighlight %}

When a powered wheel is created a pinGroup is passed in and used to create the DCMotor object.  I went into more detail about pinGroups in a previous post.  This parameter is not needed for an unpowered wheel.

{% highlight cpp %}
// ------------------ Constructors --------------------------
// This is for a powered wheel so we need a motor and a set of GPIO pins to connect 
// it to the microcontroller. 
Wheel::Wheel(uint8_t pinGroup, float wheelDiameter, uint8_t wheelType) 
  :motor(pinGroup), diameter(wheelDiameter), type(wheelType)
{ 
}

// This is for an unpowered wheel so no motor gets attached. 
Wheel::Wheel(float wheelDiameter, uint8_t wheelType) 
  :diameter(wheelDiameter), type(wheelType)
{    
} 
{% endhighlight %}

### Wheel State

It would be useful to know how far the wheel has travelled since we turned the robot on.  This information is returned by the wheel’s currentPosition() function.  We need to translate the number of encoder pulses into a distance travelled.  For that we take our wheel diameter multiplied by Pi and divide that by the number of encoder pulses per revolution to come up with a distance per encoder pulse.  Multiplying distancePerPulse by the total number of pulses since the robot started gives us our current X position.

{% highlight cpp %}
// ---------------------------------------------------
// Get wheel position in meters
// ---------------------------------------------------
float Wheel::currentPosition() {
return motor.encoder.pulses * distancePerPulse;
}
{% endhighlight %}

### DCMotor Class

The robot uses DC motors to actuate the wheels.  The DCMotor class includes the motor’s configuration together the functions to control it.  In the following diagram the configuration is shown on the left and the motor control functions are listed on the right.

![Robots]({{site.url}}{{site.baseurl}}/assets/images/Two-Wheeled-Robot-Slides.015.jpeg)

## Motor Configuration

The pinGroup is passed in from the DriveTrain class and is used to associate the motor with the micro controller’s GPIO pins.  The attached wheel encoder is implemented by the Encoder class.  In order to control the motor speed a periodic timer interrupt is used.  The timers are implemented with static class methods since we cannot use interrupts within member functions.  For the two-wheeled robot I have created two instances of the static timer function, one for each wheel.  Each instance is referred to by its pinGroup number.  The following code shows the configuration of the DC motor. 

{% highlight cpp %}
class DCMotor
{
  public:
        
    DCMotor() {} // Default constructor

    // Constructor to connect motor GPIO pins to microcontroller
    DCMotor(uint8_t pinGroup);

    // Encoder attached to the motor
    Encoder encoder;

    // Set the wheel speed   
    void setSpeed(float pulseSetpoint);
    
  private:
    
    uint8_t pinGroup_; // motor GPIO pins 

    static DCMotor * instances [2];

    // Encoder interrupt routines
    static void motorISR0 (void *pArg)
    {
      if (DCMotor::instances [0] != NULL)
        DCMotor::instances [0]->setPower_();
    } 
    
    static void motorISR1 (void *pArg)
    {
      if (DCMotor::instances [1] != NULL)
        DCMotor::instances [1]->setPower_();
    }
    
    // Motor speed variables
    int32_t pulsesLast_ = 0; 
    int32_t pulsesPerSec_ = 0;
    int error_ = 0;      
    float pPart_ = 0.0, iPart_ = 0.0; // PI control
    int PWM_ = 0; // Current PWM
    int pulseSetpoint_ = 0; // Current pulse setpoint
    
    // PI control. Adjust gain constants as necessary.
    const float Kp = 0.1, Ki = 0.1; // gain constants

    // Set motor power
    void IRAM_ATTR setPower_();

    // Apply power to the motor
    void IRAM_ATTR applyPower_(int dir, int PWM);
}; 
{% endhighlight %}

The constructor uses the pinGroup that it got from the DriveTrain in order to attach its encoder.  The pinGroup is also saved in the DCMotor class for later use.  The motor direction and PWM pins are associated with the micro controller.  The setup of the PWM pin varies depending on the architecture of the micro controller, so I have abstracted into its own hardware specific function.  A static timer interrupt is created depending on the pinGroup.  The callback function for each interrupt can be seen in our DCMotor class declaration.

{% highlight cpp %}
 // -------------------Constructors -----------------------------------
DCMotor::DCMotor(uint8_t pinGroup) 
  :encoder(pinGroup), pinGroup_(pinGroup)
{   
  // Connect motor to GPIO pins
  pinMode(motorPinGroup[pinGroup].motorDir1, OUTPUT); // motor direction
  pinMode(motorPinGroup[pinGroup].motorDir2, OUTPUT); // motor direction
  pinMode(motorPinGroup[pinGroup].enable, OUTPUT);

  // Setup PWM signal
  ledcSetup(pinGroup_, freq, resolution); // create a PWM channel 
  ledcAttachPin(motorPinGroup[pinGroup_].enable, pinGroup_); // attach channel to pin

  // Start motor power timers 
  switch (pinGroup)
  {
  case 0: 
    {
      const esp_timer_create_args_t periodic_timer_args = {.callback = &motorISR0};
      esp_timer_create(&periodic_timer_args, &motorTimer0);
      esp_timer_start_periodic(motorTimer0, speedCtrlPeriodMicros); // Time in milliseconds (50)
      instances [0] = this; 
    }
    break;
    
  case 1: 
    {
      const esp_timer_create_args_t periodic_timer_args = {.callback = &motorISR1};
      esp_timer_create(&periodic_timer_args, &motorTimer1);
      esp_timer_start_periodic(motorTimer1, speedCtrlPeriodMicros); // Time in milliseconds (50)
      instances [1] = this;
    }
    break;
    
  } // end of switch
}
{% endhighlight %}

## Motor Speed Control

The first thing we’d want to do is set the speed of the motor.  Our motor speed depends greatly on how much load its under.  When you look at the specifications of a motor you’ll find its No-load Speed.  This is how fast the motor will run if you applied full power and held it in the air.  There’s nothing slowing the motor down like the weight of a robot chassis.  It’s under no load.  On our real world robot however, the speed of the motor will depend on such factors as the weight of the robot, whether it’s going up or down hill, and what type of surface it’s running on.  This makes our task of controlling the speed much more difficult.

So to make sure that the robot runs at a set speed under all of these conditions we’ll need to control the amount of power that’s applied to the motor.  For that we can implement a control loop.  Let’s say that our goal is to have the robot move at 1/2 meter per second.  We’ll direct the controller to apply a certain amount of power to the motors.  We’ll observe how fast the robot is moving using the wheel encoders (sensor).  If our robot is moving too slowly then we’ll apply more power.  If it’s moving too fast then we apply less power.  The difference between the required speed and the actual speed is our error.  This is done in a continuous loop while the robot is moving.  The control loop is shown below.

![Robots]({{site.url}}{{site.baseurl}}/assets/images/Two-Wheeled-Robot-Slides.006.jpeg)

Within the speed control loop we check how fast the motor is spinning at each instant of time and adjust the power accordingly.   We count the number encoder pulses per second and make sure that we maintain the required pulse rate by adjusting the power.  There’s more information about encoder pulses later in the post.  Since the motor really only cares about encoder pulses it doesn’t need know about the translational speed of the robot so we just need to give it a pulse rate and let it track against that.

{% highlight cpp %}
// -------------------------------------------------------- 
// Sets the local pulseSetpoint which is picked up in the
// setPower_() periodic timer. This will give pulses between
// 60 and 600.  600 pulses per/sec is about 35 cm/sec
// --------------------------------------------------------
void DCMotor::setSpeed(float pulseSetpoint) {

  // PulseSetpoint drives the action of the setPower_() routine
  pulseSetpoint_ = pulseSetpoint;
  encoder.wheelDirection = sgn(pulseSetpoint);
} 
{% endhighlight %}

When we created the DCMotor object it launched a periodic timer that executes the setPower() function.  In the example I’ve set the frequency to 25 milliseconds, so it will execute the setPower() function 40 times per second.

### PID Control

A PID controller is a control loop feedback mechanism that calculates the difference (error) between a desired setpoint and the actual output from a process.  If the error value is non-zero then it applies a correction to process input.  PID stands for Proportional, Integral, Derivative.  For the motor control process I’ve found that I only needed the PI part of the mechanism so I wrote a few simple lines of code to implement it.  There’s a very well documented Arduino library that provides a full implementation of a PID controller.  The diagram below shows a generalized PI control flow.  In our motor control case, r(t) would be the required encoder pulses per second and e(t) would be the difference between the current pulses coming from the encoder y(t) and the required pulses.  The motor input PWM signal is represented by u(t).  Kp and Ki are tunable parameters that provide the proportional and integral gain value constants for the controller.

![/Robots]({{site.url}}{{site.baseurl}}/assets/images/Control-Theory-Slides.007.jpeg)

### The Code

The first step is to get the number of encoder pulses since the last time period and multiply it by the periods per second.  This gives us the number of pulses per second which can then be used by the PI controller.  The PI controller calculates the error between the current pulses and the required pulses and resets the value of the PWM signal.  For my robot I’ve found the following controller gain constants to be reasonable.  You can play around with these to get the optimal values for your situation.  For the proportional part the control gain should be quite a bit less than one otherwise you would get a very large value for the PWM signal.  This would cause a sudden jolt as the robot starts up.  The integral part should be adjusted so as you don’t keep oscillating around the setpoint value.  Ok, maybe I should write a separate article on this?

There’s a situation where the integral part of the calculation drops below zero as the motor shuts down.  This results in a negative PWM signal that I’ve taken care of in the code.  Once the new PWM value is computed I figure out what direction the motor is going in and execute the applyPower() function.

{% highlight cpp %}
void IRAM_ATTR DCMotor::setPower_() {

  // Get the number of pulses since the last period
  int32_t pulses = encoder.pulses;  
  int32_t pulsesThisPeriod = abs(pulses - pulsesLast_);
     
  // Save the last pulses
  pulsesLast_ = pulses;

  // Compute the error between requested pulses/sec and actual pulses/sec
  pulsesPerSec_ = pulsesThisPeriod * periodsPerSec;
  error_ = abs(pulseSetpoint_) - pulsesPerSec_; 
  
  // --- PI control ---
  pPart_ = Kp * error_; // Proportional
  iPart_ += Ki * error_; // Integral

  // We've put the setpoint to zero, stopping the motors
  if (iPart_ < 0.0) { 
    iPart_ = 0.0; // Don't let integral part go negative
  }

  // Compute the PWM
  PWM_ = int(pPart_ + iPart_);

  // Motors are shutting down
  if (pulseSetpoint_ == 0 && iPart_ == 0.0) { 
    PWM_ = 0;
  } 
 
  // Apply the power with the direction and PWM signal
  applyPower_(sgn(pulseSetpoint_), PWM_);
} 
{% endhighlight %}

The applyPower() function sets the direction of the motor and writes the PWM signal to the motor’s enable pin.  Writing the PWM signal may be different depending on the hardware so I’ve abstracted it out into a separate function.  On the Arduino platform it’s a simple analogWrite function.  On the ESP32 it’s a little more complicated.

{% highlight cpp %}
// ------------------------------------------------
// Apply power to motor 
// ------------------------------------------------
void IRAM_ATTR DCMotor::applyPower_(int dir, int PWM) {

  int level;
  if(dir >= 0) {
    level = HIGH; 
  } else {
    level = LOW; 
  }

  digitalWrite(motorPinGroup[pinGroup_].motorDir2, level); // Direction HIGH forward, LOW backward
  digitalWrite(motorPinGroup[pinGroup_].motorDir1, (!level)); // Write the opposite value
  
  // See setupPWM(pinGroup) in Config.h
  ledcWrite(pinGroup_, abs(PWM));
} 
{% endhighlight %}

### Encoder Class

The robot has an optical encoder attached to each wheel.  The pinGroup is a structure of GPIO pins that are used to attach the encoders to the micro controller.  The PPR variable stands for Pulses Per Revolution.  This is the number of pulses that the encoder generates for each revolution of its motor.  In my case there are 341 pulses per revolution.  The pulses variable tracks the number of encoder pulses since we started the robot and is incremented via a hardware interrupt.

![Robots]({{site.url}}{{site.baseurl}}/assets/images/Two-Wheeled-Robot-Slides.016.jpeg)

### The Code

Since we can’t use a member function for an interrupt in C++ the Encoder class sets up a static function for each interrupt.  This is not an ideal situation since we can’t just create additional encoder objects without adding more functions.  The interrupt will increment or decrement the pulses variable depending on whether the motor is moving forward or backwards.

{% highlight cpp %}
 class Encoder
{  
   public:

    // Class variables
    static const int PPR = 341;  // Encoder Count per Revolutions 
    
    // Default constructor
    Encoder() {}

    // Constructor to connect encoder GPIO pins to microcontroller
    Encoder(uint8_t pinGroup);

    volatile int32_t pulses;
    int8_t wheelDirection = 0;

    static Encoder * instances [2];
   
  private:

    uint8_t pinGroup_;

    // Encoder interrupt routines
    static void encoderISR0 ()
    {
      if (Encoder::instances [0] != NULL)
        Encoder::instances [0]->encoderFired_();
    } 
    
    static void encoderISR1 ()
    {
      if (Encoder::instances [1] != NULL)
        Encoder::instances [1]->encoderFired_();
    }
      
    // Checks encoder A
    void IRAM_ATTR encoderFired_();   
};
{% endhighlight %}

In the constructor we setup an interrupt using the encoder A and B pins of the motor pinGroup.  I’m using a quadrature encoder, hence the need for channels A and B.  The interrupts get attached depending on whether it’s the left or right encoder.   The interrupt will fire as the GPIO pin goes from low too high.  The instances array is assigned a pointer to the member that we just created.

{% highlight cpp %}
// ---------------------  Constructors -------------------------------
Encoder::Encoder(uint8_t pinGroup) 
  :pinGroup_(pinGroup)
{   
  // Connect encoder to GPIO pins 
  pinMode(motorPinGroup[pinGroup].encoderA, INPUT); //  Left encoder, channel A
  digitalWrite(motorPinGroup[pinGroup].encoderA, HIGH); // turn on pullup resistors
  pinMode(motorPinGroup[pinGroup].encoderB, INPUT); //  Left encoder, channel B
  digitalWrite(motorPinGroup[pinGroup].encoderB, HIGH); // turn on pullup resistors

  // Attach interrupts
  switch (pinGroup)
  {
  case 0: 
    attachInterrupt (motorPinGroup[0].encoderB, encoderISR0, RISING);  // Left encoder
    instances [0] = this; 
    break;
    
  case 1: 
    attachInterrupt (motorPinGroup[1].encoderA, encoderISR1, RISING); // Right encoder
    instances [1] = this;
    break; 
       
  } // end of switch

  // Switch on interrupts
  sei();  
  // Initialize pulses. 
  pulses = 0;
} 
{% endhighlight %}

So one of the reasons for using a quadrature encoder is that it can detect which direction the motor is spinning by tracing the offset between channels A and B.  I wasn’t able to get this working on the ESP32 for some reason so I just used a variable to determine whether the pulses should be incremented of decremented.  I’ll probably try and get this working at some point.  Since the pulses variable is four bytes long the mutex is used to ensure that the increment and decrement occurs as an atomic operation.

{% highlight cpp %}
void IRAM_ATTR Encoder::encoderFired_() {
  // pulses is 4 bytes so make sure that the write is not interupted
  portENTER_CRITICAL_ISR(&timerMux);
  
  if (wheelDirection > 0) {
    pulses++;
  } else {
    pulses--;  
  } 
  portEXIT_CRITICAL_ISR(&timerMux);
}
{% endhighlight %}

That completes the two-wheeled robot model for now.  There will probably be other additions to the model as more capabilities are added to the project.  The [next post](/twr-model-control) will detail the control loops that move the robot in a straight line and rotate it on the spot.

