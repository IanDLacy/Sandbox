/* Begin Metadata

       Name:    smartCar.ino
    Created:    12/4/2018 1:29:34 AM
    Authors:    Chris Sayen:  Code
                   Ian Lacy:  Program Summary \ Goal
                              Program Map \ Function Tree
                              Descriptions
                              Comments

End Metadata */


/* Begin Program Summary \ Goal

	The vehicle travels forward until it encounters an obstacle as determined from data gathered from its ultrasonic sensor.
	It then stops at a safe distance, and scans its forward field of view (180 degrees).
	The scan, enabled by the servo on which the ultrasonic sensor is mounted, gathers data on obstructions blocking the vehicle.
	From the data gathered, the vehicle decides which direction it should try next.
	The vehicle uses the aforementioned data to orient itself to avoid the obstacle.
	The program then repeats.

End Program Summary \ Goal */


/* Begin Program Map \ Function Tree

[Compilier Initialization]
	| - Include libraries to control the ultrasonic sensor and the servo that aims it.
	| - Create aliases for the pins that will be utilized in this program.

[Program Initialization]
	| - Declare and set initial values for global variables used in this program.
	| - Declare the objects that will be used to control the ultrasonic sensor and the servo that aims it.

[Vehicle Initialization]
	| - Function "setup" is the standard Sketch function that runs once at runtime.
	| - - Configure pin modes.
	| - - Configure the object controlling the servo that aims the ultrasonic sensor.

[Main Loop]
	| - Function "loop" is the standard Sketch function that loops forever after "setup" runs.
	| - - Function "test" is a custom function for testing components. It can be activated in function "loop".
	| - - Function "forward" moves the vehicle forward while no obstacle is detected (using function "noObstacle", then initiates function "initLook".
	| - - - Function "noObstacle" checks whether or not there is an obstacle closer than "obsDis" parameter, and retuns true if not.
	| - - - - Function "readRangefinder" is used to pull data from the ultrasonic sensor and convert it to a distance value.
	| - - - - - Function "microsecondsToCentimeters" converts microseconds to centimeters. This is a helper for function "readRangeFinder".
	| - - - Function "initLook" stops the motors, then initiates functions "lookRight", "lookLeft", and "decideDirection", in that order.
	| - - - - Function "lookRight" scans the right half of the vehicle's forward viewing area and activates function "lookLeft" if no obstacle is detected (using function "noObstacle") closer than the threshold (though "lookLeft" will still be activated by function "initLook" regardless. The purpose of this function to gather data used for function "decideDirection".
	| - - - - - Function "noObstacle".
	| - - - - - - Function "readRangefinder".
	| - - - - - - - Function "microsecondsToCentimeters". <--|
	| - - - - - Function "lookLeft". <--|
	| - - - - Function "lookLeft" scans the left half of the vehicle's forward viewing area and activates function "decideDirection" if no obstacle is detected (using function "noObstacle") closer than the threshold (though "decideDirection" will still be activated by function "initLook" regardless. The purpose of this function to gather data used for function "decideDirection".
	| - - - - - Function "noObstacle".
	| - - - - - - Function "readRangefinder".
	| - - - - - - - Function "microsecondsToCentimeters". <--|
	| - - - - - Function "decideDirection". <--|
	| - - - - Function "decideDirection" picks which direction the vehicle will turn based on the 'clarity' data of each half of the vehicle's foward field of view as gathered by functions "lookRight" and "lookLeft", activating function "initTurn" accordingly.
	| - - - - - Function "initTurn" activates functions "turnRight" and "turnLeft" according to the information passed to it through its parameters. It also translates the degree values from function "decideDirection" to an amount for the turnng functions.
	| - - - - - - Functions "turnRight" and "turnLeft" are nearly identical and are used to activate the motors to spin the vehicle by the amounts passed by function "initTurn". After turning, the function reactivates function "forward".
	| - - - - - - - Function "forward". <--|

End Program Map \ Function Tree */


/* Begin Program */

// Include libraries to control the ultrasonic sensor and the servo that aims it.
#include <Servo.h>           // Include library to control servo under ultrasonic sensor.
#include <NewPing.h>         // Include library to control ultrasonic sensor.

// Create aliases for the pins that will be utilized in this program.
#define enA 8                // Compiler will replace all instances of "enA" with "8".
#define enB 9                // Compiler will replace all instances of "enB" with "9".
#define in1 4                // Compiler will replace all instances of "in1" with "4".
#define in2 5                // Compiler will replace all instances of "in2" with "5".
#define in3 6                // Compiler will replace all instances of "in3" with "6".
#define in4 7                // Compiler will replace all instances of "in4" with "7".
#define trig 3               // Compiler will replace all instances of "trig" with "3".
#define echo 2               // Compiler will replace all instances of "echo" with "2".

// Declare and set initial values for global variables used in this program.
int maximumRange = 200;      // Maximum range for ultrasonic sensor in centimeters.
int pwm = 255;               // Default value (0-255) for Pulse Width Modulation to control drive motor speed.
int pos = 0;                 // Variable representing the Sensor Servo position.												
long duration;               // Variable to contain the duration returned from the ultrasonic sensor.
long dist;                   // Variable to contain distance read from ultrasonic sensor.
long cm;                     // Variable to hold the distance value as converted from the duration read from the ultrasonic sensor.
int i;                       // Generic counter for loops.
int servoDelay = 2;          // Milliseconds to pause the program during function "lookRight" if "lookLeft" does not get called.
int leftDegrees = 90;        // Number of degrees the sensor servo will turn left.
int rightDegrees = 90;       // Function "lookRight" resets this to the difference between 90 and a counter.
int far = 100;               // Threshold distance (cm) for detecting [or not detecting?] an obstacle in function "lookRight".
int near = 30;               // Threshold distance (cm) for detecting an obstacle.
int turnAttemptCounter = 0;

// Declare the objects that will be used to control the ultrasonic sensor and the servo that aims it.
Servo RangeFinderServo;                        // Create object "RangeFinderServo" as new instance of library "Servo".
NewPing sonar(trig, echo, maximumRange);       // Create object "sonar", declare trigger and echo pins, and set maximum range.


// Function "setup" is the standard Sketch function that runs once at runtime.
void setup()                      // Declare valueless function "setup" with no parameters.
{
	// Configure pin modes.
	pinMode(enA, OUTPUT);         // Set pin "enA" as an "OUTPUT".
	pinMode(enB, OUTPUT);         // Set pin "enB" as an "OUTPUT".
	pinMode(in1, OUTPUT);         // Set pin "in1" as an "OUTPUT".
	pinMode(in2, OUTPUT);         // Set pin "in2" as an "OUTPUT".
	pinMode(in3, OUTPUT);         // Set pin "in3" as an "OUTPUT".
	pinMode(in4, OUTPUT);         // Set pin "in4" as an "OUTPUT".

	// Configure the object controlling the servo that aims the ultrasonic sensor.
	RangeFinderServo.attach(11);  // Attach object "RangeFinderServo" to pin "11".
}


// Function "test" is a custom function for testing components. It can be activated in function "loop".
void test()                                    // Declare valueless function "test" with no parameters.
{											  
	// Move the vehicle forward at full speed.
	
	// Configure left motor polarity for forward rotation.
	digitalWrite(in1, LOW);                   // Set half of H-Bridge A to open.
	digitalWrite(in2, HIGH);                  // Set half of H-Bridge A to closed.
	
	// Configure right motor polarity for forward rotation.
	digitalWrite(in3, LOW);                   // Set half of H-Bridge B to open.
	digitalWrite(in4, HIGH);                  // Set half of H-Bridge B to closed.
	
	// Apply power to both motors.
	analogWrite(enA, pwm);                    // Apply power to motor A (left) at full voltage (pwm is currently "255").
	analogWrite(enB, pwm);                    // Apply power to motor B (right) at full voltage (pwm is currently "255").
	
	// Move the servo for the ultrasonic sensor starting at 90 degrees, until position is 0 degrees.
	for (pos = 90; pos >= 0; pos -= 1)        // Sensor Servo position is "90". While greater than or equal to "0", do the following, then decrease by "1".
	{
		RangeFinderServo.write(pos);          // Move the Sensor Servo position to the current value from above.
		delay(5);                             // Pause the program for 5 milliseconds. 
	}

	// Move the servo for the ultrasonic sensor starting at 0 degrees, ultil position is 180 degrees
	for (pos = 0; pos <= 180; pos += 1)       // Sensor Servo position is "0". While less than or equal to "180", do the following, then increase by "1". 
	{
		RangeFinderServo.write(pos);          // Move the Sensor Servo position to the current value from above.
		delay(5);                             // Pause the program for 5 milliseconds.
	}

	// Kill power to both motors.
	analogWrite(enA, 0);                      // Kill power to motor A (left).
	analogWrite(enB, 0);                      // Kill power to motor B (right).
}

// Function "noObstacle" checks whether or not there is an obstacle closer than "obsDis" parameter, and retuns true if not.
bool noObstacle(int obsDis)                    // Declare boolean function "noObstacle" with integer parameter "obsDis" (obstacle distance).
{
	dist = readRangefinder();                  // Take a reading from the ultrasonic sensor and store it in this variable.
	if (dist > obsDis)                         // If the reading is greater than the value of the parameter passed through the function, do the following.
	{
		return true;                           // Function passes boolean value "true".
	}
	else                                       // Otherwise do the following.
	{
		return false;                          // Function passes boolean value "false".
	}
}

// Function "forward" moves the vehicle forward while no obstacle is detected (using function "noObstacle", then initiates function "initLook". 
void forward()                                // Declare valueless function "forward" with no parameters.
{
	turnAttemptCounter = 0;
	digitalWrite(in1, LOW);                   // See comments for function "test" above.
	digitalWrite(in2, HIGH);                  // ~~
	digitalWrite(in3, LOW);                   // ~~
	digitalWrite(in4, HIGH);                  // ~~
	analogWrite(enA, pwm);                    // ~~
	analogWrite(enB, pwm);                    // ~~
	while (noObstacle(near));                 // While the value of function "noObstacle", with the value of "near" as its parameter, returns true, do nothing, otherwise continue.
	initLook();                               // When an obstable is encountered (see above), call function "initLook".
}

// Function "initLook" stops the motors, then initiates functions "lookRight", "lookLeft", and "decideDirection", in that order. 
void initLook()                               // Declare function "initLook" to be called when an obstacle is detected.
{
	leftDegrees = 90;                         // Set "leftDegrees" to "90".
	rightDegrees = 90;                        // Set "rightDegrees" to "90".
	analogWrite(enA, 0);                      // Stop Motor A (left).
	analogWrite(enB, 0);                      // Stop Motor B (right).
	lookRight();                              // Call function "lookRight" with no parameters.
	lookLeft();                               // Call function "lookLeft" with no parameters.
	decideDirection();                        // Call function "decideDirection" with no parameters.
}

// Function "lookRight" scans the right half of the vehicle's forward viewing area and activates function "lookLeft" if no obstacle is detected (using function "noObstacle") closer than the threshold (though "lookLeft" will still be activated by function "initLook" regardless. The purpose of this function to gather data used for function "decideDirection".
void lookRight()                              // Declare valueless function "lookRight" with no parameters.
{
	for (pos = 90; pos >= 0; pos -= 1)        // Set position value to "90". While greater than or equal to "0", do the following, then decrease by "1".
	{
		RangeFinderServo.write(pos);          // Set Sensor Servo position to value from above.
		if (noObstacle(far))                  // If the value of "noObstacle", passed with parameter of value equal to "far" (see above) returns true, do the following.
		{
			rightDegrees = (-1 * pos) + 90;   // Set "rightDegrees" to the difference between "90" and the current Sensor Servo position.
			RangeFinderServo.write(90);       // Move the Sensor Servo position to 90 degrees.
			lookLeft();                       // Call function "lookLeft" with no parameters.
		}
		delay(servoDelay);                    // Pause the program for milliseconds equal to the value of "servoDelay".
	}
	RangeFinderServo.write(90);               // Move the Sensor Servo position to 90 degrees.
	delay(100);                               // Pause the program for 10 milliseconds.
	return;
}

// Function "lookLeft" scans the left half of the vehicle's forward viewing area and activates function "decideDirection" if no obstacle is detected (using function "noObstacle") closer than the threshold (though "decideDirection" will still be activated by function "initLook" regardless. The purpose of this function to gather data used for function "decideDirection".
void lookLeft()                               // Declare valueless function "lookLeft" with no parameters.
{
	for (pos = 90; pos <= 180; pos += 1)      // Set position value to "90". While less than or equal to "180", do the following, then increase by "1".
	{
		RangeFinderServo.write(pos);          // Set Sensor Servo position to value from above.
		if (noObstacle(far))                  // If the value of "noObstacle", passed with parameter of value equal to "far" (see above) returns true, do the following.
		{
			leftDegrees = pos - 90;           // Set "leftDegres" to the difference between the current Sensor Servo position and "90".
			RangeFinderServo.write(90);       // Move the Sensor Servo Position to 90 degrees.
			delay(100);                        // Pause the program for 10 mlliseconds.
			decideDirection();                // Call function "decideDirection" with no parameters.
		}
		delay(servoDelay);                    // Pause the program for milliseconds equal to the value of "servoDelay".
	}
	RangeFinderServo.write(90);               // Move the Sensor Servo position to 90 degrees.
	delay(100);                                // Pause the program for 10 milliseconds.
	return;
}

// Function "decideDirection" picks which direction the vehicle will turn based on the 'clarity' data of each half of the vehicle's foward field of view as gathered by functions "lookRight" and "lookLeft", activating function "initTurn" accordingly.
void decideDirection()                        // Declare valueless function "decideDirection" with no parameters.
{
	if (leftDegrees > rightDegrees)           // If the value of "leftDegrees" is greater than the value of "rightDegrees", do the following.
	{
		turn("right", rightDegrees);
	}
	else                                      // Otherwise, do the following.
	{
		turn("left", leftDegrees);
	}
	return;
}

void turn(String direction, int degrees)
{
	degrees = degrees * 2; // car turns approx half a degree per milisecond
	digitalWrite(in2, LOW);
	digitalWrite(in1, HIGH);
	digitalWrite(in4, LOW);
	digitalWrite(in3, HIGH);
	analogWrite(enA, pwm); 
	analogWrite(enB, pwm);
	delay(100);

	if (direction == "right")
	{
		digitalWrite(in2, LOW);
		digitalWrite(in1, HIGH);
		digitalWrite(in3, LOW);
		digitalWrite(in4, HIGH);
		analogWrite(enA, pwm);
		analogWrite(enB, pwm);
		delay(degrees);
	}
	else                                       // Otherwise, do the following.
	{
		digitalWrite(in1, LOW);
		digitalWrite(in2, HIGH);
		digitalWrite(in4, LOW);
		digitalWrite(in3, HIGH);
		analogWrite(enA, pwm);
		analogWrite(enB, pwm);
		delay(degrees);
	}
	forward();                              // Call function "forward" with no parameters.
}

void turnLeft(int turn)                    // Function "turnLeft" is exactly the same as "turnRight" except for the following.
{                                          //
	digitalWrite(in2, LOW);                //
	digitalWrite(in1, HIGH);               //
	digitalWrite(in4, LOW);                //
	digitalWrite(in3, HIGH);               //
	analogWrite(enA, pwm);                 //
	analogWrite(enB, pwm);                 //
	delay(100);                            //
	for (i = 0; i < turn; i++)             // Differently named local variable as the parameter.
	{                                      // Opposite spin direction.
		digitalWrite(in1, LOW);            //
		digitalWrite(in2, HIGH);           //
		digitalWrite(in4, LOW);            //
		digitalWrite(in3, HIGH);           //
		analogWrite(enA, pwm);             //
		analogWrite(enB, pwm);             //
		delay(10);                         //
		analogWrite(enA, 0);               //
		analogWrite(enB, 0);               //
		delay(10);                         //
	}                                      //
	forward();                             //
}                                          //

// Function "readRangefinder" is used to pull data from the ultrasonic sensor and convert it to a distance value.
long readRangefinder()                                // Declare long function "readRangefinder" with no parameters.
{
	pinMode(trig, OUTPUT);                            // Set pin "trig" as an "OUTPUT".
	digitalWrite(trig, LOW);                          // "LOW" to pin "trig".
	delayMicroseconds(2);                             // Pause the program for 2 microseconds.
	digitalWrite(trig, HIGH);                         // "HIGH" to pin "trig".
	delayMicroseconds(10);                            // Pause the program for 10 microseconds.
	digitalWrite(trig, LOW);                          // "LOW" to pin "trig".
	pinMode(echo, INPUT);                             // Set pin "echo" as an "INPUT".
	duration = pulseIn(echo, HIGH);                   // Set variable "duration" to the value of function "pulseIn" with parameters "echo" and "HIGH".
	cm = microsecondsToCentimeters(duration);         // Set variable "cm" to the returned value of function "microsecondsToCentieters" as was passsed variable "duration".
	return cm;                                        // Function passes the value of variable "cm".
}

// Function "microsecondsToCentimeters" converts microseconds to centimeters. This is a helper for function "readRangeFinder".
long microsecondsToCentimeters(long microseconds)            // Declare long function "microsecondsToCentimeters" with long parameter "microseconds".
{
	return microseconds / 29 / 2;                            // Function passes the result of a formula of some sort.
}

// Function "loop" is the standard Sketch function that loops forever after "setup" runs.
void loop()                  // Declare valueless function "loop" with no parameters.
{
	forward();               // Call function "forward" with no parameters.
	/*test();*/              // Call function "test" with no parameters.
}

/* End Program */

