/*******************************************************************************
* race.cpp
*
* Justin Creaby 2018
* This program runs an autonomous car around a predefined path.
* 
* The car will stop when a position of 0, 0 is desired or by hitting control C
* 
* Example of calling this function:
* 
*		sudo ./race 4
* 
*******************************************************************************/
extern "C" {
    #include <rc_usefulincludes.h>
	#include <roboticscape.h>
}

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <deque>
#include "encoderFunctions.h"
#include "readPathFileToVector.h"
//#include "URG04LX.hpp"
//#include "../../BreezyLiDAR/cpp/URG04LX.cpp"
//#include "../../BreezyLiDAR/c/hokuyo.h"
//#include "../../BreezyLiDAR/c/hokuyo.c"//#include "../../BreezyLiDAR/cpp/URG04LX.cpp"

//static const char * DEVICE = "/dev/ttyACM0";


/*******************************************************************************
* void on_pause_released() 
*	
* Make the Pause button toggle between paused and running states.
*******************************************************************************/
void on_pause_released(){
	rc_set_state(RUNNING);
	return;
}



int main(int argc, char *argv[]){
	if(argc != 3 )
	{
		printf("    Incorrect usage.\n");
		printf("    Need to specify amount of time (sec) to calibrate gyros.\n");
		printf("    Need to provide a path file.\n");
		printf("    Example: sudo ./race 4 pathFile.txt\n");
		return 0;
	}
	
	// Initialize LEDs
	rc_set_led(GREEN,OFF);
	rc_set_led(RED,ON);
	
	// Timing Variables
	float timeStep = 0.01f; // Time step of program, (seconds).
	int timeStepCount = 0; // Used for printing out to the screen
	float timeElapsed = 0.0f; // Used for printing time passed
	uint64_t startTimerNanoSeconds; // Timer for the timing while loop
	int initializeCount = 0; // Counter for initializing the ECS motor and steering
	
	// Distance & Speed Variables
	float propGain = 0.03f;
	float intGain = 0.02f;
	float straightSpeed = 3.0f; // any faster than 3.0 will cause it to wheel spin
	float curveSpeed = 2.0f;
	float obstacleSpeed = 2.0f;
	float lastVelSetpoint = straightSpeed/2.0f;
	float velocityAverageTime = 0.5f;
	float velocityRateLimit = 1.0f;
	float integratorError = 0.0f;
	int encoderPos = rc_get_encoder_pos(1); // Current encoder position
	int lastEncoderPos = rc_get_encoder_pos(1); // Previous encoder position
	float distanceIncrement = 18.288f/151.0f; //  Distance travelled between pulses.

	std::deque<int> encoderHistory ((int) (velocityAverageTime/timeStep),0); // Deque to hold the recent history of encoder pulses

	// Office path
	// Read in the path from a csv text file.
	std::string pathFileName = argv[2];
	
	std::vector<double> pathX, pathY;
	readPathFile(pathFileName, pathX, pathY); //TODO: Pass in and populate pathX and pathY pointers
	
	//float pathX[65] = {0.75,0.75,0.75,0.75,0.75,0.75,0.75,1.25,1.75,2.25,2.7,3.15,3.6,4.05,4.5,4.95,5.4,5.85,6.3,6.75,7.2,7.65,8.1,8.55,9.05,9.55,10.05,10.05,10.05,10.05,10.05,10.05,10.05,10.05,10.05,10.05,10.05,10.05,10.05,9.55,9.05,8.55,8.1,7.65,7.2,6.75,6.3,5.85,5.4,4.95,4.5,4.05,3.6,3.15,2.7,2.2,1.7,1.2,0.95,0.75,0.75,0.75,0.75,0.75,0.75};
	//float pathY[65] = {7.16,8.16,9.16,10.16,11.16,12.16,13.16,13.66,13.66,13.66,12.66,11.66,10.66,9.66,8.66,7.66,6.66,5.66,4.66,3.66,2.66,1.66,0.66,0.66,0.66,0.66,1.16,2.16,3.16,4.16,5.16,6.16,7.16,8.16,9.16,10.16,11.16,12.16,13.16,13.66,13.66,13.66,12.66,11.66,10.66,9.66,8.66,7.66,6.66,5.66,4.66,3.66,2.66,1.66,0.66,0.66,0.66,0.66,1.16,2.16,3.16,4.16,5.16,6.16,7.16};
	
	// Initial Starting Position
	unsigned int pathIndex = 0; // Starting path index
	float xPosition = pathX[pathIndex]; // Initial X position
	float yPosition = pathY[pathIndex]; // Initial Y position
	
	// Steering / direction variables
	float steeringAngle = 0.0f;
	float lidarSteeringAngle = 0.0f;
	rc_imu_data_t data; // Struct to hold gyro data
	float gyroZBias; // Gyro Z bias
	int gyroCount = 1;
	double gyroCalTime = atof(argv[1]);
	int maxGyroCount = (int) (gyroCalTime / timeStep);
	int calcGyroBias = 1;
	float gyroHeading = 0.0f; // Initial vehicle heading
	float steerBias = -0.125f; // Steering bias, 0 degrees = -0.125 PWM
	float steeringSaturationAngle = 28.0f; // degrees
	float steeringRateLimit = 40.0f;
	float lastSteeringAngle = 0.0f;
	float wheelBase = 0.333375f; // 0.333375 m (13.125 inches)

	// Initialize hardware first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to run rc_initialize(), are you root?\n");
		return -1;
	}

	// Initialize ESC motor and steering.
	printf("Initializing ESC motor & steering\n");
	while (initializeCount< (int) (0.1f / timeStep)) // Send commands for 0.1 seconds.
	{
		rc_send_servo_pulse_normalized(1, steerBias); // Steer straight
		rc_send_esc_pulse_normalized(2, 0.50f); // Send neutral velocity
		initializeCount++;
		rc_usleep(timeStep*1000000);
	}
	rc_usleep(500000); // Pause for 0.5 second, so that any vibrations do not impact the gyro cal
	printf("Initializing motor and steering complete.\n");

	// Initialize IMU
	rc_imu_config_t conf = rc_default_imu_config();
	if(rc_initialize_imu(&data, conf)){
		fprintf(stderr,"rc_initialize_imu_failed\n");
		return -1;
	}
	
	// Calibrate Gyro
	printf("Calibrating Gyro Bias\n");
	if(rc_read_gyro_data(&data)<0)
	{
		printf("read gyro data failed\n");
	}
	gyroZBias = data.gyro[2]; // Set initial bias to the current gyro reading.
	while (calcGyroBias)
	{
		if(rc_get_state()==EXITING)
		{
			return 0;
		}
		
		if(rc_read_gyro_data(&data)<0)
		{
			printf("read gyro data failed\n");
		};
		gyroZBias = gyroZBias+(data.gyro[2]-gyroZBias)/gyroCount;
	
		gyroCount++;
		if (gyroCount >= maxGyroCount)
		{
			calcGyroBias = 0;
		}
		rc_usleep(timeStep*1000000);
		printf("Elapsed time: %f Calibrating Gyro Bias: Z Bias = %f\n", gyroCount*timeStep, gyroZBias);
	}
	printf("Elapsed time: %f Calibrating Gyro Bias: Z Bias = %f\n", gyroCount*timeStep, gyroZBias);
		
	// Set state to paused, stay in this loop until pause button is pressed
	printf("Press PAUSE button to start race.\n");
	rc_set_pause_released_func(&on_pause_released);
	rc_set_led(GREEN,ON);
	rc_set_led(RED,OFF);
	rc_set_state(PAUSED);
	while(rc_get_state()==PAUSED)
	{
		rc_usleep(100000);
	}
	printf("Pause was pressed, starting race.\n");
	rc_set_led(GREEN,OFF);
	rc_set_led(RED,ON);
	
	printf("timeElapsed, pathIndex, pathX[ii], pathY[ii], steeringAngle, gyroHeading, xPosition, yPosition, velocity, velSetpoint, ESCCommand, integratorError\n");
	
	// Test calling another function
	myPrintFunction();
	
	// Starting main loop
	while(rc_get_state()!=EXITING)
	{
		startTimerNanoSeconds = rc_nanos_since_boot();
		
		// Get gyro data and update heading
		rc_read_gyro_data(&data);
		gyroHeading += (data.gyro[2] - gyroZBias)*timeStep*1.0f; // 1.0125 worked ok for first runs. 1.005 works well, turns inside a little. This gain is needed when the time of the loop is not quite real time.
		// I believe the gyro is under measuring and thus over turning

		// Get encoder value and update x/y position
		encoderPos = rc_get_encoder_pos(1);
		encoderHistory.pop_back(); // Remove from back of encoder history deque
		if (encoderPos > lastEncoderPos)
		{
			encoderHistory.push_front (1); // Add pulse value of 1 to deque. Used for velocity calculation
			float SIncrement = distanceIncrement*(sin(gyroHeading*0.0174532925199f)); // 2.0*PI/360.0 =  0.0174532925199
			float CIncrement = distanceIncrement*(cos(gyroHeading*0.0174532925199f));
			// if (reverse)
			// {
			// 	// distanceIncrement is opposite in reverse
			// 	xPosition = xPosition+SIncrement;
			// 	yPosition = yPosition-CIncrement;
			// }
			// else
			// {
				xPosition = xPosition-SIncrement;
				yPosition = yPosition+CIncrement;
			// }
		}
		else
		{
			encoderHistory.push_front (0); // Add pulse value of 0 to deque
		}
		lastEncoderPos = encoderPos;
		
		// Calculate Velocity from Encoder Pulses
		int encoderHistorySum = 0;
		for (unsigned int ii = 0; ii<encoderHistory.size(); ii++)
		{
			encoderHistorySum += encoderHistory.at(ii);
		}
		float velocity = (float) (encoderHistorySum) * distanceIncrement / velocityAverageTime;
		
		// Determine if car is stuck
		// if (velocity == 0 && reverse == 0)
		// {
		// 	stuckCount++;
		// 	if (stuckCount > (int) (2.0 / timeStep) ) // Put car in reverse after 1 second of being stuck
		// 	{
		// 		reverse = 1;
		// 		forward = 0;
		// 		stuckCount = 0;
		// 	}
		// 	//printf("stuck, stuckCount = %d, (int) (1.0 / timeStep) = %d\n", stuckCount, (int) (1.0 / timeStep));
		// }
		// else
		// {
		// 	stuckCount = 0;
		// }

		// Determine Goal Point
		// While within 1 meter of the desired point, increase the index of desired point
        while (sqrt((pathX[pathIndex] - xPosition)*(pathX[pathIndex] - xPosition) + (pathY[pathIndex] - yPosition)*(pathY[pathIndex] - yPosition)) < 2.0f)
        {
            pathIndex++;
			if (pathIndex >= pathX.size() - 1)
			{
				printf("\n1st break\n");
				break;
			}
        }
	
		if (pathIndex >= pathX.size() - 1)
    	{
    	  	printf("2nd break\n");
    	  	rc_send_servo_pulse_normalized(1, steerBias);
    	  	break;
    	}	

		// VELOCITY SETPOINT & ESC COMMAND (SPEED CONTROL) CALCULATION //
		float velSetpoint;
		// if ( (pathIndex >= 26 && pathIndex <= 60) || (pathIndex >= 96 && pathIndex <= 128) )
		// //if (pathIndex >= 3 && pathIndex <= 11)
		// {
		//  	velSetpoint = curveSpeed; // MEDIUM
		//  	if ( (pathIndex >= 40 && pathIndex <= 49) || (pathIndex >= 99 && pathIndex <= 128) )// || (pathIndex >= 120 && pathIndex <= 128) )
		//  	//if (pathIndex >= 6 && pathIndex <= 11)
		//  	{
		//  		velSetpoint = obstacleSpeed; // SLOW
		//  	}
		// }
		// else
		// {
			 velSetpoint = straightSpeed; // FAST
		// }
	
		// Rate limit the velocity
    	float velRateLim_timeStep = velocityRateLimit * timeStep;
    	if ((velSetpoint - lastVelSetpoint) > velRateLim_timeStep)
    	{
    	    velSetpoint = lastVelSetpoint + velRateLim_timeStep;
    	}
    	else if ((-velSetpoint + lastVelSetpoint) > velRateLim_timeStep)
    	{
    	    velSetpoint = lastVelSetpoint - velRateLim_timeStep;
    	}
        lastVelSetpoint = velSetpoint;
		
		float eVelSetpoint = velSetpoint - velocity;
		integratorError = integratorError + eVelSetpoint * timeStep;
		float ESCCommand = eVelSetpoint * propGain + integratorError * intGain;
		if (ESCCommand < 0.0f)
		{
			ESCCommand = 0.0f;
			if (intGain>0.0f)
			{
				integratorError = integratorError - eVelSetpoint * timeStep; // Anti-windup
			}
		}
		if (ESCCommand > 0.25f)
		{
			ESCCommand = 0.25f;
			if (intGain>0.0f)
			{
				integratorError = integratorError - eVelSetpoint * timeStep; // Anti-windup
			}
		}
		ESCCommand = ESCCommand + 0.5f; // 0.5 = zero speed. Add this offset.
		
		// STEERING ANGLE CALCULATION //
		// Calculate the desired steering angle based on the Pure Pursuit method
    	float dE = pathX[pathIndex] - xPosition;
    	float dN = pathY[pathIndex] - yPosition;
    	float dist2DesiredPoint = sqrt(dE*dE + dN*dN);
    	float goalPointAngle = -1.0f*atan2(dE,dN); // same as from matlab script: float matlabangle = -(PI/2-atan2(dN,dE));
		float alpha = goalPointAngle - gyroHeading*0.0174532925199f;
    	float k = 2.0f * sin(alpha) / dist2DesiredPoint;

    	steeringAngle = atan(k*wheelBase) * 57.295779513082323f * (2.5f);	// 360/(2*PI) = 57.295779513082323
    
    	// Saturate the steering angle
    	steeringAngle = (steeringAngle > steeringSaturationAngle) ? steeringSaturationAngle : steeringAngle; // Max
    	steeringAngle = (steeringAngle < -steeringSaturationAngle) ? -steeringSaturationAngle : steeringAngle; // Min

    	// Rate limit the steering angle
    	float strRateLim_timeStep = steeringRateLimit * timeStep;
    	if ((steeringAngle - lastSteeringAngle) > strRateLim_timeStep) 
    	{
    		steeringAngle = lastSteeringAngle + strRateLim_timeStep;
    	}
    	else if ((-steeringAngle + lastSteeringAngle) > strRateLim_timeStep)
    	{
    		steeringAngle = lastSteeringAngle - strRateLim_timeStep;
    	}
		lastSteeringAngle = steeringAngle;
    	
    	// Convert from steering angle to PWM
    	float steeringPWM = -0.0275f*steeringAngle + steerBias; // steeringSlope = (0.68 - (-0.75)) / (-29 - (23)) = 1.43 / -52 = -0.0275 ; // (y2 - y1) / (x2 - x1)

		// Send steering annd velocity command
		// if (1);//(forward)
		// {
		 	rc_send_esc_pulse_normalized(2, ESCCommand);
		 	rc_send_servo_pulse_normalized(1, steeringPWM);
		// 	reverseSet = 0;
		// 	neutralSet = 0;
		// 	//printf("forward\n");
		// }
		
		// if (0);//(reverse)
		// {
		// 	//rc_send_esc_pulse_normalized(2, 0.42);
		// 	//rc_send_servo_pulse_normalized(1, steerBias); // Steer straight
		// 	reverseCount++;
		// 	if (reverseCount > (int) (2.0 / timeStep)) // Reverse for 1 second
		// 	{
		// 		reverseCount = 0;
		// 		forward = 1;
		// 		stuck = 0;
		// 		reverse = 0;
		// 	}
			
		// 	if (!reverseSet)
		// 	//send a backwards signal for 1 sample
		// 	{
		// 		rc_send_esc_pulse_normalized(2, 0.43);
		// 		reverseSet = 1;
		// 		//printf("back for 1 sample\n");
		// 	}
		// 	else if (!neutralSet)
		// 	{
		// 		rc_send_esc_pulse_normalized(2, 0.5);
		// 		neutralCount++;
		// 		if (neutralCount > (int) (0.1 / timeStep))
		// 		{
		// 			neutralSet = 1;
		// 			neutralCount = 0;
		// 		}
		// 		//printf("neutral for 0.1 seconds\n");
		// 	}
		// 	else{
		// 		rc_send_esc_pulse_normalized(2, 0.42);
		// 		//printf("backwards\n");
		// 	}
			
		// 	// Reset velocity control variables
		// 			        	printf("stuck vel setpoint = %f\n",velSetpoint );
  //      	printf("stuck last vel setpoint = %f\n",lastVelSetpoint );
		// 	lastVelSetpoint = 0;
		// 	integratorError = 0;
		// 	//printf("reverse\n");
		// 	//printf("reverse, reverseCount = %d, (int) (1.0 / timeStep) = %d\n", reverseCount, (int) (1.0 / timeStep));
		// }
		

		// Print to the screen
		if (timeStepCount % 100 == 0)
		{
		//printf("\r");
	    printf("%7.2f   | %8d | %10.2f | %10.2f | %10.2f | % 10.2f | % 10.2f | % 9.2f | % 8.2f | % 10.2f | % 10.2f | % 10.2f \n",timeElapsed, pathIndex, pathX[pathIndex], pathY[pathIndex], steeringAngle, gyroHeading, xPosition, yPosition, velocity, velSetpoint, ESCCommand, integratorError);
		//fflush(stdout);
		}

		timeElapsed += timeStep;
		timeStepCount++;
		
		// Stay in this loop until the amount of time for the loop has reached timeStep
		while ( (rc_nanos_since_boot() - startTimerNanoSeconds) < ( (uint64_t) (timeStep * 1000000000)) )
		{
			//printf("time Step time =  %lldns\n", rc_nanos_since_boot()-startTimerNanoSeconds);
			// Wait until 1/frequ milliseconds has elapsed
		}
	}
	
	rc_send_servo_pulse_normalized(1, steerBias);
	rc_send_esc_pulse_normalized(2, 0.5f); //Apply the brake at the end. Don't do this. Let the car roll to a stop instead to ensure it crosses the finish line.
	rc_cleanup();
	return 0;
}
